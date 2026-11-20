// ROS 2 node for the Novint Falcon using libnifalcon.
//
// Publishes:
//   falcon/position  geometry_msgs/PointStamped  end-effector position [m], offset-corrected
//   falcon/buttons   sensor_msgs/Joy             grip buttons (4-button grip)
// Subscribes:
//   falcon/force     geometry_msgs/Vector3       force to render [N], same axes as falcon/position
//
// The USB/haptic loop runs in its own thread as fast as the device allows;
// ROS publishing runs on a timer at `publish_rate` Hz.
//
// Local haptic effects, added to falcon/force inside the ~1 kHz loop (all settable
// live with `ros2 param set /falcon_node <name> <value>`):
//   hold_force_idle / _active    virtual friction [N] with no button / any button held
//   hold_stiffness               (the clutch): an anchor trails the grip by at most
//                                hold_force/hold_stiffness and a spring pulls toward it,
//                                so the grip stays where it is left but moves when pushed
//   gravity_comp                 constant upward force [N] against the grip's weight
//   damping                      viscous damping at any speed [N*s/m]
//   speed_damping / free_speed   extra damping above free_speed [m/s], only while a button
//                                is held, so fast robot motions get progressively heavier

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "falcon/core/FalconDevice.h"
#include "falcon/comm/FalconCommLibUSB.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"

using namespace libnifalcon;
using namespace std::chrono_literals;

class FalconNode : public rclcpp::Node
{
public:
  FalconNode() : Node("falcon_node")
  {
    device_index_ = declare_parameter<int>("device_index", 0);
    publish_rate_ = declare_parameter<double>("publish_rate", 500.0);
    max_force_ = declare_parameter<double>("max_force", 5.0);          // N, Falcon peaks ~9 N
    force_timeout_ = declare_parameter<double>("force_timeout", 0.1);  // s, zero force if stale
    z_offset_ = declare_parameter<double>("z_offset", 0.11);          // m, centers the workspace
    frame_id_ = declare_parameter<std::string>("frame_id", "falcon");
    hold_force_idle_ = declare_parameter<double>("hold_force_idle", 0.8);    // N, no button
    hold_force_active_ = declare_parameter<double>("hold_force_active", 1.2);  // N, clutch held
    hold_stiffness_ = declare_parameter<double>("hold_stiffness", 300.0);  // N/m
    gravity_comp_ = declare_parameter<double>("gravity_comp", 0.0);        // N, +up
    damping_ = declare_parameter<double>("damping", 3.0);                  // N*s/m
    speed_damping_ = declare_parameter<double>("speed_damping", 30.0);     // N*s/m
    free_speed_ = declare_parameter<double>("free_speed", 0.05);           // m/s, hand speed
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto & p : params) {
          const auto & n = p.get_name();
          if (n == "hold_force_idle") hold_force_idle_ = p.as_double();
          else if (n == "hold_force_active") hold_force_active_ = p.as_double();
          else if (n == "hold_stiffness") hold_stiffness_ = p.as_double();
          else if (n == "gravity_comp") gravity_comp_ = p.as_double();
          else if (n == "damping") damping_ = p.as_double();
          else if (n == "speed_damping") speed_damping_ = p.as_double();
          else if (n == "free_speed") free_speed_ = p.as_double();
          else if (n == "max_force") max_force_ = p.as_double();
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    if (!initDevice()) {
      throw std::runtime_error("Could not initialize the Falcon");
    }

    pos_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("falcon/position", 10);
    btn_pub_ = create_publisher<sensor_msgs::msg::Joy>("falcon/buttons", 10);
    force_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      "falcon/force", 10,
      [this](const geometry_msgs::msg::Vector3 & msg) {
        // NaN would slip past the max_force clamp (NaN > x is false) and reach the motors
        if (!std::isfinite(msg.x) || !std::isfinite(msg.y) || !std::isfinite(msg.z)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring non-finite falcon/force");
          return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        // Same axis swap as publish(): ROS y is libnifalcon's raw axis 2 (depth), ROS z is raw axis 1
        cmd_force_ = {msg.x, msg.z, msg.y};
        last_force_time_ = now();
      });

    last_force_time_ = now();
    running_ = true;
    io_thread_ = std::thread(&FalconNode::ioLoop, this);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish(); });

    RCLCPP_INFO(get_logger(), "Falcon ready. Publishing on falcon/position, falcon/buttons.");
  }

  ~FalconNode() override
  {
    running_ = false;
    if (io_thread_.joinable()) io_thread_.join();
    dev_.setForce({0.0, 0.0, 0.0});
    dev_.runIOLoop();
    dev_.close();
  }

private:
  bool initDevice()
  {
    dev_.setFalconComm<FalconCommLibUSB>();
    dev_.setFalconFirmware<FalconFirmwareNovintSDK>();
    dev_.setFalconKinematic<FalconKinematicStamper>();
    dev_.setFalconGrip<FalconGripFourButton>();

    unsigned int count = 0;
    dev_.getDeviceCount(count);
    RCLCPP_INFO(get_logger(), "Falcons found: %u", count);
    if (count == 0) {
      RCLCPP_ERROR(get_logger(), "No Falcon found. Is it powered and passed through to this machine (usbipd / VM USB menu)?");
      return false;
    }
    if (!dev_.open(device_index_)) {
      RCLCPP_ERROR(get_logger(), "Could not open Falcon %d (permissions? udev rule?)", device_index_);
      return false;
    }

    // The Falcon has no persistent firmware; load it on every power-up.
    if (!dev_.isFirmwareLoaded()) {
      RCLCPP_INFO(get_logger(), "Loading firmware...");
      bool ok = false;
      for (int i = 0; i < 10 && !ok; ++i) {
        ok = dev_.getFalconFirmware()->loadFirmware(
          true, NOVINT_FALCON_NVENT_FIRMWARE_SIZE,
          const_cast<uint8_t *>(NOVINT_FALCON_NVENT_FIRMWARE));
      }
      if (!ok) {
        RCLCPP_ERROR(get_logger(), "Firmware load failed. Replug USB + power and retry.");
        return false;
      }
    }

    // Homing: user pulls the grip all the way out and pushes it back in.
    dev_.getFalconFirmware()->setHomingMode(true);
    dev_.getFalconFirmware()->setLEDStatus(FalconFirmware::RED_LED);
    RCLCPP_INFO(get_logger(), "Home the Falcon: pull the grip fully out, then push it in.");
    auto start = std::chrono::steady_clock::now();
    auto last_status = start;
    while (rclcpp::ok()) {
      dev_.runIOLoop();
      if (dev_.getFalconFirmware()->isHomed()) break;
      // Once a second, show which axes are homed, the raw encoders and how many packets came
      // back, so a stuck homing can be told apart from a dead link (packets stays at 0).
      if (std::chrono::steady_clock::now() - last_status > 1s) {
        last_status = std::chrono::steady_clock::now();
        auto fw = dev_.getFalconFirmware();
        unsigned int h = fw->getHomingModeStatus();
        auto enc = fw->getEncoderValues();
        RCLCPP_INFO(get_logger(), "Homing: axes homed [%c%c%c], encoders [%d %d %d], packets %lu",
                    (h & FalconFirmware::ENCODER_1_HOMED) ? '1' : '-',
                    (h & FalconFirmware::ENCODER_2_HOMED) ? '2' : '-',
                    (h & FalconFirmware::ENCODER_3_HOMED) ? '3' : '-',
                    enc[0], enc[1], enc[2], static_cast<unsigned long>(fw->getLoopCount()));
      }
      if (std::chrono::steady_clock::now() - start > 60s) {
        RCLCPP_ERROR(get_logger(), "Homing timed out.");
        return false;
      }
    }
    if (!rclcpp::ok()) return false;  // Ctrl+C while homing
    dev_.getFalconFirmware()->setLEDStatus(FalconFirmware::GREEN_LED);
    RCLCPP_INFO(get_logger(), "Homed.");
    return true;
  }

  static double norm(const std::array<double, 3> & v)
  {
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  }

  // Total force for the next I/O cycle, in libnifalcon's raw axes. Caller holds mtx_.
  std::array<double, 3> computeForce(const std::array<double, 3> & pos, double dt)
  {
    std::array<double, 3> force{0.0, 0.0, 0.0};
    if ((now() - last_force_time_).seconds() < force_timeout_) {
      force = cmd_force_;
    }

    if (!haptics_init_) {
      anchor_ = pos;
      prev_pos_ = pos;
      haptics_init_ = true;
    }
    // Low-passed velocity: raw 1 kHz differences are dominated by encoder quantization
    if (dt > 0.0 && dt < 0.05) {
      double a = dt / (dt + 0.01);  // ~16 Hz cutoff
      for (int i = 0; i < 3; ++i) vel_[i] += a * ((pos[i] - prev_pos_[i]) / dt - vel_[i]);
    }
    prev_pos_ = pos;

    // Virtual friction: drag the anchor along once the grip is more than `slack` away,
    // so the spring force never exceeds hold_force
    std::array<double, 3> e{pos[0] - anchor_[0], pos[1] - anchor_[1], pos[2] - anchor_[2]};
    bool clutch = buttons_ != 0;
    double hold_force = clutch ? hold_force_active_ : hold_force_idle_;
    double slack = hold_stiffness_ > 0.0 ? hold_force / hold_stiffness_ : 0.0;
    double en = norm(e);
    if (en > slack) {
      for (int i = 0; i < 3; ++i) {
        e[i] *= slack / en;
        anchor_[i] = pos[i] - e[i];
      }
    }

    double speed = norm(vel_);
    double b = damping_;
    if (clutch && speed > free_speed_) b += speed_damping_ * (speed - free_speed_) / speed;
    for (int i = 0; i < 3; ++i) force[i] -= hold_stiffness_ * e[i] + b * vel_[i];
    force[1] += gravity_comp_;  // raw axis 1 is vertical (ROS z)

    // Clamp force magnitude for safety
    double mag = norm(force);
    if (mag > max_force_) {
      for (auto & f : force) f *= max_force_ / mag;
    }
    return force;
  }

  void ioLoop()
  {
    uint64_t loops = 0;
    auto t0 = std::chrono::steady_clock::now();
    auto t_prev = t0;
    std::array<double, 3> force{0.0, 0.0, 0.0};

    while (running_ && rclcpp::ok()) {
      dev_.setForce(force);
      if (!dev_.runIOLoop()) continue;  // no fresh data this cycle

      auto t_now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(t_now - t_prev).count();
      t_prev = t_now;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        position_ = dev_.getPosition();
        buttons_ = dev_.getFalconGrip()->getDigitalInputs();
        have_data_ = true;
        force = computeForce(position_, dt);
      }

      // Report the achieved loop rate every 5 s; handy for judging usbip latency
      if (++loops % 1000 == 0) {
        double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (dt > 5.0) {
          RCLCPP_INFO(get_logger(), "I/O loop rate: %.0f Hz", loops / dt);
          loops = 0;
          t0 = std::chrono::steady_clock::now();
        }
      }
    }
  }

  void publish()
  {
    std::array<double, 3> pos;
    unsigned int buttons;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!have_data_) return;
      pos = position_;
      buttons = buttons_;
    }

    geometry_msgs::msg::PointStamped p;
    p.header.stamp = now();
    p.header.frame_id = frame_id_;
    // libnifalcon's raw axis 2 is the push/pull (depth) axis -- that's the one
    // that needs the centering offset, and it maps to ROS's Y here, not Z.
    p.point.x = pos[0];
    p.point.y = pos[2] - z_offset_;
    p.point.z = pos[1];
    pos_pub_->publish(p);

    sensor_msgs::msg::Joy j;
    j.header = p.header;
    for (int i = 0; i < 4; ++i) j.buttons.push_back((buttons >> i) & 1);
    btn_pub_->publish(j);
  }

  FalconDevice dev_;
  std::thread io_thread_;
  std::atomic<bool> running_{false};
  std::mutex mtx_;

  std::array<double, 3> position_{0.0, 0.0, 0.0};
  std::array<double, 3> cmd_force_{0.0, 0.0, 0.0};
  unsigned int buttons_ = 0;
  bool have_data_ = false;
  rclcpp::Time last_force_time_;

  // Local haptics state (raw axes), guarded by mtx_
  bool haptics_init_ = false;
  std::array<double, 3> anchor_{0.0, 0.0, 0.0};
  std::array<double, 3> prev_pos_{0.0, 0.0, 0.0};
  std::array<double, 3> vel_{0.0, 0.0, 0.0};
  double hold_force_idle_, hold_force_active_, hold_stiffness_, gravity_comp_, damping_, speed_damping_, free_speed_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  int device_index_;
  double publish_rate_, max_force_, force_timeout_, z_offset_;
  std::string frame_id_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pos_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr btn_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr force_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<FalconNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("falcon_node"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
