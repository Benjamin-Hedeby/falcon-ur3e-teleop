"""Live MuJoCo UR3e for the real teleop stack: stands in for the UR3e + ur_robot_driver in twist
mode, so Falcon -> falcon_node -> falcon_ur_teleop.py (mode:=twist) drives the simulated arm.

Provides what the bridge uses from the real driver:
  sub  /twist_controller/twist (TwistStamped, frame 'base'); zero after 0.1 s without a command,
       like the patched twist_controller
  pub  /force_torque_sensor_broadcaster/wrench (frame tool0_controller = peg tip, 250 Hz; force
       and torque about the TCP, like the UR)
       /joint_states (100 Hz)
       /speed_scaling_state_broadcaster/speed_scaling (100 %)
       /io_and_status_controller/robot_program_running (true, latched)
       TF base_link -> base (static), base_link -> tool0 (flange), base_link -> tool0_controller
  srv  /controller_manager/list_controllers, /controller_manager/switch_controller (report the
       twist and force-mode controllers active), /io_and_status_controller/zero_ftsensor
       /force_mode_controller/start_force_mode, stop_force_mode: refused (UR force mode is not
       simulated; use the default insert_method:=admittance)
       /mujoco/reset (Trigger): arm back to the start pose

Run with ROS sourced, using the MuJoCo venv (it sees rclpy through ROS's PYTHONPATH):
    source /opt/ros/jazzy/setup.bash && source ~/ur3e_ws/install/setup.bash
    ~/ur3e_ws/mujoco/.venv/bin/python sim_ros.py [--offset DX DY]
or all of it at once: ros2 launch falcon_ros2 ur_mujoco_teleop.launch.py
"""
import argparse
import multiprocessing as mp
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import TransformStamped, TwistStamped, WrenchStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from ur_msgs.srv import SetForceMode

import sim

TWIST_TIMEOUT = 0.1    # s, like the patched twist_controller
WRENCH_PERIOD = 0.004  # s of sim time (250 Hz; the real sensor streams 500 Hz)
TF_PERIOD = 0.01       # s of sim time
VIEW_PERIOD = 1 / 30   # s of wall time
SPIN_PERIOD = 0.005    # s of sim time between ROS callback checks. No executor thread (it would
                       # fight the physics loop for the GIL), and not every step: each rclpy
                       # spin_once costs a few hundred microseconds; twists arrive at 100 Hz
ACTIVE = ['twist_controller', 'force_mode_controller', 'joint_state_broadcaster',
          'force_torque_sensor_broadcaster', 'io_and_status_controller',
          'speed_scaling_state_broadcaster']


def wrist_wrench(s):
    """Environment-on-tool (force, torque about the peg tip / TCP), world = base_link, including
    the constant offset from the peg's weight that the caller zeroes away."""
    f = s.wrist_force()
    p_ft, R_ft = sim.site_pose(s.m, s.d, 'ft_sensor')
    p_tip, _ = sim.site_pose(s.m, s.d, 'peg_tip')
    tau_ft = s.sensor_sign * (R_ft @ s.d.sensor('ft_torque').data)
    return f, tau_ft + np.cross(p_ft - p_tip, f)  # moment moved from the flange to the TCP


def quat_from_mat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.flatten())
    return q  # w, x, y, z


class FakeDriver(Node):
    def __init__(self):
        super().__init__('mujoco_ur3e')
        self.lock = threading.Lock()
        self.v_cmd = np.zeros(3)  # base_link
        self.v_time = 0.0
        self.zero_request = False
        self.reset_request = False

        self.create_subscription(TwistStamped, '/twist_controller/twist', self.on_twist, 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, '/force_torque_sensor_broadcaster/wrench', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.scaling_pub = self.create_publisher(Float64, '/speed_scaling_state_broadcaster/speed_scaling', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        running = self.create_publisher(Bool, '/io_and_status_controller/robot_program_running', latched)
        running.publish(Bool(data=True))
        self.create_timer(0.1, lambda: self.scaling_pub.publish(Float64(data=100.0)))
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        base = TransformStamped()
        base.header.frame_id, base.child_frame_id = 'base_link', 'base'
        base.transform.rotation.z, base.transform.rotation.w = 1.0, 0.0  # base = base_link turned pi about z
        self.static_tf.sendTransform(base)

        self.create_service(ListControllers, '/controller_manager/list_controllers', self.on_list)
        self.create_service(SwitchController, '/controller_manager/switch_controller',
                            lambda req, res: setattr(res, 'ok', True) or res)
        self.create_service(Trigger, '/io_and_status_controller/zero_ftsensor', self.on_zero)
        self.create_service(SetForceMode, '/force_mode_controller/start_force_mode', self.on_force_mode)
        self.create_service(Trigger, '/force_mode_controller/stop_force_mode',
                            lambda req, res: setattr(res, 'success', True) or res)
        self.create_service(Trigger, '/mujoco/reset', self.on_reset)

    def on_twist(self, msg):
        if msg.header.frame_id != 'base':
            self.get_logger().warn(f"Twist in frame '{msg.header.frame_id}' ignored (expects 'base')",
                                   throttle_duration_sec=5.0)
            return
        t = msg.twist.linear
        with self.lock:
            self.v_cmd = np.array([-t.x, -t.y, t.z])  # base -> base_link
            self.v_time = time.monotonic()

    def command(self):
        with self.lock:
            fresh = time.monotonic() - self.v_time < TWIST_TIMEOUT
            return self.v_cmd.copy() if fresh else np.zeros(3)

    def on_list(self, req, res):
        res.controller = [ControllerState(name=n, state='active') for n in ACTIVE]
        return res

    def on_zero(self, req, res):
        self.zero_request = True
        res.success = True
        return res

    def on_force_mode(self, req, res):
        self.get_logger().warn('UR force mode is not simulated; use insert_method:=admittance')
        res.success = False
        return res

    def on_reset(self, req, res):
        self.reset_request = True
        res.success = True
        return res

    def publish_state(self, s, offset, stamp, wrench=True, tf=True):
        p_tip, R_tip = sim.site_pose(s.m, s.d, 'peg_tip')
        if wrench:
            # Environment-on-tool force, zeroed like the UR's payload-compensated TCP force,
            # in the TCP frame
            f, tau = wrist_wrench(s)
            f, tau = R_tip.T @ (f - offset[0]), R_tip.T @ (tau - offset[1])
            w = WrenchStamped()
            w.header.stamp, w.header.frame_id = stamp, 'tool0_controller'
            w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = f
            w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = tau
            self.wrench_pub.publish(w)
        if tf:
            p_fl, R_fl = sim.site_pose(s.m, s.d, 'ft_sensor')
            msgs = []
            for child, p, R in (('tool0', p_fl, R_fl), ('tool0_controller', p_tip, R_tip)):
                t = TransformStamped()
                t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'base_link', child
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p
                q = quat_from_mat(R)
                t.transform.rotation.w, t.transform.rotation.x = q[0], q[1]
                t.transform.rotation.y, t.transform.rotation.z = q[2], q[3]
                msgs.append(t)
            self.tf.sendTransform(msgs)
            js = JointState()
            js.header.stamp = stamp
            js.name = list(sim.JOINT_NAMES)
            js.position, js.velocity = s.d.qpos.tolist(), s.d.qvel.tolist()
            self.joint_pub.publish(js)


def viewer_process(hole_offset, qpos_shared, closed):
    """Viewer in its own process: in the physics process its render thread competed for the GIL
    and the simulation fell behind real time. Shows the arm at the joint angles in qpos_shared."""
    s = sim.Sim(hole_offset, viewer=True, see_through=True, fast=True)
    try:
        while s.viewer.is_running():
            with qpos_shared.get_lock():
                s.d.qpos[:] = qpos_shared[:]
            mujoco.mj_kinematics(s.m, s.d)
            s.viewer.sync()
            time.sleep(VIEW_PERIOD)
    finally:
        closed.value = True
        s.viewer.close()


def reset_pose(s):
    start = s.hole_center + [0, 0, sim.HOLE_TOP + sim.START_GAP]
    q, _ = sim.ik(s.m, start, sim.R_TOOL_DOWN, s.q_ref)
    s.d.qpos[:] = q
    s.d.qvel[:] = 0.0
    s.q_ref = q.copy()
    mujoco.mj_forward(s.m, s.d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--offset', type=float, nargs=2, default=[0.010, 0.006], metavar=('DX', 'DY'),
                    help='hole offset from the start position, m')
    ap.add_argument('--headless', action='store_true', help='no viewer window (for tests)')
    ap.add_argument('--duration', type=float, default=0.0, help='stop after this many s (0: never)')
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = FakeDriver()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    s = sim.Sim(args.offset, viewer=False, see_through=True, fast=True)
    ctx = mp.get_context('spawn')  # no fork: the parent already runs rclpy threads
    qpos_shared = ctx.Array('d', s.m.nq)
    viewer_closed = ctx.Value('b', False)
    viewer = None
    if not args.headless:
        qpos_shared[:] = s.d.qpos
        viewer = ctx.Process(
            target=viewer_process, args=(args.offset, qpos_shared, viewer_closed), daemon=True)
        viewer.start()
    s.calibrate_sign()
    offset = wrist_wrench(s)  # the peg's weight, removed like the UR's payload compensation
    node.get_logger().info('Simulated UR3e ready: drive it with the Falcon (twist mode). '
                           'Close the viewer window or Ctrl+C to stop.')
    dt = s.m.opt.timestep
    t_sim, next_wrench, next_tf, next_view, next_spin = 0.0, 0.0, 0.0, 0.0, 0.0
    behind_total = 0.0
    wall0 = time.monotonic()
    try:
        while rclpy.ok() and not viewer_closed.value:
            if args.duration and t_sim >= args.duration:
                break
            if node.reset_request:
                node.reset_request = False
                reset_pose(s)
                node.get_logger().info('Reset to the start pose')
            if node.zero_request:
                node.zero_request = False
                offset = wrist_wrench(s)
            s.step(node.command())
            t_sim += dt
            stamp = node.get_clock().now().to_msg()
            if t_sim >= next_wrench:
                node.publish_state(s, offset, stamp, wrench=True, tf=t_sim >= next_tf)
                next_wrench += WRENCH_PERIOD
                if t_sim >= next_tf:
                    next_tf += TF_PERIOD
            now = time.monotonic() - wall0
            if viewer is not None and now >= next_view:
                with qpos_shared.get_lock():
                    qpos_shared[:] = s.d.qpos
                next_view = now + VIEW_PERIOD
            if t_sim >= next_spin:
                executor.spin_once(timeout_sec=0.0)
                next_spin += SPIN_PERIOD
            ahead = t_sim - (time.monotonic() - wall0)
            if ahead > 0.0005:
                time.sleep(ahead)
            if ahead < -0.25:
                node.get_logger().warn(f'Simulation running slower than real time ({-ahead:.2f} s '
                                       'behind); catching up', throttle_duration_sec=5.0)
                behind_total += -ahead
                wall0 = time.monotonic() - t_sim  # don't sprint to catch up
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if t_sim > 0:
            print(f'simulated {t_sim:.1f} s, fell behind real time by {behind_total:.2f} s in total')
        if viewer is not None and viewer.is_alive():
            viewer.terminate()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
