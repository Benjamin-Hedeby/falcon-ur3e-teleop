# Falcon → UR3e teleoperation with admittance control

A Novint Falcon haptic controller teleoperates a Universal Robots UR3e (ROS 2 Jazzy, Ubuntu
24.04). It was built for a peg-in-hole task: guiding a gripper housing into a tapered socket, a
mock-up of breeding-blanket handling for tokamak maintenance. Features:

- **Joystick, position and twist modes:** the Falcon grip drives the tool, with force feedback
  from the UR3e's wrist force/torque sensor.
- **Admittance Insert:** a virtual mass-damper-spring that yields to contact forces and lets the
  taper guide the peg in. UR force mode is available as an alternative.
- **Contact limit:** a force/torque limit that stops the arm until the clutch is re-pressed.
- **Data collection:** a trial recorder (bag + CSV + parameters) and a 500 Hz analysis script.
- **MuJoCo simulation** of the peg and socket from the CAD: offline runs and videos, or a live
  stand-in for the robot that the real teleop stack drives.

## Layout

The repository root is a ROS 2 workspace.

| Path | Contents |
|---|---|
| `src/falcon_ros2/` | The ROS 2 package: `falcon_node` (C++, libnifalcon), the teleop bridge `scripts/falcon_ur_teleop.py`, admittance and contact-limit modules, launch files, data-collection scripts, desktop launcher, tests |
| `mujoco/peg_in_hole/` | MuJoCo simulation: offline insertion runs (`sim.py`), live ROS stand-in for the UR3e (`sim_ros.py`), mesh preparation tools and the generated meshes. See its README |
| `mujoco/third_party/ur3e/` | UR3e MuJoCo model (Universal Robots' meshes and parameters, BSD-3) |
| `cad/` | Peg (`Housing_Folding_Gripper_v2.STL`) and socket (`BLANKET_SEGM_OUTBOARD_MIDDLE_GRIPPER_v2.STL`), in mm |
| `patches/` | `ur_driver_twist_timeout.patch`: zero velocity if twist commands stop for 0.1 s |
| `ur3e.repos` | The UR ROS 2 driver version used |
| `docs/` | Data-collection spec |

## Setup

Ubuntu 24.04 with ROS 2 Jazzy (`ros-jazzy-desktop`).

```bash
git clone <this repo> ~/ur3e_ws && cd ~/ur3e_ws

# UR driver (from source, with the twist-timeout patch)
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep \
    cmake build-essential git libusb-1.0-0-dev libboost-all-dev
vcs import src < ur3e.repos
git -C src/Universal_Robots_ROS2_Driver apply ../../patches/ur_driver_twist_timeout.patch
rosdep install --from-paths src --ignore-src -y

# libnifalcon (Falcon driver library), upstream commit f7c727a, unmodified
git clone https://github.com/libnifalcon/libnifalcon /tmp/libnifalcon
git -C /tmp/libnifalcon checkout f7c727aa24403ea10dc1507d7d396eeb7d55564a
cmake -S /tmp/libnifalcon -B /tmp/libnifalcon/build -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -C /tmp/libnifalcon/build -j"$(nproc)" && sudo make -C /tmp/libnifalcon/build install && sudo ldconfig

# USB access to the Falcon (VID:PID 0403:cb48), then replug it
sudo cp src/falcon_ros2/udev/99-novint-falcon.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Without root access, libnifalcon can be installed to `~/.local` instead
(`-DCMAKE_INSTALL_PREFIX=$HOME/.local`); `falcon_ros2`'s CMake looks there too.

MuJoCo simulation (separate Python environment):

```bash
python3 -m venv mujoco/.venv && mujoco/.venv/bin/pip install -r mujoco/requirements.txt
```

## Running

Desktop launcher: `src/falcon_ros2/scripts/launch_falcon_teleop.sh` asks for the target, mode,
logging and force feedback. Copy `src/falcon_ros2/falcon-teleop.desktop` to `~/Desktop` and fix
the path in its `Exec=` line. Or use the launch files directly:

```bash
ros2 launch falcon_ros2 ur_real_teleop.launch.py robot_ip:=<IP> mode:=twist   # real UR3e
ros2 launch falcon_ros2 ur_mujoco_teleop.launch.py                           # MuJoCo peg-in-hole
ros2 launch falcon_ros2 ur_sim_teleop.launch.py                              # RViz, mock hardware
ros2 launch falcon_ros2 ur_gz_teleop.launch.py                               # Gazebo
```

Home the Falcon when its LED turns red (pull the grip out, push it in). Hold any grip button as
the clutch and push the grip like a joystick. In twist mode, tap the minus button for the
admittance Insert. Most settings are live ROS parameters (`ros2 param set /falcon_ur_teleop ...`);
the header of `falcon_ur_teleop.py` lists them.

**Safety:** never use `ur_sim_teleop` / `ur_gz_teleop` or `move_to_ready.py` on the real robot:
they move the arm to a fixed pose without collision checks. On the real robot start with the
pendant speed slider at 10–20 %, the pendant TCP at the peg tip and the payload set.

## Data collection

```bash
ros2 param load /falcon_ur_teleop $(ros2 pkg prefix falcon_ros2)/share/falcon_ros2/config/data_collection.yaml
ros2 run falcon_ros2 record_trial.sh <operator> <condition> <repeat> ~/trials   # Ctrl+C to stop
ros2 run falcon_ros2 analyze_trial.py ~/trials/<name> ~/trials/<name>.csv
```

`record_trial.sh` zeroes the wrist sensor, saves the bridge parameters and trial metadata, logs
the bridge CSV and records a bag. `analyze_trial.py` resamples a trial to 500 Hz: tool pose,
Savitzky-Golay velocity and acceleration, wrench in sensor and base frames, commanded target and
contact detection, plus force plots. See `docs/data-collection-changes.md`.

## Tests

```bash
colcon test --packages-select falcon_ros2 && colcon test-result --verbose
```

## Licences

- `falcon_ros2`: BSD (see `package.xml`).
- `mujoco/third_party/ur3e`: BSD-3, Universal Robots (see its `LICENSE` and `sources.json`).
- libnifalcon and the UR driver are fetched from upstream under their own licences.
