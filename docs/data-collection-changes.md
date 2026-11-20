# Task: prepare falcon_ros2 for contact-data collection (admittance control)

## Context

This repo contains `falcon_ros2` (ROS 2 Jazzy, Ubuntu 24.04): a Novint Falcon teleoperates a UR3e through a Python bridge, `scripts/falcon_ur_teleop.py`. In joystick (rate) mode the grip offset becomes a tool velocity. A 20 Hz command loop moves a target, solves IK through MoveIt `/compute_ik` and sends one-point goals to `scaled_joint_trajectory_controller`. A 200 Hz force loop sends wrist force/torque feedback to the Falcon on `/falcon/force`. The bridge can log a CSV (`log_file` parameter, 200 rows/s).

We will use this setup to **collect data for tuning an admittance controller**. An operator uses the Falcon to guide a gripper on the UR3e into a slot (a mock-up of breeding-blanket handling for tokamak maintenance). We need force/torque together with position, velocity and acceleration, recorded safely and labelled per trial.

**Read `scripts/falcon_ur_teleop.py`, `src/falcon_node.cpp`, the launch files and `CMakeLists.txt`/`package.xml` before changing anything.** Parameter and topic names below come from a system overview dated 20 Oct 2026. If the code differs, follow the code and tell me what differed.

## Ground rules

- Keep normal teleop behaviour unchanged, except for the new contact limit, which is on by default.
- All new parameters must be **live-tunable** (`ros2 param set`), declared with descriptions, using floats (`0.0`, not `0`), the same way as the existing parameters.
- Append new CSV columns **at the end**, so existing analysis scripts keep working.
- No custom message types. Use standard messages (`geometry_msgs`, `std_msgs`, `std_srvs`).
- Don't touch `move_to_ready.py` or the simulation launch files' start-pose logic.
- **Never run anything against the real robot.** Test only in simulation (`ur_sim_teleop.launch.py` / `ur_gz_teleop.launch.py`) and with unit tests.
- Don't commit or push unless I ask.

## Required changes

### 1. Contact force/torque limit (safety)

There is currently no contact-force limit. In joystick mode the target can run up to 5 cm ahead of the tool, so pushing into the slot builds force quickly.

- New parameters:
  - `contact_force_limit`: N, default `20.0`. `0.0` disables it.
  - `contact_torque_limit`: Nm, default `2.0`. `0.0` disables it.
  - `contact_backoff`: m, default `0.0`. Optional small retreat along the measured force direction after a trip.
- Check in the **200 Hz force loop**, not the 20 Hz command loop. Use the contact wrench (raw sensor minus the clutch-time zero), with at most a light filter (~50 Hz) so noise spikes don't trip it. The 5 Hz haptic filter is too slow for this.
- On a trip:
  1. Immediately cancel the active trajectory goal, the same way releasing the clutch does, so the controller holds the arm.
  2. Set the target to the current TCP pose (plus `contact_backoff` along the force direction, if non-zero).
  3. Latch: ignore operator motion commands until the clutch is released and pressed again.
  4. Log a throttled warning with the force/torque values.
- Expose the state: publish `std_msgs/Bool` on `~/limit_tripped`, and add a CSV column `limit_tripped`.
- Put the trip decision in a small pure function so it can be unit-tested without ROS.

### 2. Make the lead limit a parameter

The "target never more than 5 cm ahead of the actual tool" limit is hard-coded (I think). Make it the parameter `max_lead` (m, default `0.05`, live). For data collection we'll set it to `0.01`.

### 3. Publish the commanded target and clutch state

The commanded target is currently only in the CSV. For ros2 bag recording, publish:
- `~/target` as `geometry_msgs/PoseStamped` in the robot base frame, stamped with the node clock, every command tick while the clutch is held.
- `~/clutch` as `std_msgs/Bool`.

### 4. Extend the CSV log

Append, at the end:
- `torque_x/y/z`: raw wrist torque, same frame convention as `sensor_x/y/z` (document which frame).
- `limit_tripped`: 0/1.
- `ros_time`: node clock time in seconds, so the CSV can be synced with bag timestamps. The existing `time` column is wall-clock.

### 5. Data-collection parameter preset

Add `config/data_collection.yaml` for the bridge node, to be loaded with `ros2 param load /falcon_ur_teleop config/data_collection.yaml`:

```yaml
falcon_ur_teleop:
  ros__parameters:
    rate_max_speed: 0.02
    rate_boost_max_speed: 0.04
    max_tcp_speed: 0.05
    max_lead: 0.01
    ft_deadband: 0.0
    ft_gain: 0.2          # keep identical across all trials
    ft_filter_hz: 5.0     # keep identical across all trials
    contact_force_limit: 20.0
    contact_torque_limit: 2.0
```

Check the node name used in the launch files and adjust the top-level key if it differs. Install the `config/` folder in `CMakeLists.txt`.

### 6. Trial recording script

Add `scripts/record_trial.sh`, installed with `install(PROGRAMS …)`. Usage:

```bash
ros2 run falcon_ros2 record_trial.sh <operator> <condition> <repeat> [outdir]
# e.g.  record_trial.sh op1 off+1mm_careful 03 ~/trials
```

It should:
1. Check that ROS is sourced and that the bridge node is running.
2. Build a trial name: `<YYYYMMDD-HHMMSS>_<operator>_<condition>_r<repeat>`.
3. Zero the UR force/torque sensor via the UR driver's zero service (I believe it's `/io_and_status_controller/zero_ftsensor`, `std_srvs/Trigger`; check with `ros2 service list`). Print a warning and continue if the service isn't available, as in simulation.
4. Dump the bridge parameters to `<outdir>/<name>_params.yaml` (`ros2 param dump`), and write `<outdir>/<name>_meta.yaml` with operator, condition, repeat and start time.
5. Set the bridge `log_file` parameter to `<outdir>/<name>.csv`.
6. Start `ros2 bag record -o <outdir>/<name>` with: `/force_torque_sensor_broadcaster/wrench`, `/joint_states`, `/tf`, `/tf_static`, `/speed_scaling_state_broadcaster/speed_scaling`, `/falcon/position`, `/falcon/buttons`, `/falcon/force`, `/falcon_ur_teleop/target`, `/falcon_ur_teleop/clutch`, `/falcon_ur_teleop/limit_tripped`. Verify the real topic names in the running sim first, and skip any that don't exist with a warning.
7. On Ctrl+C: stop the bag cleanly, set `log_file` back to `''`, and print where the files are.

## Optional (do after 1–6 work)

### 7. Per-trial analysis script

`scripts/analyze_trial.py <bag_dir> <csv>` reads the bag with `rosbag2_py` and outputs `<name>_processed.csv` at 500 Hz with:
- time (s, from first sample), clutch, limit_tripped
- tool position from TF (base → tool frame used by the bridge)
- velocity and acceleration from a Savitzky–Golay filter (`scipy.signal.savgol_filter`, ~25-sample window, order 3) on position. **Do not difference raw position twice.**
- force and torque, 6 axes, raw and in the base frame
- commanded target (interpolated from `~/target`)
- `in_contact`: |force| above a threshold estimated from the first 1 s of the trial (the clutch-held baseline), e.g. mean + 5·std

It should also save a quick plot: force along each axis against insertion depth and against velocity.

## Testing / acceptance

- Unit test (pytest) for the contact-limit function: trips at the limit, doesn't trip at `0.0` (disabled), stays latched until the clutch cycles.
- `colcon build --packages-select falcon_ros2` passes.
- In `ur_sim_teleop.launch.py` (mock hardware), normal teleop still works, and `~/target`, `~/clutch` and `~/limit_tripped` publish.
- Trip test in simulation: publish a fake wrench above the limit on the topic the bridge subscribes to (or use Gazebo contacts if that's what feeds the bridge in `ur_gz_teleop`). The goal is cancelled, motion stops, `limit_tripped` is true, and it clears after re-clutching.
- `ros2 param load` with `config/data_collection.yaml` applies without errors.
- `record_trial.sh` produces a bag, a CSV with the new columns, and the params and meta YAML files, and stops cleanly on Ctrl+C.
- Finish with a short summary of what changed, anything that differed from this spec, and the exact commands for one recorded trial.
