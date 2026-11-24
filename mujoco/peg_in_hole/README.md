# Peg-in-hole simulation (MuJoCo)

Simulated UR3e inserting the folding-gripper housing (peg) into the blanket-segment socket
(hole), using the **same admittance code as the real robot**
(`src/falcon_ros2/scripts/admittance_insert.py`, imported by `falcon_ur_teleop.py`).

All commands from this folder, with the MuJoCo virtual environment:

```bash
cd ~/ur3e_ws/mujoco/peg_in_hole
../.venv/bin/python sim.py                    # insertion with admittance (saves plots + CSV)
../.venv/bin/python sim.py --no-admittance    # same insertion, stiff robot, for comparison
../.venv/bin/python sim.py --viewer           # watch it live in the MuJoCo viewer
../.venv/bin/python sim.py --video            # also save insertion.mp4 (with a text overlay)
../.venv/bin/python sim.py --offset 0.015 0.0 --speed 0.01 \
    --mass 40 40 60 --damping 1000 1000 1000 --stiffness 400 400 0
```

Results go to `outputs/<time>_<label>/`: `summary.txt`, `data.csv`, `insertion.png`, and
`insertion.mp4` with `--video`. In the viewer and videos the socket is see-through, so the peg is
visible inside it. `outputs/comparison.mp4` shows the admittance and stiff runs side by side.

## Live: drive it with the Falcon

The simulation can stand in for the real UR3e + driver, so the real stack drives it: Falcon ->
`falcon_node` -> `falcon_ur_teleop.py` in twist mode -> `sim_ros.py`. Start it from the desktop
shortcut (target "MuJoCo peg-in-hole") or:

```bash
ros2 launch falcon_ros2 ur_mujoco_teleop.launch.py              # viewer window, Falcon
ros2 launch falcon_ros2 ur_mujoco_teleop.launch.py offset_x:=0.015 offset_y:=0.0 log_file:=~/ff_logs
ros2 service call /mujoco/reset std_srvs/srv/Trigger              # arm back to the start
```

Drive exactly as on the robot: hold a grip button (not minus) as the clutch, tap minus (clutch
released, peg out of contact) for the admittance Insert, feel the contact through the Falcon.
Closing the viewer window ends the session. UR force mode isn't simulated; use the default
`insert_method:=admittance`.

For speed the live version uses coarser socket tiles (`build_hole_tiles.py --coarse`, seats
~4 mm higher than the fine tiles), the peg hull split into slabs (`split_peg.py`) and a 2 ms
physics step; the viewer runs in its own process. On this laptop (i5-10210U VM) the full
stack, viewer included, runs at roughly half real time, so the arm moves in slow motion; the
simulation alone keeps up with real time. The bridge uses about a full core in twist mode, the
biggest share.

## What is simulated

- **Robot:** the UR3e model in `../third_party/ur3e` (Universal Robots' meshes and parameters,
  BSD-3) with a stiff joint servo tracking an integrated
  velocity reference, like `speedl` on the real arm: a blocked peg builds up force.
- **Bridge:** every 10 ms (the twist-mode rate) the wrist force goes through `AdmittanceInsert`;
  the command is the operator velocity plus the admittance velocity, capped at 0.25 m/s. After an
  admittance limit stop the arm stops, as in the bridge.
- **Operator:** a scripted Falcon: clutch held, pushing straight down at `--speed`.
- **Sensor:** a 6-axis force/torque sensor between flange and peg. `sim.py` checks its sign
  against the peg's weight and a known push before every run.

## Geometry files (`meshes/`, regenerate if the STLs change)

```bash
../.venv/bin/python prepare_meshes.py     # STL (mm) -> OBJ (m), peg convex hull
../.venv/bin/python build_hole_tiles.py   # hole collision tiles along its inner surface
../.venv/bin/python seat_depth.py         # exact seat height from the CAD surface (17.8 mm)
../.venv/bin/python seat_depth.py --tiles # same against the tiles (20.9 mm)
```

MuJoCo collides every mesh as its convex hull, which would fill the hole. A CoACD convex
decomposition bulged into the cavity (peg seated 13+ mm too high), so the hole's collision
shape is built from thin tiles that follow its inner surface instead. The peg's collision shape
is its convex hull (contact is on its outer surface).

## Known limits

- The peg's slot orientation is held fixed (translation-only admittance), and starts aligned.
- Contact forces are somewhat jagged during sliding; partly MuJoCo contact against a stiff servo.
- The arm and the peg don't collide with each other or the socket's outside.
- Friction is a guess (0.3).
