#!/usr/bin/env bash
# Desktop launcher for the Falcon -> UR3e teleop: asks what to run (real robot or a simulation),
# the robot IP, control mode and logging in small dialogs, then runs the launch file in this
# terminal. Choices are remembered in ~/.config/falcon_teleop.conf.
#
# Normally started from the desktop shortcut (falcon-teleop.desktop), which opens a terminal:
#   gnome-terminal -- bash <workspace>/src/falcon_ros2/scripts/launch_falcon_teleop.sh

WS="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../../.." && pwd)"
CONF="$HOME/.config/falcon_teleop.conf"
LOG_DIR="$HOME/ff_logs"
TITLE="Falcon UR3e teleop"

# Started from the desktop, ~/.bashrc isn't read: use the same ROS settings it sets, so
# `ros2 param set` from a normal terminal reaches these nodes
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-67}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

finish() {
  echo
  read -rp "Press Enter to close this window."
  exit "${1:-0}"
}

# Defaults, then the remembered choices
ROBOT_IP=""
MODE="rate"
LOGGING="no"
FEEDBACK="yes"
[ -f "$CONF" ] && . "$CONF"

if [ ! -f "$WS/install/setup.bash" ]; then
  zenity --error --title="$TITLE" --text="Workspace not built: $WS/install/setup.bash is missing.\nRun colcon build in $WS first."
  finish 1
fi
. /opt/ros/jazzy/setup.bash
. "$WS/install/setup.bash"

# Two sessions would fight over the Falcon and the same ROS topics/services
if pgrep -f "^\S*python[0-9.]* \S*(falcon_ur_teleop|sim_ros)\.py|^\S*/lib/falcon_ros2/falcon_node" >/dev/null; then
  zenity --error --title="$TITLE" --text="A teleop session is already running (falcon_node, the bridge or the MuJoCo simulation).\n\nClose it first: Ctrl+C in its terminal window, or close the MuJoCo viewer."
  exit 1
fi

TARGET=$(zenity --list --radiolist --title="$TITLE" --text="What do you want to run?" \
  --width=420 --height=260 --column="" --column="Target" \
  TRUE "Real UR3e" FALSE "RViz simulation" FALSE "Gazebo simulation" \
  FALSE "MuJoCo peg-in-hole") || exit 0

if [ "$TARGET" = "MuJoCo peg-in-hole" ]; then
  MODE_LABEL="Twist + Insert"  # the MuJoCo simulation stands in for the robot in twist mode
else
  RATE_ON=FALSE; POS_ON=FALSE; TWIST_ON=FALSE
  case "$MODE" in position) POS_ON=TRUE ;; twist) TWIST_ON=TRUE ;; *) RATE_ON=TRUE ;; esac
  MODE_LABEL=$(zenity --list --radiolist --title="$TITLE" --text="Control mode" \
    --width=460 --height=260 --column="" --column="Mode" \
    $RATE_ON "Joystick (rate)" $POS_ON "Position" $TWIST_ON "Twist + Insert (real robot / MuJoCo)") || exit 0
  case "$MODE_LABEL" in Position) MODE=position ;; Twist*) MODE=twist ;; *) MODE=rate ;; esac
  if [ "$MODE" = twist ] && [ "$TARGET" != "Real UR3e" ]; then
    zenity --error --title="$TITLE" --text="Twist + Insert needs the real robot or the MuJoCo peg-in-hole simulation: RViz and Gazebo can't execute velocity commands."
    exit 1
  fi
fi

LOG_DEFAULT=()
[ "$LOGGING" = yes ] || LOG_DEFAULT=(--default-cancel)
if zenity --question --title="$TITLE" "${LOG_DEFAULT[@]}" \
     --text="Log force feedback to a CSV file in $LOG_DIR?" --ok-label="Log" --cancel-label="Don't log"; then
  LOGGING=yes
  LOG_ARG="log_file:=$LOG_DIR"
else
  LOGGING=no
  LOG_ARG="log_file:=''"
fi

FEEDBACK_ARG=()
if [ "$TARGET" = "Real UR3e" ] || [ "$TARGET" = "MuJoCo peg-in-hole" ]; then
  FB_DEFAULT=()
  [ "$FEEDBACK" = yes ] || FB_DEFAULT=(--default-cancel)
  if zenity --question --title="$TITLE" "${FB_DEFAULT[@]}" --ok-label="Feedback on" \
       --cancel-label="Feedback off" \
       --text="Contact force feedback on the Falcon?\n\nOff: you only feel the joystick's centring spring; the admittance still guides the peg. Switch it while running with:\nros2 param set /falcon_ur_teleop force_feedback false"; then
    FEEDBACK=yes
  else
    FEEDBACK=no
  fi
  FEEDBACK_ARG=("force_feedback:=$([ "$FEEDBACK" = yes ] && echo true || echo false)")
fi

if ! lsusb 2>/dev/null | grep -qi "0403:cb48"; then
  zenity --question --title="$TITLE" --ok-label="Continue anyway" --cancel-label="Cancel" \
    --text="No Falcon found on USB (0403:cb48).\n\nCheck it's powered and connected to this VM (VM USB menu)." || exit 0
fi

case "$TARGET" in
  "Real UR3e")
    ROBOT_IP=$(zenity --entry --title="$TITLE" --text="Robot IP address:" --entry-text="$ROBOT_IP") || exit 0
    if ! [[ "$ROBOT_IP" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]; then
      zenity --error --title="$TITLE" --text="'$ROBOT_IP' is not an IP address."
      exit 1
    fi
    zenity --question --title="$TITLE" --ok-label="Start" --cancel-label="Cancel" --width=420 \
      --text="Before starting the real robot:\n\n• Arm jogged to an open pose, elbow bent\n• Pendant speed slider low (10–20 %)\n• Workspace clear$([ "$MODE" = twist ] && printf '\n• Pendant TCP set to the peg tip, payload set (mass + CoG)')\n\nAfter launch: home the Falcon (LED red → green), then press Play on the External Control program." \
      || exit 0
    CMD=(ros2 launch falcon_ros2 ur_real_teleop.launch.py "robot_ip:=$ROBOT_IP")
    [ -f "$HOME/ur3e_calibration.yaml" ] && CMD+=("kinematics_params_file:=$HOME/ur3e_calibration.yaml")
    ;;
  "RViz simulation")  CMD=(ros2 launch falcon_ros2 ur_sim_teleop.launch.py) ;;
  "Gazebo simulation") CMD=(ros2 launch falcon_ros2 ur_gz_teleop.launch.py) ;;
  "MuJoCo peg-in-hole")
    if [ ! -x "$WS/mujoco/.venv/bin/python" ]; then
      zenity --error --title="$TITLE" --text="MuJoCo isn't set up: $WS/mujoco/.venv is missing (see $WS/mujoco/peg_in_hole/README.md)."
      exit 1
    fi
    CMD=(ros2 launch falcon_ros2 ur_mujoco_teleop.launch.py "mujoco_dir:=$WS/mujoco") ;;
  *) exit 0 ;;
esac
if [ "$TARGET" = "MuJoCo peg-in-hole" ]; then
  CMD+=("$LOG_ARG" "${FEEDBACK_ARG[@]}")  # always twist mode
else
  CMD+=("mode:=$MODE" "$LOG_ARG" "${FEEDBACK_ARG[@]}")
fi

mkdir -p "$(dirname "$CONF")"
printf 'ROBOT_IP=%q\nMODE=%q\nLOGGING=%q\nFEEDBACK=%q\n' "$ROBOT_IP" "$MODE" "$LOGGING" "$FEEDBACK" > "$CONF"

echo "=== $TARGET, $MODE_LABEL mode, logging: $LOGGING${FEEDBACK_ARG:+, force feedback: $FEEDBACK} ==="
echo "Press Ctrl+C in this window to stop everything."
echo "\$ ${CMD[*]}"
echo
# Ctrl+C should stop the launch but not this script, so the window stays open to read the output.
# (A handler, not `trap '' INT`: an ignored signal would be inherited by the launch.)
trap 'true' INT
"${CMD[@]}"
finish $?
