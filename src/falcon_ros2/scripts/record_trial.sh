#!/usr/bin/env bash
# Record one contact-data trial: a ros2 bag, the bridge's CSV log, its parameters and metadata.
#
#   ros2 run falcon_ros2 record_trial.sh <operator> <condition> <repeat> [outdir]
#   e.g.  ros2 run falcon_ros2 record_trial.sh op1 off+1mm_careful 03 ~/trials
#
# Start it with the arm still and out of contact (the wrist sensor and the bridge's contact
# limit are zeroed first, and the contact limit keeps that zero for the whole trial), drive the
# trial, then press Ctrl+C: the bag is closed, the bridge stops logging, and the files are listed.
# Files: <outdir>/<name>/ (bag), <name>.csv, <name>_params.yaml, <name>_meta.yaml, with
# <name> = <YYYYMMDD-HHMMSS>_<operator>_<condition>_r<repeat>.
set -u

NODE=/falcon_ur_teleop
ZERO_SRV=/io_and_status_controller/zero_ftsensor
TOPICS_WANTED=(
  /force_torque_sensor_broadcaster/wrench
  /joint_states
  /tf
  /tf_static
  /speed_scaling_state_broadcaster/speed_scaling
  /falcon/position
  /falcon/buttons
  /falcon/force
  /falcon_ur_teleop/target
  /falcon_ur_teleop/clutch
  /falcon_ur_teleop/limit_tripped
  /twist_controller/twist  # twist mode: the commanded velocity
)

warn() { echo "WARNING: $*" >&2; }
# ros_list node|topic|service [names...]: what's in the ROS graph. One discovery pass can miss
# entries on a busy graph, and the daemon's cache can lag, so take the union of the daemon's list
# and up to three direct passes, stopping early once all the given names have been seen.
ros_list() {
  local kind=$1; shift
  local found
  found=$(ros2 "$kind" list 2>/dev/null)
  for _ in 1 2 3; do
    local missing=0 n
    for n in "$@"; do grep -qx "$n" <<<"$found" || missing=1; done
    [ $# -gt 0 ] && [ $missing = 0 ] && break
    found=$(printf '%s\n%s\n' "$found" "$(ros2 "$kind" list --no-daemon --spin-time 3 2>/dev/null)")
  done
  sort -u <<<"$found" | grep -v '^$'
}
die() { echo "ERROR: $*" >&2; exit 1; }

if [ $# -lt 3 ] || [ $# -gt 4 ]; then
  sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
  exit 1
fi
OPERATOR=$1 CONDITION=$2 REPEAT=$3 OUTDIR=${4:-$HOME/trials}
for v in "$OPERATOR" "$CONDITION" "$REPEAT"; do
  [[ "$v" =~ ^[A-Za-z0-9._+-]+$ ]] || die "'$v': use only letters, digits and . _ + - (it becomes part of file names)"
done

# 1. ROS sourced and the bridge running
command -v ros2 >/dev/null || die "ros2 not found: source /opt/ros/jazzy/setup.bash and the workspace first"
ros_list node "$NODE" | grep -qx "$NODE" || die "the bridge ($NODE) isn't running (or is on another ROS_DOMAIN_ID)"

# 2. Trial name and files
NAME="$(date +%Y%m%d-%H%M%S)_${OPERATOR}_${CONDITION}_r${REPEAT}"
mkdir -p "$OUTDIR" || die "can't create $OUTDIR"
OUTDIR=$(cd "$OUTDIR" && pwd)  # absolute: the bridge writes the CSV from its own working directory
BAG="$OUTDIR/$NAME" CSV="$OUTDIR/$NAME.csv"
PARAMS="$OUTDIR/${NAME}_params.yaml" META="$OUTDIR/${NAME}_meta.yaml"
echo "=== Trial $NAME ==="

# 3. Zero the wrist force/torque sensor (UR driver); not available in every simulation
if ros_list service "$ZERO_SRV" | grep -qx "$ZERO_SRV"; then
  echo "Zeroing the force/torque sensor (keep the arm still and out of contact)..."
  if timeout 10 ros2 service call "$ZERO_SRV" std_srvs/srv/Trigger 2>/dev/null | grep -q "success=True"; then
    echo "  zeroed"
    ZEROED=true
  else
    warn "zeroing failed or timed out; continuing without it"
    ZEROED=false
  fi
else
  warn "$ZERO_SRV not available (simulation?); continuing without zeroing"
  ZEROED=false
fi
# The bridge's contact limit keeps one zero for the whole trial: take it now (after the sensor
# zero has settled), with the arm still and out of contact
sleep 0.3
if timeout 10 ros2 service call "$NODE/zero_contact_limit" std_srvs/srv/Trigger 2>/dev/null \
    | grep -q "success=True"; then
  echo "  contact-limit zero taken for this trial"
  CL_ZEROED=true
else
  warn "could not zero the bridge's contact limit (it will zero at the first clutch press instead)"
  CL_ZEROED=false
fi

# 4. Parameters and metadata
ros2 param dump "$NODE" > "$PARAMS" 2>/dev/null || warn "could not dump the bridge parameters"
MODE=$(ros2 param get "$NODE" mode 2>/dev/null | sed -n 's/^String value is: //p')
cat > "$META" <<EOF
operator: '$OPERATOR'
condition: '$CONDITION'
repeat: '$REPEAT'
start_time: '$(date --iso-8601=seconds)'
bridge_mode: '${MODE:-unknown}'
ft_sensor_zeroed: $ZEROED
contact_limit_zeroed: $CL_ZEROED
ros_domain_id: '${ROS_DOMAIN_ID:-0}'
host: '$(hostname)'
bag: '$BAG'
csv: '$CSV'
params: '$PARAMS'
EOF

# 5. Bridge CSV log
ros2 param set "$NODE" log_file "$CSV" 2>&1 | grep -q successful || warn "could not start the bridge CSV log"

# 6. Bag: only topics that exist right now
AVAILABLE=$(ros_list topic "${TOPICS_WANTED[@]}")
TOPICS=()
for t in "${TOPICS_WANTED[@]}"; do
  if grep -qx "$t" <<<"$AVAILABLE"; then
    TOPICS+=("$t")
  else
    warn "topic $t doesn't exist, not recorded"
  fi
done
[ ${#TOPICS[@]} -gt 0 ] || die "none of the topics exist"

STOPPING=false
cleanup() {
  $STOPPING && return
  STOPPING=true
  echo
  echo "Stopping..."
  if kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID"
    for _ in $(seq 50); do kill -0 "$BAG_PID" 2>/dev/null || break; sleep 0.2; done
    kill -0 "$BAG_PID" 2>/dev/null && { warn "bag didn't stop in 10 s, terminating"; kill -TERM "$BAG_PID"; }
  fi
  wait "$BAG_PID" 2>/dev/null
  ros2 param set "$NODE" log_file "''" 2>&1 | grep -q successful || warn "could not stop the bridge CSV log"
  echo "end_time: '$(date --iso-8601=seconds)'" >> "$META"
  echo "=== Trial $NAME saved ==="
  echo "  bag:    $BAG"
  echo "  csv:    $CSV"
  echo "  params: $PARAMS"
  echo "  meta:   $META"
  exit 0
}
trap cleanup INT TERM

# Job control only for this: a background job in a script otherwise starts with SIGINT ignored, and
# the bag couldn't be stopped cleanly. (Not for the whole script: with it, ros2 CLI discovery
# returned wrong results.)
set -m
ros2 bag record -o "$BAG" --topics "${TOPICS[@]}" > "$OUTDIR/${NAME}_bag.log" 2>&1 &
BAG_PID=$!
set +m
sleep 2
kill -0 "$BAG_PID" 2>/dev/null || { cat "$OUTDIR/${NAME}_bag.log" >&2; ros2 param set "$NODE" log_file "''" >/dev/null 2>&1; die "ros2 bag record failed"; }
echo "Recording ${#TOPICS[@]} topics. Drive the trial, then press Ctrl+C to stop."
while kill -0 "$BAG_PID" 2>/dev/null; do
  wait "$BAG_PID"
done
warn "the bag recorder exited on its own (see ${NAME}_bag.log)"
cleanup
