#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
NAV_WS="$REPO_ROOT/Nav2_rea_ws"
ARM_WS="$REPO_ROOT/tzb_ws"
DOMAIN="${P7_ROS_DOMAIN_ID:-$(( $$ % 102 ))}"
SCENARIO="${P7_SCENARIO:-normal}"
FAULT="${P7_FAULT:-}"
EXPECTED="${P7_EXPECTED_STATE:-}"
LOG_DIR="${P7_LOG_DIR:-/tmp/p7-offline-$(date -u +%Y%m%dT%H%M%SZ)}"

if [[ ! "$DOMAIN" =~ ^[0-9]+$ ]] || (( DOMAIN < 0 || DOMAIN > 101 )); then
  echo "P7_ROS_DOMAIN_ID must be an integer from 0 through 101" >&2
  exit 2
fi
if [[ -n "${P7_CHASSIS_DEVICE:-}${P7_ARM_DEVICE:-}" ]]; then
  echo "P7 refuses externally supplied serial paths; it creates both PTYs itself" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required at /opt/ros/humble/setup.bash" >&2
  exit 2
fi
if [[ ! -d "$NAV_WS/src/p6_autonomous_mission" || ! -d "$ARM_WS/src/lite_motion_planner" ]]; then
  echo "Run from a complete lunar-cave-robot P7 worktree" >&2
  exit 2
fi

OPERATOR_ACTION="${P7_SIMULATED_OPERATOR_ACTION:-}"
if [[ -z "$OPERATOR_ACTION" && -t 0 ]]; then
  read -r -p 'Simulated offline operator: type enable-offline-simulator to enable the PTY arm controller: ' OPERATOR_ACTION
fi
if [[ "$OPERATOR_ACTION" != "enable-offline-simulator" ]]; then
  echo "P4e stays DISARMED. Set P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator for the explicit simulated operator action." >&2
  exit 2
fi

export P7_REPO_ROOT="$REPO_ROOT"
export P7_NAV_WS="$NAV_WS"
export P7_ARM_WS="$ARM_WS"
export P7_ROS_DOMAIN_ID="$DOMAIN"
export P7_SCENARIO="$SCENARIO"
export P7_FAULT="$FAULT"
export P7_EXPECTED_STATE="$EXPECTED"
export P7_SIMULATED_OPERATOR_ACTION="$OPERATOR_ACTION"
export P7_LOG_DIR="$LOG_DIR"
export P7_PTY_DIR="$LOG_DIR/runtime/pty"
export P7_CONTROLLER_LOG="$LOG_DIR/controller-events.jsonl"
export P7_ARM_DRIVER_PARAMS="$LOG_DIR/arm-driver.yaml"
export P7_ARM_DRIVER_PID_FILE="$LOG_DIR/arm-driver.pid"
export P7_CHASSIS_BRIDGE_PID_FILE="$LOG_DIR/chassis-bridge.pid"
export P7_ARM_RESTART_LOG="$LOG_DIR/arm-driver-restarted.log"
export P7_MISSION_TIMEOUT_SEC="${P7_MISSION_TIMEOUT_SEC:-260}"
export P7_NAV_TIMEOUT_SEC="${P7_NAV_TIMEOUT_SEC:-90}"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
unset ROS_DISCOVERY_SERVER || true
mkdir -p "$P7_PTY_DIR" "$LOG_DIR"
# Avoid accepting a stale ready file or stale PTY links when a caller reuses LOG_DIR.
rm -f "$P7_PTY_DIR/pty-ready.json" "$P7_PTY_DIR/chassis-device" \
  "$P7_PTY_DIR/chassis-device.next" "$P7_PTY_DIR/arm-device" "$P7_PTY_DIR/arm-device.next"

if [[ "${P7_SKIP_BUILD:-0}" != "1" ]]; then
  echo "Building Nav2 and P5b chassis workspaces independently..."
  (
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    cd "$NAV_WS"
    colcon build --base-paths src --packages-up-to p6_autonomous_mission chassis_serial_bridge \
      --build-base build/p7 --install-base install/p7 --parallel-workers "${P7_BUILD_JOBS:-4}"
  )
  echo "Building planner and P4e arm-driver workspace independently..."
  (
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    cd "$ARM_WS"
    colcon build --base-paths src --packages-up-to lite_motion_planner rm_serial_driver \
      --packages-skip orbbec_camera --build-base build/p7 --install-base install/p7 \
      --parallel-workers "${P7_BUILD_JOBS:-4}"
  )
fi

for install in "$NAV_WS/install/p7/setup.bash" "$ARM_WS/install/p7/setup.bash"; do
  if [[ ! -f "$install" ]]; then
    echo "Missing independent P7 install: $install" >&2
    exit 2
  fi
done

cat > "$P7_ARM_DRIVER_PARAMS" <<YAML
/**:
  ros__parameters:
    device_name: "$P7_PTY_DIR/arm-device"
    baud_rate: 115200
    flow_control: "none"
    parity: "none"
    stop_bits: "1"
    tzb.arm_serial_enabled: true
    tzb.include_arm_serial_extension: true
    tzb.arm_command_topic: "/tzb_catch/arm_joint_command"
    tzb.gripper_command_topic: "/tzb_catch/gripper_command"
    tzb.trajectory_gripper_topic: "/p6/arm_task/gripper_command"
    tzb.trajectory_cancel_topic: "/p6/arm_task/cancel"
    tzb.trajectory_cancel_ack_topic: "/serial/arm_task/cancel_ack"
    tzb.arm_state_topic: "/p2_offline/arm_joint_state"
    tzb.arm_safety_status_topic: "/serial/arm_safety/status"
    tzb.arm_safety_reset_service: "/serial/arm_safety/reset"
    tzb.arm_safety_enable_service: "/serial/arm_safety/enable"
    tzb.trajectory_epoch_state_path: "$P7_PTY_DIR/arm-epoch-high-water"
    tzb.arm_serial_angle_unit: "rad"
    tzb.arm_command_publish_hz: 100.0
    tzb.arm_command_lease_sec: 0.1
    tzb.arm_command_max_age_sec: 0.1
    tzb.arm_command_future_tolerance_sec: 0.05
    tzb.arm_feedback_timeout_sec: 0.5
    tzb.arm_rearm_quiet_sec: 0.1
    tzb.arm_safety_status_hz: 20.0
    tzb.arm_joint_names: ["J1_joint", "J2_joint", "J3_joint", "J4_joint", "J5_joint", "J6_joint"]
YAML

pids=()
cleanup() {
  set +e
  local pid
  if [[ -f "$P7_ARM_DRIVER_PID_FILE" ]]; then
    pids+=("$(cat "$P7_ARM_DRIVER_PID_FILE")")
  fi
  for pid in "${pids[@]}"; do
    [[ -n "$pid" ]] && kill -INT -- "-$pid" 2>/dev/null
  done
  sleep 1
  for pid in "${pids[@]}"; do
    [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null
  done
  sleep 1
  for pid in "${pids[@]}"; do
    [[ -n "$pid" ]] && kill -KILL -- "-$pid" 2>/dev/null
  done
  for pid in "${pids[@]}"; do
    [[ -n "$pid" ]] && wait "$pid" 2>/dev/null
  done
  # ROS 2 Humble ignores ROS2CLI_DISABLE_DAEMON. Stop only the CLI daemon
  # bound to this scenario's isolated domain after all scenario processes exit.
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    (
      set +u
      source /opt/ros/humble/setup.bash
      set -u
      ROS_DOMAIN_ID="$DOMAIN" ros2 daemon stop >/dev/null 2>&1
    )
  fi
}
trap cleanup EXIT INT TERM

setsid bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  exec python3 "$P7_REPO_ROOT/tools/p7_offline/pty_stm32_simulator.py" --runtime-dir "$P7_PTY_DIR"
' >"$LOG_DIR/pty-simulator.log" 2>&1 &
SIM_PID=$!; pids+=("$SIM_PID")

ready_file="$P7_PTY_DIR/pty-ready.json"
for _ in $(seq 1 100); do
  [[ -s "$ready_file" ]] && break
  kill -0 "$SIM_PID" 2>/dev/null || { cat "$LOG_DIR/pty-simulator.log" >&2; exit 1; }
  sleep 0.1
done
if [[ ! -s "$ready_file" ]]; then
  echo "PTY simulator did not publish its ready record" >&2
  exit 1
fi
python3 - "$ready_file" "$P7_PTY_DIR" <<'PY'
import json, os, re, sys
record=json.load(open(sys.argv[1], encoding='utf-8'))
root=os.path.realpath(sys.argv[2])
if record.get('mode') != 'offline-pty-only':
    raise SystemExit('PTY simulator mode mismatch')
for key in ('chassis_device','arm_device'):
    path=record[key]
    if os.path.dirname(os.path.abspath(path)) != root or not os.path.islink(path):
        raise SystemExit(f'{key}: path={path!r} root={root!r} parent={os.path.dirname(os.path.abspath(path))!r} islink={os.path.islink(path)}')
    target=os.path.realpath(path)
    if not re.fullmatch(r'/dev/pts/[0-9]+', target):
        raise SystemExit(f'{key} target is not a PTY: {target}')
print('PASS serial isolation: only runtime-created PTYs are configured')
PY

setsid bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  source "$P7_NAV_WS/install/p7/setup.bash"
  exec ros2 launch chassis_serial_bridge serial.launch.py \
    device_name:="$P7_PTY_DIR/chassis-device" command_topic:=/cmd_vel
' >"$LOG_DIR/chassis-bridge.log" 2>&1 &
BRIDGE_PID=$!; pids+=("$BRIDGE_PID")
echo "$BRIDGE_PID" > "$P7_CHASSIS_BRIDGE_PID_FILE"

setsid bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  source "$P7_ARM_WS/install/p7/setup.bash"
  exec ros2 run rm_serial_driver node --ros-args --params-file "$P7_ARM_DRIVER_PARAMS"
' >"$LOG_DIR/arm-driver.log" 2>&1 &
ARM_PID=$!; pids+=("$ARM_PID")
echo "$ARM_PID" > "$P7_ARM_DRIVER_PID_FILE"

setsid bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  source "$P7_ARM_WS/install/p7/setup.bash"
  exec ros2 launch lite_motion_planner p3b_offline.launch.py \
    offline_domain_id:="$P7_ROS_DOMAIN_ID" autostart:=false p6_staged_tasks:=true \
    publish_synthetic_target:=true synthetic_arm_feedback:=false use_rviz:=false
' >"$LOG_DIR/arm-planner.log" 2>&1 &
ARM_PLANNER_PID=$!; pids+=("$ARM_PLANNER_PID")

setsid bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  source "$P7_NAV_WS/install/p7/setup.bash"
  exec ros2 launch p6_autonomous_mission p6_autonomous_mission.launch.py \
    domain_id:="$P7_ROS_DOMAIN_ID" scenario:="$P7_SCENARIO" \
    arm_backend:=p4e_pty auto_start:=false p4e_status_required:=true \
    chassis_command_topic:=/p7/chassis/decoded_command chassis_link_status_required:=true \
    navigation_timeout_sec:="$P7_NAV_TIMEOUT_SEC" \
    event_log_path:="$P7_LOG_DIR/mission-events.jsonl"
' >"$LOG_DIR/nav2-mission.log" 2>&1 &
NAV_PID=$!; pids+=("$NAV_PID")

sleep 1
{
  echo '=== /navigate_to_pose ==='
  (set +u; source /opt/ros/humble/setup.bash; source "$NAV_WS/install/p7/setup.bash"; set -u; ros2 action info /navigate_to_pose) 2>&1 || true
  for topic in /cmd_vel /p7/chassis/decoded_command /p7/sim/wire_status /chassis/serial_link_status /p2_offline/arm_joint_state /tzb_catch/arm_joint_command /serial/arm_safety/status /p6/mission_state; do
    echo "=== $topic ==="
    (set +u; source /opt/ros/humble/setup.bash; source "$NAV_WS/install/p7/setup.bash"; set -u; ros2 topic info "$topic" --verbose) 2>&1 || true
  done
} >"$LOG_DIR/ros-graph.log"
# TF publishers can start after lifecycle/graph discovery. Keep tf2_echo alive
# until it resolves the transform, with a bounded deadline, instead of treating
# a fixed five-second startup delay as a transform failure.
setsid timeout 30s bash -c 'source /opt/ros/humble/setup.bash; source "$P7_NAV_WS/install/p7/setup.bash"; exec stdbuf -oL ros2 run tf2_ros tf2_echo map base_link' \
  >"$LOG_DIR/tf-map-base.log" 2>&1 &
TF_PID=$!; pids+=("$TF_PID")
tf_deadline=$((SECONDS + 30))
while (( SECONDS < tf_deadline )); do
  if grep -q 'Translation:' "$LOG_DIR/tf-map-base.log"; then
    kill -INT -- "-$TF_PID" 2>/dev/null || true
    wait "$TF_PID" 2>/dev/null || true
    break
  fi
  if ! kill -0 "$TF_PID" 2>/dev/null; then
    break
  fi
  sleep 0.2
done
if ! grep -q 'Translation:' "$LOG_DIR/tf-map-base.log"; then
  echo "TF map->base_link was not resolved within 30 seconds; see $LOG_DIR/tf-map-base.log" >&2
  exit 1
fi

echo "P7 offline integration started in isolated ROS_DOMAIN_ID=$DOMAIN (localhost only)."
echo "Decoded wire command topic: /p7/chassis/decoded_command; arm feedback: /p2_offline/arm_joint_state"
echo "A simulated operator enable is recorded; no reset service is called."
echo "Logs: $LOG_DIR"
set +e
(
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  exec python3 "$P7_REPO_ROOT/tools/p7_offline/mission_control.py"
) 2>&1 | tee "$LOG_DIR/mission-control.log"
status=${PIPESTATUS[0]}
set -e
if [[ $status -ne 0 ]]; then
  echo "P7 mission controller failed (status=$status); logs: $LOG_DIR" >&2
  exit "$status"
fi
