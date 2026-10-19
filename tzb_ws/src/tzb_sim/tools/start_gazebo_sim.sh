#!/usr/bin/env bash
set -euo pipefail

env_or() {
  local name="$1" fallback="$2" value
  value="$(printenv "$name" 2>/dev/null || true)"
  if [[ -n "$value" ]]; then printf '%s' "$value"; else printf '%s' "$fallback"; fi
}

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
SIM_PKG="$(cd -- "$SCRIPT_DIR/.." && pwd)"
WS_ROOT="$(cd -- "$SIM_PKG/../.." && pwd)"
P7_UNDERLAY="$(env_or TZB_GZ_P7_UNDERLAY "$HOME/lunar-robot/worktrees/issue-48-p0-p1/tzb_ws/install/p7/setup.bash")"
P1_OVERLAY="$(env_or TZB_GZ_P1_OVERLAY "$HOME/lunar-robot/worktrees/issue-48-p0-p1/tzb_ws/install/issue48")"
OVERLAY="$(env_or TZB_GZ_OVERLAY "$WS_ROOT/install/issue48-p2")"
DOMAIN="$(env_or TZB_GZ_ROS_DOMAIN_ID 216)"
LOG_DIR="$(env_or TZB_GZ_LOG_DIR "$HOME/tzb-study-output/issue48-p2-gazebo/logs")"
RUNTIME_DIR="$(env_or XDG_RUNTIME_DIR /tmp)"
PID_FILE="$(env_or TZB_GZ_PID_FILE "$RUNTIME_DIR/tzb_issue48_gazebo.pid")"
AUTOSTART="$(env_or TZB_GZ_AUTOSTART true)"
LISTEN_NAV_STATUS="$(env_or TZB_GZ_LISTEN_NAV_STATUS false)"
USE_RVIZ="$(env_or TZB_GZ_USE_RVIZ true)"
GRASP_PIPELINE="$(env_or TZB_GZ_GRASP_PIPELINE true)"

if [[ ! "$DOMAIN" =~ ^[0-9]+$ ]] || (( 10#$DOMAIN > 232 )); then
  echo "ROS domain must be an integer from 0 through 232 (got: $DOMAIN)." >&2
  exit 1
fi
if [[ -f "$PID_FILE" ]]; then
  old_pid="$(cat "$PID_FILE")"
  if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
    old_cmd="$(tr '\0' ' ' < "/proc/$old_pid/cmdline" 2>/dev/null || true)"
    if [[ "$old_cmd" == *"ros2 launch tzb_sim gazebo_grasp.launch.py"* ]]; then
      echo "P2 Gazebo is already running: pid=$old_pid domain=$DOMAIN"
      echo "log: $(ls -1t "$LOG_DIR"/gazebo_*.log 2>/dev/null | head -n 1)"
      exit 0
    fi
    echo "Refusing to replace live PID file $PID_FILE (pid $old_pid is not P2 Gazebo)." >&2
    exit 1
  fi
  rm -f "$PID_FILE"
fi
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$P7_UNDERLAY" || ! -f "$P1_OVERLAY/local_setup.bash" ]]; then
  echo "ROS Humble, preserved P7 underlay, or P1 overlay is missing; refusing a partial launch." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$P7_UNDERLAY"
source "$P1_OVERLAY/local_setup.bash"
set -u
if [[ ! -f "$OVERLAY/local_setup.bash" ]]; then
  echo "Building only tzb_sim into the isolated P2 overlay."
  (cd "$WS_ROOT" && colcon --log-base log/issue48-p2 \
    build --base-paths src --packages-select tzb_sim lite_motion_planner \
    --build-base build/issue48-p2 --install-base install/issue48-p2)
  OVERLAY="$WS_ROOT/install/issue48-p2"
fi
set +u
source "$OVERLAY/local_setup.bash"
set -u

OPTIONAL_CAMERA_PREFIX="$(env_or TZB_GZ_OPTIONAL_CAMERA_PREFIX "$HOME/tzb-delivery-check-ascii/tzb-repro-v2-2026-10-16/tzb_ws/install")"
for pkg in orbbec_camera orbbec_camera_msgs; do
  if [[ -d "$OPTIONAL_CAMERA_PREFIX/$pkg" ]]; then
    current_prefixes="$(printenv AMENT_PREFIX_PATH 2>/dev/null || true)"
    if [[ -n "$current_prefixes" ]]; then
      export AMENT_PREFIX_PATH="$current_prefixes:$OPTIONAL_CAMERA_PREFIX/$pkg"
    else
      export AMENT_PREFIX_PATH="$OPTIONAL_CAMERA_PREFIX/$pkg"
    fi
  fi
done

export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1
export IGN_PARTITION="tzb-issue48-p2-$DOMAIN"
export GZ_IP=127.0.0.1
export GZ_PARTITION="tzb-issue48-p2-$DOMAIN"
for env_path in /proc/[0-9]*/environ; do
  candidate="${env_path#/proc/}"
  candidate="${candidate%/environ}"
  [[ "$candidate" =~ ^[0-9]+$ ]] || continue
  if grep -aFq "GZ_PARTITION=$GZ_PARTITION" "$env_path" 2>/dev/null; then
    candidate_cmd="$(tr "\0" " " < "/proc/$candidate/cmdline" 2>/dev/null || true)"
    if [[ "$candidate_cmd" == *"ign gazebo"* ]]; then
      echo "Found an existing P2 Gazebo process (pid $candidate); run stop_gazebo_sim.sh before restarting." >&2
      exit 1
    fi
  fi
done
export DISPLAY="$(env_or DISPLAY :0)"
export XAUTHORITY="$(env_or XAUTHORITY /run/user/1000/gdm/Xauthority)"
export YOLO_NODE_PYTHON="$(env_or YOLO_NODE_PYTHON "$HOME/robot_p0/.venv/bin/python")"
export POSE_ESTIMATOR_PYTHON="$(env_or POSE_ESTIMATOR_PYTHON "$HOME/robot_p0/.venv/bin/python")"
MODEL_DIR="$(env_or TZB_OPENVINO_MODEL "$HOME/tzb-study-output/issue48-p0-p1-2026-10-18/ska_1_reference/best_openvino_model/blue_block")"
if [[ ! -x "$YOLO_NODE_PYTHON" || ! -x "$POSE_ESTIMATOR_PYTHON" ]]; then
  echo "Set YOLO_NODE_PYTHON and POSE_ESTIMATOR_PYTHON to the preserved perception interpreter." >&2
  exit 1
fi
if [[ ! -f "$MODEL_DIR/yoloe-26l-seg.xml" || ! -f "$MODEL_DIR/yoloe-26l-seg.bin" ]]; then
  echo "OpenVINO model XML/BIN pair is missing: $MODEL_DIR" >&2
  exit 1
fi
export TZB_OPENVINO_MODEL="$MODEL_DIR"

mkdir -p "$LOG_DIR" "$(dirname -- "$PID_FILE")"
LOG_FILE="$LOG_DIR/gazebo_$(date +%Y%m%d_%H%M%S).log"
setsid ros2 launch tzb_sim gazebo_grasp.launch.py \
  autostart:="$AUTOSTART" listen_navigation_goal_status:="$LISTEN_NAV_STATUS" \
  use_rviz:="$USE_RVIZ" enable_grasp_pipeline:="$GRASP_PIPELINE" \
  >"$LOG_FILE" 2>&1 < /dev/null &
PID=$!
echo "$PID" > "$PID_FILE"
sleep 1
if ! kill -0 "$PID" 2>/dev/null; then
  rm -f "$PID_FILE"
  echo "P2 launch exited during startup; inspect $LOG_FILE" >&2
  exit 1
fi
echo "Started P2 Gazebo physical simulation: pid=$PID domain=$ROS_DOMAIN_ID localhost-only=1"
echo "Gazebo transport partition: $IGN_PARTITION"
echo "autostart=$AUTOSTART listen_navigation_goal_status=$LISTEN_NAV_STATUS use_rviz=$USE_RVIZ grasp_pipeline=$GRASP_PIPELINE"
echo "log: $LOG_FILE"
echo "stop: $SCRIPT_DIR/stop_gazebo_sim.sh"
