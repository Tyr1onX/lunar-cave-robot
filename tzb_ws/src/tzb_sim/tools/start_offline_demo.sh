#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
LOG_DIR="${TZB_SIM_LOG_DIR:-$HOME/.local/state/tzb_sim}"
PID_FILE="${TZB_SIM_PID_FILE:-${XDG_RUNTIME_DIR:-/tmp}/tzb_sim_offline_demo.pid}"
DOMAIN="${TZB_SIM_ROS_DOMAIN_ID:-214}"
if [[ ! "$DOMAIN" =~ ^[0-9]+$ ]] || (( 10#$DOMAIN > 232 )); then
  echo "ROS domain must be an integer from 0 through 232 (got: $DOMAIN)." >&2
  exit 1
fi

if [[ -f "$PID_FILE" ]]; then
  old_pid="$(cat "$PID_FILE")"
  if [[ "$old_pid" =~ ^[0-9]+$ ]] && kill -0 "$old_pid" 2>/dev/null; then
    old_cmd="$(tr '\0' ' ' < "/proc/$old_pid/cmdline" 2>/dev/null || true)"
    if [[ "$old_cmd" == *"ros2 launch tzb_sim demo.launch.py"* ]]; then
      echo "tzb_sim is already running (pid $old_pid; domain $DOMAIN)"
      echo "log: $(ls -1t "$LOG_DIR"/demo_*.log 2>/dev/null | head -n 1)"
      exit 0
    fi
    echo "Refusing to replace live PID file $PID_FILE (pid $old_pid is not this demo)." >&2
    exit 1
  fi
  rm -f "$PID_FILE"
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble." >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ -n "${TZB_SIM_SETUP_BASH:-}" ]]; then
  set +u
  source "$TZB_SIM_SETUP_BASH"
  set -u
else
  UNDERLAY_SETUP_BASH="${TZB_SIM_UNDERLAY_SETUP_BASH:-}"
  if [[ -z "$UNDERLAY_SETUP_BASH" ]]; then
    for candidate in "$WS_ROOT/install/p7/setup.bash" "$WS_ROOT/install/p1/setup.bash" "$WS_ROOT/install/setup.bash"; do
      if [[ -f "$candidate" ]]; then UNDERLAY_SETUP_BASH="$candidate"; break; fi
    done
  fi
  if [[ -z "$UNDERLAY_SETUP_BASH" || ! -f "$UNDERLAY_SETUP_BASH" ]]; then
    echo "No dependency underlay found. Build the workspace or set TZB_SIM_UNDERLAY_SETUP_BASH." >&2
    exit 1
  fi
  set +u
  source "$UNDERLAY_SETUP_BASH"
  set -u
  OVERLAY_LOCAL_SETUP="${TZB_SIM_OVERLAY_LOCAL_SETUP_BASH:-$WS_ROOT/install/issue48/local_setup.bash}"
  if [[ ! -f "$OVERLAY_LOCAL_SETUP" ]]; then
    echo "No tzb_sim overlay found. Build with --install-base install/issue48 or set TZB_SIM_SETUP_BASH." >&2
    exit 1
  fi
  set +u
  source "$OVERLAY_LOCAL_SETUP"
  set -u
fi

# pose_estimator lists the optional camera driver as an exec dependency even
# when launch_camera=false. Add only its preserved package prefixes; the demo
# never starts this driver and current pipeline packages stay earlier in path.
OPTIONAL_CAMERA_PREFIX="${TZB_SIM_OPTIONAL_CAMERA_PREFIX:-$HOME/tzb-delivery-check-ascii/tzb-repro-v2-2026-10-16/tzb_ws/install}"
if [[ -d "$OPTIONAL_CAMERA_PREFIX/orbbec_camera" ]]; then
  export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:+$AMENT_PREFIX_PATH:}$OPTIONAL_CAMERA_PREFIX/orbbec_camera"
fi
if [[ -d "$OPTIONAL_CAMERA_PREFIX/orbbec_camera_msgs" ]]; then
  export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:+$AMENT_PREFIX_PATH:}$OPTIONAL_CAMERA_PREFIX/orbbec_camera_msgs"
fi

export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
export YOLO_NODE_PYTHON="${YOLO_NODE_PYTHON:-$HOME/robot_p0/.venv/bin/python}"
export POSE_ESTIMATOR_PYTHON="${POSE_ESTIMATOR_PYTHON:-$HOME/robot_p0/.venv/bin/python}"
REFERENCE_MODEL_DIR="$HOME/tzb-study-output/issue48-p0-p1-2026-10-18/ska_1_reference/best_openvino_model/blue_block"
REPOSITORY_MODEL_DIR="$WS_ROOT/src/best_openvino_model/blue_block"
MODEL_DEFAULT="$REPOSITORY_MODEL_DIR"
if [[ -f "$REFERENCE_MODEL_DIR/yoloe-26l-seg.xml" && -f "$REFERENCE_MODEL_DIR/yoloe-26l-seg.bin" ]]; then
  MODEL_DEFAULT="$REFERENCE_MODEL_DIR"
fi
export TZB_OPENVINO_MODEL="${TZB_OPENVINO_MODEL:-$MODEL_DEFAULT}"
if [[ ! -x "$YOLO_NODE_PYTHON" || ! -x "$POSE_ESTIMATOR_PYTHON" ]]; then
  echo "Set YOLO_NODE_PYTHON and POSE_ESTIMATOR_PYTHON to the perception Python interpreter." >&2
  exit 1
fi
if [[ ! -f "$TZB_OPENVINO_MODEL/yoloe-26l-seg.xml" || ! -f "$TZB_OPENVINO_MODEL/yoloe-26l-seg.bin" ]]; then
  echo "OpenVINO XML/BIN pair is incomplete in: $TZB_OPENVINO_MODEL" >&2
  exit 1
fi

mkdir -p "$LOG_DIR" "$(dirname -- "$PID_FILE")"
LOG_FILE="$LOG_DIR/demo_$(date +%Y%m%d_%H%M%S).log"
setsid ros2 launch tzb_sim demo.launch.py autostart:=true use_rviz:=true \
  >"$LOG_FILE" 2>&1 < /dev/null &
PID=$!
echo "$PID" > "$PID_FILE"
sleep 1
if ! kill -0 "$PID" 2>/dev/null; then
  rm -f "$PID_FILE"
  echo "Launch exited during startup; inspect $LOG_FILE" >&2
  exit 1
fi
echo "Started isolated visual grasp demo: pid=$PID domain=$ROS_DOMAIN_ID localhost-only=1"
echo "log: $LOG_FILE"
echo "stop: $SCRIPT_DIR/stop_offline_demo.sh"
