#!/usr/bin/env bash
set -Ee -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROBOT_ROOT="$(dirname "$WS")"
if (($# < 1 || $# > 2)); then
  echo "用法：$0 INPUT_BAG [OUTPUT_REPLAY_BAG]" >&2
  exit 2
fi
INPUT_BAG="$(realpath "$1")"
OUTPUT_BAG="${2:-$ROBOT_ROOT/bags/p2b_replay_$(date -u +%Y%m%dT%H%M%SZ)}"
OUTPUT_BAG="$(realpath -m "$OUTPUT_BAG")"
[[ -d "$INPUT_BAG" ]] || { echo "输入 Bag 不存在：$INPUT_BAG" >&2; exit 2; }
[[ "$INPUT_BAG" != "$OUTPUT_BAG" ]] || { echo '输入和输出 Bag 不能相同。' >&2; exit 2; }
[[ ! -e "$OUTPUT_BAG" ]] || { echo "拒绝覆盖已有路径：$OUTPUT_BAG" >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo '未找到 ROS 2 Humble。' >&2; exit 2; }
[[ -f "$WS/install/offline/setup.bash" ]] || { echo '缺少 P2 离线构建 overlay。' >&2; exit 2; }

# Software-only and local-only; never launches a camera, serial, base, or arm driver.
source /opt/ros/humble/setup.bash
source "$WS/install/offline/setup.bash"
[[ "${ROS_DISTRO:-}" == humble ]] || { echo "当前不是 ROS Humble：${ROS_DISTRO:-unset}" >&2; exit 2; }
set -u
export ROS_DOMAIN_ID="${P2B_ROS_DOMAIN_ID:-187}"
export ROS_LOCALHOST_ONLY=1

existing_nodes="$(ros2 node list 2>/dev/null || true)"
if [[ -n "$existing_nodes" ]]; then
  echo "ROS 域 $ROS_DOMAIN_ID 已有节点，拒绝与其他运行混用：" >&2
  printf '%s\n' "$existing_nodes" >&2
  exit 1
fi

"$WS/tools/p2b/audit_rgbd_bag.py" --stage capture "$INPUT_BAG"
mkdir -p "$(dirname "$OUTPUT_BAG")"
launch_log="$(mktemp)"
qos_file="$(mktemp)"
launch_pid=""
recorder_pid=""
cleanup() {
  if [[ -n "$recorder_pid" ]] && kill -0 "$recorder_pid" 2>/dev/null; then
    kill -INT "$recorder_pid" 2>/dev/null || true
    wait "$recorder_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  rm -f "$launch_log" "$qos_file"
}
trap cleanup EXIT INT TERM
cat > "$qos_file" <<'QOS'
/camera/color/image_raw:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile
/camera/depth/image_rect_raw:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile
/camera/depth/color/points:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile
/camera/color/camera_info:
  history: keep_last
  depth: 10
  reliability: best_effort
  durability: volatile
/tf:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: volatile
/tf_static:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: transient_local
QOS

# Explicitly disable camera launch; only YOLO and pose_estimator are started.
ros2 launch pose_estimator perception_bringup.launch.py launch_camera:=false \
  >"$launch_log" 2>&1 &
launch_pid=$!

ready=false
for _ in $(seq 1 90); do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    cat "$launch_log" >&2
    echo '感知节点提前退出；没有播放输入 Bag。' >&2
    exit 1
  fi
  yolo_publishers="$(ros2 topic info --verbose /yolo/detections 2>/dev/null | awk -F: '/Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}')"
  pose_publishers="$(ros2 topic info --verbose /pose_estimator/object_pose 2>/dev/null | awk -F: '/Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}')"
  if [[ "$yolo_publishers" =~ ^[1-9][0-9]*$ && "$pose_publishers" =~ ^[1-9][0-9]*$ ]]; then
    ready=true
    break
  fi
  sleep 1
done
if [[ "$ready" != true ]]; then
  cat "$launch_log" >&2
  echo 'YOLO 或 pose_estimator 未就绪；没有播放输入 Bag。' >&2
  exit 1
fi

# Dynamic discovery records only the replay inputs plus YOLO/pose outputs.
topic_regex='^(/camera/color/image_raw|/camera/depth/image_rect_raw|/camera/depth/color/points|/camera/color/camera_info|/tf|/tf_static|/serial/arm_joint_state|/joint_states|/yolo/detections|/pose_estimator/object_pose)$'
ros2 bag record --storage sqlite3 --regex "$topic_regex" \
  --qos-profile-overrides-path "$qos_file" --output "$OUTPUT_BAG" &
recorder_pid=$!
sleep 3
if ! kill -0 "$recorder_pid" 2>/dev/null; then
  echo '回放结果 Bag 记录器未能启动；没有播放输入 Bag。' >&2
  exit 1
fi

printf '在 ROS_DOMAIN_ID=%s、localhost-only 环境回放：%s\n' "$ROS_DOMAIN_ID" "$INPUT_BAG"
printf '输出完整感知回放 Bag：%s\n' "$OUTPUT_BAG"
ros2 bag play --rate "${P2B_REPLAY_RATE:-0.5}" "$INPUT_BAG"
sleep 2
kill -INT "$recorder_pid" 2>/dev/null || true
wait "$recorder_pid" 2>/dev/null || true
recorder_pid=""

"$WS/tools/p2b/audit_rgbd_bag.py" --stage replay "$OUTPUT_BAG"
printf '\n注意：此流程验收真实输入 Bag 的 YOLO 与 3D 位姿估计；不会启动规划器或硬件控制。\n'
