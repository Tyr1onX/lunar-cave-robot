#!/usr/bin/env bash
set -Ee -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROBOT_ROOT="$(dirname "$WS")"
OUTPUT=""
CONFIRM_RECORDING=false
CHECK_ONLY=false

usage() {
  cat <<'USAGE'
检查当前 ROS 图中的真实传感器话题；默认只做检查，不启动设备，也不开始录制。

用法：
  record_rgbd_bag.sh [--check-only] [--confirm-recording] [--output BAG_DIRECTORY]

只有确认相机驱动和关节状态发布器已由用户在现场启动后，才使用
--confirm-recording 开始记录。此脚本本身绝不启动任何驱动或机器人程序。
默认输出：~/robot_p0/bags/p2b_<UTC 时间戳>
USAGE
}

while (($#)); do
  case "$1" in
    --check-only) CHECK_ONLY=true; shift ;;
    --confirm-recording) CONFIRM_RECORDING=true; shift ;;
    --output)
      (($# >= 2)) || { echo "--output 缺少目录" >&2; exit 2; }
      OUTPUT="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 /opt/ros/humble/setup.bash；需要在 ROS 2 Humble 主机运行。" >&2
  exit 2
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "当前 ROS_DISTRO=${ROS_DISTRO:-<未设置>}，拒绝跨发行版录制。" >&2
  exit 2
fi
set -u

required_topics=(
  /camera/color/image_raw
  /camera/depth/image_rect_raw
  /camera/depth/color/points
  /camera/color/camera_info
  /tf
  /tf_static
)
required_types=(
  sensor_msgs/msg/Image
  sensor_msgs/msg/Image
  sensor_msgs/msg/PointCloud2
  sensor_msgs/msg/CameraInfo
  tf2_msgs/msg/TFMessage
  tf2_msgs/msg/TFMessage
)

active_topics=()
failed=0
printf 'ROS_DOMAIN_ID=%s  ROS_DISTRO=%s\n' "${ROS_DOMAIN_ID:-0}" "$ROS_DISTRO"
printf '检查实时 ROS 图；本步骤不会启动相机或其他设备节点。\n'

for i in "${!required_topics[@]}"; do
  topic="${required_topics[$i]}"
  expected="${required_types[$i]}"
  actual="$(ros2 topic type "$topic" 2>/dev/null | head -n 1 || true)"
  if [[ "$actual" != "$expected" ]]; then
    printf 'FAIL %-38s type=%s (expected %s)\n' "$topic" "${actual:-absent}" "$expected"
    failed=1
    continue
  fi
  info="$(ros2 topic info --verbose "$topic" 2>/dev/null || true)"
  publishers="$(awk -F: '/Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"$info")"
  if [[ ! "$publishers" =~ ^[1-9][0-9]*$ ]]; then
    printf 'FAIL %-38s type=%s publisher_count=%s\n' "$topic" "$actual" "${publishers:-unknown}"
    failed=1
    continue
  fi
  printf 'PASS %-38s type=%s publishers=%s\n' "$topic" "$actual" "$publishers"
  active_topics+=("$topic")
done

joint_topics=()
for topic in /serial/arm_joint_state /joint_states; do
  actual="$(ros2 topic type "$topic" 2>/dev/null | head -n 1 || true)"
  [[ "$actual" == "sensor_msgs/msg/JointState" ]] || continue
  info="$(ros2 topic info --verbose "$topic" 2>/dev/null || true)"
  publishers="$(awk -F: '/Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"$info")"
  if [[ "$publishers" =~ ^[1-9][0-9]*$ ]]; then
    printf 'PASS %-38s type=%s publishers=%s (只订阅记录)\n' "$topic" "$actual" "$publishers"
    joint_topics+=("$topic")
  fi
done

if ((${#joint_topics[@]} == 0)); then
  echo 'FAIL 未发现正在发布的 /serial/arm_joint_state 或 /joint_states。'
echo '     此脚本不会启动串口或关节驱动；需要先由用户确认并启动只读状态来源。'
  failed=1
fi
active_topics+=("${joint_topics[@]}")

if ((failed)); then
  echo '采集前检查未通过；没有创建 Bag，也没有启动设备节点。' >&2
  exit 1
fi

if [[ "$CHECK_ONLY" == true ]]; then
  echo '采集前检查通过；根据 --check-only 请求，没有开始录制。'
  exit 0
fi
if [[ "$CONFIRM_RECORDING" != true ]]; then
  echo '采集前检查通过；当前仅完成检查。若已现场确认驱动与状态话题安全运行，再显式加 --confirm-recording。'
  exit 0
fi

if [[ -z "$OUTPUT" ]]; then
  OUTPUT="$ROBOT_ROOT/bags/p2b_$(date -u +%Y%m%dT%H%M%SZ)"
fi
OUTPUT="$(realpath -m "$OUTPUT")"
if [[ -e "$OUTPUT" ]]; then
  echo "目标已存在，拒绝覆盖：$OUTPUT" >&2
  exit 2
fi
mkdir -p "$(dirname "$OUTPUT")"
qos_file="$(mktemp)"
cleanup() { rm -f "$qos_file"; }
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
for topic in "${joint_topics[@]}"; do
  cat >> "$qos_file" <<QOS
$topic:
  history: keep_last
  depth: 50
  reliability: reliable
  durability: volatile
QOS
done

printf '\n开始 ROS Bag 录制到：%s\n按 Ctrl-C 正常结束并完成索引。\n' "$OUTPUT"
ros2 bag record --storage sqlite3 --output "$OUTPUT" \
  --qos-profile-overrides-path "$qos_file" "${active_topics[@]}"
