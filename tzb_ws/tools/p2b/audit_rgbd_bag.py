#!/usr/bin/env python3
"""Read-only structural and timing audit for P2b ROS 2 RGB-D bags."""
from __future__ import annotations
import argparse
import bisect
import math
import sys
from collections import Counter, defaultdict, deque
from pathlib import Path

RAW_TYPES = {
    "/camera/color/image_raw": "sensor_msgs/msg/Image",
    "/camera/depth/image_rect_raw": "sensor_msgs/msg/Image",
    "/camera/depth/color/points": "sensor_msgs/msg/PointCloud2",
    "/camera/color/camera_info": "sensor_msgs/msg/CameraInfo",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}
JOINT_TOPICS = ("/serial/arm_joint_state", "/joint_states")
DETECTION_TOPIC = "/yolo/detections"
POSE_TOPIC = "/pose_estimator/object_pose"
EXPECTED_ARM_JOINTS = {f"J{i}_joint" for i in range(1, 7)}


def stamp_ns(stamp):
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else None


def clean_frame(frame):
    return (frame or "").lstrip("/")


def percentile(values, percentile_value):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(percentile_value * len(ordered)) - 1))
    return ordered[index]


def nearest_deltas(reference, candidates):
    if not reference or not candidates:
        return []
    candidates = sorted(candidates)
    output = []
    for value in reference:
        index = bisect.bisect_left(candidates, value)
        nearby = candidates[max(0, index - 1):min(len(candidates), index + 1)]
        if nearby:
            output.append(min(abs(value - other) for other in nearby))
    return output


def load_model_labels(path):
    try:
        import yaml
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        names = data.get("names", {})
        if isinstance(names, dict):
            return {str(value) for value in names.values()}
        if isinstance(names, list):
            return {str(value) for value in names}
    except Exception:
        pass
    labels = set()
    in_names = False
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if line.startswith("names:"):
            in_names = True
            continue
        if in_names:
            if line and not line[0].isspace():
                break
            if ":" in line:
                labels.add(line.split(":", 1)[1].strip().strip("'\""))
    return labels


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS Bag 目录（本工具只读）")
    parser.add_argument("--stage", choices=("capture", "replay"), default="capture")
    parser.add_argument("--target-class", default="blue_block")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--sync-tolerance-ms", type=float, default=50.0)
    parser.add_argument("--joint-max-gap-ms", type=float, default=500.0)
    parser.add_argument("--tf-max-gap-ms", type=float, default=100.0)
    parser.add_argument("--model-metadata", type=Path,
                        default=Path(__file__).resolve().parents[2] / "src/best_openvino_model/blue_block/metadata.yaml")
    args = parser.parse_args()
    if not args.bag.is_dir():
        parser.error(f"Bag 目录不存在：{args.bag}")
    try:
        import rosbag2_py
        from rosidl_runtime_py.utilities import get_message
        from rclpy.serialization import deserialize_message
    except Exception as exc:
        print(f"FAIL 无法导入 ROS 2 Humble rosbag2_py/消息运行库：{exc}", file=sys.stderr)
        return 2

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
        topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    except Exception as exc:
        print(f"FAIL 无法打开 SQLite3 ROS Bag：{exc}", file=sys.stderr)
        return 2

    print(f"Bag: {args.bag}\nStage: {args.stage}\nTopics: {len(topic_types)}")
    errors, warnings = [], []
    counts = Counter()
    stamps = defaultdict(list)
    frames = defaultdict(Counter)
    image_sizes = defaultdict(Counter)
    cloud_fields = set()
    camera_intrinsics, joint_samples, pose_records = [], [], []
    tf_edges = set()
    tf_live_edges = set()
    tf_dynamic_times = defaultdict(list)
    detection_classes = Counter()
    deserializers = {}

    for topic, expected_type in RAW_TYPES.items():
        actual = topic_types.get(topic)
        if actual != expected_type:
            errors.append(f"{topic}: 类型为 {actual or '缺失'}，预期 {expected_type}")
    joint_topics = [name for name in JOINT_TOPICS if topic_types.get(name) == "sensor_msgs/msg/JointState"]
    if not joint_topics:
        errors.append("缺少 /serial/arm_joint_state 和 /joint_states 两种 JointState 话题")
    if args.stage == "replay":
        replay_types = {
            DETECTION_TOPIC: "yolo_node/msg/DetectionArray",
            POSE_TOPIC: "geometry_msgs/msg/PoseStamped",
        }
        for topic, expected_type in replay_types.items():
            actual_type = topic_types.get(topic)
            if actual_type != expected_type:
                errors.append(f"回放验收 {topic} 类型为 {actual_type or '缺失'}，预期 {expected_type}")

    while reader.has_next():
        topic, raw, _bag_time = reader.read_next()
        counts[topic] += 1
        type_name = topic_types.get(topic)
        if not type_name:
            continue
        try:
            cls = deserializers.get(type_name)
            if cls is None:
                cls = get_message(type_name)
                deserializers[type_name] = cls
            msg = deserialize_message(raw, cls)
        except Exception as exc:
            errors.append(f"{topic}: 反序列化 {type_name} 失败：{exc}")
            continue

        header = getattr(msg, "header", None)
        msg_stamp = stamp_ns(getattr(header, "stamp", None)) if header else None
        if msg_stamp is not None:
            stamps[topic].append(msg_stamp)
        frame = clean_frame(getattr(header, "frame_id", "")) if header else ""
        if frame:
            frames[topic][frame] += 1

        if type_name == "sensor_msgs/msg/Image":
            image_sizes[topic][(int(msg.width), int(msg.height), str(msg.encoding))] += 1
        elif type_name == "sensor_msgs/msg/PointCloud2":
            cloud_fields.update(field.name for field in msg.fields)
        elif type_name == "sensor_msgs/msg/CameraInfo":
            k = list(msg.k)
            if len(k) >= 9:
                camera_intrinsics.append((float(k[0]), float(k[4]), int(msg.width), int(msg.height), frame))
        elif type_name == "sensor_msgs/msg/JointState":
            joint_samples.append((topic, msg_stamp, list(msg.name), list(msg.position)))
        elif type_name == "tf2_msgs/msg/TFMessage":
            for transform in msg.transforms:
                parent = clean_frame(transform.header.frame_id)
                child = clean_frame(transform.child_frame_id)
                if parent and child:
                    edge = (parent, child)
                    tf_edges.add(edge)
                    transform_stamp = stamp_ns(transform.header.stamp)
                    if topic == "/tf":
                        tf_live_edges.add(edge)
                        if transform_stamp is not None:
                            tf_dynamic_times[edge].append(transform_stamp)
        elif topic == DETECTION_TOPIC and hasattr(msg, "detections"):
            for detection in msg.detections:
                class_name = str(getattr(detection, "class_name", "")).strip()
                if class_name:
                    detection_classes[class_name] += 1
        elif topic == POSE_TOPIC and hasattr(msg, "pose"):
            p, q = msg.pose.position, msg.pose.orientation
            values = [float(p.x), float(p.y), float(p.z), float(q.x), float(q.y), float(q.z), float(q.w)]
            norm = math.sqrt(sum(value * value for value in values[3:]))
            pose_records.append((msg_stamp, frame, values, norm))

    print("\n消息数量：")
    for topic in sorted(counts):
        print(f"  {topic}: {counts[topic]}")
    for topic in RAW_TYPES:
        if counts[topic] == 0:
            errors.append(f"{topic}: Bag 中没有消息")
    timed_topics = ("/camera/color/image_raw", "/camera/depth/image_rect_raw", "/camera/depth/color/points")
    for topic in timed_topics:
        coverage = len(stamps[topic]) / max(1, counts[topic])
        print(f"header 时间戳覆盖率 {topic}: {coverage:.1%}")
        if coverage < 0.95:
            errors.append(f"{topic}: 非零 header 时间戳覆盖率低于 95%")
        out_of_order = sum(later < earlier for earlier, later in zip(stamps[topic], stamps[topic][1:]))
        if out_of_order:
            warnings.append(f"{topic}: 按 Bag 顺序发现 {out_of_order} 个倒序 header 时间戳")

    for topic, sizes in sorted(image_sizes.items()):
        print(f"图像 {topic}: {dict(sizes)}")
        if not sizes or any(width <= 0 or height <= 0 for width, height, _ in sizes):
            errors.append(f"{topic}: 图像宽高无效")
    if not {"x", "y", "z"}.issubset(cloud_fields):
        errors.append(f"点云缺少 x/y/z 字段，实际字段：{sorted(cloud_fields)}")
    else:
        print(f"点云字段：{sorted(cloud_fields)}")

    valid_intrinsics = [(fx, fy, width, height, frame) for fx, fy, width, height, frame in camera_intrinsics
                        if fx > 0 and fy > 0 and width > 0 and height > 0]
    if not valid_intrinsics:
        errors.append("CameraInfo 没有有效的图像尺寸或正焦距 fx/fy")
    else:
        fx, fy, _, _, info_frame = valid_intrinsics[0]
        print(f"CameraInfo: {len(valid_intrinsics)} 帧有效；示例 fx={fx:.3f}, fy={fy:.3f}, frame={info_frame or '<empty>'}")

    cloud_frames = frames["/camera/depth/color/points"]
    color_frames = frames["/camera/color/image_raw"]
    info_frames = frames["/camera/color/camera_info"]
    print(f"相机帧：RGB={dict(color_frames)} 点云={dict(cloud_frames)} CameraInfo={dict(info_frames)}")
    if not cloud_frames:
        errors.append("点云消息没有 frame_id")
    if not color_frames:
        errors.append("RGB 消息没有 frame_id")
    if color_frames and cloud_frames and set(color_frames) != set(cloud_frames):
        warnings.append("RGB 与彩色注册点云的 frame_id 不一致；需确认驱动对齐输出及下游坐标假设")
    if info_frames and cloud_frames and set(info_frames) != set(cloud_frames):
        warnings.append("CameraInfo 与彩色注册点云的 frame_id 不一致；需确认实际光学坐标系")

    graph = defaultdict(set)
    for parent, child in tf_edges:
        graph[parent].add(child)
        graph[child].add(parent)
    base = clean_frame(args.base_frame)
    for cloud_frame in cloud_frames:
        parents = {cloud_frame: None}
        queue = deque([cloud_frame])
        while queue and base not in parents:
            current = queue.popleft()
            for neighbor in graph[current]:
                if neighbor not in parents:
                    parents[neighbor] = current
                    queue.append(neighbor)
        if base not in parents:
            errors.append(f"TF 中没有从点云帧 {cloud_frame} 到规划基座 {base} 的连通路径")
            continue
        path_edges, node = [], base
        while parents[node] is not None:
            previous = parents[node]
            directed = (node, previous) if (node, previous) in tf_edges else (previous, node)
            path_edges.append(directed)
            node = previous
        print(f"TF: {cloud_frame} 与 {base} 拓扑连通（路径边数={len(path_edges)}）")
        dynamic_edges = []
        for edge in path_edges:
            reverse = (edge[1], edge[0])
            edge_times = tf_dynamic_times.get(edge) or tf_dynamic_times.get(reverse)
            if edge in tf_live_edges or reverse in tf_live_edges:
                if not edge_times:
                    errors.append(f"TF 动态边 {edge[0]}->{edge[1]} 没有非零时间戳")
                else:
                    dynamic_edges.append((edge, edge_times))
        if not dynamic_edges:
            warnings.append(f"TF 路径 {cloud_frame}->{base} 没有 /tf 动态变换；仅验证了拓扑，需现场确认机械臂状态到相机 frame 的实时 TF")
        else:
            for edge, edge_times in dynamic_edges:
                tf_deltas = nearest_deltas(stamps["/camera/color/image_raw"], edge_times)
                if not tf_deltas:
                    errors.append(f"TF 动态边 {edge[0]}->{edge[1]} 没有可与 RGB 比对的时间戳")
                    continue
                tf_p95 = percentile(tf_deltas, 0.95) / 1e6
                print(f"TF 时间同步 {edge[0]}->{edge[1]}: p95={tf_p95:.2f} ms")
                if tf_p95 > args.tf_max_gap_ms:
                    errors.append(f"TF 边 {edge[0]}->{edge[1]} 与 RGB 时间差 p95 超过 {args.tf_max_gap_ms:.1f} ms")
    if not tf_edges:
        errors.append("/tf 与 /tf_static 中没有有效 TF 变换")

    rgb_stamps = stamps["/camera/color/image_raw"]
    for other in ("/camera/depth/color/points", "/camera/depth/image_rect_raw"):
        deltas = nearest_deltas(rgb_stamps, stamps[other])
        if not deltas:
            errors.append(f"缺少可用于同步检查的 {other} 非零 header 时间戳")
            continue
        p95_ms = percentile(deltas, 0.95) / 1e6
        within = sum(delta <= args.sync_tolerance_ms * 1e6 for delta in deltas)
        print(f"时间同步 RGB↔{other}: 配对={len(deltas)}，p95={p95_ms:.2f} ms，阈值内={within}/{len(deltas)}（{args.sync_tolerance_ms:.1f} ms）")
        if p95_ms > args.sync_tolerance_ms:
            errors.append(f"RGB 与 {other} 最近时间戳配对 p95 超过 {args.sync_tolerance_ms:.1f} ms")

    info_deltas = nearest_deltas(rgb_stamps, stamps["/camera/color/camera_info"])
    if info_deltas:
        print(f"时间同步 RGB↔CameraInfo: p95={percentile(info_deltas, 0.95) / 1e6:.2f} ms（CameraInfo 可低频发布，需结合帧率判断）")
    else:
        warnings.append("CameraInfo 缺少非零 header 时间戳，未能做时间对齐统计")

    good_joint_sample = False
    for _topic, timestamp, names, positions in joint_samples:
        if len(names) != len(positions) or timestamp is None:
            continue
        by_name = dict(zip(names, positions))
        if EXPECTED_ARM_JOINTS.issubset(by_name) and all(
            math.isfinite(float(by_name[name])) for name in EXPECTED_ARM_JOINTS
        ):
            good_joint_sample = True
            break
    if not good_joint_sample:
        errors.append("没有含 J1_joint 至 J6_joint、数组长度匹配且位置有限的关节状态样本")
    joint_timestamps = [sample[1] for sample in joint_samples if sample[1] is not None]
    if rgb_stamps and joint_timestamps:
        deltas = nearest_deltas(rgb_stamps, joint_timestamps)
        p95_ms = percentile(deltas, 0.95) / 1e6
        print(f"RGB↔关节状态最近时间差: p95={p95_ms:.2f} ms（上限 {args.joint_max_gap_ms:.1f} ms）")
        if p95_ms > args.joint_max_gap_ms:
            errors.append(f"关节状态相对 RGB 过期：p95 {p95_ms:.2f} ms")
    elif joint_timestamps:
        warnings.append("RGB header 时间戳缺失，无法评估关节状态时间差")

    if args.model_metadata.is_file():
        try:
            model_labels = load_model_labels(args.model_metadata)
            if args.target_class not in model_labels:
                errors.append(f"模型元数据没有目标类别 {args.target_class!r}；现有类别：{sorted(model_labels)}")
            else:
                print(f"模型类别匹配：{args.target_class} ∈ {sorted(model_labels)}")
        except Exception as exc:
            warnings.append(f"模型类别元数据读取失败：{exc}")
    else:
        warnings.append(f"未找到模型元数据：{args.model_metadata}")

    if args.stage == "replay":
        detection_frames = frames[DETECTION_TOPIC]
        if not detection_frames:
            errors.append("YOLO DetectionArray 没有 frame_id")
        elif color_frames and set(detection_frames) != set(color_frames):
            errors.append("YOLO DetectionArray frame_id 与输入 RGB frame 不一致")
        detection_stamps = stamps[DETECTION_TOPIC]
        if counts[DETECTION_TOPIC] and len(detection_stamps) / counts[DETECTION_TOPIC] < 0.95:
            errors.append("YOLO DetectionArray 非零 header 时间戳覆盖率低于 95%")
        detection_deltas = nearest_deltas(detection_stamps, rgb_stamps)
        if detection_deltas:
            detection_p95 = percentile(detection_deltas, 0.95) / 1e6
            print(f"时间同步 RGB↔YOLO 检测: p95={detection_p95:.2f} ms")
            if detection_p95 > args.sync_tolerance_ms:
                errors.append("YOLO 检测与 RGB 时间戳不同步")
        else:
            errors.append("YOLO DetectionArray 缺少可与 RGB 比对的 header 时间戳")
        if not detection_classes:
            errors.append("回放输出中没有带类别名称的 YOLO 检测")
        elif args.target_class not in detection_classes:
            errors.append(f"检测到类别 {sorted(detection_classes)}，没有目标类别 {args.target_class!r}")
        else:
            print(f"YOLO 检测类别：{dict(detection_classes)}")
        if not pose_records:
            errors.append("回放输出中没有 object_pose")
        else:
            cloud_frame_set = set(cloud_frames)
            invalid, wrong_frame = 0, 0
            for timestamp, frame, values, norm in pose_records:
                if timestamp is None or not all(math.isfinite(value) for value in values) or not 0.9 <= norm <= 1.1:
                    invalid += 1
                if frame not in cloud_frame_set:
                    wrong_frame += 1
            print(f"目标位姿：{len(pose_records)} 条；无效数={invalid}；与点云帧不一致={wrong_frame}；示例 frame={pose_records[0][1] or '<empty>'}")
            if invalid:
                errors.append(f"{invalid} 条目标位姿时间戳/数值/四元数无效")
            if wrong_frame:
                errors.append(f"{wrong_frame} 条目标位姿 frame_id 不属于彩色点云坐标系")
            pose_stamps = [record[0] for record in pose_records if record[0] is not None]
            deltas = nearest_deltas(pose_stamps, stamps["/camera/depth/color/points"])
            if deltas:
                p95_ms = percentile(deltas, 0.95) / 1e6
                print(f"位姿↔点云时间差: p95={p95_ms:.2f} ms")
                if p95_ms > args.sync_tolerance_ms:
                    errors.append("目标位姿时间戳与输入点云不同步")
            else:
                errors.append("无法比较目标位姿和点云时间戳")

    if warnings:
        print("\nWARN：")
        for item in warnings:
            print(f"  - {item}")
    if errors:
        print("\nFAIL：")
        for item in errors:
            print(f"  - {item}")
        print("\n结果：FAIL（真实 Bag 结构或同步验收未通过）")
        return 1
    print("\n结果：PASS（此阶段 Bag 检查通过；不代表机械臂实机动作或比赛精度已验证）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
