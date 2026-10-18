# YOLO → 位姿估计 → P6 隔离软件验收

## 数据接口

实际源码定义的输入输出为：

| 步骤 | 输入 | 输出 | 依据 |
|---|---|---|---|
| YOLO 目标检测 | `/camera/color/image_raw` (`sensor_msgs/Image`) | `/yolo/detections` (`yolo_node/DetectionArray`)、`/yolo/annotated_image` | `tzb_ws/src/yolo_node/launch/yolo_node.launch.py`、`yolo_detector.py` |
| 目标位姿估计 | `/camera/depth/color/points` (`sensor_msgs/PointCloud2`)、`/camera/color/camera_info`、`/yolo/detections` | `/pose_estimator/object_pose` (`geometry_msgs/PoseStamped`) | `tzb_ws/src/pose_estimator/config/params.yaml`、`pose_estimation_node.py` |
| P6 任务协调 | 可配置 `target_topic`，另依赖 Nav2 Action 和软件状态接口 | `/p6/mission_state`、`/p6/mission_events` 与任务请求 | `Nav2_rea_ws/src/p6_autonomous_mission/launch/p6_autonomous_mission.launch.py`、`mission_coordinator.py` |

P6 独立离线启动仍保留原默认目标 Topic `/p2_offline/pose_estimator/object_pose`。新组合 Launch 显式将 `target_topic` 设置为位姿节点实际默认发布的 `/pose_estimator/object_pose`，不改变旧离线入口的现有默认行为。

## 组合 Launch 安全边界

`tools/perception/p6_yolo_pose_synthetic.launch.py` 使用唯一 ROS Domain、`ROS_LOCALHOST_ONLY=1` 和 CPU YOLO；相机由合成 RGB-D 发布器代替，P6 使用 cave simulator，任务结果和取消回执由独立的合成反馈节点提供。Launch 不包含相机、雷达、串口桥、底盘或机械臂驱动。

模型权重由测试环境通过 `model_path` 参数只读引用，不打包进仓库，也不复制到测试结果目录。合成位姿阈值与图像、点云仅用于接口验证，不代表现场检测置信度、相机标定、位姿精度或硬件性能。

ThinkBook 运行示例：先在隔离目录构建并加载 ROS 2 Humble 下的 P6、`yolo_node` 和 `pose_estimator` 安装前缀，再从只读路径指定已有模型目录：

```bash
P6_SYNTHETIC_PYTHON=/path/to/ROS-enabled-python \
YOLO_NODE_PYTHON=/path/to/ROS-enabled-python \
POSE_ESTIMATOR_PYTHON=/path/to/ROS-enabled-python \
python3 tools/perception/p6_yolo_pose_integration_smoke.py \
  --scenario normal \
  --model-dir /path/to/blue_block_model \
  --domain-id 219
```

将 `--scenario` 依次设为 `normal`、`no_target`、`low_confidence`、`timeout`、`cancel`、`recovery`。`model-dir` 需要包含 `metadata.yaml`、`yoloe-26l-seg.xml` 和 `yoloe-26l-seg.bin`。每次运行结果保存在独立临时目录，包含 P6 JSONL 事件和组合 Launch 日志。

## Console 可观测性边界

Agent Graph 能列出视觉、位姿和 P6 节点及其 Topic 名称；现有视觉接口可读取 RGB 与 YOLO 标注图像。固定链路遥测 allowlist 当前覆盖雷达、Point-LIO、`/scan` 和 `/map`，不包含 DetectionArray 或 P6 mission state/event 的消息内容。因此系统诊断可以对比节点与 Topic 的存在证据，但不能声称 Console 已展示 YOLO 检测项、位姿样本或 P6 任务事件正文。该限制在当前变更中保持显式，不将 Graph 可见误述为任务故障诊断已完成。

## 验收解释

六个合成场景分别检查检测与位姿输出、无目标与置信度门控不产生目标、缺少深度点云时不产生位姿、恢复深度输入后链路恢复、以及 P6 取消确认。正常与恢复场景均以 P6 确认目标并收到合成抓取结果为验收点；完整往返导航与卸载由既有 P6 场景测试覆盖。结果只证明隔离软件接口和模拟任务反馈工作；执行端、真实 RGB-D 时间同步、现场 TF/标定和自主任务闭环仍需独立验收。

当前模型在纯色无目标合成图上，即使推理阈值为 `0.0005`，也会产生低分候选框。无目标场景设置候选阈值 `0.0001`、发布门控 `0.1`；低置信度场景使用可见合成目标、候选阈值 `0.0001` 和发布门控 `0.5`，并从 YOLO 时间日志断言最大分数低于门控。其余场景使用 ROS Launch 中与当前 YOLO 启动配置一致的 `0.1` 推理和发布阈值。该测试只验证接口与门控行为，不代表现场置信度校准。
