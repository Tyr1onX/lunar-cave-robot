# P9-I：视觉推理恢复与离线感知闭环

日期：2026-10-13。本文记录 P9-I 独立分支上的实际构建与离线测试。PR #14/#16 仍是独立工作；本 PR 以最新 `main` 为目标，只整理感知运行、依赖和验证，不修改 Mac GUI。测试期间没有启动传感器驱动、控制器、串口、电机或实体机器人。

## 1. YOLOE 蓝色方块模型根因与修复

NUC 旧 `tzb_ws/src/best_openvino_model/blue_block/` 工作区找到配套 OpenVINO IR、metadata、提示词和 prompt embedding。原文件保持不变；验证副本位于 NUC 临时隔离目录。本仓库不提交 111 MB 权重、prompt 文件或 embedding 文件。

| 文件 | 字节数 | SHA-256 |
| --- | ---: | --- |
| `yoloe-26l-seg.xml` | 639,550 | `b5adb9a893197deb049e69e53b6c9cc0b1511a50eb23522ae7f1d125e27e1d34` |
| `yoloe-26l-seg.bin` | 111,726,400 | `860ba6b59b598c650e81b35f2357578db895f69efe78886adfd60d150c54da72` |
| 修复后 `metadata.yaml` | 440 | `d0adabde239e7ce9786d7d38b37b85037f2a695f9d9b24bbd3c13114d2182bf5` |
| `prompts.txt` | 22 | `b26c6b55f893146f15c5fe6568c8d6186306eeb72f1269850dd55cbe4822723c` |
| `prompt_embeddings.npz` | 4,480 | `cabdd576ee23604f3d01aa94b92f5b77cea702e11fe64d83b64527ab8e8c5d16` |

embedding 中的 prompt 名称与文本 `blue_block`、`blue block` 对应，矩阵形状为 `(1,2,512)`。IR 只暴露一个 RGB 图像输入，提示 embedding 已烘焙在导出图中；测试验证 prompt/embedding/metadata 名称一致，不把 `.npz` 当作动态 ROS 输入。原导出声明 `task=segment`、`head=YOLOESegment26`，Ultralytics 版本 `8.4.164`。OpenVINO IR 实际接口为输入 `[1,3,640,640]`、输出 `[1,300,38]` 与 `[1,32,160,160]`。

原仓库的精简 metadata 漏掉了导出属性 `end2end: true`。在不更改模型任务类型的情况下，以同一个 XML/BIN 和只移除该字段的临时 metadata 复现旧错误：Ultralytics 将 300 个检测行误当作通道，按 `300 - 4 - 2 = 294` 推出 mask dimension，随后在 `13×294` 与 `32×25600` 矩阵乘法失败。恢复导出真实标记 `end2end: true` 后，Ultralytics 8.4.164 / OpenVINO 2025.4.1 成功完成直接推理并生成 6 个框及 `(6,640,640)` masks。没有把任务类型改成 detect，也没有以 `haomu` 权重代替 blue-block 模型。

节点对模型目录名不符合 Ultralytics `*_openvino_model` 约定的情况创建独立临时别名。别名只链接目标 XML/BIN 和 metadata，过滤 macOS 压缩包常见的 `._*` AppleDouble 文件；每个节点实例拥有独立临时目录，避免并发实例互相删除模型链接。单独的 ROS smoke 测试特意放入伪造 `._decoy.xml/.bin`，确认模型加载选择正确。

## 2. ROS YOLO 节点验证

Ubuntu 22.04.5、ROS 2 Humble、Python 3.10、Ultralytics 8.4.164 和 OpenVINO 2025.4.1 上，实际 `yolo_detector.py` ROS 节点通过隔离 topic 接收合成 640×640 BGR 图像并发布 `yolo_node/DetectionArray` 与标注 `sensor_msgs/Image`：

- 两种类别文本均有输出：`class_id=1 / blue block` 与 `class_id=0 / blue_block`；一次运行得到 4 条记录，confidence 为 `0.0011198`、`0.0008570`、`0.0005725` 和 `0.0005594`。
- 每个框的坐标、宽高、有限置信度及图像范围断言通过；`DetectionArray` 保留输入时间戳与 `synthetic_rgb_optical_frame`。标注图像为 `640×640 bgr8`，时间戳与 frame 同样保留。
- 节点日志内推理总耗时约 244 ms（Infer 241 ms）；测试从首次发布到两个输出均收到的 wall latency 约 3.37 s，包含 ROS 发现/启动和输出等待。
- 测试显式将 `conf_threshold` 和 `publish_conf_threshold` 设为 `0.0005` 以暴露低分输出；这张合成图的所有结果都低于默认 `0.1` 门限，故默认配置下会被过滤。此次没有下调生产门限；需要获准的标注图片/真实数据之后才能评估门限与分类效果。这些结果不证明类别识别准确、置信度可用于抓取，也不代表真实相机画面效果。

## 3. 位姿 ROS 接口与异常输入

实际 `pose_estimation_node` 接收离线生成的 `DetectionArray + PointCloud2 + CameraInfo`，输出 `geometry_msgs/PoseStamped`。输入点云由仓库的 `half_block.pcd` 模板投影得到，包含 1,634 个有效投影点；CameraInfo 为合成的 640×480、`fx=fy=600`，坐标系是 `synthetic_camera_color_optical_frame`。最近一次输出位置 `(-0.00031, 0.00044, 0.64873)`、四元数范数 `1.000000`、wall latency `277.9 ms`；消息 frame 与输入点云时间戳保持一致。

该模板/合成场景构造不包含真实相机噪声、遮挡、深度误差或外参，因此位置数值不是精度指标。多次运行的 RANSAC 输出略有变化；例如此前一次合成运行的 x 为 `0.01352 m`。这些结果只证明 ROS 消息与算法链可完成一次软件输出，不能描述为真实目标定位精度或稳定性验收。

P9-I 为位姿点云解析加上输入布局检查：确认 XYZ 字段存在、类型为单值 FLOAT32、offset 不超出 `point_step`、`row_step` 与 payload 长度足够；支持有行尾填充与大端 FLOAT32。错误缺字段点云以明确 warning 跳过，不再由 `KeyError` 产生 traceback。测试同时验证有组织点云 ROI/全量解码、行尾 padding、大端数据以及缺少 Y/Z 的消息。

ROS 负例额外覆盖空 `DetectionArray`（不发布 PoseStamped）、缺 Y/Z 字段的 PointCloud2（明确拒绝、不发布位姿）；节点仍存活并随后处理有效的合成输入。YOLO 发布链与 PoseEstimator 输入/输出链分别通过 ROS smoke，但没有把实际 YOLO 低分检测直接串到位姿节点的综合测试；消息接口和坐标系行为通过，真实同步、RGB-D 对齐、相机内参/外参、TF 树、目标模板与现场目标之间的差异仍未验证。

## 4. 驱动、运行依赖与可搬迁安装

NUC 隔离构建结果：

| 包 | 版本/依赖 | 构建及检查 |
| --- | --- | --- |
| `livox_ros_driver2` | 1.2.4；外置 Livox SDK2 1.4.3 | 独立、非 symlink install 全新构建通过；`ldd` 无未解析依赖；installed RPATH 未包含临时 build/install 前缀。库来自 `/usr/local/lib/liblivox_lidar_sdk_shared.so`，SHA-256 `97f0663f71444b1fb6cf40baaa778f917bfc9716d187f01bc3b6b6bbdbcbaebe`。 |
| `orbbec_camera_msgs`、`orbbec_camera` | 2.9.3；外置官方 Orbbec SDK v2.9.3，发行包 SHA-256 `8516081b2201f841b1aa444dd203975e06891d73a2694dd2b3d864254f89ff0d` | 两包构建通过。安装后的完整 overlay 复制到另一临时目录再 source，`ldd` 全部依赖解析成功，`libOrbbecSDK.so.2` 从搬迁后相机包内 `$ORIGIN` 目录加载；RUNPATH 为 `$ORIGIN:$ORIGIN/..`，不引用原 SDK 临时解包路径。 |
| `yolo_node` | 1.0.0；Ultralytics 8.4.164、OpenVINO 2025.4.1、NumPy 1.26.4 | 随上述 overlay 构建通过；对最新模型别名逻辑再以新 build/install 前缀重建，ROS smoke 通过。 |
| `pose_estimator` | 1.0.0；Open3D 0.19.0、SciPy 1.15.3、OpenCV 4.11.0 | 包构建通过；加入 PointCloud2 字段校验后再次独立重建，集成 smoke 通过。 |

Orbbec CMake 的系统 udev 规则写入受 `INSTALL_UDEV_RULES` 显式开关保护。以 `BUILDING_PACKAGE=1`、`INSTALL_UDEV_RULES=OFF` 并设置隔离 `DESTDIR` 执行安装后，没有生成 `/etc/udev/rules.d/99-obsensor-libusb.rules`。CMake install 仍可在 `INSTALL_UDEV_RULES=ON` 时将规则放入 package share，供人工查看/安装；测试没有修改系统规则。

OpenVINO 与 Ultralytics 的运行版本已固定在 `tzb_ws/requirements.txt`，Python 3.10 CPU lock 与实测版本一致。Orbbec SDK v2.9.3、Livox SDK2 1.4.3 及相机/雷达设备配置仍需由部署环境单独提供。Livox 使用系统安装的 SDK2；Orbbec SDK 是构建时外部输入，并由安装规则随隔离 overlay 安装到包库目录。两份 SDK 均不进入 Git。

## 5. 静态入口、ROS Topic 与现场网络输入

本轮仅做了参数展示，不执行 launch：

- `ros2 launch livox_ros_driver2 mid360_static.launch.py --show-args`：`config_file` 与 `cmdline_input_bd_code` 为必需现场输入。预期发布 `/livox/lidar` (`sensor_msgs/PointCloud2`) 和 `/livox/imu` (`sensor_msgs/Imu`)。
- `ros2 launch pose_estimator camera_switch.launch.py --show-args`、`ros2 launch pose_estimator perception_bringup.launch.py --show-args`：参数可解析。默认候选 RGB `/camera/color/image_raw`、内参 `/camera/color/camera_info`、深度 `/camera/depth/image_rect_raw`、彩色点云 `/camera/depth/color/points`；ROS smoke 使用隔离 topic，未连相机。
- YOLO 输入 `/camera/color/image_raw`，输出 `/yolo/detections` (`yolo_node/DetectionArray`) 和 `/yolo/annotated_image` (`sensor_msgs/Image`)。位姿输入 RGB-D 对齐点云、CameraInfo、DetectionArray，输出 `/pose_estimator/object_pose` (`geometry_msgs/PoseStamped`)。

现场采集前必须让设备负责人确认并提供：NUC 实际雷达网卡名及静态 IPv4/掩码/路由/防火墙规则、MID-360 设备 IP 与广播码、主机接收地址、设备/主机 UDP 端口映射、选定 JSON 配置，以及实际安装外参。仓库历史 `MID360_config.json` 地址和外参只是候选值，不能直接当成现场真值。本轮没有探测传感器网络或启动驱动。

## 6. 可复跑命令与未完成项

在具有项目 ROS 2 Humble 工作区、外部授权模型包和以上依赖的隔离环境中，源入对应安装 overlay 后，可运行：

```bash
python3 tools/perception/verify_yoloe_openvino.py \
  --model /path/to/blue_block_bundle --confidence 0.0005

python3 tools/perception/offline_yolo_ros_smoke.py \
  --model /path/to/blue_block_bundle

python3 tools/perception/offline_pose_ros_smoke.py \
  --model-pcd tzb_ws/src/pose_estimator/config/half_block.pcd \
  --params tzb_ws/src/pose_estimator/config/params.yaml
```

这些脚本设置独立 ROS domain 与 localhost-only 通信，生成可重复的合成图像/点云，不需要相机、雷达或实体控制器。目标仓库和本次检查到的 NUC ROS 工作区没有 RGB-D ROS Bag，因此没有真实数据可直接回放。

仍未完成：blue-block 真实标注集准确率/召回率评估；外部 BIN 模型能否在团队设备间合法共享与稳定发布的确认；相机和雷达静态采集；RGB、深度、CameraInfo、点云、TF 的现场时间/坐标对齐；带真值的位姿误差评估；ROS Bag 录制与回放。项目相关路径中未找到该模型的训练 checkpoint `.pt` 或完整导出命令/运行记录。更准确的评估输入至少需要获准共享的带标注 RGB 及对应类别/标注格式；要重导模型还需训练 checkpoint 或训练集、数据配置、提示词/embedding 生成信息、Ultralytics/OpenVINO 导出命令和 exporter 版本。没有这些输入时不伪造训练或权重文件。
