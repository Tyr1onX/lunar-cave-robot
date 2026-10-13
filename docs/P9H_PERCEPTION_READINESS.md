# P9-H 感知链路构建与现场准备

日期：2026-10-13。本文记录从 PR #14 源码快照进行的 NUC 隔离构建和离线检查，不代表传感器或实机验收。

> **历史状态说明：** 本报告中的 blue-block 模型“后处理失败、YOLO 节点未就绪”是 P9-H 当时的真实结果。P9-I 随后查明是 metadata 遗漏 `end2end: true`，已在不改任务类型的情况下修复，并通过直接推理和实际 ROS 节点合成图像 smoke。当前结果、模型哈希和可复跑步骤以 [P9-I 感知离线验证](P9I_VISION_OFFLINE_VALIDATION.md) 为准。

## 范围与隔离

- 起始 Git 源码：PR #14 head `1ab8641777a5fd574b6c3f90a3103de6d778f15f`，其基线为 `main` `b1cfe617209a06d69000a515af82b9b95add62ef`。
- NUC 正常工作树 `~/lunar-cave-robot` 保持在 `main` `6a48dc52f5d7f5b263845bdec0639f59144a4486`，检查结束仍干净。本轮没有同步、覆盖或安装到该工作树。
- 所有临时源码、SDK 解包、编译与测试都在 NUC 的临时隔离目录完成。没有执行相机或雷达启动节点，没有连接传感器、打开控制器、访问串口或发送执行器指令。
- 新增代码只修复 Orbbec SDK 外置库路径发现、补足 `pose_estimator` 运行依赖、要求 MID-360 启动时显式提供设备广播码，并归档位姿估计默认使用的小型模型点云。没有改动 P7/P8 控制流程或固件。

## 构建结果

NUC 环境为 Ubuntu 22.04.5 / ROS 2 Humble / x86_64。此次使用 NUC 的 ROS Python 环境：OpenCV 4.11.0、Open3D 0.19.0、SciPy 1.15.3、Ultralytics 8.4.164、OpenVINO 2025.4.1。

| 软件包 | 版本 | 实际结果 |
| --- | --- | --- |
| `livox_ros_driver2` | 1.2.4 | 使用已有 Livox SDK2 头文件和库，在隔离目录构建通过；修改后的静态启动文件再次构建通过。SDK 库 SHA-256：`97f0663f71444b1fb6cf40baaa778f917bfc9716d187f01bc3b6b6bbdbcbaebe`。 |
| `orbbec_camera_msgs` | 2.9.3 | 与驱动一起构建通过。 |
| `orbbec_camera` | 2.9.3 | 使用官方 Orbbec SDK v2.9.3 解包目录构建通过；NUC 原有 SDK v1.10.37 与仓库内 v2 头文件不兼容。SDK v2.9.3 官方发行包 SHA-256：`8516081b2201f841b1aa444dd203975e06891d73a2694dd2b3d864254f89ff0d`。 |
| `yolo_node` | 1.0.0 | ROS 接口包构建通过；目标检测节点运行验证未通过，原因见“模型阻塞”。 |
| `pose_estimator` | 1.0.0 | Python 包构建通过；默认 `half_block.pcd` 已从 NUC 旧工作区找到并纳入仓库。Open3D 加载 7,500 点、毫米到米的 0.001 缩放及 FPFH 预处理通过。 |

Orbbec SDK 使用 [官方 v2.9.3 发行版](https://github.com/orbbec/OrbbecSDK_v2/releases/tag/v2.9.3)，只在隔离临时目录解包。构建显式使用 `-DORBBEC_SDK_ROOT=<SDK 根目录>` 和 `-DINSTALL_UDEV_RULES=OFF`，没有向系统安装库或写入 udev 规则。修改后的 CMake 接受 wrapper 原有 `SDK/lib/x64` 布局、官方 `lib/linux_x64` 布局以及当前发行包的 `lib` 布局，并将找到的库显式链接。仓库仍不包含 SDK 二进制。

从两个工作空间根目录分别构建；下面的 build/install/log 均指向独立目录，不能把两个工作空间合并到同一 colcon 源路径。每段命令在单独终端运行，只 source 对应工作空间：

```bash
# 终端 A：tzb_ws 感知 overlay
source /opt/ros/humble/setup.bash
unset BUILDING_PACKAGE
BUILD_ROOT=/tmp/lunar-perception-build
ORBBEC_SDK_ROOT=/path/to/OrbbecSDK_v2.9.3
colcon --log-base "$BUILD_ROOT/tzb-log" build \
  --base-paths tzb_ws/src \
  --packages-up-to yolo_node pose_estimator orbbec_camera \
  --build-base "$BUILD_ROOT/tzb-build" \
  --install-base "$BUILD_ROOT/tzb-install" \
  --cmake-args -DINSTALL_UDEV_RULES=OFF \
    -DORBBEC_SDK_ROOT="$ORBBEC_SDK_ROOT" -DCMAKE_BUILD_TYPE=Release
source "$BUILD_ROOT/tzb-install/setup.bash"
```

```bash
# 终端 B：Nav2_rea_ws Livox overlay，不 source 终端 A 的工作空间
source /opt/ros/humble/setup.bash
unset BUILDING_PACKAGE
BUILD_ROOT=/tmp/lunar-perception-build
colcon --log-base "$BUILD_ROOT/livox-log" build \
  --base-paths Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2 \
  --packages-up-to livox_ros_driver2 \
  --build-base "$BUILD_ROOT/livox-build" \
  --install-base "$BUILD_ROOT/livox-install" \
  --cmake-args -DLIVOX_SDK2_ROOT=/usr/local -DCMAKE_BUILD_TYPE=Release
source "$BUILD_ROOT/livox-install/setup.bash"
```

切换 Orbbec SDK 路径时应强制 CMake 重新配置或使用空的隔离 build 目录。P9-H 的 CMake 修复已清除库查找变量的旧缓存；在故意注入另一个路径的旧缓存后，强制重新配置仍选择了新的 SDK v2.9.3 库。

## 静态启动入口与数据接口

以下入口仅通过 `ros2 launch ... --show-args` 检查，确认包索引、launch 文件和参数解析可用；没有实际启动驱动。

| 子系统 | 已检查入口 | 预期接口 / 状态 |
| --- | --- | --- |
| MID-360 | `livox_ros_driver2 mid360_static.launch.py` | 必须传 `config_file` 与设备上的真实 `cmdline_input_bd_code`。输出 `/livox/lidar` (`sensor_msgs/PointCloud2`) 和 `/livox/imu` (`sensor_msgs/Imu`)。驱动构建不证明网络或传感器数据已通。 |
| Orbbec RGB-D | `pose_estimator camera_switch.launch.py`；内部选择 `orbbec_camera gemini_330_series.launch.py` | 当前 YAML 选择 Gemini 336L 候选型号，640×480@30；输出 RGB `/camera/color/image_raw` (`sensor_msgs/Image`)、RGB 内参 `/camera/color/camera_info` (`sensor_msgs/CameraInfo`)、对齐深度 `/camera/depth/image_rect_raw` (`sensor_msgs/Image`) 及彩色点云 `/camera/depth/color/points` (`sensor_msgs/PointCloud2`)。后两条通过 launch remap 归一化。实物型号、序列号、USB3 链路和固件仍待现场核对。 |
| YOLO | `pose_estimator perception_bringup.launch.py --show-args` | 输入 `/camera/color/image_raw`；预期发布 `/yolo/detections` (`yolo_node/msg/DetectionArray`) 与 `/yolo/annotated_image` (`sensor_msgs/Image`)。节点因当前 `blue_block` 模型运行时错误未能就绪。 |
| 位姿估计 | `pose_estimator pose_estimator.launch.py` | 输入 `/camera/depth/color/points`、`/camera/color/camera_info`、`/yolo/detections`；输出 `/pose_estimator/object_pose` (`geometry_msgs/PoseStamped`) 和可视化 Marker/ROI 点云。模型预处理已测，真实 RGB-D/检测输入同步和 6DoF 估计未测。 |

## MID-360 现场网络待确认项

仓库内 `MID360_config.json` 是历史候选参数，不可直接当成当前实机网络值。该文件含主机候选地址 `192.168.1.50`、雷达候选地址 `192.168.1.119`、LiDAR 端口 56100–56500、主机端口 56101–56501，以及历史外参。启动前由设备负责人确认：

1. NUC 实际接雷达的有线 NIC 名称、链路状态、静态 IP、掩码/前缀、路由和防火墙规则；确认它与相机/校园网接口不会冲突。
2. MID-360 实际 IP、掩码、广播码/设备标识和所用配置文件；确认当前安装固件是否沿用标准端口。
3. 主机各 UDP 收包地址是否绑定到该 NIC；命令、状态、点云、IMU、日志端口是否与设备配置一致。
4. 当前安装姿态与实际外参（roll/pitch/yaw、x/y/z）及 `livox_frame` 到机器人 TF 的关系。

本轮只检查了静态 JSON 与接口，没有探测链路、改网络、打开 UDP 套接字或验证包流。静态 launch 现将真实广播码作为必填参数，移除了原占位码。

## YOLO 模型阻塞与可恢复材料

仓库 `best_openvino_model/metadata.yaml` 对应的根模型 XML 在仓库中没有配套 `best.bin`，所以选根模型目录时不能载入。NUC 旧工作区 `~/tzb_ws/src/best_openvino_model/` 找到匹配的根 `best.bin`（10,430,496 字节，SHA-256 `46a1eaba77c4d319f6d691a1924883c930cfbd3c5edbda15b1d612aaa96092bc`）；用本机 Orbbec 示例 RGB 帧做 OpenVINO 推理可运行，但其 metadata 类别是 `haomu`，不适合作为默认 `blue_block` 检测模型。

当前 `yolo_node` 默认选择 `blue_block/`。NUC 旧工作区中找到了完整的 `yoloe-26l-seg.xml` / `.bin` 配对，XML 与仓库一致；`.bin` 为 111,726,400 字节，SHA-256 `860ba6b59b598c650e81b35f2357578db895f69efe78886adfd60d150c54da72`。同目录还存在 `prompts.txt` 和 `prompt_embeddings.npz`（SHA-256 `cabdd576ee23604f3d01aa94b92f5b77cea702e11fe64d83b64527ab8e8c5d16`）。这些文件没有从旧工作区复制到目标仓库。

尽管 XML/BIN 配对存在，当前节点启动预热及直接图像推理都在 Ultralytics 分割后处理失败：节点的 640×640 预热出现 `13×294`，现有 RGB 示例帧出现 `24×294`；两者都无法与 `32×25600` 的原型矩阵相乘。节点因此没有进入订阅/发布就绪态，也没有产生 ROS 检测消息。根 `best` 模型成功运行只证明 NUC 有可用 OpenVINO runtime，不能证明蓝色方块模型或视觉链有效。仓库标注的权重许可为 AGPL-3.0；由于权重的来源/可分发范围及这份导出能否工作尚未确认，本 PR 不提交模型权重或 prompt 二进制。

在旧工作区的 `best_openvino_model` 路径、项目相关 NUC 工作区和当前仓库中没有找到可复现的训练 checkpoint `.pt`、完整导出命令/运行日志或成对的目标场景数据。要重新导出可工作的 `blue_block` 模型，需要提供：兼容版本的训练 checkpoint（或完整训练输入：数据集 YAML、训练/验证图像与标注、训练参数）、明确的类别/文本提示词或参考图及其 embeddings、Ultralytics 源码/版本、OpenVINO 版本和导出参数，并在导出后用实际 ROS 后处理完成兼容性检查。仅凭现有 XML/BIN、metadata 和 prompt 文件无法重建训练或纠正当前输出结构。

## 离线数据检查与边界

- 在目标仓库、NUC 旧 `tzb_ws` 位姿包和导航工作区没有找到 ROS 2 `.db3`、`.mcap` 或 `.bag` 录制数据。
- 采用 OrbbecSDK_ROS2 同步验证工具随源代码已有的一张 RGB 示例帧，只用于软件推理 smoke 检查；没有复制入仓库。它没有同步深度、CameraInfo、目标真值或可验证目标类别，不能用于位姿估计准确率/端到端验收。
- 已从 NUC 旧工作区归档 `tzb_ws/src/pose_estimator/config/half_block.pcd`。SHA-256 为 `d4fe24118f3bd3d1a3a9db4fb8bef87873b486af8b0d3ec52416dabad5f5027d`。它是 7,500 点目标模型模板，不是场景点云或实测数据；在主机上可完成 Open3D/FPFH 模型预处理。
- **已通过：** ROS 包编译、launch 参数解析、PCD 模型预处理、根 `best` 模型单帧 OpenVINO smoke。
- **未通过/未完成：** 默认 blue-block YOLO 推理（模型输出/分割后处理不兼容）、YOLO ROS 图像到检测消息测试、真实相机采集、深度配准、三维位姿端到端、真实 MID-360 点云/IMU 和 ROS Bag 回放。

## 第一次实验室静态测试步骤

先确认车轮与机械臂驱动电源关闭，实体急停可用，实验场地允许单独给传感器上电。全程不启动整机运动 bringup，不运行导航目标或串口执行器节点。

1. **准备版本和构建依赖：** 等本 PR 审核后，在 NUC 的隔离 overlay 编译；Livox 使用已确认的 SDK2，Orbbec 使用官方 SDK v2.9.3。检查 `ros2 pkg executables` 和 launch `--show-args`，确认执行的是隔离 overlay。
2. **先单测 MID-360 网络：** 设备负责人确认网卡/IP/端口/广播码/外参后，把一份设备专用 JSON 放在用户配置目录，不改仓库示例。启动命令形态：

   ```bash
   ros2 launch livox_ros_driver2 mid360_static.launch.py \
     config_file:=/path/to/confirmed_mid360.json \
     cmdline_input_bd_code:=$MID360_BD_CODE
   ```

   在另一终端检查 `/livox/lidar` 与 `/livox/imu` 的类型、`ros2 topic hz`、点云时间戳和 frame；确认数据稳定后用 `Ctrl-C` 停止驱动。该步骤采集传感器数据，不会启动控制器或电机。

3. **单测相机：** 确认实物型号、USB3 线缆/端口和 SDK 后，以 `camera_switch.yaml` 选定实际型号，运行 `ros2 launch pose_estimator camera_switch.launch.py camera_config_file:=$CAMERA_CONFIG`。另开终端检查真实消息类型并分别观察频率和内容：

   ```bash
   ros2 topic type /camera/color/image_raw
   ros2 topic type /camera/color/camera_info
   ros2 topic type /camera/depth/image_rect_raw
   ros2 topic type /camera/depth/color/points
   ros2 topic hz /camera/color/image_raw
   ros2 topic hz /camera/depth/image_rect_raw
   ros2 topic hz /camera/depth/color/points
   ros2 topic echo --once /camera/color/camera_info
   ```

   同时检查深度图尺寸、有效深度比例和各消息 `frame_id`；以 `Ctrl-C` 停止。仅看到节点名或 Topic 名不算通过。
4. **录制合法回放包：** 在传感器主题检查通过后，从用户可访问且有许可的场景录制数据，至少包括：

   ```bash
   ros2 bag record -o bags/perception_static \
     /camera/color/image_raw /camera/color/camera_info \
     /camera/depth/image_rect_raw /camera/depth/color/points \
     /tf /tf_static
   ```

   MID-360 需要单独记录时再加 `/livox/lidar /livox/imu`，先观察写盘速率与空间占用。将 Bag 保存在仓库外；记录设备型号、配置、标定版本和源码 SHA，避免提交未经批准的实景数据。

5. **离线回放：** 先解决 `blue_block` OpenVINO 输出不兼容问题，准备可运行的授权模型，再使用不带相机启动的感知入口：

   ```bash
   export TZB_OPENVINO_MODEL=/path/to/compatible_blue_block_openvino_model
   ros2 launch pose_estimator perception_bringup.launch.py launch_camera:=false
   ros2 bag play bags/perception_static
   ```

   检查图像、检测数组、对齐点云、CameraInfo 和 `/pose_estimator/object_pose` 的时间戳/TF 对齐，再用已知真值评估位置和朝向误差。当前仓库没有这样的完整 Bag，且模型未通过，所以这组端到端步骤尚未执行。屏幕上的软件停止或 `Ctrl-C` 不是实体急停证明。

## 下一步阻塞

1. 确认 MID-360 实机网络/广播码/外参并确认 NUC 接口可达。
2. 确认 Orbbec 的实际设备型号、序列号和固件；选择 SDK 发布版及持久的隔离 overlay。
3. 获取 `blue_block` 的源 checkpoint 或训练/导出工件，纠正并重新验证 OpenVINO 后处理接口；不要用 `haomu` 模型替代。
4. 采集获准的 RGB、CameraInfo、对齐 RGB-D 点云、TF 和目标真值，并用 Bag 跑通检测到 6DoF 的回放闭环。
5. 相机—机械臂外参、传感器安装 TF 及任何执行器控制仍需单独审核；本轮的构建与静态检查不能替代这些验证。
