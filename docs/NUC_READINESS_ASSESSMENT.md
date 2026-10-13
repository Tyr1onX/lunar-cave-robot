# NUC 首次现场测试准备评估

检查日期：2026-10-13。通过 Mac 现有 SSH 配置对 NUC 做只读检查，并读取 GitHub `main`。本轮没有启动 ROS 节点、传感器、相机、控制器或执行器；没有访问 STM32 串口、烧录、同步或改写 NUC 工作区。

## 结论

**NUC 主机适合做环境核对和受控的单传感器静态采集准备；目前不具备直接开展整机集成测试的证据。**

NUC 可经 SSH 访问，Ubuntu 22.04.5、ROS 2 Humble、Nav2 和 rosbag2 已安装，磁盘空间充足；ROS 图当前没有活动节点，图中只看到 `/rosout` 和 `/parameter_events`。这表示本次检查没有发现正在发布传感器数据的 ROS 节点，不能据此判断传感器物理连接状态。

第一次到实验室可先做不带执行器的主机/网络核对。MID-360 静态采集需先由现场负责人核实雷达型号、地址、网线和有线网络；Orbbec、YOLO 与三维位姿链目前有构建/模型阻塞。导航、机械臂和整机运动不在当前准备范围。

## 主机与版本快照

| 项目 | 实际检查结果 |
| --- | --- |
| 主机 / 系统 | `tzb-NUC14RVS-B`；Ubuntu 22.04.5 LTS；内核 `6.8.0-40-generic`；x86_64 |
| CPU | Intel Core Ultra 9 185H，22 个逻辑核 |
| 资源 | 内存 14 GiB，总可用约 12 GiB；根分区 916 GiB，可用约 827 GiB（5% 已用）；交换分区 2 GiB，未使用 |
| 温度快照 | `TCPU` 约 44 °C，`x86_pkg_temp` 约 43 °C；仅是检查时 sysfs 读数，不是温度告警阈值或长期监控 |
| 网络接口 | Wi-Fi 接口为 `UP`，有线接口为 `DOWN`；Tailscale 接口存在。未记录设备 IP，未连接或探测传感器网络 |
| SSH | 可连接；主机运行约 2 天 21 小时 |
| NUC 统一工作树 | `~/lunar-cave-robot`，分支 `main`，HEAD `6a48dc52f5d7f5b263845bdec0639f59144a4486`，未提交修改 0 |
| GitHub `main` | `b1cfe617209a06d69000a515af82b9b95add62ef`；NUC 相对远程落后 2 个提交对象（远程可快进到该 SHA） |
| 两版本差异 | 远程新增的是 P9-E 合并及其文档/主机 CI 文件；相对 NUC 快照没有 ROS/固件生产源码差异。此次只读阶段没有同步 |
| 部署版本 | NUC 状态目录没有 `deployed-sha`，所以当前实体部署源码 SHA 未记录 |

两套工作空间仍在统一仓库内分开管理：`tzb_ws/` 与 `Nav2_rea_ws/`。各自有 `p7`、`p9b` 安装叠加层。历史包名冲突要求构建和运行时分别 source 所需工作空间，不能在同一 colcon source 空间合并或无审查地叠加。

## ROS 与构建状态

- ROS 2 Humble 可从 `/opt/ros/humble` 加载。只读图查询得到 **0 个活动 ROS 节点**，Topic 名称为 `/rosout`、`/parameter_events`。本轮没有订阅 Topic，消息频率、时间戳与数据有效性均未采样。
- 当前服务/进程列表中没有发现 Livox、Orbbec、Nav2、YOLO、ROS 或机器人控制进程。没有运行控制器、底盘或机械臂节点。
- NUC 的状态记录有两套工作空间构建通过：Nav2 工作区范围 `p6_autonomous_mission`、`chassis_serial_bridge`，机械臂工作区范围 `lite_motion_planner`、`rm_serial_driver`；记录 SHA 均为 `07a56c3a33882aa2866f1076d8597776e2579bcb`，时间为 2026-10-12 23:46Z / 23:47Z。它们是该 SHA 的历史构建记录，不是 `6a48dc5` 或最新 `b1cfe61` 的本轮重建结果。
- NUC 的完整 P7/P8 状态是旧 `latest-test.yaml` 中的 PASS，SHA `5124a68d398aa84b59a750e69636bf566f298a1c`，记录范围未知。不能用它证明当前版本重新通过 12 场景。
- 统一仓库中有上述两个工作区的 `p7`、`p9b` 构建输出和日志；没有针对最新远程 SHA 的本轮构建。

## 传感器与软件依赖

| 模块 | 当前可确认内容 | 现场测试判断 |
| --- | --- | --- |
| MID-360 | NUC 有 Livox SDK2 共享库。旧 `~/lunar-cave-p8/install/mid360` overlay 能解析出 `livox_ros_driver2_node`；当前统一仓库的 Nav2 overlay 未包含 Livox 包。静态 launch 入口 `livox_ros_driver2 mid360_static.launch.py` 在仓库文档有说明，必须提供经现场核对的 `config_file`。 | **条件具备，尚不可直接启动。** 当前有线网卡为 DOWN；传感器地址、布线、UDP 路由、使用哪个配置及外参尚未核实。旧 P8 二进制不能替代当前 `main` 的构建验证。 |
| Orbbec RGB-D | 相机驱动源码和型号配置候选在 `tzb_ws/src/OrbbecSDK_ROS2/`；P9B 的 `tzb_ws` 构建范围不含 `orbbec_camera`。`pyorbbecsdk` 未安装。仓库配置选择 Gemini 336L，但实体设备标签/型号尚未确认。 | **未就绪。** 需要负责人核对相机型号、构建/SDK 依赖、相机内参和 USB 枚举后再做独立图像/深度采集。 |
| YOLO | NUC 可导入 Ultralytics 8.4.164、OpenVINO 2025.4.1、NumPy 1.26.4、CPU 版 PyTorch 2.1.2；源码包 `yolo_node` 没有在当前安装叠加层中构建。 | **未就绪。** `best_openvino_model/metadata.yaml` 指向 `best.bin` 并注明权重未分发；目录实际只有 `best.xml` 和元数据，缺少配套 `best.bin`。软件依赖可导入不等于模型可加载。 |
| 三维位姿估计 | 源码和 `pose_estimator` 配置存在；配置订阅 `/camera/depth/color/points`、`/camera/color/camera_info`、`/yolo/detections`，输出 `/pose_estimator/object_pose`。该包未在当前 NUC 工作区安装。 | **未就绪。** 需要相机驱动、权重文件、检测消息包、点云、标定和相机到机械臂基座外参。 |
| Nav2 / 定位 / TF | ROS Humble 中安装了 Nav2 与 `slam_toolbox`；NUC 有 `p6_autonomous_mission` 和底盘桥工作区构建记录；源码提供 mapping/navigation launch。 | **仅源码/构建准备。** 没有活动里程计、定位、雷达或 TF 数据；不能执行真实导航或把离线 TF 当成实机 TF。 |
| rosbag2 | `ros-humble-rosbag2` 及相关包已安装；根分区可用约 827 GiB。 | **可以准备记录流程，尚未录包。** 需先确认单传感器 Topic 稳定、时间戳及存储目录；本轮没有创建包。 |
| STM32 / 电机 | 本轮未打开端口、未查询固件、未连接控制板，且没有部署 SHA。 | **未知/未核验。** 不允许据此开始运动测试。 |

## 首次现场测试建议与通过判据

### 0. 到场前静态条件

1. 从设备铭牌确认 MID-360 和 Orbbec 具体型号、序列号与固件版本；由网络负责人确认 NUC 有线网口、传感器地址/子网、UDP 方向、端口与路由。**不要从历史 JSON 推断当前地址或外参。**
2. 现场负责人确认执行器电源隔离、底盘/机械臂不会被本次流程启动。全程不运行整机 bringup，不打开 STM32 串口。
3. 不要在本轮直接使用 P8 旧工作区作为当前交付；先确认旧 Livox overlay 与所选配置的来源和兼容性。当前主线驱动应在受控的软件工作区按 [MID-360 静态验收说明](MID360_STATIC_ACCEPTANCE.md) 单独构建。

### 1. MID-360 原始点云与 IMU（满足网络前置条件后）

- 入口：`livox_ros_driver2 mid360_static.launch.py config_file:=<现场确认并批准的配置.json>`。仅启动雷达驱动，不启动 RViz、SLAM、Nav2、底盘或机械臂。当前旧 overlay 在 `~/lunar-cave-p8/install/mid360`；是否使用它必须先核对代码版本。
- 观察：`/livox/lidar`（`sensor_msgs/msg/PointCloud2`）和 `/livox/imu`（`sensor_msgs/msg/Imu`）。
- 通过：两 Topic 持续有数据、频率非零且合理、时间戳递增，点云空间形状和 IMU 数值合理；短时 ROS Bag 可列出并回放。
- 保存：命令、日志、所用配置的脱敏 ID、开始/结束时间、频率摘要及短 Bag。不要提交设备 IP 或私有传感器配置。
- 退出：只在驱动终端 Ctrl-C；确认节点退出。此退出动作不是传感器断电，也不是实体急停。
- 完整命令与具体观察步骤见 [MID-360 静态验收说明](MID360_STATIC_ACCEPTANCE.md)。

### 2. RGB-D / 视觉 / 位姿（当前有阻塞）

- 先确认实际 Orbbec 型号与仓库 `camera_switch.yaml` 配置是否匹配，完成 `orbbec_camera` 的独立依赖构建；相机只采集图像和深度，不启动机械臂通信。
- 观察主题按批准的型号配置确认；仓库当前统一相机配置候选为 `/camera/color/image_raw`、`/camera/depth/image_rect_raw`、`/camera/depth/color/points`。核对 `/camera/color/camera_info`、图像尺寸/频率、深度有效范围和点云坐标。
- YOLO 入口源码为 `yolo_node/launch/yolo_node.launch.py`；位姿估计入口源码为 `pose_estimator/launch/perception_bringup.launch.py`。两包尚未在 NUC 当前 overlay 构建，且 YOLO 缺 `best.bin`，因此当前不可执行。
- YOLO 通过判据：模型成功加载，输入帧时间戳连续，输出检测类别/置信度/框与可视化图同步。位姿通过判据：检测、点云与 CameraInfo 同时间域；目标姿态在已验证坐标系中稳定，误差/质量指标记录在案。
- 完成离线/静态采集前不连接机械臂驱动、不发布运动目标。异常时只停止对应相机/视觉 ROS 进程；不要触碰执行器。

### 3. TF 与导航

TF 需在有真实传感器/里程计输入时核对 `map → odom → base_link → livox_frame/camera_frame` 的方向、时间戳和唯一发布者。Nav2 只有在地图/定位/里程计、底盘模型、TF 与速度桥接逐项确认后才可进入独立的静态规划测试；本轮不启动导航。

## 结论分级

- **现在可做：** SSH 与主机资源只读检查；现场目视确认传感器型号/接线；核对网络方案；阅读/审查日志和现有构建记录。
- **满足网络与配置确认后可做：** 单独 MID-360 静态点云/IMU 采集与 rosbag2 记录。当前有线接口未联通，因此需先完成现场网络前置。
- **当前不建议：** Orbbec/YOLO/位姿实测（构建包、SDK/模型权重不足）；实机 TF、定位、Nav2、抓取或任何执行器运动。
- **尚未验证：** 传感器是否物理连接、消息是否真实发布、相机标定、MID-360 地址/外参、ROS Bag 实录、实体 MCU 反馈与固件版本、安全停止/制动/防坠。

## 后续人工需提供/确认

1. MID-360 和 Orbbec 的铭牌型号、接线图或现场确认记录；有线网络配置与传感器地址由设备负责人保管，不上传仓库。
2. Orbbec 相机具体型号及当前标定文件；摄像头到机械臂基座的标定责任人/结果。
3. 具有授权的 YOLO `best.bin` 配套文件或模型重新导出流程，并明确它是否可放入团队私有仓库。
4. 部署负责人确认 NUC 的统一源码目录可以在后续独立窗口快进至 GitHub `main`；本次未同步。
5. 硬件负责人确认控制器固件版本和安全门槛后，再单独规划任何执行器联调。
