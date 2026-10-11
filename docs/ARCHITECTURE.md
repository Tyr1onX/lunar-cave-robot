# 机器人系统架构与成员接手指南

> 仓库：`Tyr1onX/lunar-cave-robot`（内部 Private）｜统一分支基于 P3～P4b 离线归档，并补入 P5a 审计与模型测试；P4c 来源待核实。
> **文档描述的是源码中的设计与已验证边界，不等于整机实机验收。** 修改硬件、串口、运动控制或启动文件前，必须经过设备负责人核对。

## 1. 一分钟认识项目

该机器人由**底盘导航**、**RGB-D 视觉感知**、**机械臂运动规划/抓放**及可选的**导航—机械臂协调**构成，使用 Ubuntu 22.04 / ROS 2 Humble 和 colcon。

仓库故意保留两个**独立 ROS 2 工作空间**：

| 路径 | 核心职责 | 典型硬件与运行对象 |
| --- | --- | --- |
| [`tzb_ws/`](../tzb_ws/) | RGB-D 相机、YOLO、三维位姿估计、机械臂规划、抓放、机械臂串口及状态协调 | Orbbec Gemini 336L（配置可切换 D435i）、六轴机械臂/夹爪、STM32 |
| [`Nav2_rea_ws/`](../Nav2_rea_ws/) | 激光雷达驱动、IMU/点云预处理、里程计与建图、地形分析、Nav2 导航、底盘控制 | Livox MID-360、IMU、底盘串口 |
| [`p1_offline/`](../p1_offline/) | P1 软件验证探针 | 无实机硬件 |

**重要：** 两个工作空间都定义了 `rm_serial_driver` / `rm_serial_msgs`。不可在同一个终端直接叠加两套 `install/setup.bash`；不能假定两个驱动可同时在一个 ROS 图中无冲突运行。真实跨工作空间部署方式、节点命名与串口归属，仍须由系统集成人员确认。

### 系统功能链路（设计视图）

```text
【导航侧 Nav2_rea_ws】                    【机械臂侧 tzb_ws】
MID-360 / IMU                              RGB-D 相机
     │                                          │
Livox 驱动 / IMU 预处理                         ├─ RGB → YOLO 检测
     │                                          └─ 点云/相机内参 ─┐
Point-LIO 里程计/建图                                         ↓
     │                                                 pose_estimator
地形分析 → 虚拟 LaserScan → 二维地图(/map)                    │
     │                                                 目标 6DoF 位姿
Nav2 规划/控制                                                ↓
     │                                                 tzb_catch + 规划器
速度坐标变换 / 底盘串口                                        │
     │                                                 机械臂串口 → 机械臂/夹爪
     └──────────── 可选协调接口 arm2nav / nav2arm ────────────┘
                   【目前不能视为已验收的整机闭环】
```

图仅展示主要逻辑关系。具体哪一个进程向哪个控制器下发数据、哪套驱动实际接管硬件，必须与所选启动方案和 ROS Topic 实测结果对照。

## 2. 目录、模块与负责人应阅读的源码

### 机械臂 / 视觉工作空间 `tzb_ws/src/`

| 模块/ROS 包 | 作用 | 首选源码入口 |
| --- | --- | --- |
| `OrbbecSDK_ROS2/orbbec_camera`、`orbbec_camera_msgs`、`orbbec_description` | Orbbec 相机驱动、消息与模型；SDK 二进制文件未随仓库提供 | [相机选择配置](../tzb_ws/src/pose_estimator/config/camera_switch.yaml)、[相机切换启动](../tzb_ws/src/pose_estimator/launch/camera_switch.launch.py) |
| `yolo_node` | 读取 RGB、输出目标检测框；运行需要单独授权的模型及 Python 推理依赖 | [启动文件](../tzb_ws/src/yolo_node/launch/yolo_node.launch.py) |
| `pose_estimator` | 结合检测框、点云和相机参数估计目标位姿，提供配准可视化 | [联合感知启动](../tzb_ws/src/pose_estimator/launch/perception_bringup.launch.py)、[参数](../tzb_ws/src/pose_estimator/config/params.yaml) |
| `tzb_description`、`engineer_0520_description` | 机械臂 URDF/网格及模型变体 | [模型选择入口](../tzb_ws/src/lite_motion_planner/launch/tzb_catch.launch.py) |
| `lite_motion_planner`、`lite_motion_msgs` | 机械臂 IK/路径与碰撞规划、`planning_node`、`tzb_catch` 抓放协调、自定义消息 | [抓放启动](../tzb_ws/src/lite_motion_planner/launch/tzb_catch.launch.py)、[规划节点启动](../tzb_ws/src/lite_motion_planner/launch/planning_node.launch.py) |
| `serial/driver`、`serial/msgs` | 与 STM32 的机械臂命令、关节反馈和夹爪通信 | [串口参数](../tzb_ws/src/serial/driver/config/serial_params.yaml) |
| `arm2nav`、`arm2nav_msgs` | 可选的目标停车建议、导航/机械臂协作状态桥 | [节点实现](../tzb_ws/src/arm2nav/arm2nav/node.py)、[消息定义](../tzb_ws/src/arm2nav_msgs/msg/Arm2Nav.msg) |
| `nav2arm` | 旧版导航完成触发抓放；依赖当前未提供的 `tzb_state_test` 接口 | [协调代码](../tzb_ws/src/nav2arm/nav2arm/coordinator.py)、[参数](../tzb_ws/src/nav2arm/config/nav2arm.yaml) |

机械臂更完整的默认 Topic 列表见 [`tzb_ws/README.md`](../tzb_ws/README.md)。两个模型 `tzb_description` 与 `engineer_0520_description` 的几何和配置并不自动互换，切换前核对 URDF、SRDF、运动学和串口关节序。

### 导航工作空间 `Nav2_rea_ws/src/`

| 目录/ROS 包 | 作用 | 首选源码入口 |
| --- | --- | --- |
| `drivers/livox_ros_driver2` | Livox MID-360 消息驱动；Livox SDK 不在本仓库 | [单雷达建图启动](../Nav2_rea_ws/src/bringup/launch/mapping_single_lidar.launch.py) |
| `drivers/serial/driver`、`drivers/serial/msgs` | 底盘串口控制；与机械臂工作空间的同名 ROS 包**不是可直接混用的同一配置** | [驱动实现](../Nav2_rea_ws/src/drivers/serial/driver/src/node.cpp) |
| `preprocess/imu_tools` | IMU 坐标转换、互补滤波 | [IMU 启动](../Nav2_rea_ws/src/preprocess/imu_tools/launch/imu_transform.launch.py) |
| `preprocess/pointcloud_tools` | 点云融合、转换为虚拟 LaserScan、PCD 工具 | [单雷达导航总启动](../Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py) |
| `localization/point_lio` | 主要单雷达方案中的激光惯性里程计、点云建图 | [配置](../Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml) |
| `localization/fast_lio`、`localization/dlio` | 保留的替代/历史定位方案，含第三方源码；不要在同一 TF 链上无计划并行启用 | [定位源码目录](../Nav2_rea_ws/src/localization/) |
| `localization/alignment` | 旧地图与当前局部坐标对齐；含旧设备绝对路径 | [配置](../Nav2_rea_ws/src/localization/alignment/config/config.yaml) |
| `localization/pcd_processor` | 点云到地图、滤波等离线/辅助处理 | [代码目录](../Nav2_rea_ws/src/localization/pcd_processor/) |
| `navigation/terrain_analysis` | 点云地形、障碍物分析，提供导航代价信息 | [参数](../Nav2_rea_ws/src/navigation/terrain_analysis/config/terrain_analysis_plus.yaml) |
| `navigation/rm_navigation`、`behavior_ext_plugins`、`fast_layer`、`dog_map` | Nav2 启动、规划/控制配置、行为树与自定义层/插件 | [Nav2 入口](../Nav2_rea_ws/src/navigation/rm_navigation/launch/bringup_rm_navigation.py) |
| `navigation/fake_vel_transform` | 将导航速度变换为底盘使用的速度坐标系 | [说明](../Nav2_rea_ws/src/navigation/fake_vel_transform/README.md)、[实现](../Nav2_rea_ws/src/navigation/fake_vel_transform/src/fake_vel_transform.cpp) |
| `interfaces` | 导航侧自定义消息接口 | [消息目录](../Nav2_rea_ws/src/interfaces/msg/) |
| `bringup` | 组合以上模块的不同运行场景、Nav2 参数和 RViz 配置 | [启动目录](../Nav2_rea_ws/src/bringup/launch/) |

## 3. 主要数据流与 ROS 接口

下表是**仓库默认配置/源码声明**，不是实机 `ros2 topic list` 的现场记录。不同 launch、参数覆盖、命名空间会使 Topic 改变。

| 发布方 → 消费方 | 常用 Topic / 消息 | 含义 |
| --- | --- | --- |
| RGB-D 相机 → YOLO | `/camera/color/image_raw` · `sensor_msgs/Image` | RGB 视频输入 |
| RGB-D 相机 → 位姿估计 | `/camera/depth/color/points` · `sensor_msgs/PointCloud2`；`/camera/color/camera_info` · `sensor_msgs/CameraInfo` | 对齐点云与内参 |
| YOLO → 位姿估计/可选桥 | `/yolo/detections` · `yolo_node/DetectionArray` | 二维目标框、类别与置信度 |
| 位姿估计 → 抓取 | `/pose_estimator/object_pose` · `geometry_msgs/PoseStamped` | 目标相对于消息中 `header.frame_id` 的 6DoF 位姿 |
| 抓取协调 → 机械臂串口 | `/tzb_catch/arm_joint_command` · `sensor_msgs/JointState` | 关节目标角；实际单位/顺序以参数核实 |
| 抓取协调 → 夹爪串口 | `/tzb_catch/gripper_command` · `std_msgs/Int8` | `1` 闭合、`-1` 张开 |
| 机械臂串口 → 抓取协调 | `/serial/arm_joint_state` · `sensor_msgs/JointState` | 实际关节反馈 |
| 任务触发 → 抓取协调 | `/tzb_catch/start` 或 `/navigation_goal_status` · `std_msgs/Int8` | 不同启动模式的触发输入 |
| 抓取协调 → 协调方 | `/tzb_catch/result` · `std_msgs/Int8` | `1` 成功、`-1` 失败 |
| 可选 `arm2nav` ↔ 外部导航方 | `/arm2nav`、`/arm2nav/command`、`/arm2nav/from_arm` · `arm2nav_msgs/Arm2Nav` | 目标停车建议、触发、返航等标志；需确认接收端是否集成 |
| 可选旧 `nav2arm` ↔ 抓取 | `tzb_state_test` 状态 + `/tzb_catch/start` / `/tzb_catch/result` | 依赖缺失的 `tzb_state_test/msg/TzbState` |
| LiDAR / IMU → Point-LIO | `/livox/lidar`、`/livox/imu`（经 IMU 预处理可能使用 `/imu/data`） | 雷达与惯性测量；以驱动配置和参数重映射为准 |
| 地形点云 → 虚拟激光 → SLAM | `/terrain_map` → `/scan` → `/map` | 单 MID-360 在线地图方案 |
| Nav2 → 底盘 | `cmd_vel_nav` → 速度平滑 → `cmd_vel` → `fake_vel_transform` → `/cmd_vel_chassis` | 代码中的预期速度控制链，串口驱动订阅 `/cmd_vel_chassis`；必须现场检查实际连接 |

### 机械臂抓放的工作顺序

1. 感知链获取 RGB、深度/点云和目标检测；`pose_estimator` 使用点云配准得到目标 `PoseStamped`。
2. `tzb_catch` 使用机械臂模型和 `lite_motion_planner` 规划接近、抓取、放置及返回等动作。
3. 实机配置可向 STM32 发送关节与夹爪命令，关节反馈回流至抓取协调节点；离线配置将这些通道重映射到 `/p2_offline/*`，只使用合成反馈。
4. 抓取结果通过 `/tzb_catch/result` 通知上层；**这个结果不等同于有实物检测的抓取成功验收**。

### 导航的单 MID-360 在线地图方案

[`mapping_navigation_single_lidar.launch.py`](../Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py) 组合了 MID-360、IMU 预处理、Point-LIO、地形分析、`pointcloud_to_laserscan`、`slam_toolbox`、Nav2 和 `fake_vel_transform`。其设计明确规定：

- **定位来源：** Point-LIO 输出 `odom_init → base_frame`。
- **地图来源：** `slam_toolbox` 根据 `/scan` 输出二维占据地图 `/map`，**不作为第二套里程计**；`transform_publish_period: 0.0` 关闭它的 TF 发布。
- **全局 TF：** 本启动方案用**唯一**的静态 `map → odom_init` 单位变换；不要额外同时启动 `alignment` 或第二个同名 TF 发布者。
- **雷达坐标：** `base_frame → livox_frame` 在该启动文件中是逻辑单位变换，注释称实体安装外参由 `MID360_config.json` 处理；须实机验证且不得重复套用外参。

注意：`base_frame` 是导航侧常见底盘坐标，`base_link`、`camera_link`、`tcp_link` 是机械臂侧常见坐标，**不能仅凭名称推断它们之间的实际 TF**。手眼标定及两工作空间间的 TF 连通性尚未验收。

## 4. 启动文件：不同场景不要混用

下表是**寻找入口的索引**，不是建议直接在无人值守实机上执行的命令。

| 使用场景 | 源码入口 | 注意 |
| --- | --- | --- |
| **安全、已验证的机械臂软件测试** | `tzb_ws/tools/p2_offline/run.sh` → `lite_motion_planner/launch/p2_offline.launch.py` | 只启动本机隔离 ROS domain 187、合成关节反馈与 `/p2_offline/*`；不连接真实硬件 |
| 机械臂实机组合入口（未验收） | `tzb_ws/src/lite_motion_planner/launch/bring_launch.launch.py` | 按顺序组合相机、YOLO、位姿估计和抓放；默认启用串口驱动，**勿当作离线测试命令** |
| 视觉链单独启动 | `tzb_ws/src/pose_estimator/launch/perception_bringup.launch.py` | 相机可切换；推理模型、相机驱动 SDK 均需由授权途径另行配置 |
| 单 MID-360 建图 | `Nav2_rea_ws/src/bringup/launch/mapping_single_lidar.launch.py` | 组合 LiDAR、IMU、Point-LIO 和 RViz；需要真实 SDK/硬件 |
| 单 MID-360 在线地图 + Nav2 | `Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py` | 包含在线地图与导航链路；**未通过本仓库的完整实机验收** |
| 旧的导航/多雷达方案 | `Nav2_rea_ws/src/bringup/launch/bringup.launch.py`、`mapping.launch.py` | 保留参考；存在已排除的原有地图和旧参数路径，不保证开箱可启动 |

在机械臂总入口中，`use_nav2arm` 和 `use_arm2nav` **都默认为 false**；旧版 `nav2arm` 需要缺失的 `tzb_state_test` 消息，`arm2nav` 需要有相应逻辑的导航侧消费者。**两种协议尚未在本仓库内确认完整互通，不应同时开启后直接控制设备。**

## 5. 已验证范围、缺口和风险

| 阶段/能力 | 已知证据 | 尚不能下的结论 |
| --- | --- | --- |
| P0 原始 ThinkBook 恢复 | 原工作目录曾完成机械臂 14 包、导航 18 包构建 | 不代表任何新电脑和新仓库直接全量构建通过 |
| 本仓库离线子集构建 | 整理暂存区：机械臂 13/13 包、导航 14/14 包成功；跳过缺失 SDK 相关包 | 不代表 Orbbec、Livox 和依赖其驱动的定位包已构建 |
| P1 | 软件规划、碰撞与合成反馈检查 | 不等于真实机械臂轨迹精度或安全性验证 |
| P2 | 本机 ROS domain 187、`ROS_LOCALHOST_ONLY=1` 下完成合成抓放/放置/返回流程 | 不等于真实相机识别、夹爪抓取、实体碰撞或下位机闭环验证 |
| P2b | RGB-D bag 的检查、录制和回放工具已留存 | 尚无获批真实 bag、手眼标定、TF 证据或完整实机抓取验收 |
| P3 / P3b | [离线复验记录](P3_P4B_VALIDATION.md)：时间戳坐标转换测试、动态 URDF/TF 与合成视觉抓放通过 | 光学 TF 是模拟夹具；未验证真实相机外参或实体抓取 |
| P4a / P4b | [离线复验记录](P3_P4B_VALIDATION.md)：通信帧解析、六关节映射、URDF 限位与 FK 测试通过 | 未验证 STM32 实机协议、零位、方向或运动安全 |
| GitHub 交付 | Private 仓库源码、配置和文档已上传 | 全新克隆环境下的构建与测试仍待复核 |

以下资产**不在 Git 中**：受授权限制的 OpenVINO/YOLO 权重（包括 `.bin`）、Orbbec/Livox SDK 二进制库、真实 bag、PCD/PGM 地图、校准结果、虚拟环境、密钥和运行日志。不得为追求一键运行而直接提交它们；应先明确所有权、来源和分发范围。

另外已有**至少 24 个源文件**包含旧主机绝对路径；`localization/alignment/config/config.yaml` 是直观例子。机械臂原始 YAML、老导航 launch、地图路径以及外部 `tzb_state_test` 依赖需要逐项清理或重新配置。第三方 GPL/Apache/MIT/BSD/AGPL 等信息见 [`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md)，发布边界见 [`RELEASE_AUDIT.md`](RELEASE_AUDIT.md)。

## 6. 新成员如何接手（推荐顺序）

1. **先认领模块与接口。** 确认你负责导航、感知、规划、底盘/机械臂串口或系统集成中的哪一块，以及该模块对接人是谁。**现有仓库没有可信的模块负责人名单，不能从提交者推断负责人。**
2. **读本页和对应 launch / YAML。** 理清输入、输出 Topic、TF、消息类型、硬件端口及是否依赖未上传资产。
3. **只用安全离线入口复现。** 先遵循仓库 [README 的两工作空间离线构建命令](../README.md#offline-builds-no-device-access)，再验证 [P2 离线流程](../tzb_ws/tools/p2_offline/README.md)。不要为了调试使用默认实机总启动。
4. **改动前记录可验证证据。** 至少写清运行环境、命令、预期与实际结果、相关 Topic/TF 和失败日志位置；日志、录制数据留在 Git 之外。
5. **涉及实机必须走负责人确认。** 包括串口设备名、关节限位/速度、底盘运动、雷达外参、手眼标定、导航安全区域、急停和人员监护，禁止未经审批的直接启动。
6. **补齐模块交接卡片。** 在后续协作中逐模块填写：负责人/备份人、入口文件、依赖设备/SDK、输入输出、测试命令、当前阻塞、配置与数据存放位置（只记录路径和访问方式，**不要提交密钥和私有文件**）。

### 当前最有价值的待办

- **P0：** 明确「导航—机械臂」采用哪一套协调协议、哪些节点负责消费 `/arm2nav` 与导航完成事件；补齐 `tzb_state_test` 的来源或去除旧依赖。
- **P0：** 由设备负责人核对实际硬件连接、两个 `rm_serial_driver` 的部署归属、TF 根节点和设备接线图。
- **P1：** 消除仍被实机 launch 使用的旧绝对路径，建立可审计的配置示例与依赖安装方式。
- **P1：** 在**全新克隆**中复现 README 所列离线构建及抓放测试；保存验证结论但不提交运行日志。
- **P2：** 获得授权的真实 RGB-D 记录与标定资料，按 [P2b 文档](../tzb_ws/tools/p2b/README.md) 逐阶段验收。

以上优先级是交接建议，不代表这些事项已有指定负责人或已批准开展实机实验。


## 7. 两套 STM32 控制器与 ROS 2 接口边界

两套固件分别运行在独立 STM32 控制器上。ROS 2 节点通过各自的主机链路发送命令；CAN 是控制器内部连接电机的总线，不是现有 ROS 2 的直接接口。两套 ROS 工作空间也不能在同一个终端叠加，见本指南前文。

### 机械臂控制器

输入工程 lunar_rover.zip 的静态源码显示目标芯片为 STM32H723。控制循环按 2 ms 周期更新六个机械臂关节参考、动力学前馈和电机控制；第七路按夹爪处理。FDCAN3 连接关节电机，FDCAN3 配置为 CAN FD/BRS：时序参数对应标称速率 1 Mbps、数据相位 2 Mbps。variables.cpp 的注释称 classic CAN，但构造参数启用了 FD 和 BRS；需要对照电机侧配置确认，不能仅按注释上实机。电机 CAN 命令与反馈由控制器固件处理，ROS 2 不直接访问 CAN。

主机协议由 USB OTG HS CDC 提供，帧布局如下：

| 方向 | 长度 | 字段 |
| --- | ---: | --- |
| ROS 主机到控制器 | 46 字节 | A5；3 个 float32 底盘速度占位字段；4A B5；使能；4 字节预留；6 个 float32 连续关节目标弧度；有符号夹爪命令；4B |
| 控制器到 ROS 主机 | 35 字节 | 5A 01 4A B5；反馈就绪标记；uint32 帧计数；6 个 float32 连续关节反馈弧度；夹爪状态；4B |

下行的底盘速度字段在机械臂固件中被解析后丢弃。六个关节角使用编码器连续弧度；+1 表示夹紧，-1 表示张开，0 表示保持。固件把偏置与坐标换算留给主机侧。

UART5 配置为 100000 baud、9 位字长、偶校验、单停止位、仅接收，用作遥控/SBUS 输入；它不是 ROS 2 机械臂命令口。当前 tzb_ws ROS 2 驱动配置的是串口式 46 字节桥和 9600 baud。源码确认的固件主机链路是 USB CDC，UART5 则为遥控接收，因此还需核实 USB CDC 端点、主机设备映射、实际线缆以及 ROS 包与固件帧字段的一致性。代码中的 4 字节预留区在固件中被忽略，不能只凭相同的 46 字节长度推断直接互通。

### 六轮底盘控制器

P5a 源码审计显示底盘使用另一块 STM32H723 控制器，FreeRTOS 控制周期为 5 ms（200 Hz）。USART1（PA9/PA10）以 115200 baud、8N1、DMA 接收主机速度命令；帧长 14 字节，依次是 A5、VX/VY/VZ 三个小端 float32、4A。协议没有 CRC、序号或时间戳。固件执行 CH8 模式门控、命令限幅/斜坡、200 ms 串口超时停车及六轮运动学。FDCAN1/2 控制六个 HM G36 电机；BMI088 用于底盘姿态反馈。固件的 FDCAN3 虽初始化，但不在当前六轮电机映射中。

Nav2 工作空间已有 ROS 2 发送端，订阅 /cmd_vel_chassis 并编码为相同的 14 字节形状，但当前配置仍为 9600 baud；标准导航 bringup 没有启动该驱动。上游 fake_vel_transform 会覆盖 angular.z，MPPI 允许非零 linear.y，而底盘没有横移自由度。串口节点定时重发缓存命令，主机侧没有新鲜度保护；ROS 串口接收端也没有发布轮速里程计、底盘 IMU 或控制器故障状态。这些问题使 ROS 2 到底盘的接口仍处于阻塞状态，详见 docs/p5a/chassis-source-audit.md。

### 联调边界

机械臂和底盘不能共享一个“46 字节通用串口协议”：机械臂固件主机端点是 USB CDC，底盘是 115200/8N1 UART；两者的控制职责、帧解释和 CAN 电机总线都不同。ROS 2 当前对机械臂的串口参数和固件 USB CDC 实现尚需端口核对，底盘 Nav2 桥还需修正波特率、启动链、运动模型、命令新鲜度和遥测接口。当前只完成源码与离线模型检查，没有实机联调或烧录。
