# 新成员技术总览

审查记录基线：GitHub main **6a48dc52f5d7f5b263845bdec0639f59144a4486**。本页按该基线的仓库源码说明系统，不把仿真和离线模拟结果说成实机能力。

## 项目由哪些部分组成

系统软件分成两个独立 ROS 2 工作空间和两块 STM32 固件：

- **Nav2_rea_ws/** 负责底盘导航、MID-360 激光雷达、IMU、定位建图、Nav2 和底盘串口桥。
- **tzb_ws/** 负责 RGB-D 相机、视觉检测、目标 3D 位姿、机械臂运动规划、抓放协调和机械臂串口。
- **firmware/chassis/** 是六轮底盘 STM32H723 固件。
- **firmware/arm/** 是机械臂 STM32H723 固件归档。其 XRobot 产品生成输入仍缺失，不能完整构建。
- **hardware/** 是机械/电气资料接收目录；当前没有找到可编辑 CAD、电路图、PCB、BOM 或接线原图。

两个 ROS 2 工作空间独立构建和运行。它们都含有历史命名相同的串口消息包；不要在一个 shell 中叠加两套 install/setup.bash，也不要直接把两个 src 合成一个 colcon 工作区。

## 模块职责、算法和源码入口

| 模块 | 主要职责与实际实现 | 源码入口 | 当前证据 |
| --- | --- | --- | --- |
| 单雷达导航与建图 | Livox 驱动输出 MID-360 数据；IMU 变换和互补滤波；Point-LIO 估计激光惯性里程计/点云地图；地形分析过滤点云并转为 /scan；slam_toolbox 维护在线 2D /map；Nav2 做全局规划、局部控制和行为树恢复 | Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py；localization/point_lio/；navigation/rm_navigation/ | 有源码及 ThinkBook/NUC 构建记录。真实雷达采集、时间同步、外参和实机建图未验收 |
| Nav2 配置 | rm_navigation/params/nav2_params.yaml 配置 DWB 局部控制器和 NavFn 全局规划器。P6/P7 离线任务另用 SmacPlanner2D 和 DWB 的 p6_autonomous_mission/config/nav2_offline.yaml | Nav2_rea_ws/src/navigation/rm_navigation/params/nav2_params.yaml；Nav2_rea_ws/src/p6_autonomous_mission/config/nav2_offline.yaml | 离线完整 12 场景通过记录来自合成场景；Nav2 配置不等于真实洞穴导航验收 |
| 底盘串口桥 | 将 /cmd_vel 转成固定长度 UART 命令；限制无效/越界速度，失联后发送零速帧 | Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp；其 config/serial_params.yaml | PTY 模拟、故障注入和离线场景有通过记录；实体 UART 未接 |
| 任务协调 | ROS 无关的任务模型限制单次触发和非法状态跳转；协调器等待 Nav2 结果、确认停车、检查新鲜目标位姿，并等待抓取结果；失败进入 ABORTED | Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_logic.py、mission_coordinator.py | 单元/静态合同和模拟任务场景通过；抓放成功不等于实物抓取 |
| 相机与检测 | Orbbec ROS 2 驱动包装发布图像、点云和相机信息；YOLO 节点给出检测框和类别 | tzb_ws/src/OrbbecSDK_ROS2/、tzb_ws/src/yolo_node/ | ROS 包源码存在；模型权重与相机采集/标定另行提供，实机图像链路未验收 |
| 目标位姿 | 从检测 ROI 裁剪点云，降采样；FPFH/RANSAC 产生一个初始 6DoF 位姿，再用 trust-region point-to-point ICP 细配准；质量评分对场景点和变换后的完整物体表面计算，并保留上一帧位姿跟踪 | tzb_ws/src/pose_estimator/pose_estimator/pose_estimation_node.py | 这是源码实现；合成/静态检查不证明真实目标姿态精度。依赖相机内参、外参、目标模型及实际点云 |
| 机械臂运动规划 | Pinocchio 读取 URDF 和关节模型；阻尼伪逆 Jacobian IK、多起点解算、关节与笛卡尔轨迹生成；碰撞场景由 hpp-fcl 检查；稀疏笛卡尔路径后由 Ruckig 做时间参数化 | tzb_ws/src/lite_motion_planner/src/kinematics_solver.cpp、planning_node.cpp、collision_scene.cpp | ROS 规划包有 ThinkBook/NUC 构建记录及离线取消竞争检查；关节零位、模型精度、碰撞几何和真实轨迹尚需硬件验证 |
| 机械臂主机通信 | 将 ROS 关节目标与反馈映射为机械臂 USB/串口消息，并用命令租约、反馈新鲜度、取消/恢复门保护主机流程 | tzb_ws/src/serial/driver/、tzb_ws/src/lite_motion_planner/、相关 arm2nav/ 协调包 | PTY 协议和软件模拟有验证；实体控制器 ACK、端口映射和安全失效行为未知 |
| 机械臂固件 | 归档的七轴应用包含关节轨迹、速度/位置纠偏和重力/惯性前馈；XRobot 生成产品入口与模块依赖缺失，完整固件不能编译 | firmware/arm/others/src/、Core/、User/ | 纯 C 状态逻辑主机测试曾通过 PR #8；完整 ARM 固件未构建，且状态机与 RTOS 控制出口存在审查缺口 |
| 六轮底盘固件 | 六个 HM G36 通过两个 CAN 总线接收 MIT 速度命令；差速轮运动学、速度斜坡、IMU yaw-rate PI 修正和航向保持 | firmware/chassis/User/APP/chassis_task.c、vehicle_config.h、hm_motor.c | 固件交叉编译和协议主机测试记录通过；PR #8 状态机测试通过，但需审查 CAN 反馈故障门和 relax 物理行为 |

## 软硬件通信关系

底盘链路是 Nav2 → /cmd_vel → ROS 串口桥 → USART1 14 字节帧 → 底盘 STM32 → FDCAN1/2 → 六台驱动器。当前协议字段和频率见 [STM32 协议说明](STM32_PROTOCOL_SPEC.md)。实体接口、端子与信号电平尚未核线。

机械臂链路是感知/规划 → 主机串口或 USB CDC 消息 → 机械臂 STM32 → FDCAN3 → 七个关节/夹爪驱动器。历史固件 USB CDC 帧为 46 字节下行、35 字节上行。帧兼容不等于实际控制板的固件版本、USB 端口或 CAN 接线已经确认。

STM32 仓库代码的 Git SHA 是源码版本，不代表实体板上的已烧录固件 SHA。每块实体控制板当前版本仍待硬件负责人确认。

## 哪些已经完成，哪些仍阻塞

**有源码或离线验证证据：**

- P0～P8 ROS 2 主线及两套工作空间完整留在 main。
- ThinkBook 上 P7/P8 命名的完整 PTY 离线故障套件在源码 SHA **07a56c3a33882aa2866f1076d8597776e2579bcb** 记录 12/12 通过；同一份记录还保留此前一次真实失败。完整套件结果见 [P9-C 验证记录](P9C_OFFLINE_VALIDATION.md)。
- ThinkBook 和 NUC 两套 ROS 工作空间构建结果、底盘 STM32 交叉编译及主机协议测试有独立记录。
- PR #8 有底盘和机械臂纯状态机主机测试；状态机接入审查及剩余时序风险见 [PR #8 审查](P9E_PR8_INTEGRATION_REVIEW.md)。

**仍未完成或不能据现有证据确认：**

- 机械臂缺失真实 XRobot 产品配置、模块锁定信息和两个生成文件，无法构建完整镜像。恢复材料清单见 [XRobot 构建恢复记录](XROBOT_ARM_BUILD_RECOVERY.md)。
- PR #8 中机械臂控制输出没有同周期安全门；仍需处理 Host 租约过期竞态、模式切换/恢复窗口和逐轴反馈新鲜度。
- 电机 disable/relax 是否制动或保持、机械臂重力负载的失能行为、独立实体急停和实际接线均须负责人确认。软件停止不等于物理急停。
- 真实相机/雷达数据、标定、控制板当前固件 SHA、实机停止距离和抓取流程尚未验收。
- 可编辑 CAD、电气原理图/PCB、BOM 和接线图没有在已搜索的存档中找到。电机厂商手册的再分发条件未确认，因此仍在原始压缩包中。

## 新成员推荐阅读顺序

1. 从本页和根目录 README 了解源码位置。
2. 查看 [系统架构](ARCHITECTURE.md)、[项目资产](PROJECT_ASSETS.md) 和 [软硬件版本矩阵](HARDWARE_SOFTWARE_MATRIX.md)。
3. 按 [P8 开发部署说明](P8_DEPLOYMENT.md) 配置 ROS 2 Humble 与依赖；两个工作区分开构建。
4. 修改 ROS 包前运行对应离线测试；对底盘协议修改运行 **firmware/chassis/tests/host/run.sh**。
5. 固件使用 [固件构建审计](STM32_BUILD_AUDIT.md) 中入口。机械臂要先取回真实 XRobot 产品配置。
6. 任何要连驱动器或电机的操作先审阅 [安全门槛](REAL_ROBOT_SAFETY_GATES.md) 和 [硬件负责人清单](HARDWARE_OWNER_CHECKLIST.md)。本仓库的离线验证不会给实机运动授权。

本页为后续教学 PDF 提供来源索引；教学资料应保留“源码实现”“离线验证”“实机未验证”之间的区分。
