# Lunar Robot Console：机器人系统拓扑审计

## 范围与证据分类

拓扑描述来自 `b379b198` 中的源码、Launch、配置和 STM32 协议；P6 协调器依据当前 main 中的源码和离线运行验收单独补充。它是静态结构模型，与 Agent 的实时状态分开保存。

| 标记 | 含义 |
| --- | --- |
| 源码 / Launch 连线 | 源码明确声明了候选输入、输出、Topic remap、TF 或串口协议；不代表该连接当前运行。 |
| Topic 名称已观察 | 当前新鲜、Domain 可用的 ROS Graph 列出 Topic 名称；不提供发布者 / 订阅者归属、数据内容或数据是否成功送达。 |
| 历史离线软件验证 | 仓库记录的 PTY、离线或合成测试；不代表当前 NUC 实机运行。 |
| 接口待核实 | 源码 / 离线文档存在接口概念，但 main 中没有可确认的运行连接，或 Agent 没有对应只读数据。 |

运行状态只复用现有 Agent 快照、工作区进程扫描、ROS Graph、Vision 读取和健康事件。Agent 断连、ROS Domain 无效 / 不匹配、Graph 过期或缺少时间证据时，下游节点与连线显示未知。历史工作区快照仅作为历史证据展示，不参加实时判定。未配置运行预期时，“未发现关联进程”不升级为故障。

## 拓扑能力矩阵

| 子系统 | 源码与接口依据 | 目前能显示的实时证据 | 未确认部分 |
| --- | --- | --- | --- |
| 监测入口 | NUC 只读 Agent `/health`、`/v1/snapshot`；ROS observer | Agent 连接、Domain、Graph 新鲜度、节点 / Topic 名称 | Agent 不读取任意 Topic 消息，不提供 ROS publisher/subscriber endpoint 关联 |
| 视觉感知 | `tzb_ws/src/pose_estimator/launch/perception_bringup.launch.py`、camera switch、YOLO、pose estimator、Arm2Nav 源码 | Vision 接口提供的 RGB / 深度帧时间与流状态；现有模块健康事件 | RGB / 深度同名 Topic 不能确认具体工作区相机来源；YOLO 检测受置信度门控，无输出不等于故障；Arm2Nav 到任务协调器未接通 |
| 雷达定位 | `mapping_navigation_single_lidar.launch.py` 启动单个 MID-360、IMU 变换 / 互补滤波、Point-LIO、地形分析、点云转 LaserScan 和 SLAM Toolbox。IMU 路径为 `/livox/imu → /livox/imu_aft_tf → /imu/data → Point-LIO`；点云路径为 Point-LIO `cloud_registered` / `odometry` → `terrain_analysis` `/terrain_map` → 虚拟 `/scan` → `slam_toolbox` `/map` | `livox/lidar`、`livox/imu`、`imu/data`、`cloud_registered`、`odometry`、`terrain_map`、`scan`、`map` 等 Topic 名称；Agent 的聚合雷达 / 定位工作区进程状态 | Agent 的 `lidar` 模块合并多个包，不能确认 MID-360 实体状态或 Point-LIO、IMU、SLAM 独立节点健康；TF frame pair、Topic publisher/subscriber 和消息内容未被读取 |
| 地图与导航 | 同一单雷达 Launch 用 `slam_toolbox_single_lidar.yaml` 订阅 `/scan` 并发布在线 `/map`。该 Launch 将 `bringup/config/nav2_params.yaml` 传给 Nav2：全局静态图层消费 `/map`，STVL 障碍点云层消费 `/terrain_map`；Nav2 velocity smoother 将 `cmd_vel_smoothed` remap 到 `/cmd_vel` | `/map`、`/scan`、`/terrain_map`、`/tf`、`/cmd_vel` 名称；工作区导航进程扫描 | 当前 Agent 不提供 Nav2 Action 服务器、生命周期状态、TF frame tree、地图内容或消费端关联；所选 Launch 的 Nav2 障碍输入为 `/terrain_map`，`/scan` 由 SLAM Toolbox 使用 |
| 任务协调 | `Nav2_rea_ws/src/p6_autonomous_mission/` 中的协调器源码与 Launch 已在 main；P6 隔离启动通过离线 ROS 2/Nav2 运行验收 | 当前 Agent 不采集 ROS Action endpoint，也没有与 P6 任务模块绑定的独立工作区进程记录 | 离线启动包含 cave simulator；实体主启动流程是否集成、Arm2Nav / PoseEstimator 是否接入、是否连接真实执行器均待确认 |
| 机械臂 | `tzb_ws` 中 `lite_motion_planner`、`rm_serial_driver`、串口协议与 `/serial/arm_joint_state`、`/serial/arm_safety/status` 源码 | Graph Topic 名称、合并的 Agent `arm` 工作区进程证据 | Agent 模块把规划器和串口驱动合并，不能按进程拆分；当前 Graph 不读取反馈消息值、所有关节状态、硬件急停或实体执行结果 |
| 底盘 | `Nav2_rea_ws` 中 `chassis_serial_bridge` 订阅 `/cmd_vel`，发布 `/chassis/serial_link_status`；USART1 使用 115200、14 字节 `A5 + 3×float32 LE + 4A` 帧 | 底盘桥进程和 Topic 名称；主机侧串口状态 Topic 名称 | 源码明确没有 STM32 ACK；`HOST_WRITE_OK` 只说明主机写入状态，不能证明 MCU 接收、驱动器状态或底盘运动 |
| 安全反馈 | 机械臂安全状态文本 Topic 与 Agent 安全快照 | Topic 名称及 Agent 当前暴露的只读安全字段 | Agent 不读取 arm safety Topic 内容；实体急停和硬件健康没有接入证据，保持未知 |

## TF 与 STM32 链路

选定的单 MID-360 Launch 中，`base_frame → livox_frame` 为源码设置的静态变换；Point-LIO 源码发布 `odom_init → base_frame`；Launch 使用静态 `map → odom_init`。`slam_toolbox` 参数设置 `transform_publish_period: 0.0`，由此避免它重复发布 `map → odom_init`。Console 显示这些来源定义，但当前 ROS Graph 只提供 `/tf`、`/tf_static` 名称，不能确认实时 transform 是否存在、更新时间或是否冲突。

## 单雷达导航链路与孤立节点复核

雷达定位详情显示点云与 IMU 两条输入分支。点云分支依次连接 MID-360、Point-LIO、地形过滤、点云转 LaserScan、SLAM Toolbox 和在线 `/map`。`/scan` 是 SLAM Toolbox 的输入，`/map` 是其在线 OccupancyGrid 输出。导航详情另外显示 `/terrain_map` 直接进入 Nav2 的 STVL 障碍层、`/map` 进入全局静态图层，以及 `/tf` 和底盘速度输出。

该单雷达 Launch 传入 `bringup/config/nav2_params.yaml`，其中 Nav2 点云层配置的是 `/terrain_map`。`navigation/rm_navigation/params/nav2_params.yaml` 的其他配置含 `/scan`，但不由此 Launch 传入；拓扑不把该备用配置误画为当前单雷达 Launch 的 Nav2 LaserScan 输入。

Launch 中 `map_saver` 与生命周期管理器负责保存在线地图；RViz 是可选查看端。这些辅助节点未作为感知 / 导航数据链路节点绘制。`lidar-stack` 和 `arm-stack` 是 Agent 工作区进程汇总证据，保留为不连线的监测项，不映射成实际 ROS 数据端点。监测入口单独展示，不与机器人内部链路相连。安全反馈详情现在包含 Agent 作为开放急停接口的端点；该虚线端点说明实体急停没有接入，不能解读为已连接。

底盘 Host bridge 到 STM32 的协议有源码和固件协议文档依据。底盘主机串口状态 Topic 不包含固件确认。机械臂协议也只保留源码级链路；当前 Agent 不读取可证明实体反馈的协议响应。

## 底盘驱动来源修正

`Nav2_rea_ws/src/drivers/serial/driver/package.xml` 定义包 `chassis_serial_bridge`，节点可执行文件为 `chassis_serial_bridge_node`，默认订阅 `/cmd_vel` 并发布 `/chassis/serial_link_status`。该包被单雷达导航 Launch 包含。

`tzb_ws/src/serial/driver/package.xml` 定义的 `rm_serial_driver` 是机械臂串口驱动，订阅机械臂关节 / 夹爪命令并发布关节状态 / 安全状态。它不属于原始导航工作区的底盘桥。Agent 的 `original-navigation` 底盘包映射已改为 `chassis_serial_bridge`，接口改为 `/cmd_vel` 与 `/chassis/serial_link_status`；`/cmd_vel_chassis` 已从该工作区 Nav2 模块的接口候选中移除。

## GUI 交互

“机器人拓扑”成为 Console 默认首页。整机图用一次出现的子系统节点展示视觉、雷达定位、任务协调、Nav2、机械臂、底盘和安全反馈；箭头表示源码中的数据方向，分支与汇合按实际接口连接。NUC Agent、ROS Domain、ROS Graph 和运行预期放在单独的只读观测栏，不与机器人数据流混画。

第一层为整机总览，第二层点击子系统进入组件链路图，第三层点击组件查看状态、异常原因、Topic、工作区 / 进程证据及源码位置。可用横向滚动查看宽链路，页面纵向滚动查看组件和证据；整机图按当前窗口可用宽度布局。

连接线区分源码 / Launch、离线验证和待核实接口。Topic 多接口边显示名称全部可见、部分可见或均未观察；这些状态只统计当前 Graph 中的名称，不代表发布 / 订阅端归属或消息传输成功。Agent 断连、Domain 不匹配或 Graph 过期时连接运行状态保持未知。直接异常突出显示在来源节点；下游只显示“可能受影响”，不继承异常状态。

P6 任务协调器源码与隔离离线 Nav2 / ROS 2 验收均在 main 中；对应主机软件边标注离线验证。实体主启动流程集成未确认，故与视觉模块的输入边保持开放标记，实时 Action endpoint 也显示未知。

直接异常只来自已有健康规则事件。来源节点异常后，源码连接图中可能受影响的节点 / 边只显示“可能受影响”；其状态不会被批量改成异常。开放接口不参与影响传播。硬件节点在缺少反馈接口时保持未知。原“系统诊断”“测试驾驶舱”“历史记录”等页面继续保留。

## 只读边界

本变更只调整 Mac Console 的拓扑定义 / 展示、Agent 已有工作区只读清单中的模块归属元数据及测试。没有添加 ROS publisher、shell 入口、运动 / 执行器接口、服务重启功能，也没有修改、移动、构建或启动 `tzb_ws`、`Nav2_rea_ws` 原始工作区。
