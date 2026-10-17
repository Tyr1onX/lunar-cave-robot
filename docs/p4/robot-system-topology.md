# Lunar Robot Console：机器人系统拓扑审计

## 范围与证据分类

拓扑描述来自 `b379b198` 中的源码、Launch、配置和 STM32 协议。它是静态结构模型，与 Agent 的实时状态分开保存。

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
| 雷达定位 | `mapping_navigation_single_lidar.launch.py` 启动 MID-360、Point-LIO、地形分析和扫描转换；Point-LIO 发布 `cloud_registered`、`odometry` 和 TF；地形分析订阅点云 / 里程计并发布 `terrain_map` | `livox/lidar`、`imu/data`、`cloud_registered`、`odometry`、`terrain_map`、`scan` 等 Topic 名称；Agent 的聚合雷达 / 定位工作区进程状态 | Agent 的 `lidar` 模块合并多个包，不能确认 MID-360 实体状态或 Point-LIO 独立节点健康；TF frame pair 未被读取 |
| 地图与导航 | 同一单雷达 Launch 中 `slam_toolbox` 发布 `/map`，设置 `map → odom_init`；Point-LIO 发布 `odom_init → base_frame`；Nav2 velocity smoother 将 `cmd_vel_smoothed` remap 到 `/cmd_vel` | `/map`、`/scan`、`/tf`、`/cmd_vel` 名称；工作区导航进程扫描 | Agent 不提供 Nav2 Action 服务器、生命周期状态、TF frame tree、地图内容或消费端关联 |
| 任务协调 | `docs/p6` 与 `tools/p6_offline` 中存在离线接口 / 验收记录 | 当前只能显示来源和接口说明 | main 中没有已接入的 ROS 任务协调器；NavigateToPose 和机械臂任务边标记为待核实，不参与下游影响传播 |
| 机械臂 | `tzb_ws` 中 `lite_motion_planner`、`rm_serial_driver`、串口协议与 `/serial/arm_joint_state`、`/serial/arm_safety/status` 源码 | Graph Topic 名称、合并的 Agent `arm` 工作区进程证据 | Agent 模块把规划器和串口驱动合并，不能按进程拆分；当前 Graph 不读取反馈消息值、所有关节状态、硬件急停或实体执行结果 |
| 底盘 | `Nav2_rea_ws` 中 `chassis_serial_bridge` 订阅 `/cmd_vel`，发布 `/chassis/serial_link_status`；USART1 使用 115200、14 字节 `A5 + 3×float32 LE + 4A` 帧 | 底盘桥进程和 Topic 名称；主机侧串口状态 Topic 名称 | 源码明确没有 STM32 ACK；`HOST_WRITE_OK` 只说明主机写入状态，不能证明 MCU 接收、驱动器状态或底盘运动 |
| 安全反馈 | 机械臂安全状态文本 Topic 与 Agent 安全快照 | Topic 名称及 Agent 当前暴露的只读安全字段 | Agent 不读取 arm safety Topic 内容；实体急停和硬件健康没有接入证据，保持未知 |

## TF 与 STM32 链路

选定的单 MID-360 Launch 中，`base_frame → livox_frame` 为源码设置的静态变换；Point-LIO 源码发布 `odom_init → base_frame`；Launch 使用静态 `map → odom_init`。Console 显示这些来源定义，但当前 ROS Graph 只提供 `/tf`、`/tf_static` 名称，不能确认实时 transform 是否存在、更新时间或是否冲突。

底盘 Host bridge 到 STM32 的协议有源码和固件协议文档依据。底盘主机串口状态 Topic 不包含固件确认。机械臂协议也只保留源码级链路；当前 Agent 不读取可证明实体反馈的协议响应。

## 底盘驱动来源修正

`Nav2_rea_ws/src/drivers/serial/driver/package.xml` 定义包 `chassis_serial_bridge`，节点可执行文件为 `chassis_serial_bridge_node`，默认订阅 `/cmd_vel` 并发布 `/chassis/serial_link_status`。该包被单雷达导航 Launch 包含。

`tzb_ws/src/serial/driver/package.xml` 定义的 `rm_serial_driver` 是机械臂串口驱动，订阅机械臂关节 / 夹爪命令并发布关节状态 / 安全状态。它不属于原始导航工作区的底盘桥。Agent 的 `original-navigation` 底盘包映射已改为 `chassis_serial_bridge`，接口改为 `/cmd_vel` 与 `/chassis/serial_link_status`；`/cmd_vel_chassis` 已从该工作区 Nav2 模块的接口候选中移除。

## GUI 交互

“机器人拓扑”成为 Console 默认首页。系统入口、视觉、雷达与导航、任务协调、机械臂、底盘及安全反馈按流程分组；展开流程后可点选模块查看源码位置、Topic / 接口、工作区来源、进程状态、Domain、Graph 时间与健康事件证据。拓扑结构来源标签与实时状态分开呈现。

直接异常只来自已有健康规则事件。来源节点异常后，源码连接图中可能受影响的节点 / 边只显示“可能受影响”；其状态不会被批量改成异常。开放接口不参与影响传播。硬件节点在缺少反馈接口时保持未知。原“系统诊断”“测试驾驶舱”“历史记录”等页面继续保留。

## 只读边界

本变更只调整 Mac Console 的拓扑定义 / 展示、Agent 已有工作区只读清单中的模块归属元数据及测试。没有添加 ROS publisher、shell 入口、运动 / 执行器接口、服务重启功能，也没有修改、移动、构建或启动 `tzb_ws`、`Nav2_rea_ws` 原始工作区。
