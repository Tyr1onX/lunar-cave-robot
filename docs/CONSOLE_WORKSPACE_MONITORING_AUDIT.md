# Lunar Robot Console 工作区监测审计

审计时间：2026-10-16 18:11（Asia/Shanghai）  
NUC：`tzb-NUC14RVS-B`  
新版仓库：GitHub `main`，`901e0abbf543587aa237cfd9e81830ad35af46b9`

## 当前能力矩阵

本次通过目录、包清单、launch 与配置路径进行静态检查；通过进程表和已运行的只读 Agent 获取运行证据。源码存在只说明接口可供配置，不代表节点已经启动。ROS graph 只反映 Agent 自身 `ROS_DOMAIN_ID=187` 的发现结果。

| 工程来源 | 源码与配置 | 当前进程证据 | 当前 ROS 通信证据 | 运行模式与判定 |
| --- | --- | --- | --- | --- |
| 新版 `~/lunar-robot/code` | `main` 为 `901e0abb`；仓库包含新版机械臂、视觉、Nav2、雷达与底盘源码。 | 当前 UID 可见进程中未发现引用新版 ROS overlay 的模块进程。Agent 自身是只读观测服务，不计作机器人模块运行。 | Agent 在 Domain 187 观测到 0 个业务节点、0 个 Topic；Graph 数据时间为 `2026-10-16T10:11:11Z`，采集时龄约 680 ms。 | 模块启动预期未配置；运行模式未知。源码不能作为仿真或实体运行证据。 |
| 原始 `/home/tzb/tzb_ws` | 检出 14 个 `package.xml`；存在相机、YOLO、位姿估计、机械臂规划与串口驱动包及对应 launch/config。Git `main` 为 `0b64eb3`；审计时 `README.md` 已有未提交改动。 | 未发现引用该工作区路径或 overlay 的可见 ROS 模块进程。 | 当前 Agent Domain 187 的 Graph 为空；未能把任何运行节点或 Topic 归属到该工作区。 | ROS Domain、运行模式和启动预期未知。YOLO 空输出不能单独判故障。 |
| 原始 `/home/tzb/Nav2_rea_ws` | 检出 18 个 `package.xml`；存在 Livox、Point-LIO/FAST-LIO、Nav2、底盘串口驱动源码，以及 bringup、mapping、单雷达导航 launch 和参数文件。该目录没有 Git 元数据，无法从工作区确定提交 SHA。 | 未发现引用该工作区路径或 overlay 的可见 ROS 模块进程。 | 当前 Agent Domain 187 的 Graph 为空；其它 ROS Domain 未由 Agent 观测。 | ROS Domain、运行模式和启动预期未知；不得根据源码推断雷达、导航或底盘正在运行。 |

本次仅只读检查两个原始工作区的目录结构、包清单、README、所选接口源码和 Git 元数据；没有写入、移动或构建原始工作区。没有运行 `ros2` 命令、launch、驱动或测试脚本。原始目录保持在原路径。

## 静态接口证据与限制

| 模块 | 源码中可核对的接口 | 监测边界 |
| --- | --- | --- |
| 相机 | 原始 `tzb_ws/README.md` 和 Orbbec launch/config 中出现 RGB `/camera/color/image_raw`、深度图像、`/camera/depth/color/points` 与 `/camera/color/camera_info`。 | Topic 配置或发布者元数据不能证明图像内容有效、镜头无遮挡或相机硬件健康。需在确认当前运行预期后，检查真实帧时间和消息字段。 |
| YOLO | 订阅 `/camera/color/image_raw`；源码将 `publish_conf_threshold` 默认设为 `0.6`，检测达到发布门控并满足持续时间条件后才发布。标注图像只在存在订阅者时生成。 | 没有检测结果或标注图像不能单独作为故障。应先确认输入图像、节点进程、Domain、门控参数和持续时间，再报告异常。 |
| 位姿估计 | 订阅深度点云、CameraInfo 和 `/yolo/detections`；源码列出 `/pose_estimator/object_pose` 等输出。 | 目标位姿取决于有效点云、相机内参、检测和标定；无目标时不应把无输出直接判为故障。 |
| 机械臂 | 原始机械臂串口驱动源码发布 `/serial/arm_joint_state`，规划包另有轨迹和 RViz 关节状态接口。 | ROS 关节反馈只代表消息数据；它是否对应真实控制器、反馈是否可信，仍需设备身份、时间和硬件链路证据。仿真/RViz 关节状态不能当作实机反馈。 |
| 雷达与定位 | 原始 Nav2 配置中 Point-LIO 使用 `livox/lidar` 与 `imu/data` 输入；多个算法配置并存，实际接口会随 launch/参数选择变化。 | 源码中的某一算法配置不表示当前选择了该配置。需从运行节点、Topic 类型、消息时间与当前模式共同确认。 |
| Nav2 | bringup/单雷达导航 launch 涉及 `/scan`、`/map` 及速度输出 remap；存在 mapping、navigation 等不同入口。 | 未确认 launch 与生命周期状态前，节点缺失保持未知。地图或代价地图 Topic 存在不等于定位准确或导航可用。 |
| 原始底盘串口 | `/home/tzb/Nav2_rea_ws/src/drivers/serial/driver/src/node.cpp` 订阅 `/cmd_vel_chassis`。接收回调仅校验串口帧并重新挂接接收；源码注明接收侧 ROS 发布器尚未实现。 | 当前原始工程缺少可依赖的 ROS 底盘状态反馈 Topic。仅发现速度命令订阅不能证明底盘运动、制动或实体健康。本监测器不会订阅或发布速度命令。 |
| 新版底盘桥 | 新版 `chassis_serial_bridge` 源码包含 `/chassis/serial_link_status` 状态 Topic，并将命令 Topic 默认为 `/cmd_vel`。 | 状态 Topic 即使有数据，也只能作为软件串口链路证据，不能证明机械底盘、电机或制动健康。命令 Topic 不作为本监测功能的数据订阅目标。 |

## Agent 与模块测试服务

- `lunar-console-agent.service` 运行在 NUC loopback `127.0.0.1:8765`，工作目录为 `~/lunar-robot/code/tools/console-agent`，设置 ROS Domain 187 和 `ROS_LOCALHOST_ONLY=1`。预期 Domain 与预期节点集合均未设置。
- Agent `/v1/snapshot` 与 `/v1/vision` 返回只读采集结果；审计时 `/health` 正常。ROS graph 观测只覆盖 Agent Domain，不会自动发现其它 Domain。
- `lunar-console-module-tests.service` 运行在 `127.0.0.1:8769`，源码路径也在新版 `code/`。审计发现旧进程早于 `d398da4` 启动，而该提交更新了测试服务工作区保护和路径解析。确认 `busy=false` 后已重启此测试服务，使进程从当前已部署 `901e0abb` 加载代码；重启后 `/health` 正常、测试目录返回 11 个固定测试、`busy=false`。本次没有执行测试用例。
- 用户级 systemd 当前识别到 Agent 与受限模块测试服务；未发现名称指向原始机械臂、相机、雷达、Nav2 或底盘的用户/系统服务。

## 统一监测的安全分层

1. **当前阶段：来源与运行证据。** 一个 Agent 返回新版和两个原始工作区的包、launch/config 路径；只读检查 Agent UID 可见的 `/proc` 路径、ROS overlay 环境、进程包名、显式 Domain 和显式运行模式。工作区来源是每条证据的必填字段。
2. **Domain 关联。** 只有进程显式提供可解析 Domain，且 Agent graph 新鲜时，才标记共享或不同 Domain。不同 Domain、过期 graph、缺少 Domain、无法唯一归属的 overlay 都显示未知；不会用节点缺失制造连带故障。
3. **模块故障规则。** 当前没有针对原始模块配置运行预期。新增 UI 可显示“源码存在”“进程已发现”“Graph 可见性”，但在运行预期未核实时不生成缺失节点或 Topic 的异常事件。运行模式只接受进程环境中明确设置的白名单值，其余均为未知。
4. **数据超时。** 后续只对已确认运行模式及模块预期的数据流判定超时，并记录 ROS 消息时间、Agent 接收时间、Domain、来源路径和规则 ID。YOLO 规则必须继续考虑置信度门控及标注图像订阅者条件。
5. **实体状态。** 未来仅在源码或只读反馈接口提供可靠数据时报告链路异常；ROS 通信正常不等于机械臂、底盘、电机、雷达或相机实体健康。不得订阅命令流、启动传感器或操作硬件。

本轮实现增加的是读取工作区元数据、进程证据与已有 Graph 的关联；不增加 ROS Domain 探测器、不启动新 ROS 节点、不配置预期 Domain/节点、不操作 `/cmd_vel` 或其它控制接口。
