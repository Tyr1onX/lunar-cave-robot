# P5：雷达导航链路故障定位

## 源码链路审计

本试点依据 `Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py` 和对应配置，按实际接口拆成四个观测环节：

| 环节 | 主要输入 / 输出 | 源码关系 |
| --- | --- | --- |
| 雷达采集 | `/livox/lidar`、`/livox/imu` | 单 MID-360 Launch 与驱动配置定义两个发布接口；Topic 数据不证明设备供电或测量质量。 |
| 定位与障碍处理 | 输入 `/livox/lidar`、`/imu/data`；输出 `/odometry`、`/cloud_registered`、`/terrain_map` | IMU 经过坐标变换和互补滤波；Point-LIO 发布里程计与注册点云；地形分析生成 `/terrain_map`。 |
| 扫描与地图生成 | `/terrain_map` → `/scan` → `/map` | 点云转 LaserScan 后由 SLAM Toolbox 使用；此 Launch 的 SLAM Toolbox 发布在线 OccupancyGrid。 |
| 导航规划 | `/map`、`/terrain_map`、TF；条件输出 `/cmd_vel` | Nav2 静态图层消费 `/map`，STVL 使用 `/terrain_map`，定位依赖 TF。仅在配置“导航目标活动”时才把 `/cmd_vel` 作为预期持续输出。 |

`/livox/imu_aft_tf` 是 IMU 预处理支路的中间接口；`/tf` 与 `/tf_static` 的实时 frame pair 不由 Agent 读取。Nav2 Action endpoint / result、订阅者端点和消息内容也不在现有遥测 API 中。

## 判定前提与状态

Mac Console 默认为“未配置运行预期”。选择新版或原始单雷达运行场景，并手动填写预期 ROS Domain 后，才比较该场景要求的链路。预期节点名单是可选的，留空不会进行节点缺失判断。所有设置只写入 Mac 用户偏好，不会发往 NUC Agent、ROS 参数或任何工作区。

只有 Agent 在线、快照与链路遥测同属一个 Domain、两处 ROS Graph 均为新鲜且本机预期 Domain 与实际值相同时，才评估下游阶段。Agent 断连、旧版 Agent 没有 `/v1/telemetry`、Domain 不匹配、Graph 过期、预期配置无效或消息类型不支持时，状态保持未知，不生成级联故障。

| 状态 | 含义 |
| --- | --- |
| 收到数据 · 模块处理未确认 | Agent 在时限内收到该环节配置要求的 Topic；不确认工作区来源、消费者接收、算法处理或硬件状态。 |
| 疑似链路中断 | 显式运行预期下，Agent 报告等待、超时或无发布者；用于定位第一个数据证据缺口，不等同于已确认根因。 |
| 确认的运行预期偏差 | 配置中明确列出的节点，在新鲜同 Domain ROS Graph 中未出现。结论仅针对该节点预期，不代表实体硬件故障。 |
| 上游问题 · 可能受影响 | 只根据前序第一个证据缺口标示下游可能受影响，不创建新的根因结论。 |
| 证据不足 · 未知 | 观察接口、Graph、Domain 或 Topic 数据不足以作出结论。 |

异常首次观测时间、连续观测次数和恢复状态由 Console 对实时遥测快照跟踪；证据缺口消失后活动 finding 清除。每环节详情包含可见 Topic、数据时间、接收率、帧龄、Header 到 Agent 延迟、发布端点、ROS 节点名单、相关只读日志、源码位置及可能排查方向。可能原因以排查建议呈现，不作为已确认根因。报告由 Mac 本地文件保存对话框写入用户选择的位置。

## 隔离合成验收

`.github/workflows/console-tests.yml` 的 ROS 2 Humble 容器使用 `ROS_DOMAIN_ID=47` 与 `ROS_LOCALHOST_ONLY=1`，仓库以只读挂载。`pipeline_synthetic_scenarios.py` 只创建合成 ROS 2 发布者，覆盖完整数据流、`/scan` 超时与恢复、发布节点退出和 Domain 48 发布者对 Domain 47 Agent 不可见。Swift 规则测试用同一 `/v1/snapshot` 与 `/v1/telemetry` snake-case 数据结构，覆盖未配置运行预期、正常、各阶段数据缺口、节点退出、同时多处缺口、Agent 断连、旧版 Agent、Domain 不一致、Graph 过期、日志证据和异常恢复。

合成数据只验证 Agent 接收观察和 Console 规则；它不验证真实雷达、导航、机器人硬件或实体实时工作负载。
