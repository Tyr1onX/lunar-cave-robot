# P7 整机控制链路离线集成

## 启动

ThinkBook Ubuntu 22.04 / ROS 2 Humble：

```bash
cd /home/qibai/lunar-cave-robot-p7
P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator ./tools/p7_offline/run.sh
```

入口只接受程序创建的 PTY，不接受外部串口路径；强制 `ROS_LOCALHOST_ONLY=1` 并清除 Discovery Server 配置。它把 `Nav2_rea_ws` 与 `tzb_ws` 分别构建到各自的 `install/p7`，每个 ROS 进程只加载自己的工作空间 overlay。仿真机械臂保持 P4e DISARMED，直到操作者显式给出上述环境变量；启动器调用 enable，不会自动调用 reset。运行记录包含该操作、状态、时刻和反馈年龄。

可用 `P7_ROS_DOMAIN_ID=2..101` 指定隔离域，`P7_LOG_DIR` 指定记录目录。不要设置 `P7_ARM_DEVICE` 或 `P7_CHASSIS_DEVICE`；入口会拒绝这些参数。

完整场景命令：

```bash
./tools/p7_offline/validate_scenarios.sh
```

可用逗号分隔的 `P7_CASE_FILTER` 选择重跑场景，`P7_DOMAIN_START` 指定域起始值。每场使用独立 ROS domain 和新建 PTY。日志包括任务事件、状态、Action/Topic/TF 探测、PTY 线帧计数、模拟操作者操作和最终摘要。

## 数据与控制链路

- Nav2 使用仓库实际 ROS 2 lifecycle、Smac 2D 全局规划器、DWB 控制器和 NavigateToPose Action。离线地图、雷达扫描、TF、里程计与环境几何由 ROS 仿真节点产生。
- Nav2 输出经过 velocity smoother，再进入 P5b `chassis_serial_bridge`。PTY 模拟 STM32 解码 14 字节底盘帧，只有解码出的线速度才驱动模拟底盘；里程计和 TF 再反馈 Nav2。模拟器不订阅 Nav2 原始速度作为运动输入。
- P6 在 Nav2 Action 成功、底盘已停车且距离目标满足门限后才进入识别阶段。目标由 P3/P3b 离线视觉源发布，交由实际 P3b 规划代码处理。
- P6 任务请求进入 `tzb_catch`。实际规划器发布带 P4f task ID、epoch、单调序号的关节命令；P4e 安全驱动校验安全状态、反馈新鲜度、命令租约及轨迹生命周期，再编码 46 字节 STM32 兼容帧到 PTY。模拟机械臂只按接收、解码后的命令做有限速率关节跟随，并把关联关节反馈编码回 PTY；P4e 解码反馈并发布 JointState。
- 抓取/卸载成功要同时满足规划器结果、P4e ACTIVE_HOST 状态及请求后的新鲜关节反馈。P6 记录确认后才允许返航或完成任务。

动态模拟值来自离线 ROS 节点和 PTY peer；Nav2、velocity smoother、P3b 规划器、P5b/P4e ROS 驱动和 P6 协调逻辑执行仓库中的实际 ROS 软件代码。该验收不连接硬件，也不验证电机或实体运动。

## 状态机与失败处理

任务阶段顺序为等待导航栈 → 目标导航 → 到达/停车确认 → 目标识别 → P3b 抓取 → 返航 → 到达/停车确认 → P3b 卸载 → SUCCEEDED。导航 Action 失败可按有限次数重试，任务/阶段均有超时。丢失 P4e 状态流、反馈变旧、故障锁存、目标检测超时、规划失败、取消超时或底盘通信丢失都会中止任务。重复触发被忽略，不创建第二个任务。

P6 取消确认只证明 ROS 上游不再发布该 task/epoch 的轨迹。它不证明 STM32 收到停止，更不表示电机急停。P4e 46 字节协议帧没有 P4f task ID/epoch/命令序号，也没有下游停止确认。清除控制帧使能位不能视作急停。

离线场景里的操作者 enable 是单独记录的模拟人工操作。测试不自动 reset 故障锁存；故障场景退出后也不会恢复或接管旧任务。

## 兼容性与限制

P5b 底盘 14 字节线协议和 P4e 机械臂 46 字节线协议保持不变。P4f 元数据仍通过 `JointState.header.frame_id` 的 `p4f:<task_id>:<epoch>:<seq>` 编码，仅供 ROS 主机侧使用；把元数据放进 frame_id 并非标准消息语义。后续更规范的接口应使用专用 `ArmTrajectoryCommand`（task ID、epoch、序号、关节位置及夹爪状态），并让驱动到控制器的协议显式支持确认/取消生命周期。

P4f 轨迹 epoch 与旧轨迹取消问题已经在 ROS 上游实现防重放和取消确认，但 STM32 实际停止确认仍缺少下游协议和固件配合。真实部署前还需完成固件停机/看门狗/状态回报设计评审、实体急停与机械制动验证、负载及碰撞验证、关节零位与限位标定，以及相机、雷达、底盘里程计和 TF 外参标定。本分支不更改固件或实体安全策略。
