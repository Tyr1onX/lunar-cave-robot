# P4f ThinkBook ROS 2 运行验收记录

验收分支：`p4f/arm-trajectory-safety`  
基线：`p6/autonomous-mission-offline` / `0a966061de044f66f7ce794a13a1616cb6d18b56`  
主机：ThinkBook 16 G6 AHP，Ubuntu 22.04 x86_64，ROS 2 Humble  
独立 Worktree：`/home/qibai/lunar-cave-robot-p4f-acceptance`  
验收命令：`./tools/p4f_offline/validate.sh`  
最终命令退出码：0

全部运行都设 `ROS_LOCALHOST_ONLY=1`。P6 功能仿真使用域 98，P4e PTY 使用域 99；P6 Nav2 和 arm 两个工作空间分别构建和 source。P4e 驱动另构建在 `tzb_ws/install/p4f`，没有叠加工作空间 overlay。没有连接 NUC、STM32、实体串口、相机、雷达、电机或真实机械臂。

## 构建与回归结果

- P6 Nav2 工作空间 `p6_autonomous_mission` 构建通过。
- P6 arm 工作空间 6 个包构建通过：`rm_serial_msgs`、`arm2nav_msgs`、`lite_motion_msgs`、`tzb_description`、`rm_serial_driver`、`lite_motion_planner`。
- P4e `rm_serial_driver` 在独立 `install/p4f` 前缀构建通过。
- P6 状态机及隔离契约自动化测试：8/8 通过。
- P4e 驱动 PTY 测试：全部断言通过，真实 ROS 2 驱动节点通过生成 PTY 收发原有 46 字节帧。
- 真实 RRTConnect 规划结果与取消竞争测试：通过；取消确认后 P4e 输入 topic 没有旧任务命令，较高 epoch 的后续抓取成功。
- P6 六场景：六个运行时监视器全部 PASS。

## 六个完整 ROS 任务场景

任务耗时从事件日志中的 `TRIGGER_ACCEPTED` 算到终态事件。正常流程耗时 **93.252 秒**。

| 场景 | 终态 / 耗时 | 实测结果 |
| --- | --- | --- |
| `normal` | `SUCCEEDED` / 93.252 s | 出发和返航的 Nav2 Action 都返回 SUCCEEDED；到站停车后确认合成目标；机械臂 pick、unload 均返回匹配 task id、stage、epoch 的 success。 |
| `nav_failure` | `ABORTED` / 47.451 s | 障碍物目标导致 NavigateToPose 两次以 status 6 结束；只重试一次后中止，没有 arm 请求。 |
| `target_loss` | `ABORTED` / 51.900 s | Nav2 到目标并确认静止；目标检测超时中止，没有抓取请求。 |
| `grasp_failure` | `ABORTED` / 37.351 s | 抓取执行中注入取消；catch 返回 failure，上游确认静默，未发返航 goal；取消后新 epoch 抓取完成。 |
| `communication_loss` | `ABORTED` / 41.502 s | 抓取请求已到达 catch；测试丢弃结果回传，4 秒超时后中止并取消正在执行的任务。监视到 170 条带上下文的 P4e 输入关节消息，取消确认后 0.5 秒无旧任务输出；随后新 epoch 抓取成功。 |
| `duplicate_trigger` | `SUCCEEDED` / 93.101 s | 4 次任务触发中接受 1 次、忽略 3 次；之后完整采样、返航、卸载。 |

通信中断场景覆盖了“任务已接收、执行期间丢失结果回传、协调器超时取消”的路径。取消消息本身能抵达 catch；如果 ROS 通信完全中断到取消消息也无法抵达，系统只能记录未确认并且不能证明上游已经停止，见剩余风险。

## ROS 运行证据

- Action graph 中 `/navigate_to_pose` 的 server 是 Nav2 `/bt_navigator`；client 有 `/p6_mission_coordinator`、`/bt_navigator` 和 `/waypoint_follower`。
- `/cmd_vel` 有 Nav2 behavior server、velocity smoother 和 P6 停车节点发布，订阅方是 `p6_cave_simulator`。`/odom` 的发布方是 `p6_cave_simulator`，订阅方包括 controller、BT navigator、任务协调器。图中也有模拟 `/map`、`/scan`、`/tf` 和 `/tf_static`。
- 运行时 `/tf` 的 `map → base_link` 可解析；位置随差速底盘模型变化。正常流程事件记录了目标附近位置约 (9.85, 6.41) 和返航位置约 (1.02, 1.10)，并要求 Nav2 Action 成功、位置达标、速度停车后才继续状态机。
- P6 catch 在 `/tzb_catch/arm_joint_command` 发布带 P4f 元数据的 JointState；P6 监视器实际订阅此 P4e 驱动默认输入 topic，并在取消竞争/通信中断测试中观察到输出及确认后静默。
- P4e PTY 测试使用自定义隔离 topic 运行实际 `rm_serial_driver` ROS 节点，直接测试相同的 JointState 上下文解析、epoch/序号判定和串口帧生成逻辑。六个完整 P6 场景使用 P3b 合成关节反馈，不同时运行 P4e 串口节点。

P4e PTY 时序样本：ROS 100 Hz 发布样本 499 个，中位 10.001 ms、p99 10.253 ms、最大 10.747 ms；驱动输出 511 帧，中位 9.991 ms、p99 11.600 ms、最大 12.173 ms。这是本机 ROS/PTTY 软件时序测量，不是电机控制实时性指标。

## 取消竞争专项结果

`validate.sh` 在六场景前将取消定时设为 0.36 秒，实际 Nav2 驱动到目标后启动 catch 与真实规划节点。关键顺序为：

1. planning_node 的 RRTConnect 产生真实解。
2. P6 coordinator 对抓取任务发出取消。
3. catch 收到取消后才处理已在途的规划响应；命令发布门检查任务上下文已失效，不允许规划结果恢复旧任务。
4. catch 发布 `UPSTREAM_QUIESCED`，coordinator 记录取消确认。
5. P4e 输入 topic 取消确认后 0.5 秒没有该任务的新 JointState。
6. 更高 epoch 的新抓取成功。

日志位于 `/tmp/p4f-release-acceptance/cancel-plan-race/`。之后又以更严格的 topic 消息计数复跑一次，catch 在取消前向 P4e 输入 topic 发出 1 条带标记命令，取消确认后静默，新任务接管通过；日志位于 `/tmp/p4f-final-acceptance/cancel-plan-race-final/`。

## P4e PTY 故障注入

日志：`/tmp/p4f-release-acceptance/p4e-pty.log`。覆盖并通过：

- 启动 DISARMED，协议输出仍是 46 字节；测试手动调用一次 enable 以验证有效命令路径。
- 合法六关节与带 task/epoch/序号的夹爪命令编码；旧无标记夹爪输入在 P4f 生命周期激活后被拒绝。
- 旧/未来时间戳、NaN、Inf、关节越界和关节不完整被拒绝；命令租约、反馈超时和反馈序号倒退进入故障锁存。
- 重复/倒退序号、已取消 epoch、旧 epoch 的夹爪和关节重放被拒绝；同 epoch 不匹配的更高取消请求不能清除活动任务。
- 取消清空 P4e 主机目标，ack 明确为 `ROS_HOST_CANCELLED_STM32_UNCONFIRMED`；更高 epoch 接管成功。
- PTY 断连/重连保持故障锁存；驱动进程重启为 DISARMED、不回放缓存。测试只在重新运行中显式人工 enable 并发送新命令后恢复，没有自动 reset/enable。

## 算法与模拟数据边界

实际运行的算法/ROS 节点包括 Humble Nav2 `NavigateToPose`、项目的 Nav2 全局/局部规划与速度控制节点、P3b `lite_motion_planner`、IK/碰撞检查与 OMPL RRTConnect。它们处理的是模拟输入。

模拟数据包括静态占用地图、射线生成 LaserScan、仿真 odometry/TF、合成相机坐标系目标位姿、合成关节反馈及规划场景中的样本附着状态。没有相机图像推理、真实编码器反馈、实物抓取传感器或真实底盘里程计。PTY peer 只验证串口字节协议及故障行为，不等同于 STM32 或机械臂动力学仿真。

## 失败记录与修复

验收中修复了以下软件/测试问题，最终单命令验收全通过：

- lifecycle service 发现或响应竞态会让协调器卡在 WAITING_FOR_STACK；增加短超时、重试与状态日志。
- 第一次 P4f PTY 回归在 P4f epoch 建立后仍用旧 Int8 夹爪命令，暴露出未标记入口；改为 P4f 带序号命令，并验证旧入口拒绝。
- 第一次整合验收入口在 `set -u` 下 source ROS setup 失败；入口调整 setup 环境后通过完整构建和回归。
- 验收监视器最初监听模拟 arm topic；改为监听 P4e 默认输入 topic，并增加“取消前确实收到标记消息、取消确认后静默”断言。

构建日志和全部 JSONL、Nav2、arm 及 scenario monitor 日志均保存在 `/tmp/p4f-release-acceptance/` 下。

## 取消确认边界与剩余风险

ROS 2 上游已经按任务 ID、epoch 和命令序号实现取消与防重放。catch 取消确认只在工作线程停止发布并清除回放段、TCP 轨迹及冻结目标缓存后发出；P4e 驱动另行确认主机目标失效。取消与规划响应竞争、任务结果丢失和新任务接管均由真实 ROS 节点验证。

**46 字节 STM32 帧没有任务 ID、epoch、命令序号或队列取消字段。** P4e 无法撤回已经交给 UART/STM32 的帧，也无法确认 STM32 已清空待执行轨迹或停下当前动作。协议使能位清零不是电机急停，不能证明机械臂制动距离、保持扭矩或载荷不会坠落。若任务取消 topic 本身在 ROS 网络分区中无法抵达 catch，协调器会在 5 秒后记录 `ARM_CANCEL_UNCONFIRMED` 并保持任务 ABORTED，但不能远程证明 catch 已停止。

真实机器人执行仍受这些事项阻挡：STM32 协议/固件需增加可确认的代际与序号拒绝、在途队列取消和停止状态反馈；还需完成真实机械臂/底盘传感反馈、制动/保持、P4e 故障人工恢复与受控硬件验收。本任务没有改 STM32、复位/使能安全策略或任何实体设备。

**结论：P4f 的 ROS 2 上游轨迹生命周期管理与防重放通过 ThinkBook ROS 2 / PTY 软件验收；这不表示 STM32 或真实机器人已具备可确认的安全停止能力。**
