# P7 整机控制链路离线验收

- 分支：`p7/full-control-integration`
- 基线：`p4f/arm-trajectory-safety`，`eab6eef4169e4d749950d41ce9ba19b5f72b3b57`
- 环境：ThinkBook 16 G6 AHP，Ubuntu 22.04 x86_64，ROS 2 Humble
- Worktree：`/home/qibai/lunar-cave-robot-p7`
- 结论：P7 **离线端到端 ROS 2 + PTY 串口模拟闭环通过**；未连接真实机器人或实体串口。

所有启动均设置 `ROS_LOCALHOST_ONLY=1`，每次运行用不同 `ROS_DOMAIN_ID`，底盘及机械臂串口地址均由启动器创建并验证为 `/dev/pts/*` 的符号链接。入口拒绝调用者提供的串口路径。P6 Nav2/P5b 与 P3b/P4e 分别构建到两个独立的 `install/p7` 前缀；每个 ROS 进程只加载对应 workspace。无硬件 reset，模拟使能是审计记录的显式模拟操作者操作。

## 一条命令正常运行

```bash
cd /home/qibai/lunar-cave-robot-p7
P7_ROS_DOMAIN_ID=82 P7_LOG_DIR=/tmp/p7-one-command-final \
  P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator ./tools/p7_offline/run.sh
```

- 完整命令墙钟耗时：**107.383 秒**，包含增量构建、启动和清理。
- MissionControl 任务耗时：**94.999 秒**；最终状态 `SUCCEEDED`。
- 目标导航和返航的 `NavigateToPose` Action 都返回 SUCCEEDED，P6 检查到站距离并确认停车后才进入下一阶段。
- 底盘 PTY 接收并解码 5,164 个 14 字节帧，其中 3,639 个为运动帧；模拟底盘只消费解码后速度，反馈里程计和 TF。
- 机械臂 PTY 收到 5,852 个使能 46 字节命令帧；模拟控制器按这些命令有限速率跟随并回送反馈，P4e 解码得到 4,705 个反馈帧和 1,471 次关节变化。
- 抓取/卸载期间出现 4,500 个夹爪闭合帧、754 个打开帧。P6 两次都收到匹配 task ID/epoch 的规划结果、ACTIVE_HOST 状态以及请求后的新鲜关节反馈。
- 重复关节序号、无效机械臂帧、无效底盘帧均为 0。探测记录在 `/tmp/p7-one-command-final/ros-graph.log` 和 `tf-map-base.log`。

任务状态顺序：`WAITING_FOR_STACK → NAVIGATING_TO_TARGET → CONFIRMING_TARGET_STOP → DETECTING_TARGET → ARM_PICK → NAVIGATING_HOME → CONFIRMING_HOME_STOP → ARM_UNLOAD → SUCCEEDED`。每次状态转移都写入 JSONL 事件记录。

## P7 完整场景套件

最终使用 `P7_DOMAIN_START=50` 完整执行 `./tools/p7_offline/validate_scenarios.sh`，脚本退出码 0，结尾为 `P7_E2E_SCENARIOS=PASS cases=11`。各场景在真实 ROS 2 节点与新建 PTY 上执行：

| 场景 | 终态 / MissionControl 耗时 | 运行结果 |
| --- | --- | --- |
| `normal` | `SUCCEEDED` / 95.351 s | Nav2 去程/返航、P3b 抓取、P3b 卸载均有 Action/反馈证据。 |
| `nav_failure` | `ABORTED` / 72.752 s | Nav2 Action 失败后有限重试并中止；没有 arm 请求。 |
| `target_loss` | `ABORTED` / 51.266 s | 到站、停车确认后目标识别超时；没有抓取请求。 |
| `grasp_failure` | `ABORTED` / 36.443 s | 实际抓取规划执行中取消，确认上游静默后中止。 |
| `communication_loss` | `ABORTED` / 39.922 s | 丢弃抓取结果通道；P6 不接受缺失结果，超时取消并中止。 |
| `duplicate_trigger` | `SUCCEEDED` / 94.815 s | 注入 3 次重复启动，只执行一个任务并成功一次。 |
| `arm_feedback_stop` | `ABORTED` / 37.457 s | P4e 反馈超时锁存 `FAULT_LATCHED`，状态传到 P6 并取消 arm task。 |
| `arm_disconnect` | `ABORTED` / 38.933 s | P4e 串口失联导致必需状态流过期；P6 保守中止、确认取消，未误报成功。 |
| `driver_restart` | `ABORTED` / 38.229 s | P4e 重启后保持 DISARMED；没有旧使能帧，P6 中止并取消旧任务。 |
| `chassis_disconnect` | `ABORTED` / 26.035 s | 去程底盘断线后 Nav2 失败并中止，没有 arm 请求。 |
| `chassis_disconnect_return` | `ABORTED` / 230.158 s | 抓取已确认后在返航断开底盘；两次 Nav2 限时尝试失败，P6 中止并确认取消 arm hold。 |

全套日志、Mission 事件、PTY 帧计数、Topic/Action 探测及 TF 检查保存在 `/tmp/p7-e2e-suite-acceptance-final`。之前在 domain 40 做的一次连续运行遇到 Nav2 `/planner_server/change_state` 响应超时，P6 在 WAITING_FOR_STACK 超时后 fail-closed，没有注入故障也没有误报成功；隔离域 50 上的最终完整 11 场景连续套件全部通过。为减少重复运行残留，启动器现在对自己创建的各个进程组执行限时 SIGINT/SIGTERM 后清理，并修复 ROS overlay 在 nounset 模式下加载失败的 Topic 探测。

## P4e / P4f / P5b / P6 回归

- `tools/p4f_offline/validate.sh` 退出码 0，P6 与 arm workspace 分别构建，P4e 驱动另构建到 `install/p4f`。
- P4e PTY 安全用例通过：启动 DISARMED；命令租约超时、反馈停更、倒退 STM32 反馈序号触发锁存；旧时间戳、未来时间戳、NaN/Inf、越界/不完整关节目标被拒；重复命令序号、旧 epoch、已取消任务/夹爪重放被拒；断线重连及进程重启后保持 DISARMED。46 字节协议兼容性检查通过。
- P6 状态逻辑和离线安全契约测试 **8/8** 通过。
- P4f 实际规划结果与取消竞争测试通过：取消确认后没有旧命令继续发布，更高 epoch 的新任务接管成功。
- P6 六个独立任务场景 `normal/nav_failure/target_loss/grasp_failure/communication_loss/duplicate_trigger` 全部达到预期终态。
- P5b 配置验证全部通过。真实 ROS 2 桥接 PTY 测试通过：14 字节速度帧、前进/后退/转向/停车、Vy=0、NaN/Inf/超限输入、陈旧命令零输出、断线重连先发零帧及关闭前零帧。
- 回归日志位于 `/tmp/p7-p4f-regression-final` 和 `/tmp/p7-p5b-regression-final.log`。

## 算法、模拟数据与停止确认边界

ROS 侧执行的实际软件包括 Nav2 lifecycle、Smac 2D 全局规划器、DWB 控制器、NavigateToPose Action、Nav2 velocity smoother、P5b 串口桥、P3b 视觉到机械臂规划流程、P4f task/epoch/sequence 校验与取消屏障、P4e 互锁/反馈新鲜度/租约/故障锁存，以及 P6 任务状态机。模拟数据包括洞穴地图、激光扫描、TF/里程计、视觉目标、PTY STM32 peer、差速底盘运动和按帧跟随的机械臂/关节反馈。

取消确认明确是 ROS 主机上游发布静默确认，不代表 STM32 收到停止，也不代表电机急停。机械臂仍使用兼容的 46 字节协议，底盘仍使用兼容的 14 字节协议；固件没有收到 P4f task ID、epoch 或命令序号，也没有轨迹取消确认。P4f 元数据仍借用 `JointState.header.frame_id` 的 `p4f:<task_id>:<epoch>:<seq>` 字符串，只在 ROS 主机内有效，不是标准消息语义。建议后续定义专用带版本的 `ArmTrajectoryCommand`，并经固件评审后增加下游命令/停止确认。

## 真实机器人执行前仍需完成

- STM32 固件与 P4e 驱动的协议生命周期设计：命令确认、停止确认、重启/断线后的命令代际隔离、看门狗和安全状态回报。P7 没有改固件或电机安全策略。
- 真实机械臂的独立急停、驱动器失能/制动和断电验证；关节限位、零位、负载、碰撞、保持扭矩和故障恢复的机械安全验证。
- 真实底盘的刹停距离、坡面/松散地面牵引、速度与里程计标定。
- 相机内参与畸变、相机-机械臂外参、雷达到 base 外参、base/map/arm TF 标定，真实洞穴照明与目标识别数据验收。
- 在目标 NUC 和传感器上验证实时调度、ROS QoS/频率、串口延迟与丢包、电源及 EMI、长时间任务和人工复位流程。

P7 的“端到端通过”仅表示真实 ROS 2 控制代码经过离线 PTY 串口协议闭环。它不构成真实机器人运动、机械安全或 STM32 停止确认验收。
