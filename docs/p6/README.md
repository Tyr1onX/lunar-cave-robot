# P6：离线整机自主采样软件闭环

基线是 `p4e/arm-host-safety`，提交 `ef62b84589e77c136e97dc7f68d234a1c489d498`。P6 代码位于独立分支 `p6/autonomous-mission-offline`，不合并 `main`。

## 离线启动

在 Ubuntu 22.04 / ROS 2 Humble 的完整仓库 checkout 根目录运行：

```bash
./tools/p6_offline/run.sh
```

脚本分别构建 `Nav2_rea_ws` 与 `tzb_ws` 到各自的 `install/p6`，再由两个独立子进程分别 source 对应工作空间。它设置 `ROS_LOCALHOST_ONLY=1` 与专用 `ROS_DOMAIN_ID`，启动 P3b 时固定 `use_serial_driver=false`，没有串口、雷达、相机或电机驱动节点。脚本在其他平台不会尝试接触设备，但 ROS 2 / Nav2 验证需要 Ubuntu Humble。

任务状态和 JSONL 事件记录在 `/tmp/p6-autonomous-mission/mission-events.jsonl`。Nav2 与机械臂启动日志分别写入 `/tmp/p6-autonomous-mission/nav2-mission.log` 和 `arm.log`。正常流程是：

```text
WAITING_FOR_STACK → NAVIGATING_TO_TARGET → CONFIRMING_TARGET_STOP
→ DETECTING_TARGET → ARM_PICK → NAVIGATING_HOME
→ CONFIRMING_HOME_STOP → ARM_UNLOAD → SUCCEEDED
```

只有 Nav2 `NavigateToPose` action 返回 `SUCCEEDED`、仿真里程计满足目标误差并连续静止，且机械臂按匹配 task id / stage 返回成功结果后，状态机才会前进。抓取规划失败、结果超时、目标丢失、导航失败或观察到 P4e `FAULT_LATCHED` 会进入 `ABORTED`；正在执行的 arm task 会收到取消请求。导航允许一次重试。

## 使用的真实软件与模拟数据

- 导航调用已安装的 ROS 2 Nav2 `NavigateToPose` action、Smac 2D 全局规划器、DWB 局部控制器、Nav2 costmap 和 velocity smoother。`cave_simulator` 提供静态 OccupancyGrid、射线生成的 LaserScan、`odom → base_link` 与由 Nav2 `/cmd_vel` 驱动的差速运动模型；`map → odom` 为固定初始变换。
- 机械臂复用 `lite_motion_planner`、`planning_node`、`tzb_catch` 和 P3b synthetic vision。新增 P6 pick/unload 模式：取样时抓住规划场景中的物体并保持夹爪闭合，回到观察位；卸载时在基地规划到既有 place 关节位，分离规划场景物体、打开夹爪并回观察位。
- 视觉目标来自 P3b 固定目标与 URDF/TF 转换，不是相机图像或 YOLO 检测。关节反馈由 P3b 的 `p2_offline_feedback.py` 合成回显，夹爪动作由规划器的离线理论状态记录，没有电机反馈。
- P5b `chassis_serial_bridge` 不由该整机仿真启动。既有 PTY 测试继续保留，可单独按 `docs/p5b/run-offline-validation.md` 运行；P6 模拟底盘直接消费 Nav2 平滑后的速度。

## 故障注入

P6 launch 支持 `scenario:=nav_failure|target_loss|grasp_failure|communication_loss|duplicate_trigger|normal`：

- `nav_failure` 将目标放入地图内的占用区，期望 Nav2 返回失败、重试一次后任务中止。
- `target_loss` 丢弃视觉位姿，期望检测超时中止且不触发机械臂。
- `grasp_failure` 发出匹配 task id 的取消请求，期望机械臂报告失败、机器人不返航并且任务不成功。
- `communication_loss` 丢弃 arm task 请求，期望等待结果超时并中止。
- `duplicate_trigger` 走正常任务路径，由验证监视器重复发布任务触发并确认只接受一次。
- `normal` 执行完整目标采样、返航和卸载流程。

2026-10-11 已在 ThinkBook Ubuntu 22.04 / ROS 2 Humble 上执行完整构建、正常整机任务和六个实际 ROS 场景。最终结果、日志位置、topic/TF/action 证据和未解决项见 [ROS 2 实际运行验收记录](ros2-runtime-acceptance.md)。

| 场景 | 预期终态 | 实测结果 |
| --- | --- | --- |
| `normal` | `SUCCEEDED` | 通过：Nav2 往返、目标确认、P3b 抓取与卸载均有结果确认 |
| `nav_failure` | `ABORTED`，导航最多重试一次 | 通过：Nav2 两次返回失败，重试耗尽后中止 |
| `target_loss` | `ABORTED`，不启动机械臂 | 通过：到站停车后目标检测超时中止 |
| `grasp_failure` | `ABORTED`，不返航 | 通过：取消后的机械臂失败结果导致任务中止 |
| `communication_loss` | `ABORTED`，机械臂结果超时 | 通过：请求丢弃后超时中止 |
| `duplicate_trigger` | `SUCCEEDED`，重复触发被忽略并记录 | 通过：接受 1 次、忽略 3 次，随后正常完成闭环 |

重复触发可向 `/p6/start` 发布多条 `std_msgs/Bool(data=true)`；首次之外均记录 `TRIGGER_IGNORED`。单元测试运行：

```bash
./tools/p6_offline/run_unit_tests.sh
```

完整情景自动化运行：

```bash
./tools/p6_offline/validate_scenarios.sh
```

脚本先独立构建两个工作空间，再逐项启动隔离 ROS graph，监视期望终态并在完成后清理进程。也可单跑一个情景，例如 `P6_SCENARIO=nav_failure P6_EXPECTED_STATE=ABORTED ./tools/p6_offline/run.sh`。

## 安全边界与未解决问题

仿真入口固定 synthetic arm backend、localhost ROS graph 和 P3b 的串口关闭参数；它不能连接真实设备。协调器订阅 P4e `/serial/arm_safety/status`，任意观测到 `FAULT_LATCHED` 即中止，不自动调用 reset 或 enable。当前 P3b 反馈模拟并未启动 P4e 串口主机节点，所以此项是接口监视与故障门控，不是 P4e PTY 验收，也不模拟下位机安全响应。

P4f 已在 ROS 侧为机械臂命令增加任务 ID、epoch 和序号，并验证 catch 取消、P4e 主机拒绝重放与 PTY 串口帧。46 字节协议仍不包含这些标识或 STM32 队列取消/停止确认，因此不能证明真实下游已停止。P6 也没有处理真实机械臂安全复位授权、单轴反馈健康、制动/保持和实体急停。

仓库基线的 P5b 串口桥仍是单向发送，没有底盘里程计或诊断闭环；本轮模拟器里程计由运动学模型生成。P6 离线结果不能作为真实机器人执行许可。

## 当前验证记录

Nav2 工作空间 `p6_autonomous_mission` 和机械臂工作空间 6 个依赖包已分别构建通过；P6 自动化单元测试 8/8 通过。六场景运行使用 ROS_DOMAIN_ID=97、`ROS_LOCALHOST_ONLY=1`，P6 与 P3b launch 默认域号一致。运行命令、完整任务耗时、各场景原因和关键日志见 [实际验收记录](ros2-runtime-acceptance.md)。

本次验收仅证明离线 ROS 2 软件闭环在仿真数据上通过。它不是连接真实底盘、相机、雷达或机械臂的硬件验收，也不解除本文件所列 P4e 主机恢复互锁及 P4f STM32 下游停止确认限制。
