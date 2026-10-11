# P6：离线整机自主采样软件闭环

基线是 `p4e/arm-host-safety`，提交 `ef62b84589e77c136e97dc7f68d234a1c489d498`。P6 代码位于独立分支 `p6/autonomous-mission-offline`，不合并 `main`。

## 离线启动

在 Ubuntu 22.04 / ROS 2 Humble 的完整仓库 checkout 根目录运行：

```bash
./tools/p6_offline/run.sh
```

脚本分别构建 `Nav2_rea_ws` 与 `tzb_ws` 到各自的 `install/p6`，再由两个独立子进程分别 source 对应工作空间。它设置 `ROS_LOCALHOST_ONLY=1` 与专用 `ROS_DOMAIN_ID`，启动 P3b 时固定 `use_serial_driver=false`，没有串口、雷达、相机或电机驱动节点。脚本在其他平台不会尝试接触设备，但 ROS 2 / Nav2 验证需要 Ubuntu Humble。

任务状态和 JSONL 事件记录在 `/tmp/p6_autonomous_mission_events.jsonl`。Nav2 与机械臂启动日志分别写入 `/tmp/p6-autonomous-mission/nav2-mission.log` 和 `arm.log`。正常流程是：

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

- `nav_failure` 将目标放到地图之外，期望 Nav2 返回失败、重试一次后任务中止。
- `target_loss` 丢弃视觉位姿，期望检测超时中止且不触发机械臂。
- `grasp_failure` 发出匹配 task id 的取消请求，期望机械臂报告失败、机器人不返航并且任务不成功。
- `communication_loss` 丢弃 arm task 请求，期望等待结果超时并中止。
- `duplicate_trigger` 走正常任务路径，由验证监视器重复发布任务触发并确认只接受一次。
- `normal` 执行完整目标采样、返航和卸载流程。

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

P4f 的在途轨迹 epoch、下游旧消息清空/取消确认仍未解决。P6 取消只停止 `tzb_catch` 后续任务发布并把离线任务置失败；`JointState` 本身没有 task UUID，不能证明真实串口桥或 STM32 已排空在途命令。P6 也没有处理真实机械臂安全复位授权、单轴反馈健康、制动/保持和实体急停。

仓库基线的 P5b 串口桥仍是单向发送，没有底盘里程计或诊断闭环；本轮模拟器里程计由运动学模型生成。P6 离线结果不能作为真实机器人执行许可。

## 当前验证记录

本实现环境是 macOS，不包含 ROS 2 Humble、`colcon` 或 Nav2。因此没有声称完整 ROS 节点、规划器、机械臂 C++ 包或 ROS 故障注入已经在 ThinkBook 上跑通。P6 单元逻辑与静态隔离检查可在当前环境运行；完整构建及 ROS runtime 验收须由 ThinkBook Ubuntu 22.04 按上述命令记录结果后确认。
