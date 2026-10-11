# P6 ROS 2 实际运行验收记录

验收日期：2026-10-11  
主机：ThinkBook 16 G6 AHP，Ubuntu 22.04 x86_64  
ROS：ROS 2 Humble  
工作目录：`/home/qibai/lunar-cave-robot-p6-acceptance`  
分支：`p6/autonomous-mission-offline`，起始树与 P6 分支一致

## 隔离与启动

从工作目录直接执行：

```bash
./tools/p6_offline/run.sh
```

入口默认域号为 97；校验范围为 0–101。P6 Nav2 launch 与 P3b offline launch 默认值也都为 97。入口设置 `ROS_LOCALHOST_ONLY=1`，显式给两个 launch 传入相同域号，分别 source 两个工作空间的 `install/p6`。没有叠加 Nav2 与机械臂 overlay。

在验收初期发现 ThinkBook 上已有其他本地 ROS 进程使用域号 42。localhost-only 不隔离同机同域的节点，故丢弃了域号 42 下的混合 graph 运行记录，并将默认值统一改为当时空闲的 97。最终验收期间域 97 的节点清单只有 P6、Nav2 和 P3b 功能仿真节点；没有启动串口桥、`rm_serial_driver`、电机、相机或雷达驱动。运行结束后域 97 无 P6 节点残留。

## 构建与自动化测试

`run.sh` 实际构建两个独立工作空间：

- `Nav2_rea_ws`：`p6_autonomous_mission` 构建成功。
- `tzb_ws`：6 个包构建成功：`rm_serial_msgs`、`arm2nav_msgs`、`lite_motion_msgs`、`tzb_description`、`rm_serial_driver`、`lite_motion_planner`。

之后运行 `./tools/p6_offline/run_unit_tests.sh`：8 项通过，0 项失败。测试覆盖终态、重复触发、抓取失败不得返航/卸载、导航状态转移、显式故障场景和 offline 启动隔离约束。

六个 ROS 场景由以下命令逐个完整启动；脚本仅在所有事件级断言通过时返回 0：

```bash
./tools/p6_offline/validate_scenarios.sh
```

最终实际结果：

| 场景 | 终态 | 实际事件断言 | 从接受触发到终态 |
| --- | --- | --- | ---: |
| `normal` | `SUCCEEDED` | Nav2 往返成功；到站停车；目标确认；机械臂 pick、unload 成功 | 96.951 秒 |
| `nav_failure` | `ABORTED` | Smac 在地图占用区两次规划失败；Nav2 Action 返回状态 6；只重试一次后中止 | 46.502 秒 |
| `target_loss` | `ABORTED` | Nav2 到站且停车；目标检测超时；没有 arm pick 请求 | 52.100 秒 |
| `grasp_failure` | `ABORTED` | 发送匹配 task id 的取消；收到 pick failure；没有返航目标 | 37.700 秒 |
| `communication_loss` | `ABORTED` | 丢弃 arm 请求；ARM_PICK 通信超时；没有确认抓取 | 41.851 秒 |
| `duplicate_trigger` | `SUCCEEDED` | 只接受 1 次任务触发，记录 3 次忽略；随后完整采样、返航、卸载 | 119.601 秒 |

另用默认入口单独运行一次正常任务，触发至成功用时 **93.301 秒**。完整证据位于 ThinkBook：

- 六场景：`/tmp/p6-ros-acceptance/scenarios-isolated/<scenario>/`
- 单独正常任务：`/tmp/p6-ros-acceptance/normal-observed/`
- 每个目录含 `arm.log`、`nav2-mission.log`、`scenario-monitor.log` 和 `mission-events.jsonl`。

前期验收发现并修复了默认域号、Humble local costmap 宽高参数类型、`tzb_description` 执行依赖、ROS 环境 setup 与 `set -u` 冲突、launch 清理进程组、Nav2 lifecycle 未激活时过早发 goal、导航失败目标仍可达、场景监视器仅看终态及重复触发发布时机等问题。早期失败记录仍保存在 ThinkBook 的 `/tmp/p6-ros-acceptance/scenarios`、`scenarios-rerun1` 和 `scenarios-rerun2`；最终通过只以 `scenarios-isolated` 为准。

## ROS 运行证据

在域 97 的正常任务运行期间现场检查：

- `/navigate_to_pose` 的唯一 Action server 为 Nav2 `/bt_navigator`；Action clients 包含 P6 `/p6_mission_coordinator`。
- ROS topics 中实际存在 `/map`、`/scan`、`/odom`、`/tf`、`/tf_static`、`/cmd_vel`、`/p6/mission_state`、`/p6/arm_task/request` 和 `/p6/arm_task/result`。
- 现场 `/cmd_vel` 采样为线速度 0.35 m/s、角速度 -0.042 rad/s；同期 `/odom` 位置约为 (5.593, 5.982)，后续 TF 查询得到 `map → base_link` 从约 (5.600, 5.983) 移动到 (5.948, 6.007)。`/scan` 的 frame 为 `base_link`。
- Nav2 日志记录从 (1,1) 导航至 (10,6.5)，随后从约 (9.86,6.41) 返航至 (1,1)；两次 `bt_navigator` 都记录 `Goal succeeded`。任务日志分别确认目标点误差 0.168 m、基地误差 0.101 m，并在每处确认静止。
- 正常任务事件时间为 1791724627.480（接受触发）至 1791724720.781（`SUCCEEDED`）。视觉连续确认 3 个带 `camera_color_optical_frame` 的合成目标位姿。机械臂结果按 task id 和 stage 收到两次 success。
- `planning_node` 输出 IK 与 RRTConnect 解和轨迹播放；`tzb_catch` 将规划场景物体 `E3` 附着到 `tcp_link`，使用模拟关节反馈完成 pick；返航后规划 place 轨迹、分离 `E3` 并完成卸载。

## 算法与模拟数据边界

使用的真实 ROS 2 算法/节点是 Humble Nav2 `NavigateToPose`、SmacPlanner2D 全局规划器、DWB 局部控制器、Nav2 costmap / velocity smoother，以及 P3b 的 `lite_motion_planner`、IK/碰撞检查和 OMPL RRTConnect 规划代码。这里的“真实”表示运行了项目实际算法/ROS 节点，不表示它们已经在真实传感器和执行器上校准。

模拟数据包括静态 OccupancyGrid、射线生成的 LaserScan、仿真 odometry 与差速底盘运动学、固定光学坐标系的目标位姿和 synthetic joint feedback。机械臂夹爪与样本附着状态是规划场景状态，不是实物夹持传感器读数。没有相机图像/YOLO 推理，也没有真实编码器或关节反馈。

## 未解决项与真实机器人限制

- P6 协调器订阅 P4e `/serial/arm_safety/status`，任意观察到 `FAULT_LATCHED` 会中止任务；开启 `p4e_status_required` 时，机械臂阶段还要求新鲜 `ACTIVE_HOST` 状态。但本次 synthetic offline launch 将该必需项关闭，未启动 P4e host，也未执行 P4e fault/reset/recovery 运行验收。真实执行必须在受控硬件验收中确认故障锁存与恢复互锁。
- P4f 轨迹 epoch、旧轨迹取消和下游队列清空确认仍未解决。JointState 没有 task UUID，P6 收到的取消结果不能证明真实串口桥或 STM32 已停止执行旧轨迹。
- P5b 底盘串口桥及其 PTY 测试能力保留在仓库；本次 P6 使用数学运动模型，没有运行 PTY 串口桥，也没有真实底盘里程计闭环。
- 真实相机/雷达标定、关节/夹爪反馈健康、真实通信丢包行为、急停、制动保持和实体恢复授权均未验证。

**验收结论：P6 任务级离线 ROS 2 软件闭环已通过运行验收；这不等于真实机器人可以执行任务。** 在解决 P4e/P4f 真实互锁与轨迹取消问题并完成受控硬件验收前，不应把本次结果当作实体机器人执行许可。
