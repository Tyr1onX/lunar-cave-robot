# P8 实际构建与离线集成验收报告

**日期：** 2026-10-12

**代码分支：** `p8/nuc-integration-ready`，源自 P7 `852aced76170f5435d7084073020a3b755b9f32e`

**验收主机：** ThinkBook Ubuntu 22.04 / ROS 2 Humble；NUC Ubuntu 22.04.5 / ROS 2 Humble
**硬件边界：** 本次只运行软件节点和临时 PTY；未连接或打开真实运动硬件、MID-360 或物理串口。

## 结论摘要

P8 的 **P7 整机任务级离线闭环已在两台主机上通过 ROS 2 节点与 PTY 验收**。Nav2 使用真实的 SmacPlanner2D 规划器和 DWB 控制器；速度经过 velocity smoother、P5b 桥接器和 14 字节 PTY 帧后驱动模拟底盘，再由模拟里程计、TF、地图和激光扫描反馈。P3b/P4e/P4f 规划及安全 ROS 节点经 46 字节 PTY 与模拟控制器交互，关节反馈由解码后的控制指令推进。P6 收到导航 Action 结果、停车/位姿确认、机械臂执行结果及卸载结果后才进入 `SUCCEEDED`。

这证明 ROS 软件和模拟控制链闭合，不证明真实 MCU、制动器、电机、机械臂负载或传感器安全。P5b 的 `HOST_WRITE_OK` 只说明主机成功写入本机 TTY；ROS 取消确认也不代表 STM32 已停止。

## 软件来源和隔离

- P8 基于 P7 `852aced`；MID-360 SDK2 修复来自 `codex/mid360-driver-nuc-ready` 的 `20fef12`。P7 与 MID360 源分支保持独立，未合并 `main`。
- ThinkBook 验证源码位于 `/home/qibai/lunar-cave-p8-validation`；NUC 验证/部署源码位于 `/home/tzb/lunar-cave-p8`。NUC 历史工作空间 `tzb_ws`、`Nav2_rea_ws`、`nav_ws`、`mid360_driver_dev` 未修改或构建。
- Nav2/P6/P5b、机械臂规划器/P4e 驱动、MID-360 驱动分开构建到各自 install 前缀，未将两个重名 ROS 工作空间叠加到同一 colcon source space。ROS 节点按进程加载所需 overlay。
- P7 runner 强制 `ROS_LOCALHOST_ONLY=1`，使用各次运行新建的 ROS Domain 和程序创建的 `/dev/pts/N`。运行前静态扫描确认 P7 启动脚本没有物理串口自动发现或 `/dev/ttyACM*`、`/dev/ttyUSB*` 配置入口；`/dev/ttyACM0` 只做只读占用检查，未有本次测试进程打开。

## 构建和 MID-360 检查

| 项目 | ThinkBook | NUC |
|---|---|---|
| P6/P5b 构建 | 通过，2 个目标包 | 通过，2 个目标包 |
| P3b/P4e 规划器与驱动构建 | 通过，6 个目标包 | 通过，6 个目标包 |
| MID-360 ROS 驱动构建 | 未通过：本机缺少 Livox SDK2 共享库 `/usr/local/lib/liblivox_lidar_sdk_shared.so`；未安装系统包 | 通过，1 个包；安装后 `ldd` 依赖均解析，launch 参数检查确认 `config_file` 必填 |
| P5b 静态配置检查 | 通过 | 通过 |

NUC 上仅构建并检查了 MID-360 驱动，没有启动驱动或打开传感器 UDP 端口，也没有真实点云/IMU 数据验收。NUC 当前 `enp86s0` 为 `DOWN`，默认路由经 Wi-Fi `wlo1`；现场路由、MID-360 地址、UDP 方向和防火墙后端均未核实。因此，下一次实验室会话**不能直接开始静态点云/IMU验收**：应由现场负责人先确认设备与有线链路、当前地址和路由、UDP/防火墙规则，再按 [MID-360 静态验收流程](MID360_STATIC_ACCEPTANCE.md) 单独启动驱动。

## P7 ROS 2 端到端离线场景

完整场景命令：

```bash
cd /home/tzb/lunar-cave-p8
source /opt/ros/humble/setup.bash
P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator \
P7_RESULTS_DIR=/home/tzb/lunar-cave-p8/log/p7-e2e \
./tools/p7_offline/validate_scenarios.sh
```

ThinkBook 和 NUC 的 12 个独立场景均有一次通过结果。下表列出 NUC 首轮 11 个场景以及修正验收脚本后的桥接停止重测；ThinkBook 的桥接停止场景亦使用修正脚本单独通过。耗时从任务控制器开始记录到终态，单位为秒。

| 场景 | NUC 结果 / 耗时 | ThinkBook 结果 / 耗时 | 验证结果 |
|---|---:|---:|---|
| 正常采样与返航卸载 | `SUCCEEDED` / 95.498 | `SUCCEEDED` / 95.398 | 目标和基地导航 Action 成功，停车确认，抓取、返航、卸载均有结果确认 |
| Nav2 失败 | `ABORTED` / 73.653 | `ABORTED` / 75.696 | 失败后未发机械臂请求 |
| 目标丢失 | `ABORTED` / 51.584 | `ABORTED` / 50.877 | 未发机械臂请求 |
| 抓取失败 | `ABORTED` / 37.050 | `ABORTED` / 37.957 | P4f 取消确认；无成功状态 |
| 机械臂通信中断 | `ABORTED` / 43.181 | `ABORTED` / 42.810 | 取消确认并中止 |
| 重复任务触发 | `SUCCEEDED` / 109.617 | `SUCCEEDED` / 98.005 | 一次触发被接受，多余触发被记录并忽略，只产生一个目标导航任务 |
| 机械臂反馈停更 | `ABORTED` / 37.451 | `ABORTED` / 37.423 | P4e 故障锁存传递至 P6 |
| 机械臂串口断开 | `ABORTED` / 38.064 | `ABORTED` / 38.017 | 断开/状态过期时中止并确认上游取消 |
| P4e 驱动重启 | `ABORTED` / 39.653 | `ABORTED` / 41.163 | 重启保持 `DISARMED`，未自动 reset/enable，旧任务取消 |
| 底盘 PTY 断开 | `ABORTED` / 2.739 | `ABORTED` / 1.721 | P6 故障检测延迟分别 0.010s、0.012s |
| 抓取后返航时底盘断开 | `ABORTED` / 50.005 | `ABORTED` / 51.256 | 先有已确认抓取，返航链路失败后取消并中止；检测延迟 0.016s、0.014s |
| P5b 桥接进程停止 | `ABORTED` / 2.995 | `ABORTED` / 2.466 | 通过主机状态过期发现；检测延迟 0.531s、0.503s |

NUC 正常任务的 PTY 统计包括 5,378 个底盘 14 字节帧（3,695 个非零运动帧）、5,952 个机械臂使能命令帧、4,958 个反馈帧、1,474 次关节值变化；未观察到无效或重复机械臂帧。运行图记录 `/navigate_to_pose` Action、Nav2 规划/控制组件、`/cmd_vel` 链路、`/odom`、`/tf` 与 P6 事件；`tf2_echo` 确认 `map → base_link` 可查询。正常结果日志含两次导航成功、两次 `ARRIVAL_AND_STOP_CONFIRMED`、`SAMPLE_PICK_CONFIRMED`、`P4E_EXECUTION_CONFIRMED` 和 `BASE_UNLOAD_CONFIRMED`。

测试记录了可复现性问题，但没有放宽验收条件：ThinkBook 的抓取失败场景首轮遇到 Nav2 planner lifecycle `change_state` 响应超时，任务停在启动阶段并安全超时中止；用新 Domain 单独重跑后通过。初次 NUC 12 场景聚合运行中，P6 已正确中止 `chassis_bridge_stop`，但旧测试副本没有把该 fault 纳入检测延迟断言，聚合命令以测试脚本错误失败；同步修正后的脚本在 NUC、ThinkBook 单场景重跑均通过。P5b 测试的恢复断言一度偶发失败：桥接日志显示 20ms 内已重开并写帧；根因是断言拿断线前的第一个 `HOST_WRITE_OK` 与 `DISCONNECTED` 比较。修正后仅接受断线状态之后的新健康回调，并加长观察窗口；NUC 连续两次、ThinkBook 一次完整 PTY 测试通过。首轮失败与修复后的日志均保留在本机归档。

## 回归验证

P5b 在两台主机上均完成 ROS 2 `velocity_smoother → chassis_serial_bridge → PTY` 运行检查：正反向、转向、停止、限幅、无效数值归零、100ms 过期命令归零、断线检测、替换 PTY 后以零速恢复、新鲜命令恢复和正常关闭最终零速。ThinkBook 修复后的执行输出为 `RESULT=PASS_ROS2_PTY_INTEGRATION`；NUC 修复后的同一测试连续两次通过。这只覆盖主机到 PTY 行为。

ThinkBook 执行 `tools/p4f_offline/validate.sh` 完成回归：

- P4e 驱动 ROS 2/PTy 安全注入通过，包括启动 `DISARMED`、命令租约故障锁存、非法/过期/未来/非有限指令拒绝、反馈超时、取消、旧 epoch 重放拒绝、PTY 重连锁存及 ROS 驱动重启后不自动恢复。
- 8 项 P6 单元/静态契约测试全部通过。
- 真实规划器计算结果与取消竞争通过；取消确认后没有继续发布旧轨迹，较高 epoch 的新任务可接管。
- P6 六场景回归通过：`normal`、`duplicate_trigger` 为 `SUCCEEDED`；`nav_failure`、`target_loss`、`grasp_failure`、`communication_loss` 为 `ABORTED`。

此 P4f 回归使用 P3b 合成机械臂反馈；P7 的整机闭环则另行验证了通过 P4e PTY 驱动的模拟控制器反馈。

## 日志位置

- ThinkBook：`/home/qibai/lunar-cave-p8-validation/log/p8/`
- NUC：`/home/tzb/lunar-cave-p8/log/`
- 原先保存在项目外的 P7/P4f/P5b 验收日志和本次 P8 的 P7、P4f、P5b 日志，已备份到本机 `work/p8-validation-archive/2026-10-12/`；未将大体积运行产物提交进代码分支。

## 面向真实机器人尚未验证的部分

P7 仿真目标检测使用合成数据；地图、扫描、里程计、TF 和底盘运动由模拟器生成；抓取成功依据 P4e/PTy 模拟控制器返回的关节反馈、命令帧和夹爪帧确认。MID-360、本地化、实机相机、真实抓取坐标及任务规划未组成物理闭环。

真实运动前的阻塞项、控制器需要配合的停车/取消确认，以及从传感器静态采集到受控运动的顺序，见 [实体机器人安全门槛](REAL_ROBOT_SAFETY_GATES.md)。在 STM32 看门狗、机械臂取消与序号防重放、急停/制动、限位负载、人工恢复互锁及传感器标定验证完成前，不能宣称允许真实任务执行。
