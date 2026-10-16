# P8 软件集成与 NUC 隔离部署

> **历史 P8 基线。** 下文命令和路径记录的是原有 P8 验收，不用于新版构建或测试。当前源码位于 `~/lunar-robot/code`；新版操作见[工作区布局](WORKSPACE_LAYOUT.md)和[部署流程](DEPLOYMENT_WORKFLOW.md)。P8 工作区和结果保留在 `~/lunar-robot/archive/p8/`。

## 版本来源与集成

P8 分支以 `p7/full-control-integration` 的 `852aced76170f5435d7084073020a3b755b9f32e` 为基线。MID-360 修复来源为 `codex/mid360-driver-nuc-ready`：驱动修复 `20fef12` 已通过 cherry-pick 集成；NUC 构建记录与静态验收流程来源于 `ea1fc0f`、`189cf2f`，在 P8 中整理为脱敏部署文档。P8 不合并 `main`，不改 P4e/P4f 安全策略或 STM32 协议；P6 的改动只增加 P5b 主机串口状态协调。

代码集成修改了 P5b 桥接器以发布**主机本地串口写状态**，P6 可在运行任务时监视它并快速失败中止。这个状态不是 STM32 回执；离线场景验证的是 P5b 主机进程到 PTY peer 的写失败检测，不证明控制器或电机已经停止。

## NUC 独立部署与 overlay 隔离

部署目录为 `/home/tzb/lunar-cave-p8`，不得在历史工作空间中构建。建议用独立源码树的以下目录：

- `build/p7`、`install/p7`：Nav2、P6、P5b；
- `tzb_ws/build/p7`、`tzb_ws/install/p7`：P3b 规划器、P4e 驱动；
- `build/mid360`、`install/mid360`：Livox ROS 驱动独立依赖链；
- `log/`：构建、P7 场景和点云验收记录。

只读审计确认 NUC 历史 `tzb_ws` 与 `Nav2_rea_ws` 都含有 `rm_serial_driver`、`rm_serial_msgs` 重名包。P8 集成源码中，P5b 已用 `chassis_serial_bridge` 取代 Nav2 侧旧底盘 `rm_serial_driver`；当前 P8 源码只在 `rm_serial_msgs` 上重名，机械臂 `rm_serial_driver` 由 `tzb_ws` 提供。不得将两个历史工作空间或两个源码树放入同一个 colcon source 空间，也不得在同一 ROS 进程中无审查地同时 source 冲突 overlay。离线启动器分别启动进程，各进程只加载所需 overlay。

构建整机离线软件：

```bash
source /opt/ros/humble/setup.bash
cd /home/tzb/lunar-cave-p8/Nav2_rea_ws
colcon --log-base log/p7 build --base-paths src \
  --packages-up-to p6_autonomous_mission chassis_serial_bridge \
  --build-base build/p7 --install-base install/p7 --parallel-workers 4

source /opt/ros/humble/setup.bash
cd /home/tzb/lunar-cave-p8/tzb_ws
colcon --log-base log/p7 build --base-paths src \
  --packages-up-to lite_motion_planner rm_serial_driver \
  --packages-skip orbbec_camera --build-base build/p7 --install-base install/p7 \
  --parallel-workers 4
```

独立构建 MID-360 驱动见 [静态点云与 IMU 验收流程](MID360_STATIC_ACCEPTANCE.md)。Point-LIO、Fast-LIO、相机与地图构建链不是编译或运行 P7 离线任务的前置条件。

## 离线运行安全约束

P7 使用 `tools/p7_offline/run.sh`，强制 `ROS_LOCALHOST_ONLY=1` 和隔离 ROS Domain；启动前检查运行目录中新建的 `/dev/pts/N` 符号链接，只将这些 PTY 传给 P5b/P4e 驱动。它拒绝外部串口路径；不做设备自动发现，不启动 Livox、相机、NUC、STM32 或真实 bringup。P6 在 P7 模式强制检查 P4e 安全状态、关节反馈和 P5b 主机链路状态。

完整离线场景运行命令：

```bash
cd /home/tzb/lunar-cave-p8
P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator \
P7_RESULTS_DIR=/home/tzb/lunar-cave-p8/log/p7-e2e \
./tools/p7_offline/validate_scenarios.sh
```

只运行一次完整正常任务（导航、抓取、返航和卸载）：

```bash
cd /home/tzb/lunar-cave-p8
source /opt/ros/humble/setup.bash
P7_ROS_DOMAIN_ID=76 \
P7_SCENARIO=normal \
P7_EXPECTED_STATE=SUCCEEDED \
P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator \
P7_LOG_DIR=/home/tzb/lunar-cave-p8/log/p7-normal \
./tools/p7_offline/run.sh
```

只有看到运行日志中的 `PASS serial isolation: only runtime-created PTYs are configured` 后，才继续离线场景。P4e 故障恢复需要操作者动作；P7 仅有明确记录的**模拟操作者使能**，不自动 reset。ROS 取消确认表示上游停止发布，不能解释为 STM32 停止确认或急停。

## 主机侧底盘链路状态语义

P5b 发布 `/chassis/serial_link_status`：

- `HOST_WRITE_OK`：当前主机进程持有打开的 TTY，并且最近一个 14 字节帧完整写入了本机 TTY 文件描述符；没有控制器接收回执。
- `DISCONNECTED`：设备路径为空、TTY 打开失败、写失败或 peer 断开后主机侧不可写。
- P6 可要求状态持续新鲜；状态丢失、过期或非健康值会取消 Nav2 Action、停止任务并进入 `ABORTED`，不会等待 Nav2 的整段重试超时。

这个改进缩短已检测到的 PTY/主机串口断开响应时间。USB-串口芯片仍在但 MCU 断电、TX 线断开、UART 丢帧或 MCU 忙死时，OS 写调用可能继续成功；因此它不是物理控制器健康监测，也不能证明底盘刹停。

## 验收记录

P8 实际构建、离线运行结果、ROS 包/TF/Action 检查和测试日志位置记录在 [P8 验收报告](P8_VALIDATION_REPORT.md)。没有现场点云/IMU 运行结果时，不得将静态数据流或 SDK 构建记为 MID-360 实机验收通过。
