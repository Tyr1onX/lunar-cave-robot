# P9-F：机械臂安全输出集成与主机验证

更新日期：2026-10-13。P9-F 分支 `p9f/arm-safety-integration` 从 PR #8 的代码 `c441e8dd2f666dfdca000f3b4e94826f14204209` 和最新 `main` `b1cfe617209a06d69000a515af82b9b95add62ef` 建立。PR #8 保持开放；本分支不合并到 `main`。

## 本轮代码修改

- `Motor_DM::Ctrl()` 的 MIT 输出和恢复命令都经过 `arm_output_gate`。输出门在一个 FreeRTOS 临界区内复核当前遥控在线/保持状态、控制模式、Host 租约、七轴反馈新鲜度及驱动器故障、FDCAN bus-off、发送失败、接收失败和 FDCAN 错误事件，再把命令加入 CAN TX 队列。模式或授权失效后，输出门立即锁闭；状态任务下一轮采样不再是控制输出的唯一保护。
- Host 租约在每次轮询和接受新帧前检查。超过 100 ms 后会锁存过期，后续 enable 帧不能刷新旧租约；只有安全握手重置后才能重新建立 disabled→enabled 流程。100 ms 本身仍在租约内，101 ms 已过期。
- `ArmResume()` 在每条清错/使能命令前后、分组等待后、软启动后和恢复控制任务前检查授权。清错必须等待七轴各自的新 CAN 反馈；使能后必须收到每轴报告 enabled 的新反馈。失联、模式变化、驱动故障、CAN 异常或应答超时会中止恢复、锁住输出并保持控制任务挂起。
- `Joint::ControlStep()` 和 `Gripper_Update()` 不再在未初始化时自动发送 enable。所有 MIT 输出统一经过 `Motor_DM::Ctrl()` 的输出门。FDCAN RX 读取失败或接收长度不是 8 字节时会计数并丢弃该次接收，不能使用旧帧头/负载更新关节反馈。
- 每轴新鲜度使用最近一次真实 CAN 反馈的 HAL tick 与驱动状态位判断；未知/故障状态不算健康。没有合成传感器反馈或推测应答。

## ThinkBook 离线结果

验证机器为 ThinkBook `qibai-ThinkBook-16-G6-AHP`，Ubuntu 22.04，GCC 11.4。测试用本轮工作树的隔离文件副本及 `/tmp` 构建目录，未改动 ThinkBook 原有项目工作区。

| 验证项 | 结果 | 实际覆盖 |
| --- | --- | --- |
| 底盘协议解析器 | PASS | 编译生产解析/命令代码，mock HAL |
| 底盘安全状态机 | PASS | 启动锁定、链路丢失锁存、人工复位和模式切换 |
| 机械臂纯 C 状态机 | PASS | 安全输入和故障锁存 |
| 机械臂 Host/恢复集成 | PASS | 真实 Host 帧解析、100/101 ms 租约边界、超时后旧 enable、及时续租、重复 enable、TX 失败、CAN bus-off/warning、RX 读取失败/短帧、单轴驱动故障、清错中断恢复、分批使能中断恢复和成功恢复 |
| 生产遥控解析到关节输出集成 | PASS | 同一测试可执行文件链接实际 `Remote.c` UART 回调、输出门和 `roboarm.cpp`；通过有效帧驱动 S2 模式切换，切换后和两次 `ControlStep()` 之间切出再切回都阻止 MIT 输出；100 ms 反馈边界可用、101 ms 过期，单轴故障关闭全臂运动输出 |
| 生产遥控选择器代数 | PASS | 另一个主机用例验证相同选择器帧不改变代数、切走再切回累积两次变化、S1 组选择变化同样使旧授权失效 |
| ROS `lite_motion_planner` | PASS | ThinkBook ROS 2 Humble 隔离构建 `arm2nav_msgs`、`lite_motion_msgs` 与 `lite_motion_planner`，共 3 个包 |
| 完整机械臂 STM32 镜像 | NOT BUILT | 缺少真实 XRobot 产品入口和生成输入，见 [XRobot 构建恢复记录](XROBOT_ARM_BUILD_RECOVERY.md) |
| P7/P8 12 场景全套 | 本轮未运行 | 最近完整通过仍为历史代码 SHA `07a56c3a33882aa2866f1076d8597776e2579bcb` 的 12/12 记录；P9-F 不声称本轮复跑通过 |

ThinkBook 上按工作流的相同命令序列复跑：底盘协议解析器、底盘 FSM、机械臂 FSM、机械臂 Host/恢复集成、生产 `Remote.c` 到实际 `Joint::ControlStep()` 的输出门集成、选择器代数测试全部 PASS。机械臂集成脚本另连续运行 3 次，三次均通过。脚本在 Linux GCC 11.4 下以 `-Wall -Wextra -Werror -pedantic` 编译，没有缺失文件时跳过测试的路径。

Host/恢复测试编译实际 `Host.cpp`、`task_error.cpp`、`Motor_DM`、`Bsp_CAN`、CAN 回调和输出门；USB、RTOS、HAL 时钟与调度点由测试桩控制。另一测试将生产 `Remote.c` 的 UART 帧解析、输出门、`Motor_DM` 和生产 `Joint::ControlStep()` 连接在同一进程中。模式授权比较遥控帧解析出的 S1/S2 代数与当前模式，能捕获两个控制周期之间发生的“切出再切回”。源码配置将 UART5 和 FDCAN3 中断优先级都设为 5，`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 也为 5，因此这些中断处于 FreeRTOS task critical 的屏蔽范围。主机测试仍不能测量 MCU 实际临界区时长、总线时延、HAL/DMA 异常和真实调度抖动。

测试可以确认软件在所模拟的 CAN 状态帧上拒绝单轴故障和过期反馈，但仍需硬件负责人确认达妙反馈状态字段和 enable/clear 命令的应答时序是否与实际电机型号、配置一致。错误回调记录 warning/passive/bus-off 事件，TX/RX HAL 操作失败也会锁存；现有实现不根据原始 TEC/REC 数值猜测硬件阈值。

## PR #8 的剩余审查项

软件输出门和恢复路径的主机级覆盖已补齐，但 PR #8 仍应保持开放，等待以下材料与审查：

1. XRobot 对应产品的真实生成配置、`User/xrobot_main.hpp`、`Modules/CMakeLists.txt` 和匹配版本的模块源码，使机械臂目标工程能在不伪造文件的条件下完整构建。
2. 硬件负责人确认七轴电机/驱动器型号、反馈状态与应答语义、实际固件版本、失能后的重力负载保持/下坠行为、制动方式和独立急停接线。
3. 对新增固件控制行为进行人工代码审查，并在确认负载保持策略后另行制定无动力接口核对和受控联调方案。P9-F 没有改变真实失能、制动或重力保持策略。

本轮没有连接控制器、打开 STM32 串口、烧录固件、启动电机或执行真实机器人运动。离线 PASS 仅证明软件逻辑在模拟接口条件下按断言工作，不代表实体停止、保持或急停能力。

## 轨迹生成器描述更正

ROS `TrajectoryGenerator` 对相邻关节路点使用 `max(|Δq|/vmax, control_period)` 作为时间间隔，并在路点写入零速度和零加速度；源码没有调用 Ruckig，也没有施加加速度/jerk 约束。固件使用 `route_planning.hpp` 的三次位置规划和 `velocity_planning.hpp` 的三段恒 jerk 速度规划，两者均不使用 Ruckig。规划器状态见 [技术总览](TECHNICAL_OVERVIEW.md)。
