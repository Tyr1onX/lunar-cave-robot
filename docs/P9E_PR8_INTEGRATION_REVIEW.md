# P9-E：PR #8 安全状态机接入审查

> 本文记录 P9-E 对当时 PR #8 快照的审查发现。P9-F 后续分支已针对机械臂输出轮询窗口、Host 租约过期、恢复序列复核、逐轴反馈健康门和自动 enable 路径提交软件修复及主机集成测试。当前分支状态、实际测试范围和仍需硬件负责人确认的事项见 [P9-F 集成记录](P9F_ARM_SAFETY_INTEGRATION.md)。PR #8 仍保持开放，以下条目保留为历史审查依据。

审查基线：仓库 main **6a48dc52f5d7f5b263845bdec0639f59144a4486**。
审查对象：PR #8 head **c441e8dd2f666dfdca000f3b4e94826f14204209**。
截至 2026-10-13，PR #8 仍开放且可合并；本审查不修改其控制源码，也不合并该 PR。

## 接入情况

### 六轮底盘

底盘状态机已经从纯函数接入实际 RTOS 和驱动任务：

- **firmware/chassis/Core/Src/freertos.c** 的 StartDefaultTask() 每 10 ms 读取 SBUS、串口命令新鲜度、零位状态和 UART 故障，调用 chassis_safety_update()，再将 ARMED 状态写入 chassis_command_set_safety_enabled()。
- **firmware/chassis/User/APP/chassis_safety.c** 把上电状态初始化为 CHASSIS_SAFETY_BOOT_LOCKED。遥控接收离线或串口错误会锁存故障；必须回到禁用档并保持摇杆中位才能回到 DISARMED。随后选择 SBUS，或等待新鲜且为零的串口命令，才进入 ARMED。
- **firmware/chassis/User/APP/chassis_task.c** 的 Chassis_init() 只初始化电机 ID。只有收到已授权且未过期的启用命令，底盘任务才发送驱动器 enable。其余情况清空目标并周期发送 hm_motor_send_relax()。

因此，“底盘上电初始化阶段直接使能”已在软件调用路径上移除；启动不会单靠收到遥控器恢复信号自动重新使能。仍未证明 relax 的物理效果、驱动器在上电时的默认输出、CAN 总线健康监控和实际制动距离。

### 机械臂

机械臂状态机也接入 Error_tasks()、Host_Poll()、遥控恢复手势及线程挂起/恢复：

- **firmware/arm/others/src/app/arm_safety_state.c** 上电进入 BOOT_LOCKED；遥控器离线或保持请求会进入 FAULT_LATCHED。Host 模式还要求安全复位后的 disabled→enabled 握手及新鲜、有限的目标。
- **firmware/arm/others/src/app/Host.cpp** 收到 enable=0 帧会清除命令新鲜标志并产生停止事件；取消或链路超时后状态机拒绝继续采纳已缓存目标。
- 故障处理挂起 roboARM_task、保存保持位置、发送关节 disable；恢复需要遥控器的中位摇杆人工手势。Host 模式还必须重新完成协议握手。
- 以上是源码层的控制流程。机械臂关节失能是否导致下坠、disable 是否关闭力矩、保持位置是否可承载，均未经硬件确认。

## P9-E 在该历史快照发现的控制时序缺口

以下结论来自对 PR #8 实际 RTOS 调用路径的静态审查。现有主机测试只直接运行纯 C 状态机和底盘协议解析器；它没有编译或模拟 USB Host 任务、机械臂 FreeRTOS 调度、FDCAN 驱动器反馈或电机物理响应。

1. **机械臂模式切换有轮询窗口。** roboARM_task() 以约 2 ms 周期调用 Remote_UpdateJointVelocity() 并继续更新参考和发送控制；安全状态由 Error_tasks() 每 10 ms 检查。roboARM_task()/Joint::ControlStep() 没有在每个控制周期读取安全状态或模式代次。因此 Host→遥控切换后，在 Error 任务锁存并挂起之前，控制任务仍可能发送一到数拍旧控制通道对应的输出。需把安全许可检查放进控制输出路径，或提供可证明无窗口的原子停用机制，并增加模式切换边界调度测试。

2. **过期 Host 租约可能被下一帧续租。** Host_Poll() 在接受 enabled 帧时先更新 host_command_tick；状态机独立地每 10 ms 检查 Host_IsCommandFresh()。如果旧租约已经超过 100 ms，但新的 enabled 帧在下一次检查之前到达，Host 任务可能先把时戳刷新，安全状态机就看不到“过期后曾经失联”的状态，故障未锁存。应在刷新租约前同步判断旧租约是否已失效并锁存；测试需覆盖刚过期、轮询间隙内重连和恰在边界到达。

3. **恢复序列期间没有重新核验输入。** ArmResume() 包含多个 1 ms 等待，执行清错、软启动、七轴 enable，最后恢复控制任务；它没有在每一批 enable 之前重新确认遥控器在线、急停/保持状态、控制模式、Host 租约和反馈健康。输入在状态机采样后发生变化时，恢复序列仍会继续发送 enable，直到下一轮 10 ms 检查。应使恢复序列可中断，并在每次 enable 前复核授权条件；加入分批 enable 中注入失联/急停的主机测试。

4. **关节反馈只检查“曾经初始化”，没有新鲜度或健康门。** 当前 inputs.feedback_initialized 是七个 Joint::IsInited() 的逻辑与。没有逐关节反馈年龄、驱动器错误状态、FDCAN bus-off/TX 失败或使能确认。需要定义每个关节的反馈新鲜度与驱动器故障输入，并规定不健康反馈时禁止恢复。不得把首次反馈标志当作长期健康证明。

5. **输出端仍有局部自动 enable 分支。** Joint::ControlStep() 在 !inited 时会直接发送 enable；Gripper_Update() 对未初始化的 j7 也会重复发送 enable。启动任务目前先初始化 CAN，Error 任务启动时先挂起主控制任务，现有线程顺序可能避免初始输出；但这两处没有检查安全状态，依赖调度和启动顺序。恢复前需将所有驱动器 enable 命令集中到经状态机许可的函数，并测试启动顺序和单轴反馈缺失。

6. **底盘安全许可没有覆盖反馈故障。** 状态机输入有遥控在线、串口租约/错误和控制零位，但未见六路电机反馈超时、驱动器错误、单侧 FDCAN 失效或电源异常输入。需由硬件负责人确定设备可报告的故障信号，再定义软件故障策略。

## P9-E 当时结论（历史记录）

PR #8 的底盘软件集成主体清楚：上电不会从 Chassis_init() 直接使能，安全门控连接到了命令入口和底盘电机任务。机械臂故障锁存和人工恢复也已经串入任务调度和 Host 协议处理，但授权没有在每次实际控制输出处闭环，且有轮询与恢复竞态。

**建议 PR #8 保持开放。** 最小的软件阻塞项是修复机械臂过期租约续新、模式切换窗口和恢复期间重新授权，并让所有关节 enable 经过安全门；随后补齐集成主机测试，覆盖 USB 帧→Host 状态→控制输出以及每个恢复边界。逐关节健康门也应在可安全定义后加入。

> P9-F 后续已在独立分支落实上述机械臂软件修复及集成主机测试。P9-F 尚未合并，因此这份历史记录仍准确描述当时 PR #8 head；当前待审分支与未解决硬件条件见 [P9-F 集成记录](P9F_ARM_SAFETY_INTEGRATION.md)。

合并控制器行为修改前，还必须取得硬件负责人对电机失能/制动、机械臂重力负载支撑和独立急停的书面确认。完整机械臂固件编译仍受 XRobot 生成输入缺失阻塞，恢复条件记录在 [XRobot 构建恢复记录](XROBOT_ARM_BUILD_RECOVERY.md)。离线状态机通过不能证明真实停止、保持或急停能力。
