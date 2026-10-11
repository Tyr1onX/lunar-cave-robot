# P5a：六轮底盘 STM32 下位机源码审计与 ROS 2 导航接口验证

审计日期：2026-10-11  
结论状态：源码审计和离线模型完成；没有连接底盘、打开串口、启动电机或烧录固件。

## 1. 范围与证据

- 固件输入：用户提供的 `DM-car-six-wheel.zip`，只解压并检查 `Core/`、`User/`、`CMakeLists.txt` 和 CubeMX `.ioc` 中的工程源码；没有把压缩包或完整源码上传仓库。
- 协议输入：用户提供的《底盘串口通信协议.md》。协议内容仅作为待核对资料，结论以固件实现为准。
- ROS 2 代码：统一仓库 `Tyr1onX/lunar-cave-robot` 的 `main` 源码快照，基准提交 `b1173df7487d378894961a08a47d6a59aa26aab1`，包括 `tzb_ws/` 与 `Nav2_rea_ws/`。本次通过仓库源码审计，不能证明 ThinkBook 当前运行进程、安装空间或未提交文件与此快照完全一致。
- 独立离线测试：[`tools/p5a_offline/test_chassis_control_path.py`](https://github.com/Tyr1onX/lunar-cave-robot/blob/p5a/chassis-source-audit/tools/p5a_offline/test_chassis_control_path.py)。测试只使用 Python 标准库，不导入 ROS、不打开设备、不声称通过实车验收。

输入 SHA-256：`DM-car-six-wheel.zip` = `823ff6981859dbce1d05a3d4864b921bd0f9aa5165335d0caec7d12852701d2e`；《底盘串口通信协议.md》 = `49a15824329077de0e7730dd86c2348f28409c2e7e71eccdebab2f7beed2cc27`。

固件源码路径在 zip 内；下文使用 zip 相对路径。ROS 路径相对统一仓库根目录。

## 2. 总结结论

**当前底盘还不具备完整、可直接用于整机导航的 ROS 2 运动控制接口。**

已经有的部分是：Nav2 工作空间包含一个可序列化为 14 字节帧的 ROS 2 串口节点；底盘固件也实现了相同头尾和三个 `float32` 字段的接收解析，并能把有效速度命令分配给六个 CAN 电机。

尚不能构成可工作的端到端接口，主要因为：

1. 两个工作空间的串口参数都为 **9600 baud**，固件 USART1 为 **115200 baud**。
2. `Nav2_rea_ws` 的标准导航 bringup 没有启动它自己的串口驱动；其驱动订阅 `/cmd_vel_chassis`，而 Nav2 的输出先经 `fake_vel_transform`，运行时需额外启动独立驱动才会落到串口。
3. `fake_vel_transform` 忽略收到的 `angular.z`，只发其内部由 `/ai_tuoluo` 控制的固定旋转速度；默认状态下该值为 0。
4. Nav2 MPPI 配置为全向模型并允许非零 `linear.y`，而底盘运动学是左右轮差速/滑移转向，没有横移自由度。固件还把极特殊的 `VY≈1` 解释为航向锁定命令。
5. 两个 ROS 驱动都以定时器重发最后缓存的命令；没有对新 `cmd_vel` 做本机 freshness watchdog、有限值过滤和幅值限制。导航上游不再产生新消息时，串口帧仍可能持续包含旧速度，刷新 STM32 的 200 ms 看门狗。
6. ROS 串口接收路径没有发布底盘轮速里程计或 STM32 IMU；Nav2 当前定位依赖 Point-LIO/激光雷达输出 `/odometry` 与 TF，而不是底盘反馈。

## 3. STM32 工程与控制架构

### 3.1 工程结构

工程为 STM32H723 CubeMX/HAL + FreeRTOS：

- `Core/Src`、`Core/Inc`：时钟、GPIO、DMA、SPI、TIM、USART、FDCAN、FreeRTOS 和中断回调。
- `User/APP`：串口命令接收、命令快照、底盘控制循环、运动学、车辆配置和 IMU 任务。
- `User/Bsp`：FDCAN 适配、SBUS 接收与 CH8 控制模式。
- `User/Devices/HM_Motor`：HM G36 MIT 命令与反馈打包。
- `User/Devices/BMI088`、`User/Algorithm/mahony`：BMI088 SPI 驱动与姿态融合。
- `Middlewares/FreeRTOS`、CMSIS/HAL：调度与芯片驱动依赖。

`Core/Src/main.c` 初始化 USART1、UART5、FDCAN1/2/3、SPI2、DMA 等。BMI088 初始化失败时启动流程停留在重试路径。FreeRTOS 中 `defaultTask` 每约 10 ms 检查 SBUS/CH8 并发布模式命令；`INS_TASK` 运行姿态任务；`CHASSISR_TASK` 调用 `Chassis_task()`，控制周期为 5 ms（200 Hz）。

### 3.2 六电机参数与对应关系

源码将电机类型配置为 HM G36 MIT 速度模式。车辆/执行器配置在 `User/APP/vehicle_config.h`：

| 车轮 | 总线 / 命令 CAN ID | 反馈 CAN ID | 源码方向系数 | 正向直行时下发的速度符号 |
|---|---:|---:|---:|---|
| 左前 LF | FDCAN1 / 1 | `0x11` | +1 | 正 |
| 左中 LM | FDCAN1 / 3 | `0x13` | +1 | 正 |
| 左后 LR | FDCAN1 / 5 | `0x15` | +1 | 正 |
| 右前 RF | FDCAN2 / 2 | `0x12` | −1 | 负 |
| 右中 RM | FDCAN2 / 4 | `0x14` | −1 | 负 |
| 右后 RR | FDCAN2 / 6 | `0x16` | −1 | 负 |

CAN 回复 ID 按 `0x10 + 电机命令 ID` 解码。FDCAN1 接左侧电机，FDCAN2 接右侧；FDCAN3 虽在 CubeMX 初始化，但没有接入当前六轮控制映射。源码说明的 FDCAN 引脚为 FDCAN1 `PD0/PD1`、FDCAN2 `PB5/PB6`。

| 配置项 | 源码值 | 含义/限制 |
|---|---:|---|
| 轮距 `VEHICLE_TRACK_WIDTH_M` | 0.535 m | 左右轮中心面距离 |
| 轮胎直径 | 0.248 m | 半径 0.124 m |
| 外部减速比 | 未单独配置 | 运动学把 HM G36 输出轴 rad/s 直接换算车轮线速度；实物内部减速定义未由源码验证 |
| G36 位置量程 | ±12.5 rad | MIT 编码量程 |
| G36 速度量程 | ±8 rad/s | MIT 编码量程 |
| G36 Kp / Kd / 力矩量程 | 0–500 / 0–5 / ±28 Nm | 编码器输入边界 |
| 车体线速度命令上限 | ±0.6 m/s | 固件命令裁剪 |
| 偏航角速度命令上限 | ±1.5 rad/s | 固件命令裁剪 |
| 单侧线速度上限 | ±0.6 m/s | 左右侧各自裁剪 |
| 轮速发送裁剪 | ±7.6 rad/s | `0.95 × 8 rad/s` |
| 斜坡 | 2 m/s²；偏航变化率 3 rad/s² | 固件限速变化 |

方向系数是源码配置，不是对车轮实际安装和物理转向的实车验证。

### 3.3 `VX/VZ` 到六轮命令

底盘使用差速式左右轮等速分配：

```text
turn = -0.5 × 轮距 × VZ
左侧线速度  = VX + turn = VX - 0.2675 × VZ
右侧线速度  = VX - turn = VX + 0.2675 × VZ
```

每侧 LF/LM/LR 或 RF/RM/RR 收到同一个侧速度，再除以轮半径换成轮/输出轴 rad/s，并乘各自方向系数。按源码约定，正 `VZ` 为逆时针；对应左侧速度减小、右侧速度增大。右侧电机的方向系数为负，因此直行时左侧 CAN 速度为正、右侧为负。

运动学没有 `VY` 横移项。左右侧各自裁剪可能改变饱和时的 `VX:VZ` 比例。偏航控制还会根据 BMI088 陀螺仪闭环修正，最终侧速度再次受限。

### 3.4 RTOS、CAN 与传感器故障行为

- 底盘任务每 5 ms 运行，读取命令快照、作斜坡与限幅、结合陀螺仪偏航反馈计算六轮速度，并读取六路电机反馈形成底盘速度估计。
- CAN 电机帧通过 FDCAN FIFO 入队；当前配置关闭自动重传。发送函数对队列失败重试有限次数，但返回状态未向上层形成可靠故障处理。源码未发现因任一电机反馈过期/CAN 错误而锁止六轮的逻辑。
- CAN 接收回调清零缓冲区后读取报文，再以固定 8 字节交给电机解码；没有将 HAL 返回的真实 DLC 作为长度校验。短反馈帧有被按 8 字节解码并更新接收时刻的风险。
- BMI088 为 SPI2 设备，Mahony 融合姿态。启动初始化失败会卡在初始化重试；运行中驱动没有统一的 IMU 健康/过期标志，`ins_flag` 置位后不会因运行期传感器故障撤销。陀螺仪芯片 ID 不匹配时相关读数可能保留旧值，底盘偏航闭环仍可能继续使用陈旧/无效反馈。
- ROS 侧没有读取这些电机反馈或 BMI088 状态的现成底盘遥测接口。

## 4. 底盘 UART 协议与协议文档对照

### 4.1 实际帧格式

底盘 USART1 为 PA9/PA10、115200 baud、8N1、DMA 接收。实际 14 字节帧在固件解析器 `User/APP/Chaiss_Communication.c` 中为：

| 偏移 | 字节数 | 字段 | 编码 |
|---:|---:|---|---|
| 0 | 1 | 头 | `0xA5` |
| 1–4 | 4 | `VX` | IEEE-754 `float32`，小端，m/s |
| 5–8 | 4 | `VY` | IEEE-754 `float32`，小端；当前底盘不支持横移 |
| 9–12 | 4 | `VZ` | IEEE-754 `float32`，小端，rad/s |
| 13 | 1 | 尾 | `0x4A` |

固件手工按小端重组 32 位值并复制为 `float`；对 `VX/VY/VZ` 做 `isfinite` 检查。DMA idle 回调接收数据后先进入 28 字节流缓存，可拼接跨回调帧，也能连续解析多帧；找不到帧头或尾标记错误时逐字节重新同步，缓存溢出时丢弃最早字节。USART 错误路径中止并重新启 DMA。

协议没有 CRC、序号、时间戳或认证字段：有限值且头尾正确的损坏帧不能被可靠检测。限速主要由固件在接收后的控制路径实施，Nav2/ROS 发送节点本身没有等价的输入检查。

### 4.2 与《底盘串口通信协议.md》核对

| 文档描述 | 源码核对 | 结论 |
|---|---|---|
| `A5 + VX + VY + VZ + 4A`，共 14 字节 | 解析器逐字段按该格式读取 | 一致 |
| 三个 IEEE-754 小端 float | 小端组装后转 `float` | 一致 |
| CH8 低/中/高分别失能/串口/SBUS | `rc_control_mode.c` 阈值为 `<1300`、`1300..1700`、`>1700` | 一致；阈值边界按源码执行 |
| 串口命令 200 ms 超时停车 | `chassis_task.c` 检查命令更新时间；超时/未启用时给六个电机发送 relax | 一致，5 ms 轮询使触发约在 200–205 ms 后 |
| SBUS 100 ms 掉线 | `rc_sbus_update()` 根据最后帧时间判 offline，默认任务周期检查 | 一致；实际检查点会多出任务轮询延迟 |
| `VY` 字段 | `(VX≈0,VY≈1,VZ≈0)` 是航向锁特殊命令 | 必须澄清：`VY` 不是可通用使用的侧向速度 |

串口帧只有在 CH8 选择串口且模式门控已使能时才刷新有效串口命令；串口被禁用时到来的有效格式帧会被拦截，不会绕过 CH8。切换控制模式会清空旧命令/关闭控制门。

### 4.3 CH8 和超时行为

- CH8 `<1300`：失能，电机命令停车/relax，控制门关闭。
- CH8 `1300–1700`（含边界）：串口控制，只开放 USART1 指令入口。
- CH8 `>1700`：SBUS 遥控，关闭串口入口；摇杆线速度与偏航速度受固件最大值限制。
- SBUS 最后有效帧年龄超过 100 ms 被认为 offline；默认任务约每 10 ms 轮询，掉线之后模式退到失能并停车。
- 串口命令更新时间年龄大于 200 ms 时，底盘任务停止使用速度目标并对六个电机发 relax。该机制以“没有新有效串口帧”为条件，并不判断上位机的 ROS `/cmd_vel` 是否新鲜。

### 4.4 `G36_VELOCITY_KD=60.2` 异常

`vehicle_config.h` 配置 `G36_VELOCITY_KD=60.2`，而 `G36_KD_MAX=5.0`。`hm_motor_send_velocity()` 将该值传入 MIT 编码器，`hm_float_to_uint()` 先按量程裁剪，再用 12 位编码。因此实际编码为 `4095`，解码端量化值是 **5.0（Kd 量程上限）**，不是 60.2。

这不是“电机接收 60.2”的情况，但确实让速度模式固定使用 Kd 最大档。源码注释对过高 Kd 的电流/冲击/振动风险有提醒；不能仅凭代码判定该电机/机构实际安全。本次不更改该参数，也不尝试电机实测。

## 5. ROS 2 导航到 CAN 电机的真实数据流

### 5.1 Nav2 工作空间的 14 字节驱动

相关源码：

- `Nav2_rea_ws/src/drivers/serial/driver/include/rm_serial_driver/packet.hpp`
- `Nav2_rea_ws/src/drivers/serial/driver/src/node.cpp`
- `Nav2_rea_ws/src/drivers/serial/driver/config/serial_params.yaml`
- `Nav2_rea_ws/src/drivers/serial/driver/launch/serial.launch.py`

`SendPacket` 是 packed 结构：`0xA5 + linear_x + linear_y + angular_z + 0x4A`，大小正好 14 字节。节点订阅 `/cmd_vel_chassis`，把 `linear.x` 复制给 `VX`、把 `linear.y` 取负后写入 `VY`、把 `angular.z` 写入 `VZ`。4 ms 定时器把最近缓存值反复发送。参数文件设置 `/dev/ttyACM0`、9600、无流控、无校验、1 stop bit。

驱动接收路径只验证一个两字节 `5A 4A` 预留包，后续无底盘里程计/IMU 发布逻辑。其独立 `serial.launch.py` 存在，但仓库检索的 Nav2 `bringup.launch.py` 与 `mapping_navigation_single_lidar.launch.py` 均未包含该驱动。

### 5.2 Nav2 工作空间的 topic remap 与变换

`rm_navigation/launch/navigation_launch.py` 将 Nav2 controller 的 `cmd_vel` remap 为 `/cmd_vel_nav`；velocity smoother 从 `/cmd_vel_nav` 接收并将 `cmd_vel_smoothed` 输出为 `/cmd_vel`。默认根命名空间下，后续预期链路是：

```text
Nav2 controller
  → /cmd_vel_nav
  → velocity_smoother
  → /cmd_vel
  → fake_vel_transform
  → /cmd_vel_chassis
  → rm_serial_driver（必须另行启动）
  → USART1 14-byte frame
  → STM32 parser / CH8 gate / 200 ms command watchdog
  → FDCAN1/2 HM G36 电机
```

但源码中 `fake_vel_transform::cmdVelCallback()` 将 `angular.z` 设置成内部 `spin_speed_`，没有复制输入消息的 `angular.z`。构造时 `spin_speed_` 强制设为 0，只有收到 `/ai_tuoluo == 1` 才会赋值为启动参数的固定转速。因此普通 Nav2 转弯指令被丢弃。标准 `bringup.launch.py` 传 `spin_speed: 0.0`，单雷达在线导航 launch 传 `12.56`，但只有对应 `/ai_tuoluo` 信号触发后才会固定旋转；这两种都不等同于跟随 Nav2 的偏航速度命令。

`nav2_params.yaml` 中 MPPI motion model 是 `Omni`，且 `vy_max/vy_min` 允许非零侧向速度；六轮底盘无横移运动学，固件把 `VY≈1` 留作航向锁 magic 值。需要让 Nav2 与底盘使用一致的差速/滑移转向模型，确认输出 `linear.y=0`。仅在 ROS 消息类型相同并不能证明运动接口兼容。

### 5.3 `tzb_ws` 的 46 字节机械臂串口桥

相关源码：

- `tzb_ws/src/serial/driver/include/rm_serial_driver/packet.hpp`
- `tzb_ws/src/serial/driver/src/node.cpp`
- `tzb_ws/src/serial/driver/config/serial_params.yaml`

该 `SendPacket` 前 14 字节恰好也是 `A5 + 三个 float + 4A`，随后附加 32 字节机械臂扩展（`B5`、使能、序号、6 个 float 关节角、夹爪命令、`4B`），总计 46 字节。节点订阅 `/cmd_vel` 和机械臂/夹爪话题，默认参数 `tzb.arm_serial_enabled=true`、`tzb.include_arm_serial_extension=true`，并配置 100 Hz 发布；串口参数同样是 9600 baud。

底盘固件只实现 14 字节 `VX/VY/VZ` 解析，没有匹配的 `B5...4B` 机械臂扩展解析。不能把 46 字节协议当成 STM32 底盘与机械臂均已支持的统一协议。若字节流直接连到该底盘，底盘只能尝试识别其中 14 字节前缀；扩展字段并无受支持的执行语义，且流解析器继续扫描扩展字节时存在误同步风险。实际线缆/端口是否相连未由源码证明。

`tzb_ws` 驱动直接订阅 `/cmd_vel`，而 Nav2 串口驱动订阅 `/cmd_vel_chassis`；它们是两个不同节点与数据路径。不要同时把两个串口驱动配置到同一个物理端口。当前仓库参数文件指定了不同设备路径，但是否映射到不同物理接口需在实际系统上核查，不能据路径文本推断硬件接线。

### 5.4 串口超时与“Nav2 停发”

两个 ROS 串口节点都缓存最近一次速度，靠周期 timer 持续发帧，没有以最近一次 Twist 到达时间为依据自动清零或停止发送。`tzb_ws` 在 100 Hz 配置下重复发扩展帧；Nav2 14 字节驱动每 4 ms 重发。

因此，如果只有上游导航消息停止，而串口节点仍运行，底盘固件仍可能不断收到旧速度帧并刷新 200 ms watchdog，**不能保证底盘因此停车**。上游 velocity smoother 若仍存活并按自身超时发出零速度，这是另一层行为；不能代替串口节点自己的 freshness 检查。若串口进程退出、线缆断开或实际发帧停止，固件才会在约 200–205 ms 后执行本地超时 relax（前提是 CH8 处于串口模式）。

这项主机/固件联合失效策略需要单独设计、审查和确认；本次只记录风险，没有修改安全策略。

## 6. 里程计、TF 与 IMU 输入

- 当前 Nav2 配置把 `/odometry` 用作里程计 topic，底盘 base frame 配为 `base_frame_fake`。
- Nav2 仓库的 Point-LIO 在激光雷达/IMU输入有效时发布 odometry（`odometry`）和局部 TF（`odom_init → base_frame`）；`fake_vel_transform` 读取 `/odometry` 姿态，发布 `base_frame → base_frame_fake` 的零平移姿态变换。它们是导航定位链路，不是由六个车轮编码器积分得出的轮式里程计。
- 在线单 MID360 launch 通过 Livox 驱动、IMU transform 与 complementary filter 给 Point-LIO提供 `/imu/data`，配置 `mapping.imu_en: True`、`use_imu_as_input: False`。此路径取决于所选 launch 和传感器启动；仓库另一个 `bringup.launch.py` 把 Livox 驱动 add_action 注释掉，不能把两种 launch 行为混为一谈。
- STM32 BMI088 数据没有被固件 UART 帧发布给 ROS；当前串口接收代码也没有解码 chassis odometry/IMU。`Nav2_rea_ws` 的串口 ACK 接收代码不发布运动反馈，`tzb_ws` 发布的是机械臂关节状态，不是轮速/底盘 IMU。
- 因此，激光定位 TF 可为 Nav2 提供部分定位反馈，但底盘自身的轮速里程计、底盘 IMU 话题、速度闭环遥测、故障/在线状态接口均不完整。

## 7. 已实现、缺失、风险与修复状态

### 已实现

- STM32H723 + FreeRTOS 的 200 Hz 六轮控制循环。
- FDCAN1/2 六电机映射、MIT 速度帧、反馈解码和差速运动学。
- USART1 14 字节小端浮点协议，流式分包/粘包处理、头尾搜索、有限值过滤。
- CH8 失能/串口/SBUS 三模式门控；SBUS 掉线判定与串口命令超时停车。
- BMI088 启动初始化和姿态融合；Nav2 工程中有可构造 14 字节包的串口节点代码。
- 机械臂驱动 46 字节包保留了相同的 14 字节运动字段前缀，但固件不实现其机械臂扩展。

### 缺失或不完整

- 一个已在所选 Nav2 bringup 中启动、使用正确底盘物理端口和 115200/8N1 参数的底盘桥接配置。
- `angular.z` 不被 fake transform 覆盖；Nav2 运动模型与底盘差速运动学一致，且 `linear.y` 不产生无效命令。
- 串口桥命令 freshness timeout、NaN/Inf 与速度范围校验、关闭/异常时明确的安全停车路径。
- STM32 轮速里程计、IMU、驱动故障与在线状态的 ROS 发布/协议定义；`/odometry` 当前来源是激光定位。
- 对 CAN 发送错误、反馈超时、无效 DLC、运行期 IMU 故障的可靠检测和安全策略。
- 协议 CRC/序号/版本协商；如复用 46 字节扩展，需定义并在固件端实现可验证的扩展帧解析。

### 风险等级（静态审计）

| 风险 | 等级 | 依据 |
|---|---|---|
| ROS 配置 9600 与固件 115200 不一致 | 阻断 | 目前两端不能按预期正确交换帧 |
| ROS 桥无 freshness timeout，缓存速度持续重发 | 高 | `/cmd_vel` 停发不一定触发 MCU 200 ms 看门狗 |
| fake transform 覆盖 `angular.z` | 高 | Nav2 转弯速度不会按原始值送达 chassis topic |
| Nav2 Omni 与底盘差速/特殊 VY 命令不兼容 | 高 | 可生成固件不能执行的横向速度/特殊 VY 值 |
| `G36_VELOCITY_KD=60.2` 被编码限幅至 5.0 | 高待实测 | 实际 Kd 是最大值；机构、电流和温升行为未知 |
| CAN 发送状态不传播、反馈无 freshness/DLC 守卫 | 高 | 通信失效时可能继续使用旧/无效反馈或不报告失败 |
| BMI088 运行期故障未撤销健康状态 | 高 | 偏航闭环可能使用陈旧/无效 IMU 反馈 |
| 无 CRC、帧认证和状态回传 | 中 | 难以检测某些内容损坏与确认 MCU 已执行命令 |

### 软件修复评估

本次没有修改生产 ROS 驱动或固件。建议后续按接口方案单独评审：统一物理串口与 115200 配置；明确只保留一个拥有该端口的驱动；Nav2 输出适配差速底盘并保留真实偏航命令；桥接层对新鲜度、有限值、速度限幅做明确处理；定义 ACK/轮速/IMU/故障遥测及数据年龄；修正并评审 G36 Kd 的配置来源。涉及失联停车、控制权和电机限位的策略要先经用户确认后再修改。

## 8. 离线测试结果

执行命令：

```text
python3 -m unittest -v tools/p5a_offline/test_chassis_control_path.py
```

结果：**13 项通过，0 项失败**。覆盖帧字节布局和大小端、前进/后退/左右旋转/停车往返、分片与连续帧、坏头/坏尾/NaN/Inf、有限越界值裁剪、VY 航向锁特殊值、六轮方向和左右转分配、CH8/100 ms/200 ms 边界、46 字节前缀、fake transform 偏航覆盖、主机定时器重发陈旧速度、G36 Kd 限幅编码。

这是按照源码行为建立的独立 Python 模型，不是对 C/C++ 二进制或 ROS 运行时的集成测试，也不是底盘、CAN、电机或导航实车验收。测试使用 `float32` 往返容差比较。

## 9. 下一步导航联调前需补齐

1. 确认实际硬件串口设备映射、串口收发线和唯一端口所有者；把主机参数与 USART1 的 115200/8N1 对齐。
2. 选定并接入一个 14 字节 chassis bridge 到目标 Nav2 bringup；停用对同一端口的重复驱动配置。
3. 使 fake velocity transform 保留有效 `angular.z`，明确线速度坐标变换；以差速模型配置 Nav2 MPPI 并确认 `linear.y=0`。
4. 设计并审查 ROS bridge 的速度新鲜度、NaN/Inf、限幅和进程/串口异常处理；确认与 STM32 watchdog 的组合行为。此项涉及安全策略，实施前需要用户确认。
5. 定义底盘反馈：至少轮速/估算 `nav_msgs/Odometry`、时间戳、底盘 IMU、CAN 电机故障/在线状态；决定与 Point-LIO TF/里程计如何融合和避免重复 TF 发布。
6. 加入纯软件 CI/仿真测试，后续再安排隔离车轮、急停可达、限速条件下的台架验收；本报告未执行任何硬件步骤。
7. 实车前核查 G36 Kd 生效值、驱动器配置和电机/传动机构状态；未经评审不得通过盲目改参数“试出来”。

## 10. 分支保存内容

报告与离线测试保存于独立分支 `p5a/chassis-source-audit`。本阶段未修改固件参数、安全逻辑或生产 ROS 驱动；未合并、部署、烧录或连接硬件。
