# P5b：六轮底盘 ROS 2 控制链路修复与离线集成验证

**状态：** 已完成软件侧修复与离线验证。未连接底盘、机械臂或真实串口；未改动 NUC、STM32 固件、电机参数、CAN ID 或 CH8 控制权行为。此报告不构成实车验收。

## 基线与范围

- 分支基于统一归档基线 `archive/p3-p4b-offline-validation`，保留 P3/P4b 和前序 P4c/P5a 审计资产。
- 工作分支：`p5b/chassis-ros2-integration`。
- ROS 2 实际构建和节点测试在 ThinkBook 的 `/tmp/p5b-ros2-validation` 临时副本中运行；未改写 `Nav2_rea_ws` 或 `tzb_ws` 的工作树。
- P5b 软件桥接名为 `chassis_serial_bridge`，旧 Nav2 包名 `rm_serial_driver` 不再与机械臂工作空间重名。
- STM32 源码审计沿用 P5a 对 `DM-car-six-wheel.zip` 的结果；本阶段没有修改或重新打包上传固件源码。

## 修复后的运动控制数据流

```text
Nav2 controller (/cmd_vel_nav)
        │ navigation_launch.py remap
        ▼
nav2_velocity_smoother
        │ cmd_vel_smoothed -> /cmd_vel
        ▼
chassis_serial_bridge (/cmd_vel, one serial owner)
        │ 115200 8N1, 50 Hz, A5 + 3 x float32 LE + 4A
        ▼
STM32H723 USART1
        │ firmware VX/VZ differential kinematics
        ▼
CAN motor drivers -> six wheels
```

三个启用 Nav2 的 bringup launch 各自只包含一次底盘 bridge，并移除了会改变速度语义的 `fake_vel_transform`。这些 launch 要求显式传入 `chassis_device`，没有物理串口默认值。Nav2 velocity smoother 输出的实际默认名称仍为 `/cmd_vel`。

### 14 字节线协议

| 偏移 | 长度 | 字段 |
| --- | ---: | --- |
| 0 | 1 | `0xA5` 帧头 |
| 1 | 4 | `linear.x`，IEEE-754 float32，小端 |
| 5 | 4 | `linear.y`，float32，小端；桥接固定写 0 |
| 9 | 4 | `angular.z`，float32，小端 |
| 13 | 1 | `0x4A` 帧尾 |

帧中没有长度字段、序号或校验和。编解码使用逐字节小端写入，不依赖主机结构体填充或本机端序。固件将 `Vy≈1` 解释为航向锁定特殊输入，因此 Nav2 运动帧必须保持 `Vy=0`。

### 主机侧新鲜度和异常规则

- 发送周期 20 ms。启动时没有新命令，持续发送全零帧。
- 只接受有限值；任一 Twist 分量为 NaN/Inf 即清除有效缓存并在下一发送周期发零帧。
- `|linear.x| > 0.6 m/s`、`|angular.z| > 1.5 rad/s` 被拒绝并发零帧，不作静默裁剪。
- 差速底盘要求 `|linear.y| <= 0.01 m/s`；超出时拒绝整条命令。容差内也一律在线上编码为 0。其余未支持轴超过 0.01 也拒绝。
- 命令接收超过 100 ms 后，桥接从下一个 20 ms 定时发送周期开始输出零，旧速度不继续保持。调度延迟会增加少量实际延迟。
- 串口断开会清除缓存，桥接按至多每秒一次尝试重开；复连后先发零，必须收到新的有效 Twist 才恢复运动。
- 正常 ROS 关闭时析构函数尽力写一帧零速。SIGKILL、断电或主机崩溃不能保证执行析构，仍依赖固件已有 200 ms 串口超时和 CH8 许可逻辑。STM32 安全策略没有改变。
- 串口打开默认空路径；把真实串口路径错误传入启动参数仍可能打开真实设备。因此运行实车前必须人工确认路径，离线脚本只使用 PTY。

速度上限取自 P5a 固件审计中的底盘限制，`vx=0.6 m/s`、`wz=1.5 rad/s`；Nav2 smoother 的加速度限值对齐固件已审计的 `Vx=2 m/s²`、`yaw=3 rad/s²` 速度斜坡。没有变更固件、电机 PID/Kd 或电机侧限位。

## 导航配置修复

- MPPI 从 `Omni` 改为 `DiffDrive`；横向速度范围设为零，最大线速度、角速度及加速度与底盘能力一致。
- Nav2 base frame 从 `base_frame_fake` 改为 Point-LIO 实际发布的 `base_frame`。
- `mapping.launch.py` 从不存在的 `nav2_params_fake.yaml` 改用现有 `nav2_params.yaml`。
- 修正 `fake_vel_transform` 的角速度赋值，使其转发输入 `angular.z`；三个启用 Nav2 的 launch 已不再启动该节点，控制流直接使用机体坐标速度。
- `tzb_ws` 的 46 字节机械臂串口包保持不变。其导航 Twist 订阅改为默认隔离话题 `/tzb/legacy_chassis_cmd`，不再争用 Nav2 的 `/cmd_vel`。旧 14 字节底盘前缀字段仍留在机械臂线协议中，但它不是底盘 USART1 的 ROS 控制链路。

## 里程计、TF 和 IMU 接口

当前导航没有从 STM32 编码器取得 ROS 里程计，也没有从本桥接取得 IMU。现有 Point-LIO 才是 Nav2 的里程计来源：

- `/odometry` 由 Point-LIO 发布，header frame 是 `odom_init`，child frame 是 `base_frame`。
- 同一定位节点广播 `odom_init -> base_frame`。
- 单 MID-360 在线建图 launch 发布 `map -> odom_init` 静态初始变换；Nav2 配置使用 `global_frame=map`、`robot_base_frame=base_frame`、`odom_topic=/odometry`。
- Point-LIO 的 IMU 输入口来自 MID-360 经现有 IMU transform/filter 链路处理的数据，不是 STM32 BMI088。
- 这些接口在配置层与 Nav2 frame/odom 配置相符；实际 TF 连续性、时间戳、QoS、定位质量和底盘运动一致性仍需在整机环境验证。本轮没有增加新定位数据源。

## 机械臂协议与底盘协议的关系

- STM32 USART1 底盘帧：14 字节，用于底盘速度；协议和帧界定见 P5a 审计。
- `tzb_ws` 机械臂串口帧：46 字节版本，在相同前缀后追加机械臂反馈/命令扩展；它服务机械臂串口设备。即使前 14 字节看起来相同，也不能仅因字段相同就认为它连到了底盘 USART1。
- P5b 将 Nav2 底盘串口责任固定到 `chassis_serial_bridge`。机械臂 bridge 不订阅 `/cmd_vel`。实车前仍要确认两个物理串口分别连到哪块 STM32/USB VCP，不能只看 tty 名称或线协议前缀。

## 已实现、缺失和风险

### 已实现

- 115200 8N1、14 字节小端控制帧。
- 单一 ROS 节点持有底盘串口；需要显式配置设备路径。
- 速度和轴输入校验、100 ms 新鲜度零速、复连清缓存和关闭时尽力停车。
- Nav2 smoother 到桥接的 launch/topic 路由、MPPI 差速运动模型及底盘速度范围。
- 机械臂工作空间的同名包冲突和 `/cmd_vel` 订阅竞争已消除。
- 配置静态检查和真实 ROS 2 节点 PTY 集成测试。

### 尚未实现或未验证

- 桥接只发送速度，没有解析 STM32 回传帧，也没有发布轮速、驱动器健康或底盘诊断。
- ROS 里程计目前靠 Point-LIO，而不是车轮编码器；点云定位失效时没有本桥接的替代 odom。
- 不能从离线测试证明 USART1 字节到达、STM32 帧解析/CH8许可、CAN 总线、六个电机方向、实际速度跟踪或机械制动性能。
- 实际设备路径、与机械臂设备的物理区分、真实串口独占、急停/遥控器模式切换和看门狗动作必须在断电/架空轮子和现场安全流程下另行核实。
- 若 Nav2 进程突然退出，主机桥接进程仍可能继续运行并发送零帧；若桥接进程也被强制终止，固件 200 ms watchdog 是最后一道已存在的串口停止机制。

## 离线验证

在 ThinkBook ROS 2 Humble 环境中，用 `/tmp/p5b-ros2-validation` 构建了新桥接包，运行真实的 `nav2_velocity_smoother` lifecycle 节点和真实 `chassis_serial_bridge` ROS 节点。桥接目标为 PTY 模拟接收端；Domain 设为隔离值，集成脚本再将话题 remap 到专用测试前缀以避免其它残留 ROS 会话影响结果。没有打开实体设备。

测试通过项目：

1. `/cmd_vel_nav` -> 实际 velocity_smoother -> bridge -> PTY 的前进、后退、左转、右转、停车。
2. 转向角速度保留，没有 fake 节点覆盖；发送帧均为 14 字节、头尾正确、Vy 恒为 0。
3. DiffDrive smoother 的横向约束；桥接也拒绝超过容差的横移。
4. NaN、Inf、超线速度、超角速度和不支持轴均输出零帧；容差内微小 y 在线上置零。
5. 指令停止更新后 100 ms 新鲜度窗口过期，PTY 后续观察到零帧。
6. 断开第一 PTY 并把设备符号链接切到第二 PTY：复连前后无旧运动速度重放；新的有效命令才恢复。
7. 对 bridge launch 发 SIGINT，最后一帧为零速。

这是软件和虚拟串口测试，不是底盘验收；Nav2 控制器本身没有启动，`/cmd_vel_nav` 输入由测试 ROS 2 publisher 提供。另运行静态配置检查与 Python launch 语法检查。详细命令见 `docs/p5b/run-offline-validation.md`。

## 实车导航联调前必须补齐

1. 现场确认并记录底盘 USART1 的唯一 tty 路径；确认该端口与机械臂 46 字节串口实际分离。
2. 由固件/电气人员核对 CH8 处于许可的串口模式，遥控器失联和急停行为符合预期。
3. 架空车轮后先以限速零速、单方向低速和角速度命令核验六个轮的安装方向/响应，不将 PTY 结果当作这些验收。
4. 在实际运行配置启动 Point-LIO，核对 `/odometry`、`odom_init -> base_frame`、`map -> odom_init` 时间戳和唯一 TF 发布者。
5. 确认串口字节流在 STM32 上被解析、CAN 电机反馈正常、停止发布时 200 ms watchdog 实测停车，再执行地图导航测试。
6. 后续需要补充 ROS 侧底盘诊断/状态接口及编码器里程计方案；本轮没有实现，也不应宣称闭环速度反馈已就绪。
