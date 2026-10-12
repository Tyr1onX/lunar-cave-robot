# P9-C STM32 安全策略验证

日期：2026-10-13。固件 PR 分支 `p9c/stm32-safety-review` 基于 P9-B 分支 `p9b/stm32-build-and-safety-audit` 的 `0236b72c93018f2bb261c6abf79715f19384e00c`。固件源内容来自 P9-C 提交 `c383944f1bd7861d10c075253a4eccf94f31627a`。以下均为 ThinkBook 离线构建/主机测试；未打开实体 STM32 串口、未烧录、未连接电机。

## 主机测试

在 ThinkBook 固件工作树运行：

```bash
bash firmware/chassis/tests/host/run.sh /tmp/p9c-chassis-host-tests
bash firmware/arm/tests/host/run_safety_state_tests.sh /tmp/p9c-arm-host-tests
```

全部通过：

- 底盘生产 UART 解析器/命令逻辑在 mocked HAL 下的协议测试。
- 底盘安全策略测试：上电锁定、人工中立重使能、RC 丢失、UART 错误、串口租约超时、控制源切换和故障恢复。
- 机械臂纯 C 安全策略测试：上电锁定、明确重新使能、主机失能、100 ms 租约、来源丢失、反馈就绪和恢复。

主机测试验证软件状态转换，不验证输出引脚、电机制动、机械臂重力保持、独立急停或真实停止距离。

## 构建结果

底盘固件使用 ThinkBook ARM GNU GCC/G++ 10.3.1、CMake 3.22.1 和 Unix Makefiles 交叉构建通过。构建目录 `~/.cache/lunar-cave-robot/p9c-stm32-build/chassis` 与 P9-B 缓存隔离。链接结果为 text 49,712 bytes、data 148、BSS 35,672；FLASH 使用 49,864 bytes、RAM_D1 使用 35,824 bytes。

| 产物 | SHA-256 |
| --- | --- |
| `CtrlBoard-H7_IMU.elf` | `93bc092362bdd8cdc5ae74dc5e923251b3202c3257759e83a952e1cfb06804ab` |
| `CtrlBoard-H7_IMU.hex` | `f6a1c94c0202cfc95aa0269a4293e8c5ca4a4bcbaa6c109e9ef56698e06f952b` |
| `CtrlBoard-H7_IMU.bin` | `aabe05ac001f0d9d07915cea84a8c98a1ff6fc1aea3661e43acb07dfcda147e5` |
| `CtrlBoard-H7_IMU.map` | `6d7abe3d8dabfc1d3deaa8484b561b9ce4942a04905d9fab65a81fb27db52c8e` |

构建日志 SHA-256：`90b91da7f4e1c8e9198285864cb5fa8bcff933d2ee4b01402a76ead2b4951803`。未屏蔽的告警包括缺少 `OPENOCD_EXECUTABLE`（因此无 flash target）、历史 `packed` 属性被忽略、CAN 分支缩进易误读、控制器数组缺少大括号及 BMI088 校准上下界可能未初始化。

机械臂构建使用同一 GNU 工具链尝试配置，但真实 XRobot 工程输入 `firmware/arm/Modules/CMakeLists.txt` 和 `firmware/arm/User/xrobot_main.hpp` 缺失。CMake 在产品配置阶段停止，没有生成 ELF，也没有伪造文件。

## 核心源文件校验值

| 源文件 | SHA-256 |
| --- | --- |
| `firmware/chassis/User/APP/chassis_safety.c` | `12aa0d58ca48fc8d486fc99109a3c8f5a0ee656f9f1c830ef768648e46ce2d40` |
| `firmware/chassis/User/APP/chassis_task.c` | `13e14f19b45170b7161cccace17ca22990627375465bf7bbc2f81bc2c294c324` |
| `firmware/arm/others/src/app/arm_safety_state.c` | `d914ce67dc243533809c455bc3a22a9cf733f1c662e3a968afe4982d08d31a5a` |
| `firmware/arm/others/src/app/Host.cpp` | `408a65d0c4d78cecabfd849cf96b42bb3f8f5dcd324ca14d9d1547e6684adb96` |

## 未解决的 S0 与审查要求

- 底盘 CAN `relax`/disable 对驱动器和六个电机的物理效果尚未测量。
- 机械臂电机失能可能导致下坠；保持、制动或支撑策略必须由硬件负责人确认。
- 尚无独立硬件急停路径和已测量停止确认。
- 本 PR 的真实电机行为变更须由硬件负责人单独审查；审查及无动力检查完成前不得烧录或通电试运动。

状态机定义及与 ROS 2 14/46 字节协议的关系见 [P9-C 安全状态机](P9C_SAFETY_STATE_MACHINES.md)。
