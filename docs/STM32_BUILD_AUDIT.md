# P9-B STM32 构建审计

更新日期：2026-10-12。目标分支：`p9b/stm32-build-and-safety-audit`，基于 `main` `ea3f64bd37d0372ad878adaf5a33b1dc73bea38c`，验证源码 SHA 为 `732f78545d1701a89870699827cb4e55b0ac9d8c`。构建发生在 ThinkBook 的隔离分支工作树和用户缓存目录；没有更改其统一 `main` 工作树，没有触碰 NUC，也没有连接调试器、串口或实体控制器。

## ThinkBook 工具环境

ThinkBook 为 Ubuntu 22.04 x86_64，CMake 3.22.1、GNU Make 4.3；本次没有 Ninja。ARM GNU 工具链 GCC/G++ 10.3.1、binutils 2.38、newlib 3.3.0 从 Ubuntu 22.04 软件包解包到用户目录，没有安装系统包或请求管理员权限。工具包装在 `~/.cache/lunar-cave-robot/p9b-toolchain/wrapper/`，具体包版本、SHA-256 和构建日志保存在 ThinkBook 用户缓存中；仓库只记录源码和复现脚本，不提交工具链或构建产物。

## 底盘 STM32H723

- 输入归档：`DM-car-six-wheel.zip`，SHA-256 `823ff6981859dbce1d05a3d4864b921bd0f9aa5165335d0caec7d12852701d2e`。工程配置目标为 STM32H723VGTx / `CtrlBoard-H7_IMU`，含 CubeMX IOC、HAL/CMSIS、FreeRTOS、CMake、Keil、启动文件和链接脚本。
- 结果：ThinkBook 的隔离 Linux 构建 **PASS**。GNU Arm Embedded GCC 10.3.1、CMake 3.22.1、Unix Makefiles。ELF、HEX、BIN 和 MAP 均由完整构建产生。
- 本次修复：Linux 大小写敏感时 `User/APP/INS_task.c` 引用的小写 `ins_task.h` 不存在；归档实际头文件为 `INS_task.h`。此外将原有 200 ms unsigned tick 差值判断提取成纯函数供任务调用，阈值和分支行为保持不变，并对 `command_ok=0` 时此前仍读取未初始化时间戳的情况消除了无用读取。
- ELF SHA-256：`53f7c8d22d37c4750c3e0f56b8200a49a8ef2c9f1ef945991c3f1f81e9a89bfd`。
- MAP SHA-256：`7ee8a4bdf7d2a7f7d80c4a1d21b6e2587bb85446c2f489c7584c4694d9d70e0e`。
- HEX SHA-256：`a0579407d21bd8335cc326d2bb056cb8167171178183e64d3a229e2576582507`。
- BIN SHA-256：`d501b3fac056201e4356acbc9fa46033f81832969689af18a33d899ce4556440`。
- 完整日志：ThinkBook `~/.cache/lunar-cave-robot/stm32/logs/chassis-build.log`；构建产物位于 `~/.cache/lunar-cave-robot/stm32/chassis/`。本次复建报告 FLASH 使用 49,372 B / 1 MiB，RAM_D1 使用 35,816 B / 320 KiB。GCC 仍报告 BMI088 校准局部变量可能未初始化、若干 packed 属性被忽略、CAN 发送分支缩进及初始化括号警告；这些需要固件负责人结合工具链和运行边界复核。

## 机械臂 STM32H723

- 输入归档：`lunar_rover.zip`，SHA-256 `63ac07e173e148ab248b1689ccae64ba380b730d0c64b2b29d82b96307c48ab8`；包内 Git HEAD `963fd0275ddbe2c759b35250842284c6d62e90c1`。工程有 CubeMX、CMake、Keil、HAL/CMSIS、FreeRTOS、LibXR、启动及链接文件。
- 结果：ThinkBook CMake 找到 ARM GCC/G++ 10.3.1，配置 **FAIL / BLOCKED** 于 LibXR 的 XRobot 集成检查：`firmware/arm/Modules/CMakeLists.txt` 缺失。`firmware/arm/User/xrobot_main.hpp` 也缺失；源码的 `XRobot.cmake` 明确要求由 XRobot 生成这些产品文件。没有准确产品配置/模块源，因此没有运行生成器、没有编造空文件、没有产出 ELF。
- 当前用户级工具目录还缺 ARM GNU 单独分发的 newlib C++ headers/runtime。即使补齐 XRobot 输入，仍须先补齐匹配的 C++ 工具链组件再尝试编译。
- ThinkBook 配置日志：`~/.cache/lunar-cave-robot/p9b-build/arm-cmake-configure.log`。原始压缩包、ThinkBook/NUC 可访问工作区与 Git 历史检索均未找到丢失的生成文件；需要原固件工程负责人提供对应产品的生成结果或完整可复现配置。

## 底盘协议主机测试

ThinkBook 的 GCC 使用实际生产 `Chaiss_Communication.c` 与 `chassis_command.c` 编译主机测试，HAL/DMA API 由明确的测试桩替代。结果 **PASS**：覆盖 gate 初始关闭、200/201 ms 命令过期边界、HAL tick 回绕、拆帧、连续帧、垃圾与坏帧尾重同步、NaN 拒绝、heading-lock 字段和安全门控。故障注入还复现两项现存缺口：有限 float 的单比特翻转会被接受（协议无 CRC），UART 错误后残留的半帧可与后续字节拼成命令（错误恢复未清空 parser state）。这是底盘解析器源码的主机单测，不是 MCU 外设/DMA 实测。

## ROS 2 与 P4/P7 离线回归

- ThinkBook 隔离工作树在该 SHA 上分别构建 `Nav2_rea_ws` 的 2 个包及 `tzb_ws` 的 6 个包，均 **PASS**。`lite_motion_planner` 有 Pinocchio/eigenpy 对 Boost Python 头文件的 CMake 警告，但链接成功。
- P7/P8 的 12 场景脚本已尝试 3 次，完整套件状态为 **FAIL**，不可据单场景结果覆盖。三次分别在 `grasp_failure` 的触发事件未被测试订阅器观察到、`chassis_disconnect_return` 的模拟机械臂命令租约提前锁存导致故障未注入，以及 `arm_feedback_stop` 启动时 5 秒 TF 检查未解析 `map→base_link` 时退出。相关场景分别独立重跑通过；12 个场景均有单场景 PASS 结果分布在这些隔离运行中，但没有一次 12/12 完整脚本成功结束。ThinkBook 状态文件 `ros-offline-suite.yaml` 保留完整套件 FAIL，`p7-scenario.yaml` 单独记录最新的单场景结果。
- P4f 在 ThinkBook 上构建两个 ROS 工作空间、P4e PTY 驱动，运行的 20 项主机安全/协议断言、8 项 P6 单元及静态合同测试和取消竞态 **PASS**。前 5 个 P6 场景通过；完整 6 场景脚本在 `duplicate_trigger` 返航时遇到 Nav2 空规划路径与 TF 时间外推，任务按策略中止，因此 P4f 完整套件记录为 **FAIL**。独立复跑 `duplicate_trigger` 后通过；P4f 没有一次完整 6/6 套件通过记录。日志位于 ThinkBook `~/.local/state/lunar-cave-robot/runs/p4f-p9b-20261012T0932/`。
- 这些 ROS/P4f/P7 结果均为 localhost 仿真、合成反馈或 PTY；不是相机、导航传感器、STM32、CAN、电机或实机验收。P7/P8 源码与既有验证脚本没有修改。

## 复现入口及安全边界

在 Linux ThinkBook 仓库根目录，配置 `STM32CUBECLT_PATH` 指向包含 `bin/arm-none-eabi-gcc` 的工具链根目录后运行：

```bash
./tools/build_stm32_firmware.sh chassis
./tools/build_stm32_firmware.sh arm
firmware/chassis/tests/host/run.sh
```

固件构建日志/产物默认位于 `~/.cache/lunar-cave-robot/stm32/`，状态记录写入 `~/.local/state/lunar-cave-robot/tests/`。编译通过只说明源码在该工具链下可链接；没有烧录、没有打开真实 STM32 端口、没有发送 CAN、没有驱动电机，也不构成安全或实机验收。安全缺口与上位机兼容结论见[协议说明](STM32_PROTOCOL_SPEC.md)、[STM32 安全审计](STM32_SAFETY_AUDIT.md)和[后续整改方案](STM32_SAFETY_REMEDIATION_PLAN.md)。
