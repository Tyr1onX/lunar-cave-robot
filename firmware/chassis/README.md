# 六轮底盘 STM32 固件

该目录由用户提供的 `DM-car-six-wheel.zip` 整理，原压缩包 SHA-256 为 `823ff6981859dbce1d05a3d4864b921bd0f9aa5165335d0caec7d12852701d2e`。压缩包没有 Git 元数据，源码版本按原包哈希标识；原 ZIP 未被修改。

## 工程内容

- MCU：STM32H723VGT6（CubeMX 工程名 `CtrlBoard-H7_IMU`）。
- 工程入口：`CtrlBoard-H7_IMU.ioc`、根目录 `CMakeLists.txt` / `CMakePresets.json`、`MDK-ARM/CtrlBoard-H7_IMU.uvprojx`。
- 包含 Core、HAL/CMSIS、FreeRTOS、用户控制源码、STM32H723 启动文件、CMake 链接脚本、Keil scatter 文件及 CMSIS-DSP 对应的 GCC/ARM 预编译库。
- 压缩包中 2,252 个文件、约 894 MB 解压内容；当前仅保留工程源码、必要依赖、配置和许可。已排除构建目录、Keil/CMake 输出、缓存、机器专用 IDE 配置以及不相关的工程会话记录。
- CMake 构建要求 CMake 3.22+、Ninja 和 ARM GNU 工具链；Keil 工程使用 ARMCC 5.06 Update 6 记录，需安装兼容的 MDK-ARM 与 STM32H7 设备包。

```bash
cmake --preset Debug
cmake --build --preset Debug
```

本轮 ThinkBook 与 NUC 均未发现 `arm-none-eabi-gcc`，所以未执行固件编译。静态检查确认 CMake/Keil 配置引用的源目录、启动文件、链接脚本和两种工具链所需的 CMSIS-DSP 库均已归档；所有 Keil 文件引用均有对应源文件。原 Keil 项目引用的 `bsp_usart1.c` 与 `rc_pwm.c` 在原 ZIP 中也不存在且不在 CMake 源码列表，因此从归档副本中删除了这两个无效条目。CMake 文件中的固定 Windows 安装路径已改为从 `PATH` 或 `STM32CUBECLT_PATH` 查找。不得调用 `flash` 或 `flash-cmsis-dap` 目标。

## 静态接口摘要

- USART1：115200 baud、8N1、DMA；14 字节帧由 `A5`、三个小端 float32 速度值和 `4A` 组成。
- FDCAN1/2：配置为六个 HM G36 电机控制；BMI088 用于底盘姿态反馈。
- 当前 ROS 2 P5b 主机桥静态发送同一 14 字节格式和 115200 baud。无实体 UART/CAN/电机、控制器回传或制动测试。
- 固件归档不能证明实体控制板正在运行此源码版本。

归档来源、依赖许可、SHA-256 和未纳入的厂商手册见[项目资产清单](../../docs/PROJECT_ASSETS.md)。
