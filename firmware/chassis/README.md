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

P9-B 在 ThinkBook 隔离工具目录中使用 ARM GNU GCC 10.3.1 和 CMake 3.22.1 完成完整交叉编译，输出 `CtrlBoard-H7_IMU.elf/.map/.hex/.bin`。Linux 大小写敏感构建发现 `INS_task.c` 原来包含小写 `ins_task.h`，与归档中的 `INS_task.h` 不一致；已按真实头文件名修正。原有 200 ms 命令新鲜度判断抽为纯函数供底盘任务使用，阈值不变，并新增 tick 回绕/超时边界测试。构建日志及 ELF/MAP 哈希见[STM32 构建审计](../../docs/STM32_BUILD_AUDIT.md)。编译仍有既存警告，需要在后续固件整改中复查。没有烧录；不得调用 `flash` 或 `flash-cmsis-dap` 目标。

## 静态接口摘要

- USART1：115200 baud、8N1、DMA；14 字节帧由 `A5`、三个小端 float32 速度值和 `4A` 组成。
- FDCAN1/2：配置为六个 HM G36 电机控制；BMI088 用于底盘姿态反馈。
- 当前 ROS 2 P5b 主机桥静态发送同一 14 字节格式和 115200 baud。无实体 UART/CAN/电机、控制器回传或制动测试。
- 固件归档不能证明实体控制板正在运行此源码版本。

归档来源、依赖许可、SHA-256 和未纳入的厂商手册见[项目资产清单](../../docs/PROJECT_ASSETS.md)。
