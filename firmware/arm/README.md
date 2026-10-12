# 机械臂 STM32 固件

该目录是从本机 `lunar_rover.zip` 整理出的工程快照，原压缩包 SHA-256 为 `63ac07e173e148ab248b1689ccae64ba380b730d0c64b2b29d82b96307c48ab8`。包内 Git `HEAD` 为 `963fd0275ddbe2c759b35250842284c6d62e90c1`；本次同时保留了压缩包中相对该提交的团队工作区源码变化和新增文件。LibXR 依赖快照为 `783b157fb3e9e5a7801edc9083834ba3ac49801f`。原始 ZIP 未被修改。

## 工程内容

- MCU：STM32H723VGT6（CubeMX 标识 `STM32H723VGTx`）。
- 工程入口：`lunar_rover.ioc`、根目录 `CMakeLists.txt` / `CMakePresets.json`、`MDK-ARM/lunar_rover.uvprojx`。
- 包含 `Core/`、STM32 HAL、CMSIS、USB Device CDC、FreeRTOS、启动文件和链接脚本、LibXR、团队 `User/` 及 `others/` 应用源码。
- 构建使用 CMake/Ninja 与 ARM GNU 工具链；Keil 工程需 MDK-ARM 5.32 及对应的 STM32H7 设备包。

## 构建阻塞

当前归档中的 `User/app_main.cpp` 包含 `xrobot_main.hpp`，LibXR 的 CMake 检查还要求 `Modules/CMakeLists.txt`。这两个生成文件不在 ZIP、Git 对象历史、ThinkBook 或 NUC 可访问工程中。`Modules/modules.yaml` 与 `sources.yaml` 保留了配置，但不能替代生成文件。因此目前不能确认 CMake 工程可独立配置或编译；需要从固件负责人处恢复该产品对应的 XRobot 生成文件，或由负责人确认生成方式和模块版本后再生成。

P9-B 在 ThinkBook 上通过隔离工具目录准备了 ARM GNU GCC 10.3.1；CMake 能识别 GCC/G++，随后在 LibXR 的 XRobot 集成检查处因缺少 `Modules/CMakeLists.txt` 停止。`User/xrobot_main.hpp` 也缺失，恢复前无法生成 ELF。当前独立工具链还没有安装随 ARM GNU 工具链单独分发的 newlib C++ headers/runtime；补齐生成输入后需要一并安装，再重新构建。具体日志和工具链来源见[STM32 构建审计](../../docs/STM32_BUILD_AUDIT.md)。没有烧录；普通工程构建不是实机安全验收。

## 静态接口摘要

- USB OTG HS CDC 是 ROS 2 主机数据端点候选；源码定义 46 字节下行目标帧与 35 字节上行状态帧。
- FDCAN3 源码配置名义速率 1 Mbps、数据相位 2 Mbps，并启用 FD/BRS；需与关节驱动器配置核对。源码中另有 classic CAN 注释，存在需硬件负责人确认的表述差异。
- UART5 配置为 100000 baud、9 位/偶校验的遥控接收输入，不是 ROS 2 主机命令接口。
- ROS 2 的主机帧生成逻辑与字节布局已作静态核对；没有做 USB CDC 设备映射、实体 MCU 应答或关节安全测试。

STM32 当前运行版本未知。源码变化、依赖来源和归档排除项见[项目资产清单](../../docs/PROJECT_ASSETS.md)与[软硬件版本对应表](../../docs/HARDWARE_SOFTWARE_MATRIX.md)。第三方许可保留在各依赖目录。
