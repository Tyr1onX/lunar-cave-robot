
# 六轮底盘 STM32 工程归档状态

状态：源码审计报告与离线模型测试已纳入归档；本目录当前只有本说明，不包含可编译固件。

输入包：DM-car-six-wheel.zip  
SHA-256：823ff6981859dbce1d05a3d4864b921bd0f9aa5165335d0caec7d12852701d2e  
该哈希与 P5a 报告记录一致。完整源码审计见 docs/p5a/chassis-source-audit.md。

## 结构、许可与清理

输入包包含 STM32H723 的 Core/User 源码、CubeMX .ioc、CMake/CMakePresets、Keil MDK-ARM 工程和链接脚本，工程结构基本完整。压缩包解压约 894 MB，包含数百个 Keil/CMake 编译产物、CMSIS 预编译库、目标 ELF/AXF/HEX/map/object 文件、J-Link/CMake 日志和本机路径信息。

工程根目录没有 LICENSE；发现的许可文件只覆盖 CMSIS、ST HAL、FreeRTOS 等第三方组件。团队内部再分发授权和第一方源码许可均未找到。Private 仓库不替代这些授权。取得书面授权前，不上传工程源码、压缩包、预编译库、日志或其他固件二进制。

## 接口摘要

固件 USART1 使用 115200 baud、8N1、DMA；14 字节帧为 A5、VX/VY/VZ 三个小端 float32、4A。控制器通过 FDCAN1/2 管理六个 HM G36 电机，并以 BMI088 姿态反馈参与控制。当前 Nav2 ROS 2 串口驱动配置 9600 baud，且标准 bringup 未启动该桥；其完整阻塞与风险列表见 P5a 审计。未进行电机/CAN/UART 实机测试。
