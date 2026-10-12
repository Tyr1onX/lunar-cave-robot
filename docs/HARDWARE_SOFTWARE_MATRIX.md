# 软硬件版本对应表

更新日期：2026-10-12。仓库中的源码版本随 Git 提交整体追踪；运行 `./tools/device_status.sh` 可查询 GitHub、ThinkBook 和 NUC 的实时 SHA。表中固件 ZIP 哈希用于识别归档来源，不能代表实体控制板的运行版本。

| 模块 | 源码版本/来源 | 控制接口 | 构建状态 | 实机状态 |
| --- | --- | --- | --- | --- |
| ROS 2 主系统 | 本仓库当前 main SHA；P8 主线 | ROS 2 Humble；两个独立 colcon 工作空间 | P8 已在 ThinkBook、NUC 有历史构建/离线记录；P9 合并后复验状态见设备状态脚本 | 未完成整机验收 |
| 底盘 STM32 | P9-B 分支源码；输入 ZIP SHA-256 `823ff6981859dbce1d05a3d4864b921bd0f9aa5165335d0caec7d12852701d2e` | USART1 115200/8N1；`A5 + 3×float32 little-endian + 4A`，14 字节；FDCAN1/2 | ThinkBook ARM GNU GCC 10.3.1 完整构建通过，产物与日志见[构建审计](STM32_BUILD_AUDIT.md)；生产解析器主机测试通过 | 控制板固件版本未知；UART、CAN、电机、停车均未验收 |
| 机械臂 STM32 | P9-B 分支源码；输入 ZIP SHA-256 `63ac07e173e148ab248b1689ccae64ba380b730d0c64b2b29d82b96307c48ab8`；包内 Git HEAD `963fd0275ddbe2c759b35250842284c6d62e90c1` | 固件配置 USB OTG HS CDC，46 字节下行/35 字节上行；UART5 为 RC 接收。ROS 主机接口使用设备配置，端点映射仍待确认 | ThinkBook CMake 已找到 ARM GCC/G++，但因 `Modules/CMakeLists.txt` 缺失而配置终止；`User/xrobot_main.hpp` 也缺失，未生成 ELF | 控制板固件版本未知；CDC 设备映射、帧应答与运动未验收 |
| MID-360 | 主仓库 `Nav2_rea_ws` 的 Livox 驱动源码 | UDP；驱动依赖 Livox SDK | NUC 既有记录为驱动构建通过；P9 不启动驱动 | 未采集真实点云/IMU |
| 相机 | `tzb_ws` 中相机驱动包装和视觉节点 | USB，相机具体型号及接口映射以设备配置为准 | ROS 包沿 P8 主线；本轮未启动相机 | 未采集图像或完成标定 |

## 使用规则

- 设备脚本显示的仓库 SHA 是源代码版本；NUC 的“当前部署版本”只从显式部署标记读取。同步 Git 源码不会设置部署标记，也不会启动程序。
- 不得将固件归档 SHA 或 STM32 工程 Git HEAD 当作控制板当前固件版本。只有读取控制器版本或由硬件负责人确认后，才能填写实机版本。
- STM32 协议比对属于静态源码核查，不构成 ROS 与固件联调或安全验证。
