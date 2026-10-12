# 项目阶段状态与交接

更新日期：2026-10-12。GitHub `main` 基线 `ea3f64bd37d0372ad878adaf5a33b1dc73bea38c` 包含 P0～P8 ROS 2 主线和 P9-A 项目资产整理。P9-B 在独立分支继续完善构建与安全审计；分支及逐项结果见 `docs/STM32_BUILD_AUDIT.md`。仓库保持 Private；用户已确认团队自有工程可在此私有仓库内部归档。第三方组件仍遵循其原许可证，未据此推断公开发布权。

## 已归档

- `tzb_ws/` 与 `Nav2_rea_ws/` 保持两个独立 ROS 2 工作空间，包含机械臂规划、视觉、相机包装、导航、定位建图和底盘主机桥等 P8 主线源码。
- `firmware/arm/` 已归档机械臂 STM32H723 CubeMX、CMake、Keil、启动/链接配置、HAL/CMSIS、FreeRTOS、LibXR 及团队源码；工程仍缺两个 XRobot 生成文件。
- `firmware/chassis/` 已归档六轮底盘 STM32H723 CubeMX、CMake、Keil、启动/链接配置、HAL/CMSIS、FreeRTOS、DSP 库和团队源码；对不存在的旧 Keil 源文件条目已作明确清理。
- 原始 ZIP 哈希、固件文件清单、发现的历史工作区和未找到的机械/电气源资料见 `docs/PROJECT_ASSETS.md`。
- 软硬件接口与验证状态见 `docs/HARDWARE_SOFTWARE_MATRIX.md`。设备实时 SHA 查询和安全同步入口见 `docs/DEVICE_STATUS.md`、`docs/DEPLOYMENT_WORKFLOW.md`。

## 验收状态

- P8 历史验收：ThinkBook 和 NUC 在旧 P8 工作区记录各 12 个 P7 PTY 离线场景通过。NUC MID-360 驱动曾构建成功；未启动传感器采集。
- P9-B 底盘固件在 ThinkBook 隔离副本上使用 ARM GNU GCC 10.3.1 完整构建通过，产出 ELF/MAP/HEX/BIN；修复了 Linux 大小写敏感下的 `INS_task.h` include，并为现有命令租约判定增加可测的纯函数入口。生产解析器/命令快照主机测试（mock HAL）通过。
- P9-B 机械臂 CMake 可检测 ARM GCC/G++，但在 XRobot 检查处因缺少 `Modules/CMakeLists.txt` 中止；同时缺少 `User/xrobot_main.hpp`。没有伪造生成文件或产品配置；因此未产出 ELF。
- P9-B ThinkBook 隔离分支工作树上 `Nav2_rea_ws` 2 包、`tzb_ws` 6 包构建通过，底盘固件生产解析器主机测试通过。P7/P8 12 场景全套多次运行均因测试环境时序提前退出，最新全套状态为 FAIL；12 个场景分别在独立运行中均取得过 PASS。P4f 驱动 PTY 断言、P6 8 项测试和取消竞态通过；6 场景全套在 Nav2 返航的空路径/TF 时间问题退出，单独重跑重复触发场景通过。精确 SHA、范围、日志位置和失败原因见 `docs/STM32_BUILD_AUDIT.md` 与 ThinkBook 状态目录记录。不能将这些分散的单场景通过记成全套通过。
- P9-B ROS/P4f/P7 构建与离线测试没有连接实体控制器；P7/P8 源码和原有验证脚本没有改动。当前代码分支的安全、构建和测试结果不能描述为实机验收。
- P9 新统一 clone 的 ROS 2 构建和离线验证，以每端的实时状态查询及 P9 测试记录为准。不能将测试脚本未运行、历史通过或 PTY 模拟通过描述为实机验收。
- 当前没有确认控制器固件实装版本、真实 UART/USB/CAN 应答、真实雷达点云/IMU、相机标定、物理急停、制动或抓取运动。

## 尚需补齐

1. 从工程生成者处取得机械臂两个缺失生成文件或完整、可复现的生成配置。
2. 确认实体控制板当前固件 SHA/版本及对应构建记录。
3. 找回可编辑机械 CAD、原理图/PCB、BOM、接线图和硬件配置资料；目前只发现仓库已有的 URDF/STL 仿真网格。
4. 由机械臂固件负责人提供对应产品的 XRobot 生成文件或可复现配置，并补齐 C++ runtime 后重新构建；随后另行进行受控硬件接口、安全与实机验收。
5. 由负责人确认电机厂商 PDF 的团队分发权限后再决定是否归档。

完整源码交接状态：ROS 2 主线和两套团队 STM32 输入源码已集中到 Git；ThinkBook 上底盘有独立编译证据，机械臂仍缺两个 XRobot 生成文件及匹配的 C++ runtime。P7/P8 与 P4f 的完整套件目前都有测试时序失败记录，即使各场景独立复跑已通过，也不能将全套标为通过。机械/电气原始设计未找到，实体版本和实机安全状态未验收，因此整个软硬件项目尚未达到无条件完整交接状态。
