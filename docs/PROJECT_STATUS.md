# 项目阶段状态与交接

更新日期：2026-10-12。GitHub `main` 包含 P0～P8 ROS 2 主线和 P9 项目资产整理。仓库保持 Private；用户已确认团队自有工程可在此私有仓库内部归档。第三方组件仍遵循其原许可证，未据此推断公开发布权。

## 已归档

- `tzb_ws/` 与 `Nav2_rea_ws/` 保持两个独立 ROS 2 工作空间，包含机械臂规划、视觉、相机包装、导航、定位建图和底盘主机桥等 P8 主线源码。
- `firmware/arm/` 已归档机械臂 STM32H723 CubeMX、CMake、Keil、启动/链接配置、HAL/CMSIS、FreeRTOS、LibXR 及团队源码；工程仍缺两个 XRobot 生成文件。
- `firmware/chassis/` 已归档六轮底盘 STM32H723 CubeMX、CMake、Keil、启动/链接配置、HAL/CMSIS、FreeRTOS、DSP 库和团队源码；对不存在的旧 Keil 源文件条目已作明确清理。
- 原始 ZIP 哈希、固件文件清单、发现的历史工作区和未找到的机械/电气源资料见 `docs/PROJECT_ASSETS.md`。
- 软硬件接口与验证状态见 `docs/HARDWARE_SOFTWARE_MATRIX.md`。设备实时 SHA 查询和安全同步入口见 `docs/DEVICE_STATUS.md`、`docs/DEPLOYMENT_WORKFLOW.md`。

## 验收状态

- P8 历史验收：ThinkBook 和 NUC 在旧 P8 工作区记录各 12 个 P7 PTY 离线场景通过。NUC MID-360 驱动曾构建成功；未启动传感器采集。
- P9 固件静态归档：两个工程的必要源文件、工程配置、启动文件、链接配置和所需构建依赖已做完整性核验。设备上没有 `arm-none-eabi-gcc`，因此没有 STM32 编译结果。
- 机械臂 CMake 依赖两个未找到的 XRobot 生成文件 `User/xrobot_main.hpp` 和 `Modules/CMakeLists.txt`，当前不能独立配置/编译。
- P9 新统一 clone 的 ROS 2 构建和离线验证，以每端的实时状态查询及 P9 测试记录为准。不能将测试脚本未运行、历史通过或 PTY 模拟通过描述为实机验收。
- 当前没有确认控制器固件实装版本、真实 UART/USB/CAN 应答、真实雷达点云/IMU、相机标定、物理急停、制动或抓取运动。

## 尚需补齐

1. 从工程生成者处取得机械臂两个缺失生成文件或完整、可复现的生成配置。
2. 确认实体控制板当前固件 SHA/版本及对应构建记录。
3. 找回可编辑机械 CAD、原理图/PCB、BOM、接线图和硬件配置资料；目前只发现仓库已有的 URDF/STL 仿真网格。
4. 在安装所需工具链后构建两套固件；随后另行进行受控硬件接口、安全与实机验收。
5. 由负责人确认电机厂商 PDF 的团队分发权限后再决定是否归档。

完整源码交接状态：ROS 2 主线和已确认的两套团队 STM32 输入源码已集中到 Git；机械臂固件暂缺两个生成文件，机械/电气原始设计未找到，实体版本与实机状态未验收，因此整个软硬件项目尚未达到无条件完整交接状态。
