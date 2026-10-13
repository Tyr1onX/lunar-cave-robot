# 项目阶段状态与交接

更新日期：2026-10-13。当前 GitHub `main` 基线为 `b1cfe617209a06d69000a515af82b9b95add62ef`；P9-F 分支基于该版本和 PR #8 head `c441e8dd2f666dfdca000f3b4e94826f14204209`。PR #8 仍开放。最近完整 P7/P8 离线套件在代码 SHA `07a56c3a33882aa2866f1076d8597776e2579bcb` 上通过 12/12，P9-F 本轮未重新运行该套件。仓库保持 Private；团队自有工程授权仅用于该私有仓库内部归档，第三方组件仍遵循其原许可证。

## P9-F 分支验证状态

- ThinkBook `qibai-ThinkBook-16-G6-AHP`、Ubuntu 22.04、GCC 11.4：底盘生产协议解析器、底盘安全状态机、机械臂纯 C 状态机、Host/RTOS 恢复集成，以及生产 `Remote.c`→安全输出门→`Joint::ControlStep()` 集成均 PASS。场景包括 100/101 ms 租约边界、过期旧 enable、快速切出再切回、重复 enable、TX/RX 失败、FDCAN 故障事件、单轴驱动故障、清错/分批使能期间断联和成功恢复。详细范围与模拟边界见 [P9-F 集成记录](P9F_ARM_SAFETY_INTEGRATION.md)。
- ThinkBook ROS 2 Humble 在隔离工作区对 `arm2nav_msgs`、`lite_motion_msgs`、`lite_motion_planner` 三包构建通过。该构建验证了删除未使用 Ruckig 依赖后的规划包配置。
- 机械臂 STM32 完整目标未构建：真实 XRobot 产品入口和生成模块仍缺失。底盘完整交叉编译结果仍以已有 P9-B 审计记录为准，本轮未重编译固件。
- GitHub Actions 的强制主机测试步骤已改为直接运行，不再静默跳过缺失的 FSM/集成测试；待本 PR 推送后核对 GitHub 检查结果。
- 未连接实体控制器、未打开 STM32 串口、未烧录、未启动电机。真实失能后的制动/重力负载保持和独立急停仍需硬件负责人确认。

## P9-E 审查时的历史状态

## P9-E 当前源码与验证状态

- P9-E 审查基线 SHA：6a48dc52f5d7f5b263845bdec0639f59144a4486。最近验证的可执行源码快照为 07a56c3a33882aa2866f1076d8597776e2579bcb；main 在该源码之后仅有文档提交。
- ThinkBook P7/P8 完整 12 场景离线套件在上述代码 SHA 上通过，运行记录 20261012T234558Z-699490，退出码 0；此前 arm_disconnect 运行曾因 Nav2 进度超时和恢复 TF 外推失败，未能注入故障而失败。后来统一工作目录整套重跑通过，未放宽断言。
- P9-C 记录的 P7 完整套件两轮均为 12/12。P4f/P6 的 PTY 驱动检查、逻辑/静态测试、取消竞争与模拟场景也有通过记录。详情见 P9C_OFFLINE_VALIDATION.md；这些均不是实机验收。
- ThinkBook 和 NUC 的 Nav2（2 包）及机械臂 ROS 工作区（6 包）构建均有通过记录。底盘 STM32 使用 ARM GNU GCC 10.3.1 交叉编译通过；机械臂因真实 XRobot 产品配置和生成文件缺失而未能完整构建。
- 本轮在 ThinkBook Ubuntu/GCC 11.4 上复跑：main SHA 6a48dc52f5d7f5b263845bdec0639f59144a4486 的底盘生产协议/命令主机测试 PASS；PR #8 精确 head c441e8dd2f666dfdca000f3b4e94826f14204209 的底盘和机械臂纯 C 安全 FSM 主机测试均 PASS。此前底盘交叉编译记录也通过。Mac 本机链接器无法读取安装 SDK 的 stub 描述文件，故使用 ThinkBook 复核；无设备连接。P9-E 审查发现机械臂实际控制路径仍有状态轮询窗口、Host 租约过期后续新竞态、恢复序列无复核和逐轴反馈新鲜度缺口；详见 P9E_PR8_INTEGRATION_REVIEW.md。PR #8 保持开放。
- 未连接实体 STM32，不烧录，不启用电机。底盘 relax 和机械臂 disable/enable 的真实制动、保持与重力负载行为仍待硬件负责人确认；独立实体急停也未验收。
- P9-E 新增 GitHub Actions 工作流，在 PR 与 main push 上运行底盘协议主机测试；当底盘/机械臂安全状态机测试源码存在时，也会以主机编译运行。工作流不访问串口、STM32 或电机。

## 已归档

- `tzb_ws/` 与 `Nav2_rea_ws/` 保持两个独立 ROS 2 工作空间，包含机械臂规划、视觉、相机包装、导航、定位建图和底盘主机桥等 P8 主线源码。
- `firmware/arm/` 已归档机械臂 STM32H723 CubeMX、CMake、Keil、启动/链接配置、HAL/CMSIS、FreeRTOS、LibXR 及团队源码；工程仍缺两个 XRobot 生成文件。
- `firmware/chassis/` 已归档六轮底盘 STM32H723 CubeMX、CMake、Keil、启动/链接配置、HAL/CMSIS、FreeRTOS、DSP 库和团队源码；对不存在的旧 Keil 源文件条目已作明确清理。
- 原始 ZIP 哈希、固件文件清单、发现的历史工作区和未找到的机械/电气源资料见 `docs/PROJECT_ASSETS.md`。
- 软硬件接口与验证状态见 `docs/HARDWARE_SOFTWARE_MATRIX.md`。设备实时 SHA 查询和安全同步入口见 `docs/DEVICE_STATUS.md`、`docs/DEPLOYMENT_WORKFLOW.md`。

## 验收状态

### P9-C 离线软件验证

- ThinkBook 上两轮完整 P7 12/12 场景通过，使用 DWB 离线地图代价配置修正和按场景关闭 ROS 2 CLI daemon。没有放宽测试断言。结果及源码 SHA-256 见 `docs/P9C_OFFLINE_VALIDATION.md`。
- P4f/P6 使用新结果目录和隔离 ROS domain 完成：19 项 ROS PTY 驱动检查、10 项逻辑/静态测试、规划取消竞态及六个 P6 合成场景通过。
- 上述结果是离线结果，不代表实体传感器、电机、底盘停止距离、机械臂承载或实机任务验收。

- P8 历史验收：ThinkBook 和 NUC 在旧 P8 工作区记录各 12 个 P7 PTY 离线场景通过。NUC MID-360 驱动曾构建成功；未启动传感器采集。
- P9-B 底盘固件在 ThinkBook 隔离副本上使用 ARM GNU GCC 10.3.1 完整构建通过，产出 ELF/MAP/HEX/BIN；修复了 Linux 大小写敏感下的 `INS_task.h` include，并为现有命令租约判定增加可测的纯函数入口。生产解析器/命令快照主机测试（mock HAL）通过。
- P9-B 机械臂 CMake 可检测 ARM GCC/G++，但在 XRobot 检查处因缺少 `Modules/CMakeLists.txt` 中止；同时缺少 `User/xrobot_main.hpp`。没有伪造生成文件或产品配置；因此未产出 ELF。
- P9-B ThinkBook 隔离分支工作树上 `Nav2_rea_ws` 2 包、`tzb_ws` 6 包构建通过，底盘固件生产解析器主机测试通过。该结论仅代表 P9-B 隔离工作树当时的历史快照；后续 P9-C/P9-D 在同一主线代码快照上完成 P7/P8 12 场景全套 PASS。P4f 驱动 PTY 断言、P6 8 项测试和取消竞态通过；6 场景全套在 Nav2 返航的空路径/TF 时间问题退出，单独重跑重复触发场景通过。精确 SHA、范围、日志位置和失败原因见 `docs/STM32_BUILD_AUDIT.md` 与 ThinkBook 状态目录记录。不能将这些分散的单场景通过记成全套通过。
- P9-B ROS/P4f/P7 构建与离线测试没有连接实体控制器；P7/P8 源码和原有验证脚本没有改动。当前代码分支的安全、构建和测试结果不能描述为实机验收。
- P9 新统一 clone 的 ROS 2 构建和离线验证，以每端的实时状态查询及 P9 测试记录为准。不能将测试脚本未运行、历史通过或 PTY 模拟通过描述为实机验收。
- 当前没有确认控制器固件实装版本、真实 UART/USB/CAN 应答、真实雷达点云/IMU、相机标定、物理急停、制动或抓取运动。

## 尚需补齐

1. 从工程生成者处取得机械臂两个缺失生成文件或完整、可复现的生成配置。
2. 确认实体控制板当前固件 SHA/版本及对应构建记录。
3. 找回可编辑机械 CAD、原理图/PCB、BOM、接线图和硬件配置资料；目前只发现仓库已有的 URDF/STL 仿真网格。
4. 由机械臂固件负责人提供对应产品的 XRobot 生成文件或可复现配置，并补齐 C++ runtime 后重新构建；随后另行进行受控硬件接口、安全与实机验收。
5. 由负责人确认电机厂商 PDF 的团队分发权限后再决定是否归档。

完整源码交接状态：ROS 2 主线和两套团队 STM32 输入源码已集中到 Git；底盘有独立编译证据，机械臂仍缺 XRobot 产品配置、模块锁定信息和两个生成文件，无法完整构建。P7/P8 最近完整离线套件记录为 12/12 通过，历史失败仍保留在验证记录中。机械/电气原始设计未找到，实体固件版本和实机安全状态未验收，因此尚不能称为完整的软硬件实机交接。
