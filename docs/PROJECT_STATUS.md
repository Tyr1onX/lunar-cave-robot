# 项目阶段状态与交接

更新日期：2026-10-13。P9-E 审查时 GitHub main 基线为 6a48dc52f5d7f5b263845bdec0639f59144a4486；P9-E PR 只增加文档与主机侧 CI，不改变 ROS/固件源码。最近一次完整 P7/P8 离线套件使用代码 SHA 07a56c3a33882aa2866f1076d8597776e2579bcb，完整结果 12/12 通过。PR #8 仍开放。仓库保持 Private；团队自有工程授权仅用于该私有仓库内部归档，第三方组件仍遵循其原许可证。

## P9-G NUC 与 Mac 控制台检查

本轮检查开始时 GitHub `main` 为 `b1cfe617209a06d69000a515af82b9b95add62ef`。NUC `~/lunar-cave-robot` 为干净的 `main` `6a48dc52f5d7f5b263845bdec0639f59144a4486`，与远程相比落后 2 个提交对象；差异是 P9-E 合并与文档/主机 CI 文件，本轮没有同步或修改 NUC。NUC 未登记 `deployed-sha`。

只读系统检查确认 Ubuntu 22.04.5、ROS 2 Humble、Nav2、rosbag2、Livox SDK2 库、14 GiB 内存和约 827 GiB 根分区空闲。ROS graph 没有活动节点，只看到 `/rosout` 和 `/parameter_events`；没有检查真实传感器反馈。NUC 两套 ROS 工作区的 PASS 记录位于 SHA `07a56c3a33882aa2866f1076d8597776e2579bcb`，P7/P8 完整套件记录是较旧且范围未知的 SHA `5124a68d398aa84b59a750e69636bf566f298a1c`。这些记录不是本轮对最新 `main` 的重跑。

首次现场测试判断、依赖阻塞、观察主题和安全限制见 [`NUC_READINESS_ASSESSMENT.md`](NUC_READINESS_ASSESSMENT.md)。NUC 可准备 MID-360 单传感器静态采集，但当前有线网卡 DOWN，必须先确认现场网络和配置。Orbbec 和模型状态后来在 P9-H 分支重新构建与核对，当前结论见下节及 [`P9H_PERCEPTION_READINESS.md`](P9H_PERCEPTION_READINESS.md)。实体 STM32 版本、设备反馈和停止行为均未核验。

Mac 端第一版只读界面位于 `tools/mac-console/`，复用 `device_status.sh` 并经固定 SSH 快照读取 NUC 主机与 ROS 图。源码、技术选型、运行方式和后续阶段见 [`MAC_CONSOLE_ARCHITECTURE.md`](MAC_CONSOLE_ARCHITECTURE.md)。当前页面只列 Topic 名称、不采样消息；不启动设备或发布控制指令。

## P9-H 感知集成复核（未合并分支）

- 本轮基于 PR #14 head `1ab8641777a5fd574b6c3f90a3103de6d778f15f`，未改动 NUC 的 `~/lunar-cave-robot`（仍为干净 SHA `6a48dc52f5d7f5b263845bdec0639f59144a4486`）。所有构建和模型测试位于 NUC 的临时隔离目录。
- ROS 2 Humble / Ubuntu 22.04.5 上隔离构建通过：Livox `livox_ros_driver2` 1.2.4、Orbbec `orbbec_camera_msgs`/`orbbec_camera` 2.9.3、`yolo_node` 1.0.0、`pose_estimator` 1.0.0。Orbbec 使用外置官方 SDK v2.9.3；系统原 v1.10.37 SDK 与 wrapper v2 头文件 ABI 不匹配。构建入口、依赖、SDK SHA 与复现说明见 [`P9H_PERCEPTION_READINESS.md`](P9H_PERCEPTION_READINESS.md)。
- 已检查 MID-360 静态 launch、Orbbec Gemini 330 系列 launch、相机选择 launch 和感知 bringup 的 `--show-args`，没有启动任何驱动节点。MID-360 launch 已将真实广播码设为必填项；仓库内主机/雷达 IP、端口和外参仅为未核实的历史候选值。
- NUC 旧 `~/tzb_ws/src/best_openvino_model/` 找到根 `best.bin` 和 blue-block XML/BIN/提示词文件，但没有将模型二进制放入仓库。根 `best` 单帧推理可运行但类别为 `haomu`。默认 blue-block 模型在 Ultralytics 8.4.164 / OpenVINO 2025.4.1 的分割后处理出现 `13×294`/`24×294` 对 `32×25600` 维度错误，ROS YOLO 节点不能完成初始化；不存在可复现的 `.pt` 训练 checkpoint 或完整导出记录。
- `pose_estimator` 默认使用的 `half_block.pcd` 已从 NUC 历史工作区按 SHA-256 `d4fe24118f3bd3d1a3a9db4fb8bef87873b486af8b0d3ec52416dabad5f5027d` 归档。Open3D 加载 7,500 点并完成尺度修正与 FPFH 预处理；这只验证参考模型预处理，不是目标场景或位姿估计验收。
- 当前未发现可用的场景 RGB-D Bag；使用一张 Orbbec 源码目录现有的 RGB 验证帧进行软件 smoke 检查，未将图像放入仓库。相机—YOLO—3D 位姿闭环、真实 Topic 数据、MID-360 UDP/网络和实机标定仍未验证。详细测试边界及首轮静态采集/回放步骤见 [`P9H_PERCEPTION_READINESS.md`](P9H_PERCEPTION_READINESS.md)。

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
