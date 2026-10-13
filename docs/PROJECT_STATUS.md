# 项目阶段状态与交接

更新日期：2026-10-13。P9-I 开始时 GitHub `main` 为 `38b9578a66015d02fe637a5c9a2b424af6a247a7`；本页新增的 P9-I 改动仍在独立 PR 分支，未合并。P9-E 的 `6a48dc52f5d7f5b263845bdec0639f59144a4486` 是历史审计基线，不是当前 main。最近一次完整 P7/P8 离线套件使用代码 SHA `07a56c3a33882aa2866f1076d8597776e2579bcb`，结果 12/12 通过。PR #8 仍开放。仓库保持 Private；团队自有工程授权仅用于该私有仓库内部归档，第三方组件仍遵循其原许可证。

## P9-I 视觉推理恢复与离线接口验证（未合并）

- 本轮在 NUC 临时隔离副本中基于 PR #14 源快照和 PR #16 感知修复构建/测试；NUC 正式仓库工作树没有同步或改动。P9-I 分支以最新 `main` 为目标，不改 Mac GUI 源码；PR #14 与 #16 的叠加关系仍需在各自 PR 中处理。
- `blue_block` 模型原始 OpenVINO 合同为输入 `[1,3,640,640]`，输出 `[1,300,38]` 和 `[1,32,160,160]`。运行 metadata 漏掉 `end2end: true` 令 Ultralytics 把 300 行误作通道数，复现 `13×294` 对 `32×25600` 的分割掩膜矩阵错误。恢复真实导出标记后，Ultralytics 8.4.164 / OpenVINO 2025.4.1 直接推理通过；模型任务仍为 YOLOE `segment`，没有更改成别的任务或替换成 `haomu`。
- NUC ROS Humble 隔离构建通过：`livox_ros_driver2` 1.2.4（SDK2 1.4.3）、`orbbec_camera_msgs`/`orbbec_camera` 2.9.3（官方 SDK v2.9.3）、`yolo_node` 1.0.0、`pose_estimator` 1.0.0。最新 YOLO 与位姿 PointCloud2 校验源码分别在独立隔离 build/install 前缀重新构建。
- 实际 ROS YOLO 节点接收合成 RGB 图后发布 `DetectionArray` 与标注图像，坐标、置信度范围、图像尺寸、时间戳和 frame 字段断言通过；合成 DetectionArray + organized PointCloud2 + CameraInfo 使实际位姿节点输出有限位置、单位四元数、同帧同时间戳的 `PoseStamped`。空检测与缺 XYZ 字段点云的负例均无位姿输出，节点继续运行。数值、耗时和复现命令见 `docs/P9I_VISION_OFFLINE_VALIDATION.md`。
- Orbbec 安装前缀搬迁后的 `ldd` 解析通过：SDK 从相机包自身相对库目录加载，未留下临时 SDK 路径；在 `BUILDING_PACKAGE=1`、`INSTALL_UDEV_RULES=OFF`、带隔离 `DESTDIR` 的安装试验未产生 udev 规则。Livox 通过全新非 symlink 隔离安装构建，依赖均解析且安装二进制没有临时工作树 RPATH。
- blue-block BIN、提示词和 embedding 二进制继续留在 ThinkBook/NUC 原始历史工程及本轮临时副本，不进入 Git；仓库只加入修复后的、安全脱敏 metadata 和可复跑脚本。测试图像和点云都是合成输入；没有 ROS Bag，没有真实相机/雷达、实机识别率或真实位姿精度结论。依赖模型授权与外部 SDK 版本仍要按各自条款和来源获取。
- 本轮没有启动 Livox/Orbbec 驱动节点，只检查 `--show-args`；未连接 STM32、未触碰串口/电机、不烧录、不启动任何真实运动。

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
