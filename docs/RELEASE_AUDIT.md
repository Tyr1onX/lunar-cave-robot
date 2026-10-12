# P9-B 固件构建与安全结论 — 2026-10-12

P9-B 基于 `main` `ea3f64bd37d0372ad878adaf5a33b1dc73bea38c`，在 `p9b/stm32-build-and-safety-audit` 独立分支进行；完整构建日志和产物 SHA 见[STM32 构建审计](STM32_BUILD_AUDIT.md)。

- ThinkBook ARM GNU GCC 10.3.1 完整构建底盘 STM32 成功；修正 Linux 下头文件大小写，并将原 200 ms 命令年龄谓词抽为可单测函数但保持阈值不变。固件 ELF、MAP、HEX、BIN 保存在用户缓存，不提交。
- ThinkBook 对 `Nav2_rea_ws` 2 个包与 `tzb_ws` 6 个包的独立 ROS 2 构建均通过；底盘生产解析器与命令租约主机测试通过。该 P9-B 源码快照的 P7/P8 12 场景及 P4f 六场景完整套件未通过；其具体失败和单场景结果仍按当时源码记录。P9-C 后续已在 `main` 修复并连续两轮通过 P7 全套，详见下节。
- 机械臂 CMake 找到 ARM GCC/G++ 后因 XRobot 生成的 `Modules/CMakeLists.txt` 缺失而停止；`User/xrobot_main.hpp` 也未找到。没有生成空文件或猜测产品配置。
- 底盘主机侧测试直接编译生产 parser/command 逻辑并 mock HAL，结果 PASS；测试确认有限数据位翻转可穿过无 CRC 帧校验、UART 错误恢复会保留半帧。
- ROS 2 到两套固件的静态协议字段已对照。机械臂 MCU 不消费主机任务序号，USB 失联和 ROS 取消没有固件停止确认；底盘初始化先发送驱动使能帧，缺少总线级停车确认。
- 安全审计仍有 S0 阻塞项；未开串口、未烧录、未启动驱动器。仓库保持 Private。

# P9-C 离线稳定性审计 — 2026-10-13

ThinkBook 两轮 P7 完整离线场景各 12/12 通过；P4f/P6 构建、PTY 驱动、10 项逻辑/静态检查、规划取消竞态及六个 P6 场景通过。测试使用独立 ROS domain、新结果路径、PTY 和合成数据，不连接实体执行器。P7 的 DWB 离线地图配置与 Humble CLI daemon 清理原因、场景耗时、验证目录和源码 SHA-256 见 [P9-C 离线验证](P9C_OFFLINE_VALIDATION.md)。机械臂物理保持、实体急停、底盘制动和实机传感器链路仍未验收。

P9-C 软件 PR #7 已于 `a0c20df34b1b00cc6a00bea30df13d7ebb02f073` 合并到 `main`。STM32 安全状态机 PR #8 仍基于 P9-B PR #6 独立等待硬件负责人审查；主机测试与底盘交叉编译不代表物理停止或保持已验证。

以下 P9-A 和更早章节是历史审计记录；若与本节的后续构建结论冲突，以 P9-C 的验证记录为准。

# P9-A 当前归档结论 — 2026-10-12

本节覆盖本轮资产归档；下文为阶段历史审计。用户已明确确认机械臂和六轮底盘是战队自有工程，并授权归档到 Private 团队仓库。因此下文早期“无项目级许可证/内部授权所以不上传”的结论已不适用于这两套团队工程。仓库保持 Private；团队授权不改变第三方许可证，也不代表可公开发布。

- 两套 STM32H723 工程已从原始 ZIP 整理进 `firmware/arm/`、`firmware/chassis/`，保留 CubeMX、CMake、Keil、启动/链接配置、源码和必要依赖。输入 ZIP 哈希、源码来源、筛选依据和逐文件校验入口见 [`PROJECT_ASSETS.md`](PROJECT_ASSETS.md) 与 `firmware/SOURCE_SHA256SUMS`。
- 底盘工程静态结构完整；机械臂工程缺少 `User/xrobot_main.hpp`、`Modules/CMakeLists.txt`，不能独立配置。ThinkBook 和 NUC 均未找到 `arm-none-eabi-gcc`，本轮未编译任何固件。实体控制器版本未知。
- P0～P8 ROS 主线没有被历史源码覆盖。旧 `nav_ws.zip`、`serial_t.zip` 和 NUC 的 `mid360_driver_dev` 均经对照后未导入。原设备工作区保持不变，P9 统一 clone 独立管理。
- 可访问路径中没有找到可编辑 CAD、原理图/PCB、BOM 或接线图。底盘 ZIP 内电机厂商 PDF 的再分发条件未确认，未上传并保留在原包。
- 未打开真实 STM32 串口、未烧录、未启动机器人或电机。新统一目录离线测试以设备状态脚本记录；历史 P8 PTY 通过不代表实机验收。

# Pre-publication audit — 2026-10-11

本页末尾追加了 2026-10-12 的 ThinkBook 全项目源码归档审计；较早章节是历史阶段记录，不应覆盖新审计结论。

Status: **private repository import authorized; not cleared for public release**. Staging: `~/lunar-cave-robot-staging/` on ThinkBook; NUC unchanged.

## Source snapshots

- `tzb_ws/` from branch `p2b/real-vision-data`, commit `b5bd744` (P0, P1, P2, P2b fixes carried as working-tree content, not Git history).
- `Nav2_rea_ws/` from branch `p0/offline-build`, commit `081ba74`.
- `Nav2_rea_ws/src/localization/dlio/` copies the **modified working tree**, not its nested Git directory or history (upstream `b446a3d`); it includes the restored local changes.
- `p1_offline/` includes only the Python probes; outdated absolute-path P1 YAML removed; excludes run logs and transient helper scripts.
- P2/P2b scripts under `tzb_ws/tools/` retained; README reflects incomplete real-sensor verification.

## P3–P4b private archive update

The integration branch imports only the four source patches in dependency order (P3 `f91bc6a`, P3b `e402434`, P4a `67359bd`, P4b `9d48d5d`), creating new commits from the current private `main`; the original Git history is not imported. See [P3–P4b validation](P3_P4B_VALIDATION.md) for test commands, observed results, and remaining hardware boundaries. Navigation contents were not changed.

The original STM32 firmware project and any firmware ZIP remain outside Git: permission to redistribute, third-party terms, and compiled firmware artifacts have not been cleared. The upper-computer protocol tests are not hardware acceptance evidence. Existing public-release ownership and license blockers below remain unchanged.

## Intentionally excluded

- All `.git` data, virtual environments, build/install/log outputs, Python caches, PID files.
- Local RGB-D bags, private PCD/PGM maps and map scans, large upstream demonstration images and saved Orbbec sample frames.
- Model `.bin` weights (`best.bin`, `yoloe-26l-seg.bin` and other OpenVINO binaries); their `*.xml` architecture and sanitized metadata may remain.
- Bundled Orbbec SDK `SDK/lib` precompiled objects; locally installed Livox SDK `vendor/livox_sdk` binaries.
- Any files not explicitly in the source workspaces or approved offline test resources.

## Open risks before publishing

1. **Ownership / permission:** NUC-recovered team code and DLIO modifications require project-owner publication approval.
2. **Licensing:** mixed GPL-2.0, Apache-2.0, MIT, BSD-like, AGPL-3.0; verify source-level attributions and distribution obligations. The absence of a repository-wide LICENSE is intentional.
3. **Binary provenance:** Orbbec proprietary/redistributable subcomponents and Ultralytics trained models are withheld.
4. **Portability:** legacy hardware-facing YAML/C++ contains previous-machine absolute paths; P2 offline launch resolves package paths, but real robot bringup requires separate review.
5. **Completeness:** P2b has no approved real-world camera bag, TF/calibration evidence, or closed-loop arm acceptance. Missing `tzb_state_test` prevents some runtime integration.
6. **Reproducibility:** full 14+18 package builds require hardware SDK dependencies omitted from this publishable snapshot. Verify safe offline subset separately.

**Do not make this repository public before resolving ownership, licenses, model rights and legacy configuration risks.**

## Local staging verification

- Arm: 13 packages compiled; the Orbbec camera driver was excluded because its binary SDK is not distributed. The pose estimator's optional camera runtime dependency was corrected in staging only.
- Navigation: 14 packages compiled; the Livox driver, FAST-LIO, Point-LIO and pointcloud_tools were excluded pending a separately supplied Livox SDK.
- Safe P2 test: synthetic grasp, detach, placement and return to observe position completed on ROS domain 187 with localhost-only transport. The process was then intentionally interrupted.
- Python and Bash syntax checks passed. No physical robot or sensor driver was activated.
- At least 24 source files still contain prior-machine absolute path references. Hardware bringup is not portable or approved for unattended operation.
- Exact candidate inventory and checksums are stored outside Git in the ThinkBook home directory, as lunar-cave-robot-preupload-manifest.txt and lunar-cave-robot-preupload-sha256.txt.


## 2026-10-11 统一归档分支审计

### 分支与历史边界

统一分支从 archive/p3-p4b-offline-validation 的提交 bb6e61f0ba330cc060bd2aaa92cdf73391ce7dbc 建立。没有改写或覆盖该分支，也没有合并 main、P5b 或旧源码仓库的 Git 历史。P5a 仅将同一目标仓库中的审计文档和离线模型测试文件作为普通文件纳入新树；来源提交分别为 1a5a467c012e268f0154e32cc09d2c544e03bb84 和 92b1203d6fccb70a874e2811596734f0d1565c0b。

P4c 核查结果（旧 unified-engineering-assets 分支当时）：目标仓库分支列表没有 p4c/arm-control-safety；短 SHA 2964e87 无法解析，故当时未导入报告/测试。之后的 P5b 分支归档了 P4c 审计报告和故障注入脚本，见本文末尾的 P5b 统一归档修订审计。原始提交来源仍待核实。

### STM32 输入包审计与处置

| 输入包 | 结构与风险 | 处置 |
| --- | --- | --- |
| lunar_rover.zip，SHA-256 63ac07e173e148ab248b1689ccae64ba380b730d0c64b2b29d82b96307c48ab8 | 4037 个文件；含 Core/User、CubeMX IOC、CMake/CMakePresets、Keil 项目配置。根目录没有 LICENSE；包内 Git 配置指向 champ719/lunar_rover，HEAD 为 963fd0275ddbe2c759b35250842284c6d62e90c1。上游仓库未声明 GitHub license。包内还含 Git/submodule 元数据、LibXR/Eigen 等第三方源码、数百个构建/缓存文件、Keil 目标产物及一个无关可执行文件。许可文件只覆盖各自组件，不能授权整个机械臂工程；内部团队再分发授权未找到。 | 不上传源码、压缩包、Git 历史、子模块或目标文件。只保留 firmware/arm/README.md 审计说明。 |
| DM-car-six-wheel.zip，SHA-256 823ff6981859dbce1d05a3d4864b921bd0f9aa5165335d0caec7d12852701d2e | 与 P5a 审计报告记录的输入哈希一致。2252 个文件，约 894 MB 解压内容；含 STM32/CubeMX/CMake/Keil 配置。工程根目录没有 LICENSE；只发现 CMSIS、ST HAL、FreeRTOS 等组件许可。压缩包包含数百个 Keil/CMake 编译产物或二进制库，以及 J-Link/CMake 机器日志和乱码命名资料。未发现凭据文件名或私钥/常见访问令牌模式；绝对路径命中集中在日志、临时构建数据及测试/样例内容。团队内部再分发授权未找到。 | 不上传源码、压缩包、供应商二进制、日志或构建目录。只保留 firmware/chassis/README.md 和 P5a 已审计报告/模型测试。 |

机械臂包中的构建/IDE 日志与生成目录包含机器路径候选；Eigen 上游测试脚本也出现绝对路径候选。底盘包中的 JLinkLog.txt、build_log.txt 和 CMakeConfigureLog.yaml 有本机路径。最终归档不包含这些文件。两份包都没有项目根许可证或团队内部发布授权记录；Private 仓库权限不能替代许可或作者授权。

### .gitignore 与大文件检查

已补充 firmware 路径下的 CMake、Keil、调试缓存、日志和目标文件忽略规则。未加入任何固件 ZIP、可执行文件、静态库、HEX、ELF、map 或 object 文件。归档基线已有两个超过 10 MB 的资产（AstraPlus.stl 约 32.8 MB、DLIO 示例 GIF 约 14.4 MB），它们没有在本次修改中新增或更改；仍需在公开发布前另行审查。

### 离线验证

- 本机从 P5a 分支取出的 Python 标准库模型测试：13 项通过，0 项失败；覆盖 UART 帧、六轮分配、模式/超时边界及 ROS 速度路径模型，不访问 ROS、串口、CAN 或硬件。
- P3–P4b 的历史验收记录：23 tests，0 errors、0 failures、0 skipped，P3b 动态 TF 与合成抓放流程通过。本机为 macOS，未安装 ROS 2、colcon 或 arm-none-eabi-gcc，因此本次没有重跑这些 ROS 工作空间测试或编译 STM32 工程。
- P5a 测试不是底盘固件/ROS 集成测试；P3–P4b 的历史通过结果也不是实机验收。没有修改 ROS 工作空间源文件，没有连接、部署或烧录硬件。

旧 archive/unified-engineering-assets 分支只增加审计文档、P5a 模型测试、固件缺口说明和忽略规则；P3–P4b 源码树与其基线保持一致。后续 P5b 集成修订另见本文末尾。


## 2026-10-11 P5b 统一归档修订审计

### 代码基线、历史边界与选入文件

新分支 archive/unified-engineering-assets-p5b 从 p5b/chassis-ros2-integration 的 HEAD eefd91e5d4e378e15b34dc37872413f097131132 直接建立。该 HEAD 以 archive/p3-p4b-offline-validation 的 bb6e61f0ba330cc060bd2aaa92cdf73391ce7dbc 为共同祖先；P5b 分支在共同祖先上有 25 个提交。没有合并 main、没有覆盖旧统一归档分支，也没有把来源不同仓库的旧历史接入。

选择性带入旧统一归档的 README.md、docs/ARCHITECTURE.md、docs/RELEASE_AUDIT.md、docs/PROJECT_STATUS.md、.gitignore 以及 firmware/arm/README.md 和 firmware/chassis/README.md；另修正 P5b 基线自带的 docs/p5b/run-offline-validation.md 克隆分支说明。P5b 基线原有 ROS 2 bridge、Nav2 配置/launch、P4c/P5a/P5b 报告和离线测试都保留；P5b 的 P5a 审计报告版本优先保留，不用旧归档副本覆盖。新提交只改文档/忽略规则并新增固件授权说明，不改写 P5b 实现。

### P4c 和 P5a 来源

P4c 审计报告 docs/P4c_control_safety_audit.md 由提交 728d7d056b88bc5f144d9b08c483beb04ddb31d5 纳入，PTY 故障注入脚本 tzb_ws/src/serial/driver/test/test_p4c_control_safety.py 由提交 4a194196d92afa297e75c2ac49182186dc896d8b 纳入。原始报告短 SHA 2964e87 仍未能在目标仓库解析，来源仓库、完整 SHA 和此前 HEAD 差异仍待核实；这两个仓库内提交能追踪归档副本，不能证明原始历史来源。

P5a 审计和模型测试原始来源分别为 1a5a467c012e268f0154e32cc09d2c544e03bb84、92b1203d6fccb70a874e2811596734f0d1565c0b；P5b 基线中的归档提交分别为 8c82add5231c473fc1c91113b345290cfde58736、7c406c0d9163b1e0d3f09f0615ffbc158c5e33b9。

### 固件许可与上传范围

机械臂 lunar_rover.zip 与六轮底盘 DM-car-six-wheel.zip 仍不上传。两套工程都缺少项目级授权声明和团队内部再分发授权证明；第三方组件许可证不能替代整个工程的作者/权利人许可，Private 仓库也不构成许可。保留两份 firmware README 的来源、构建文件与授权阻塞说明。没有把 ZIP、固件源码、供应商二进制、编译产物或日志加入新分支。

### 本轮差异和安全检查

P5b 分支的桥接实现、包名、参数、启动入口和 tests 均作为新分支基线保留。最新 P5b HEAD 删除已被 chassis_serial_bridge 替代的旧串口 sender；此删除保持有效，没有恢复旧 packet.hpp/node.hpp，也没有用 archive 分支旧版实现覆盖新代码。P4c、P5a、P5b 的报告和测试路径均在基线树中。

归档文档没有增加密钥、凭据、固件二进制或机器日志。P5b 的三个 Nav2 bringup 需要显式 chassis_device，配置验证确认桥接使用 115200、14 字节帧、20 ms 周期、100 ms 超时和 DiffDrive 模型。底盘固件仍未上传。

### 本轮离线验证结果与限制

- P5a Python 标准库模型回归：13 项通过，0 失败。
- P5b 静态配置验证：5 组检查全部通过，覆盖三个 bringup 单一串口所有权、Nav2/smoother topic 路由、差速运动限制与超时、桥接帧参数、机械臂/底盘命令隔离。
- Python 语法检查：P4c/P5b 离线脚本和 P5a 测试通过。
- P5b 原报告记载在 ThinkBook Ubuntu 22.04 / ROS 2 Humble 上构建 chassis_serial_bridge，并用实际 velocity_smoother、bridge 节点与 PTY 模拟接收端通过集成验证。该历史报告列出 7 类控制路径及断线/超时/关闭检查。
- 当前执行端是 macOS，未安装 ROS 2、rclpy 或 colcon。因此本轮没有重新构建 ROS 包，也没有重跑 P5b/P4c ROS PTY 测试；调用相应命令分别因缺少 colcon、rclpy、ros2 而停止。P3–P4b 历史 23 项 ROS 测试也没有在此主机重跑。
- 以上离线数据均不涉及实体串口、CAN、电机、相机、雷达或实车。P5b 是单向 ROS 命令链路，不是底盘反馈闭环或实机验收。

## 2026-10-12 ThinkBook 全项目源码归档审计

### 检查范围与 Git 基线

- 通过只读 SSH 核对 ThinkBook 系统、用户主目录下明确的 `lunar-cave-*` 工程、`robot_p0` 内嵌 Git 仓库、`teacher_robot_sources`、常用工程投递目录、可访问挂载点及本地分支。没有遍历无关个人目录，也没有修改 ThinkBook 原工程。
- ThinkBook 上检查到 `lunar-cave-p8-validation`（`p8/nuc-integration-ready`，`f6005a0`）、P4f 工作区（`p4f/arm-trajectory-safety`，`8accb4e`，含未提交改动）、P6、P7、staging/main 和 `robot_p0` 下 P0～P4c 的嵌套工作区。另见 P0 初始 bundle 及 `teacher_robot_sources/imported/{nav_ws.zip,serial_t.zip}`。检查过的候选根目录中未发现 CubeMX `.ioc`、Keil `.uvproj[x]` 工程文件或机械臂/底盘 ZIP 的另一个副本。
- GitHub 仓库保持 Private。审计时最新 `main` 是 `35e388f6a064fcb6e229d08d74f2eea6126cae9f`；远端历史包含 P0～P8 阶段分支，ThinkBook P8 七个领先提交已经在该 `main` 中。工作空间 `tzb_ws/`、`Nav2_rea_ws/` 与 P8 基线的 Git 差异为空。
- P4f 工作区的 10 个已跟踪改动中，7 个与 P8 相同；4 个未跟踪说明/脚本也已在 P8。仅 3 个源文件存在独有差异：P6 launch、`mission_coordinator.py` 和 `tzb_catch.cpp`。逐段比较发现它们回退了 P8 的底盘状态接入、任务命令 keepalive/hold、反馈检查、安全故障门控和目标朝向处理。它们是旧行为回退，未导入。P0/P6/P7 的旧工作区仅有已被 P8 替代的旧版本或第三方 SDK/示例产物，没有复制整套历史工作区。

### STM32 固件工程及授权处置

| 工程 | 结构/硬件与工具链 | 编译与处置 |
| --- | --- | --- |
| 六轮底盘 `DM-car-six-wheel.zip` | 本次用户提供的包 SHA-256 `823ff6981859dbce1d05a3d4864b921bd0f9aa5165335d0caec7d12852701d2e`；2252 项、解压约 894 MB。含 STM32H723VGTx、CubeMX `.ioc`、Core/Drivers/Middlewares/User、CMake/CMakePresets、Keil MDK-ARM 工程、启动文件和链接脚本，工程结构基本完整。包还含构建产物、预编译库和日志；根目录无项目 LICENSE。 | ThinkBook 与本机均无 `arm-none-eabi-gcc`、STM32CubeIDE 或 Keil/armclang，未编译。第一方及团队内部共享授权未确认，第三方 HAL/CMSIS/FreeRTOS 许可证不能授权整个工程；不上传源码、二进制、日志或 ZIP。细节见 [`firmware/chassis/README.md`](../firmware/chassis/README.md)。 |
| 机械臂 `lunar_rover.zip` | 既有审计记录 SHA-256 `63ac07e173e148ab248b1689ccae64ba380b730d0c64b2b29d82b96307c48ab8`、包内 Git HEAD `963fd0275ddbe2c759b35250842284c6d62e90c1`；记录的工程包含 STM32H723、CubeMX、CMake/Keil、启动及链接文件，结构基本完整。此次在 ThinkBook 候选路径没有找到原始 ZIP，因此本轮未重新核对包内容。 | 无本机 STM32 工具链，未编译。项目及内部共享授权没有确认，历史审计所述上游无项目许可证；不上传源码、子模块、预编译内容或 ZIP。具体限制见 [`firmware/arm/README.md`](../firmware/arm/README.md)。 |

两套工程均保留 MCU/工具链、启动/链接、HAL/CMSIS、通信、电机及当前 ROS 2 接口的静态审计摘要，但**当前仓库没有可编译的两套固件工程**。底盘固件串口 115200/8N1、14 字节协议与 P5b ROS 主机帧参数相符；未证明 UART/CAN/电机互通。机械臂包审计记录为 USB OTG HS CDC 的 46 字节下行/35 字节上行，而当前 ROS 2 驱动使用串口设备配置，设备映射及字段兼容仍待有授权的受控验证。未连接或驱动实体硬件。

### 其他候选源码及替代关系

- ThinkBook `robot_p0`、P6、P7 以及 staging 的 ROS 代码被主线 P8 取代；P0 中旧工作区有旧版文件和 Orbbec SDK 二进制、示例图片等，只在对照后排除，没有将旧工作空间叠加到当前树。
- `nav_ws.zip` SHA-256 `0d4d3cb04085fc6bd5f5a351e84124f33866ad643b224e2381f1027f7f7a925c`：对照 `Nav2_rea_ws` 有 206 个相同文件、21 个不同文件、26 个包内独有文件、143 个主线独有文件。包内 `competition_online_navigation` 启动/参数、目标序列及 Nanling 地图属于额外候选，但归档说明将其标为教师提供，未找到可在团队私有仓库共享的授权；另含编辑器配置、缓存、运行日志、PCD/地图数据。没有导入。
- `serial_t.zip` SHA-256 `49f0cefbe386ab812ec38084d8d960fa4fe47776bcc16e54df6c8a6cb2328476`：10 项、约 38 KB；其中 ROS 驱动源码与 P8 当前串口实现不同，P8 含后续协议安全策略。压缩包内源文件标注与包元数据中的许可证不一致，仓库链接亦与相机项目无关，无法据此确认权利。没有导入旧代码或协议文档。
- DM 底盘 ZIP 之前的部分解压副本仅有 64/2252 项，不能作为完整工程；本次未将它当作可编译源树。旧包里的 `.hex/.elf/.map/.axf`、预编译 SDK、缓存和日志均排除。第三方源码没有新增，故本轮不改 `THIRD_PARTY_NOTICES.md`。

### 本轮改动、提交安全与验证

- 本轮没有新增项目代码；仅补充 README、架构、状态和本审计记录。没有更改两个 ROS 2 工作空间、P7/P8 实现或现有测试脚本，也没有恢复旧源码覆盖新实现。
- 变更范围不含固件、密钥、令牌、绝对设备路径、编译输出、缓存、模型权重、录制数据或第三方源码。仓库 Private 状态保持不变。
- P5a 离线模型：13/13 通过。P5b 静态配置：ThinkBook ROS 2 环境中 5/5 组通过；本机 macOS 缺少 PyYAML，故不在本机执行该项。
- 另在 ThinkBook 上从最新 `main` 的干净 Git 快照隔离运行 P4e/P6/P7 验收，没有覆盖原工作区：P4e 所需机械臂工作空间 6 个包、P6 与底盘桥接 2 个包、P7 机械臂工作空间 6 个包均构建成功（仅有 Pinocchio/eigenpy 对 Python 头文件的 CMake 提示）。P4e ROS 驱动 PTY 故障注入 20 项通过；P6 的 8 项逻辑/静态单测、规划取消竞态和 6 个场景通过。
- P7 12 个正常/故障场景均在隔离运行中得到预期结果。首次顺序执行在第 6 个 `duplicate_trigger` 场景遇到 Nav2 空路径和 controller patience 超时，整条脚本因此提前退出；不改源码单独复跑该场景成功，随后剩余 6 个场景均通过。因此 12 个场景各自均有 PASS 结果，但**首次全套脚本没有一次性跑完**，且保留了一个离线 Nav2 规划不稳定记录。
- P5a 标准库模型测试 13/13 通过；P5b 静态配置检查在 ThinkBook ROS 2 环境 5/5 组通过。当前 macOS 没有 PyYAML，本机执行 P5b 检查因 `ModuleNotFoundError: yaml` 停止，故改在已装依赖的 ThinkBook 上验证。四份本轮文档的相对 Markdown 链接与 `git diff --check` 均通过。
- 没有 STM32 编译器，因此固件仅完成结构与静态完整性审查，**未编译**。离线测试不是 STM32、传感器、实体串口、CAN、电机、刹停或机器人运动验收。

**交接结论：** P0～P8 ROS 2 源码、测试和配置已集中于私有 `main`；当前不需要从 ThinkBook 恢复一份未被替代且授权明确的 ROS 2 代码。但两套 STM32 第一方工程的内部共享许可未确认，机械臂原包本轮也未找到，因此还不能宣称仓库已包含全部获准共享的机器人源码或满足完整源码交接条件。

