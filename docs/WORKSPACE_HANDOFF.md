# 原始工程保护与新版交接

核查和迁移日期：2026-10-16。本文把原始工程、历史快照和新版 checkout 分开记录。本轮设备上的新版副本、构建结果和测试结果写入 `~/lunar-robot/`；NUC 的 user-level systemd 配置改为引用该新根目录，并在 `tests/config/` 保存了副本。没有对原始路径发起写入、移动、删除或重新构建。缺少历史基线的部分另行注明，不据此宣称旧目录从未变化。

## 原始工程的准确位置

| 设备 | 原始工程 | 主要内容 |
| --- | --- | --- |
| NUC `tzb-NUC14RVS-B` | `/home/tzb/tzb_ws` | 相机、YOLO/位姿、机械臂模型/规划和主机串口接口 |
| NUC `tzb-NUC14RVS-B` | `/home/tzb/Nav2_rea_ws` | Nav2、定位/建图、MID-360/Livox 和底盘主机接口 |
| ThinkBook `qibai-ThinkBook-16-G6-AHP` | `/home/qibai/robot_p0/tzb_ws` | P0 原始机械臂/视觉 ROS 工作区 |
| ThinkBook `qibai-ThinkBook-16-G6-AHP` | `/home/qibai/robot_p0/Nav2_rea_ws` | P0 原始导航/雷达 ROS 工作区 |

只读定位命令：

```bash
ssh nuc 'ls -ld /home/tzb/tzb_ws /home/tzb/Nav2_rea_ws'
ssh thinkbook 'ls -ld /home/qibai/robot_p0/tzb_ws /home/qibai/robot_p0/Nav2_rea_ws'
```

NUC `tzb_ws` 的唯一可见 Git 提交为 `0b64eb3344e20cf028dfe6acb708c6d7b2ec19c3`。只发现一个已跟踪修改：`README.md` 增加接口说明；初始提交版 SHA-256 为 `c082afc636190c8fdd43efd1699ea33e48fcfb04437710f01396a204f2c27bc5`，当前 SHA-256 为 `8aeddd6fa1291a8082cdb658358cf0c3d152b4d4819fa5eab077fee62310de68`。`git diff` 未显示已跟踪源码、配置或启动文件修改或删除。两个抽查源码的 Git blob 与唯一提交相同。

这些 Git 证据只覆盖该提交之后的已跟踪文件。`tzb_ws` 还存在被忽略的模型、配置和日志，且没有更早的 Git 基线，无法证明首次提交前的状态，也无法排除忽略文件曾被覆盖、删除或移动。

NUC `/home/tzb/Nav2_rea_ws` 没有 Git 元数据和交接前哈希清单。当前包含 `src/`、`build/`、`install/`、`log/`。本次较早的目录盘点记录文件数为 `1521/3563/452/427`，本次复核记录为 `1472/2779/264/376`（顺序均为上述四个子目录）；没有此前文件名/哈希清单，无法确定哪些文件对应这些计数差异，也不能判断发生时间或原因。文件计数不是文件内容证明。可核实的本轮 P7 进程工作目录和报告路径都在 `/home/tzb/lunar-robot/code` / `tests/`，但这不能补出该原始目录的历史基线。ThinkBook 两个 P0 工作区是 Git 仓库：`tzb_ws` 在 `2964e87793045836d16dc4f0ce2499dbddefb66e`，`Nav2_rea_ws` 在 `081ba746272bfe7964c5ba477fad818805b3bbdc`；当前只读检查没有跟踪修改、暂存修改或未跟踪文件。该状态不能证明首次克隆之前的历史。

**结论：** 现有证据没有显示我们把新版覆盖到 NUC 原始工程；这次迁移和新版测试也没有使用上述原始目录。对缺少历史基线的部分不作“从未修改”的推断。原目录继续原样保留，后续新版同步、构建、测试和部署都禁止指向它们。

## 新版位置和版本关系

- **GitHub 私有仓库 `Tyr1onX/lunar-cave-robot` 的 `main`** 是新版正式源码来源。
- **ThinkBook `/home/qibai/lunar-robot/code`** 用于开发、构建和离线验证。
- **NUC `/home/tzb/lunar-robot/code`** 是独立新版 checkout，Console 服务从这里运行；不表示新版已获准实机部署。
- 两台设备 `code` 都在 `c2f4a5ffb6fc764713af6040b11cd648b750b3a5`，分支为 `main`，工作树干净。
- 目录用途、实际树和迁移内容见[工作区布局](WORKSPACE_LAYOUT.md)。历史 P7/P8、脏工作树、模型、构建结果、测试状态和日志已按设备归档到 `archive/` / `tests/`；未为统一版本而重置或删除。2026-10-16 又将 NUC 五个迁移前旧目录整体移动到 `~/lunar-robot/archive/old-copies/`，归档内保留路径映射与 Git 历史。此后这些五个旧路径不再存在于 `/home/tzb/` 顶层；原始的 `~/tzb_ws`、`~/Nav2_rea_ws` 仍保留原处。
- 旧版 NUC `~/lunar-cave-robot` 与其它历史工作树保持原位。P7/P0 等进程此前仍使用旧路径，因此没有移动这些目录。

新版同步脚本默认目标为 `~/lunar-robot/code`，只对来源正确、干净且可快进的 `main` 执行同步。新版 ROS 构建只使用 `code/` 内仓库源码；报告、测试状态、ROS_HOME 和配置保存在 `tests/`；视觉与仿真在 `lab/`。没有任何新版脚本将 `/home/tzb/tzb_ws` 或 `/home/tzb/Nav2_rea_ws` 设为构建或部署目录。

## 新版测试方式和当前边界

先确认来源版本和工作树：

```bash
cd ~/lunar-robot/code
git rev-parse HEAD
git status --short --branch
```

命令行完整 P7/P8 PTY 离线套件：

```bash
./tools/run_offline_validation.sh
```

单独 P7 离线场景：

```bash
./tools/run_p7_scenario.sh normal
```

Mac Lunar Robot Console 的“模块测试”页使用固定白名单测试 ID；验证 NUC `code` 的 SHA、模式和报告状态。模块测试报告、日志和 P6/P7 场景结果位于 NUC `~/lunar-robot/tests/state/lunar-cave-robot/module-tests/`。GUI“停止”取消当前测试，完整命令行套件可以 `Ctrl-C` 请求停止。历史记录可从 GUI 重新打开，或根据报告 run ID 读取对应 JSON/log。不要因 SSH/GUI 断开就假设远端测试已停止。

本轮在当前 NUC SHA `c2f4a5ffb6fc764713af6040b11cd648b750b3a5` 上，P6 六场景 `SIMULATION`、`arm_fk` 和 YOLO 合成图像 `OFFLINE` 测试通过；P7 十二场景套件 `SIMULATION` 也完成 12/12 通过。P7 完整通过 run ID 为 `f443293461dc4e2baf548cfdba1b5ad8`，完整报告和场景结果位于 NUC `tests/state/lunar-cave-robot/module-tests/`。Mac GUI 实际连接 NUC 后，`arm_fk` OFFLINE 再次通过（run ID `b972d12a96b64729b2076f6f5488518e`）；GUI 启动的 P7 进入隔离 `ROS_DOMAIN_ID=5` 后通过“停止测试”取消，最终状态 `CANCELLED`、用时 15.21 秒（run ID `cc62c961eb3e472a94853f2a80df44e5`）；测试服务随后报告 `busy=false`。各报告记录源码 SHA；P7 场景使用临时 PTY 与合成反馈，无真实串口或运动硬件。`OFFLINE`/`SIMULATION` 通过不等于实机通过。不要把只读 `LIVE` 观察或仿真驾驶当作硬件验收。

本轮没有启动实体机器人、雷达、相机驱动或运动控制器，没有运行模拟驾驶，也没有烧录 STM32。新版真实传感器、驱动器、运动与安全链路尚未完成实机验收。

本轮 Mac GUI 连通的是 NUC 的只读 Agent 和受限测试服务，不是“实体机器人在线”的证明。Domain 187 快照没有相机等真实数据发布者，部署 SHA 也未记录；当前不能判定真实机器人已运行。

## 新版相对原始工程的主要变化

原始工程已包含 Nav2、Livox/MID-360、相机/YOLO、位姿估计、机械臂规划和串口驱动。将这些既有源码或模型复制到新版仓库不算新增能力。我们新增/修改的部分和已知验证边界：

| 模块 | 我们新增或修改 | 已完成验证 | 未完成的实机验收 |
| --- | --- | --- | --- |
| 底盘与任务 | P5b 主机桥、P6 协调/链路失效保护、P7 隔离 PTY 场景、P8 集成验证 | 旧 SHA `07a56c3a33882aa2866f1076d8597776e2579bcb` 有历史 12/12 记录；本轮当前 SHA 的 P6 六场景和 P7 十二场景均通过，P7 run ID `f443293461dc4e2baf548cfdba1b5ad8` | MID-360、真实底盘接口/电机反馈、制动和真实任务链路 |
| 机械臂与固件 | P4e/P4f 串口安全、命令新鲜度、取消/重放保护；P9-B 固件快照和主机协议测试 | ROS PTY、逻辑/静态和仿真有历史通过记录；本轮 `arm_fk` 通过 | PR #8 STM32 安全状态机仍开放；缺真实 XRobot 生成输入；未连接或烧录控制器 |
| 感知与位姿 | P9-H/P9-I 模型 metadata/装载修复、PointCloud2 校验、合成 ROS smoke | 本轮 NUC 复制出的视觉环境经重建，模型检查和合成 YOLO/点云用例通过 | 真实相机/深度、模型准确率、手眼标定、定位精度和机械臂闭环 |
| Mac Console | 只读观测、隔离仿真、机械臂 3D 预览、固定 ID 模块测试与取消 | Swift/Python 本地验证；NUC Agent 与模块服务连接；P6/P7/YOLO 离线报告 | 只读和仿真页面不构成实机运动验证；真实图像与运动流程未验收 |

更早的模块记录见 `docs/P9C_OFFLINE_VALIDATION.md`、`docs/P9I_VISION_OFFLINE_VALIDATION.md`、`docs/P9E_PR8_INTEGRATION_REVIEW.md`、`docs/STM32_BUILD_AUDIT.md` 和 `tools/mac-console/acceptance/v0.3c-module-tests.md`。历史结论只对各自报告记录的 SHA、环境和输入有效。

## 后续部署与验收

1. 从 GitHub `main` 获取正式代码，在 ThinkBook `~/lunar-robot/code` 开发；通过 PR 合并后，再按审核的 commit 同步 NUC 的 `~/lunar-robot/code`。
2. 目前两台设备都位于本次核验 SHA；后续 NUC 代码同步只同步该独立路径，不重置旧目录或历史基线。
3. 真实机器人启动仍需负责人审核目标 SHA、参数和 `docs/REAL_ROBOT_SAFETY_GATES.md` 的安全门槛，按传感器、底盘、机械臂逐项验收。
4. 解除真实机械臂固件构建阻塞前，需要产品对应的 XRobot 生成输入并修复 PR #8 记录的安全时序缺口。真实硬件验收完成前，不把任何模拟/离线记录标记为实机通过。

## NUC 备份与校验（2026-10-16）

- **GitHub 的职责：** 新版正式源码、固件源码、技术文档及提交历史。ThinkBook 与 NUC 的 `~/lunar-robot/code` 已核实对应 GitHub `main` SHA `c2f4a5ffb6fc764713af6040b11cd648b750b3a5`；GitHub **不是** NUC 用户主目录和实时运行环境的完整备份。
- **ThinkBook 独立快照：** `/home/qibai/lunar-robot/archive/nuc-backups/2026-10-16-nuc/`，约 **4.8 GB**，权限受限。NUC 源端只读，通过 SSH 向 ThinkBook 同步；没有写入、构建或移动 NUC 原始 `/home/tzb/tzb_ws` 和 `/home/tzb/Nav2_rea_ws`。
- **快照内容：** `original/` 完整保存两套原始 ROS 工作空间；`models/` 保存视觉模型；`configs/` 保存 Console systemd 服务/覆盖配置、Orbbec udev 规则、设备操作入口和 Ubuntu 软件包版本清单；`records/` 保存测试报告、ROS 测试日志、串口日志；`project/` 保存当前新版代码 checkout 及视觉/仿真实验环境；`hardware-support/` 保存 MID-360 开发工作区、历史 `nav_ws`、`serial_t`、OrbbecSDK、OpenVINO 工具和 Livox-SDK2。
- **实际校验：** NUC 两套原始工程与 ThinkBook 的 **12,970 个普通文件 SHA-256 完全一致**，**636 个符号链接路径与目标完全一致**；当前视觉模型 5 个文件 SHA-256 完全一致。ThinkBook 备份总计 **31,538 个普通文件、665 个符号链接**，生成 `BACKUP_SHA256SUMS` 后执行全量本地自检并通过。恢复检查可在备份根目录运行 `./verify.sh`。
- **保留边界：** 此备份**不是 Ubuntu 整盘镜像**，未备份 SSH 私钥、所有个人应用及系统缓存；NUC 的 `~/lunar-robot/archive/` 是历史重复副本集合，本次没有再次全量复制它，ThinkBook 的既有 `~/lunar-robot/archive/` 仍保存独立历史工作树。此次快照不代表 NUC 全部磁盘字节、所有历史档案或真实控制器当前固件已被备份。逐项来源、排除范围与恢复注意事项见 ThinkBook 快照中的 `README.md`。
- **恢复与后续：** 先在隔离目录恢复、校验哈希并修复 ROS/venv/systemd 绝对路径，不直接覆盖原始工作空间；不得把代码/模型可恢复等同于机器人实机运行验收。未来新增的数据需单独再备份；不可再采集的设备标定和关键记录建议另有一份加密离线或异地副本。
