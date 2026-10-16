# 工作区边界、原始工程核查与新版测试

核查日期：2026-10-16（Asia/Shanghai）。本页记录本次能从 Git、文件哈希、SSH 状态和既有测试报告确认的事实。核查期间没有向 NUC 或 ThinkBook 写文件、构建、启动机器人或连接运动控制器。

## 先记住的路径边界

| 用途 | 设备和绝对路径 | 说明 |
| --- | --- | --- |
| 原始机械臂/视觉 ROS 工程 | NUC `tzb-NUC14RVS-B`：`/home/tzb/tzb_ws` | 原工作空间，禁止新版同步、构建或部署脚本写入 |
| 原始导航/雷达 ROS 工程 | 同一 NUC：`/home/tzb/Nav2_rea_ws` | 原工作空间，禁止新版同步、构建或部署脚本写入 |
| 新版正式源码 | GitHub 私有仓库 `Tyr1onX/lunar-cave-robot` 的 `main` | 唯一正式源码来源；本次核查 SHA：`c2f4a5ffb6fc764713af6040b11cd648b750b3a5` |
| 新版开发和离线验证 | ThinkBook：预期目录 `~/lunar-cave-robot` | 本轮 SSH 连接超时，当前 SHA 和工作区状态未核实 |
| NUC 新版源码/部署候选目录 | `/home/tzb/lunar-cave-robot` | 与两个原始目录分开；本次仍落后 GitHub `main` 12 个提交，未记录部署 SHA |
| NUC Mac Console 模块测试目录 | `/home/tzb/lunar-cave-robot-module-tests` | 当前 `main` SHA 的独立测试 checkout，不是原始工程或已部署版本 |

仓库里的 `tzb_ws/` 和 `Nav2_rea_ws/` 是 GitHub 新版 checkout 内的源码副本。它们与 NUC 上 `/home/tzb/tzb_ws`、`/home/tzb/Nav2_rea_ws` 同名，但不是同一目录。新版构建只允许在 `~/lunar-cave-robot` 这样的新版 checkout 中运行；构建产物写入该 checkout 内部。

通过 Mac 上已有 SSH 别名只读定位原始目录：

```bash
ssh nuc 'ls -ld /home/tzb/tzb_ws /home/tzb/Nav2_rea_ws'
```

原始工作空间职责：`tzb_ws` 包含相机 ROS 包装、YOLO/目标位姿、机械臂模型与规划、机械臂主机串口接口；`Nav2_rea_ws` 包含导航、定位/建图、MID-360/Livox 驱动和底盘相关主机接口。职责来自原目录内容及仓库架构/资产记录，不表示这些模块已完成实机验收。

## NUC 原始工程核查

### `/home/tzb/tzb_ws`

- Git 根目录为该路径，`origin` 指向 `zwt-0214/tzb`；当前分支 `main`，唯一可见提交为 `0b64eb3344e20cf028dfe6acb708c6d7b2ec19c3`（2026-10-10 12:19 +08，`first commit`）。
- 本次 `git status --porcelain --untracked-files=all` 只显示一个已跟踪修改：`README.md`。`git diff` 为 92 行新增、1 行替换，没有已跟踪源码、配置或启动文件差异，也没有已跟踪删除。
- 已知 README 修改得到单独确认：初始版本只有 `# tzb`；现版本增加 ROS 话题接口说明，覆盖 `yolo_node`、`pose_estimator`、`tzb_catch`、`rm_serial_driver` 及可选协调节点。文件时间为 2026-10-10 12:42 +08。初始提交版 SHA-256 为 `c082afc636190c8fdd43efd1699ea33e48fcfb04437710f01396a204f2c27bc5`，当前 README 为 `8aeddd6fa1291a8082cdb658358cf0c3d152b4d4819fa5eab077fee62310de68`。
- 两个已跟踪样本的当前 Git blob 与唯一提交中的 blob 相同：`src/yolo_node/yolo_node/yolo_detector.py` 为 `c374c2b4f9351f959ca49e1fe5fb5c8973e78935`；`src/lite_motion_planner/launch/bring_launch.launch.py` 为 `dc3fce0b3f9a202eab2eceede84faedbbc7762c1`。这支持“这两个跟踪文件自该提交以来未改变”，不覆盖未跟踪文件或首次提交前的历史。
- `git status --ignored` 还列出 `log/`、多个被忽略的 `package.xml`、Orbbec 配置及模型文件。它们不受上述 Git 状态/提交哈希证明。唯一提交没有更早父提交或交接前完整清单，因此无法证明首次提交前源码是否变化，也无法排除被忽略文件曾被覆盖、删除或移动。

### `/home/tzb/Nav2_rea_ws`

- 目录存在，顶层为 `src/`、`build/`、`install/`、`log/`；本次盘点分别发现 1,521、3,563、452、427 个文件。`build/install/log` 是编译/运行输出，应与源代码变化分开看。
- 该目录没有 Git 元数据，也没有交接前文件清单或哈希基线。只能确认本次读取时的路径和内容布局；无法证明接手前后哪些源码/配置变化，也无法用历史证据排除覆盖、删除或移动。

**核查结论：** 现有证据没有显示新版脚本覆盖了这两个绝对路径；本轮也没有写入它们。`tzb_ws` 的已跟踪源码/配置/启动文件相对唯一提交无差异，已知 README 是唯一已跟踪修改。由于首次提交前无基线、存在被忽略文件，且 `Nav2_rea_ws` 没有 Git/哈希历史，不能据此宣称原始工程从未被改动。保留两处现状，不迁移、不清理、不重新编译。

## 三端新版工作区和历史目录

- **GitHub：** `main` 是正式源码来源。本次 SHA 为 `c2f4a5ffb6fc764713af6040b11cd648b750b3a5`。
- **ThinkBook：** 用于开发、构建和离线验证。本轮 `ssh thinkbook` 超时；只保留预期路径 `~/lunar-cave-robot` 的既有记录，不把 Mac 上的 Git 工作树当作 ThinkBook 实际状态。
- **NUC 新版 checkout：** `/home/tzb/lunar-cave-robot` 在 `main`，HEAD 为 `6a48dc52f5d7f5b263845bdec0639f59144a4486`，本地工作区干净；相对本次 GitHub `main` 落后 12 个提交。未执行同步、更新或重置。
- **NUC Console 测试 checkout：** `/home/tzb/lunar-cave-robot-module-tests` 在 `feat/console-module-tests-expand`，HEAD 为 GitHub 当前 `main` SHA，工作区干净。`lunar-console-module-tests.service` 当前运行，脚本把 ROS 限定在 `ROS_DOMAIN_ID=187` 和 `ROS_LOCALHOST_ONLY=1`。本次检查没有 `running/cancelling` 测试报告；最近的 P7 模拟报告为 `interrupted`，此前另有一个完整通过报告，二者各自记录了不同的代码 SHA，不能当作当前 `main` 的同 SHA 验收。NUC 缺少 `deployed-sha` 记录，进程检查没有发现 ROS、Livox、Orbbec、底盘桥或 YOLO 运行进程；这只是当时的软件状态，不能代替对设备供电状态的测量。
- **历史目录：** NUC `/home/tzb/lunar-cave-p8`（P8 分支 HEAD `f6005a076997a7e05e079408f7e84e1b6b9bdb8f`，相对其本地 bundle 领先 7 个提交）、`/home/tzb/lunar-field-lab-p9i` 和 `/home/tzb/lunar-console-sim-v03a` 均保留。历史 P7/P8 结果与测试记录也保留在用户状态目录。不要为对齐版本重置、覆盖或清理这些目录。

新版同步脚本的默认 NUC 目标是 `~/lunar-cave-robot`，并检查仓库来源、`main`、干净状态及快进关系。不要将 `NUC_REPO` 指向 `/home/tzb/tzb_ws` 或 `/home/tzb/Nav2_rea_ws`；新版构建和测试命令也只能从新版 Git checkout 的根目录执行。同步源码不等于部署或启动机器人。

## 新版当前测试和状态检查

先在目标 checkout 确认源码版本和工作区状态；查询 GitHub 当前 `main` 可用只读命令 `git ls-remote`：

```bash
cd ~/lunar-cave-robot
git rev-parse HEAD
git status --short --branch
git ls-remote git@github.com:Tyr1onX/lunar-cave-robot.git refs/heads/main
./tools/device_status.sh
```

执行已建立的完整 P7/P8 PTY 离线套件：

```bash
cd ~/lunar-cave-robot
./tools/run_offline_validation.sh
```

这会在**该新版 checkout** 内分别构建两个 ROS 工作空间，使用运行时创建的 PTY、合成数据和隔离 ROS domain；结果写到 `$XDG_STATE_HOME/lunar-cave-robot/runs/`（默认 `~/.local/state/lunar-cave-robot/runs/`），并记录测试 SHA、分支、范围和结果。开始执行后应在日志看到 `PASS serial isolation: only runtime-created PTYs are configured`。构建目录为 checkout 内的 `tzb_ws/build/p7`、`Nav2_rea_ws/build/p7` 等；它们不是 NUC 原始工作空间。

Mac Lunar Robot Console 的“模块测试”复用固定 allowlist 中的 `OFFLINE`、`SIMULATION` 和 `LIVE` 检查。优先在 ThinkBook 跑离线/模拟验证。GUI 的**停止**按钮取消当前测试；命令行完整套件用 `Ctrl-C` 请求停止，随后检查退出结果和记录。SSH/GUI 断开本身不会取消 NUC 上正在执行的测试，恢复连接后应检查状态并显式停止。Console 测试结果保存为 NUC `~/.local/state/lunar-cave-robot/module-tests/<run-id>.json` 和 `.log`；`./tools/device_status.sh` 汇总其支持的构建与离线测试记录。每条测试记录中的 SHA 才是该次执行的源码版本；同步到某个 SHA 不代表已部署该 SHA。

判断结果属于哪类，查看 Console 模式标签和报告中的 `mode`、`input_source`、SHA 与日志：

- `OFFLINE` 使用主机测试、模拟器或合成输入，不访问真实控制器。
- `SIMULATION` 使用独立 ROS domain、合成反馈和/或 PTY；Console 的 WASD 驾驶属于仿真。
- `LIVE` 只观察已经存在的 ROS 图和固定状态话题，不启动传感器/电机驱动，不发送底盘或机械臂命令。`LIVE` 通过、话题存在或 GUI 显示图像都不能单独证明真实设备通过验收。

只有部署负责人明确记录部署 SHA，并在受控实机验收中核对真实传感器、控制器反馈、驱动使能、制动/保持和独立急停，才可记录对应硬件验收。当前 Console 的键盘命令只作用于隔离仿真，不提供实体机器人运动控制；同步、离线测试或仿真通过都不表示实体机器人正在运行。

## 相对原始工程的已完成工作

原始两个 ROS 工作空间已经包含导航/雷达、相机/YOLO、位姿估计、机械臂规划和主机串口接口。把这些目录归档进 GitHub 新版仓库不算新增功能。我们的主要新增/修改和验证边界如下：

| 模块 | 新版新增或修改 | 已有验证 | 尚未通过的实机验收 |
| --- | --- | --- | --- |
| 导航、底盘与任务 | P5b 底盘主机桥、P6 任务协调与链路失效中止、P7 隔离的 12 场景端到端 PTY 套件；P8 集成检查保留两个 workspace 边界 | ThinkBook 在源码 SHA `07a56c3a33882aa2866f1076d8597776e2579bcb` 完整 P7/P8 12/12；相关 ROS 工作区构建与 P4f/P6 离线检查有逐项记录 | 实际 MID-360、底盘串口/CAN、电机反馈、制动距离和真实任务链路 |
| 机械臂与固件 | P4e/P4f 主机串口安全、命令新鲜度和取消/重放保护；P9-B 固件归档、生产协议主机测试与构建审计 | 19 项 ROS PTY 驱动检查、10 项 P6 逻辑/静态检查及模拟场景有通过记录；底盘交叉编译在记录的 P9-B 源码快照通过 | PR #8 的 STM32 安全状态机仍开放，且 P9-E 审查列出控制时序缺口；机械臂产品构建缺少真实 XRobot 生成文件；未烧录、未接控制器 |
| 视觉与位姿 | P9-H/P9-I 修复 OpenVINO 模型 metadata/装载问题，增加 PointCloud2 输入校验和合成 ROS smoke | 合成图像可通过 YOLO ROS 输出，合成 Detection/PointCloud2/CameraInfo 可走位姿接口；测试细节见 `docs/P9I_VISION_OFFLINE_VALIDATION.md` | 真实相机/深度对齐、模型准确率、手眼标定、真实点云定位精度和传感器到机械臂闭环 |
| Mac Console | 新增 SwiftUI 只读诊断/图像页、隔离仿真驾驶、机械臂 3D 预览和固定 ID 模块测试工作台 | App/Agent 有构建与接口/离线测试记录；P7/P6 测试报告保存在 NUC 状态目录 | Console 的仿真和只读观测不是实机控制；真实相机流未验收，人工键盘验收仍有待办 |
| 源码/交付流程 | 统一 GitHub `main`、三端 SHA/测试记录、安全快进同步、固件源文件清单和设备/部署审计 | P9 文档和同步 PR 记录了来源、哈希、状态查询和失败保护 | NUC 新版源码目录需在审核后更新到目标 SHA；部署 SHA 和设备验收记录仍未填写 |

完整历史结果见 [P9-C 离线验证](P9C_OFFLINE_VALIDATION.md)、[P9-H 感知准备](P9H_PERCEPTION_READINESS.md)、[P9-I 视觉离线验证](P9I_VISION_OFFLINE_VALIDATION.md)、[P9-E PR #8 接入审查](P9E_PR8_INTEGRATION_REVIEW.md)、[STM32 构建审计](STM32_BUILD_AUDIT.md)和 [V0.3c 模块测试说明](../tools/mac-console/acceptance/v0.3c-module-tests.md)。历史结果只对各自记录的 SHA、运行环境和输入有效。

## 后续必须完成

1. ThinkBook 恢复 SSH 可达后，重新读取 `~/lunar-cave-robot` 的 SHA 和脏状态；本轮没有实时核实它。
2. 选择并审核要部署的 GitHub `main` SHA，再单独更新 NUC `/home/tzb/lunar-cave-robot`；不触碰两个原始路径和历史测试目录。完成部署前记录部署 SHA。
3. 由固件负责人提供与实际机械臂产品对应的 XRobot 生成输入，解除机械臂完整固件构建阻塞；审查并修复 PR #8 中的安全时序问题。
4. 在硬件负责人确认独立急停、底盘停止/保持、机械臂失能后的重力支撑及真实控制器反馈后，按 `docs/REAL_ROBOT_SAFETY_GATES.md` 分阶段做传感器、底盘和机械臂实机验收。
5. 采集并核验实际相机型号/标定、MID-360 网络/广播码/外参、真实数据和时间/TF 对齐；离线模型推理不能替代这些结果。
