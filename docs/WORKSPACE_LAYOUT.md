# ThinkBook 与 NUC 工作区布局

核验日期：2026-10-16。两台设备的新项目根目录为 `~/lunar-robot/`。GitHub 私有仓库 `Tyr1onX/lunar-cave-robot` 的 `main` 是新版源码来源；本次两台设备的 `code` 都在 `c2f4a5ffb6fc764713af6040b11cd648b750b3a5`，工作树干净。

## ThinkBook

```text
~/lunar-robot/
├── code/                         # GitHub main；开发、构建、离线验证
├── tests/state/                  # 迁移前测试状态的副本
└── archive/
    ├── baseline/                 # 原有源码、P0/P4f/P6/P7/P9 快照与 Git bundle
    └── p8/                        # 完整 P8 验收仓库
```

`tests/state` 从原状态目录复制了 784 个文件并校验一致。`archive/` 保存约 13 GB 的历史工作树、构建结果和日志；P4f/P9-C 的未提交内容也有快照和 diff。ThinkBook 的 `archive/baseline/README.md` 记录各副本来源与 Git 历史限制。没有单独的视觉或仿真实验目录内容，因此 ThinkBook 不创建空 `lab/`。

历史原始 ROS 工作区仍在 `/home/qibai/robot_p0/tzb_ws` 与 `/home/qibai/robot_p0/Nav2_rea_ws`，未移动、重命名或构建。旧工作区与运行中的旧进程继续保留在原位置。

`code/tzb_ws/` 和 `code/Nav2_rea_ws/` 是 GitHub 新版仓库内的源码/构建工作区；它们和 ThinkBook `robot_p0/` 下的历史目录、NUC `/home/tzb/` 下的原始目录不是同一文件夹。新版 P6/P7 构建只会使用 `code/` 中的两个工作区。

## NUC

```text
~/lunar-robot/
├── code/                         # GitHub main；Console Agent 与模块测试服务
├── tests/
│   ├── config/                   # 当前 systemd 单元、drop-in 与迁移前单元备份
│   ├── results/                  # 视觉离线验证结果
│   ├── ros-home/                 # 隔离测试 ROS_HOME 与日志
│   └── state/                    # 迁移前测试记录及新版运行报告
├── lab/
│   ├── vision/                   # P9-I 视觉代码、模型、依赖和重建后的 ROS/Python 环境
│   └── simulation/               # 独立 P6 仿真文件
└── archive/
    ├── baseline/                 # 旧 main 与迁移前视觉 venv/build/logs 副本
    └── p8/                       # 保留的 P8 Git 工作树、构建与验收记录
```

模块测试服务从 `~/lunar-robot/code/tools/console-agent` 运行，监听 NUC loopback `127.0.0.1:8769`。只读 Agent 使用 ROS Domain 187、`ROS_LOCALHOST_ONLY=1`，监听 `127.0.0.1:8765`。两项服务、ROS 日志、测试报告、视觉模型与重建环境都位于 `~/lunar-robot/`。P6/P7 构建输出由仓库测试脚本写在 `code/` 中的忽略目录，仍在该项目根目录内。

NUC 迁移前状态目录的 485 个文件已复制到 `tests/state/lunar-cave-robot/`；校验显示仅一个历史测试 JSON 的 `log_file`、`report_file` 绝对路径从旧状态目录调整到了新目录，报告其它字段和其它历史文件一致。旧状态目录保留在 `~/.local/state/lunar-cave-robot/`。ThinkBook 的 784 个状态文件校验一致，原状态目录也保留。

NUC 原始工程 `/home/tzb/tzb_ws` 和 `/home/tzb/Nav2_rea_ws` 完全独立。新版服务、构建和测试未使用这两个路径。旧版 `~/lunar-cave-robot`、`~/lunar-cave-robot-module-tests`、`~/lunar-field-lab-p9i`、`~/lunar-console-sim-v03a`、P8 工作树及其它旧目录仍保留以供回查；它们不是新版正式代码目录。

## 日常操作

只需记住 `~/lunar-robot/code`：

```bash
cd ~/lunar-robot/code
git rev-parse HEAD
git status --short --branch
```

离线 ROS 套件：

```bash
./tools/run_offline_validation.sh
```

指定 P7 场景：

```bash
./tools/run_p7_scenario.sh normal
```

Mac Lunar Robot Console 的“模块测试”页执行固定白名单 ID；报告、日志和场景产物保存在 NUC `~/lunar-robot/tests/state/lunar-cave-robot/module-tests/`。在模块测试页使用“停止”取消正在运行的测试。CLI 完整套件使用 `Ctrl-C` 请求停止。测试结束后查看 GUI 报告，或检查同目录下匹配 run ID 的 JSON、log 文件；每份报告记录源码 SHA、模式、状态、输入源和硬件访问范围。

2026-10-16 GUI 验证：Agent 在线并显示 Domain 187；`arm_fk` OFFLINE 通过（run ID `b972d12a96b64729b2076f6f5488518e`）；P7 SIMULATION 完整套件 12/12 通过（`f443293461dc4e2baf548cfdba1b5ad8`）。随后从 GUI 启动另一轮 P7，确认真实子进程进入隔离 `ROS_DOMAIN_ID=5`，再按“停止”；GUI 最终显示 `CANCELLED`、15.21 秒、工作树干净，run ID `cc62c961eb3e472a94853f2a80df44e5`。服务健康且测试目录 `busy=false`。本轮没有运行 GUI 中的 LIVE 项。

在 Console 的“系统诊断”查看 NUC 部署 SHA、ROS Domain 和当前观察到的 ROS topic/publisher；模块测试的 `LIVE` 项只读现有低带宽状态，不会启动设备。Agent 在线只说明 GUI 到 NUC 软件服务连通，不能单独证明真实机器人或驱动器正在运行。若需要判断真实设备状态，必须按安全门槛核对真实传感器流、控制器反馈和硬件状态；topic 存在本身也不算验收。本次 Domain 187 没有相机等真实数据发布者，NUC 部署 SHA 未记录。

只把模式为 `OFFLINE` 或 `SIMULATION` 且报告状态为通过的结果记作离线/仿真通过。不要把仿真结果当作真实设备通过；本次没有启动实体机器人、雷达或运动控制器，也没有烧录 STM32。新版本实机启动仍未验收。

## 部署保护

- GitHub `main` 是新版正式源码；ThinkBook 用于开发和离线验证，NUC 的独立新版目录是 `~/lunar-robot/code`。
- NUC 的 `/home/tzb/tzb_ws` 与 `/home/tzb/Nav2_rea_ws` 作为原始工程保护区，禁止新版同步、构建、测试或部署写入。
- ThinkBook 历史原始目录 `/home/qibai/robot_p0/tzb_ws` 与 `/home/qibai/robot_p0/Nav2_rea_ws` 同样保留。
- 同步脚本默认只更新 `~/lunar-robot/code` 中干净、来源正确的 `main` 工作树；不要将 `THINKBOOK_REPO` 或 `NUC_REPO` 指向原始路径、`archive/`、`lab/` 或测试目录。
- 所有新版测试状态和 ROS_HOME 固定在 `~/lunar-robot/tests/`；新版不在原始 ROS 工作区构建或运行。

原始工程检查证据及已知限制见[工作区边界与原始工程核查](WORKSPACE_HANDOFF.md)。
