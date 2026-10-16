# 新版开发、离线验证与同步

GitHub 私有仓库 `Tyr1onX/lunar-cave-robot` 的 `main` 是正式源码来源。ThinkBook 用于新版开发和离线验证；NUC 的新版代码、Console 服务和测试结果各自放在 `~/lunar-robot/`。原始 ROS 工作区和 `archive/` 不参与同步。完整目录说明见[工作区布局](WORKSPACE_LAYOUT.md)。

## 开发和提交

```bash
cd ~/lunar-robot/code
git status --short --branch
git rev-parse HEAD
```

ThinkBook 在 `code/` 的开发分支上修改，通过 PR 合并到 GitHub `main`。在合并前，可从 `code/` 运行仓库已有的主机或离线测试；测试结果写到 `~/lunar-robot/tests/`。不要从 `archive/`、`lab/`、旧 checkout 或原始 ROS 工作区开发正式修改。

## 离线验证

完整 P7/P8 PTY 离线套件：

```bash
cd ~/lunar-robot/code
./tools/run_offline_validation.sh
```

单个 P7 场景：

```bash
./tools/run_p7_scenario.sh normal
```

Mac Lunar Robot Console “模块测试”页执行固定 allowlist 的 OFFLINE/SIMULATION 测试。NUC 当前 `lunar-console-module-tests.service` 从 `~/lunar-robot/code` 运行，状态文件、报告、场景日志和 ROS_HOME 在 `~/lunar-robot/tests/`；视觉测试使用 `~/lunar-robot/lab/vision`。停止运行中的 GUI 测试请点“停止”；命令行测试用 `Ctrl-C`，然后读取退出状态与 run report。每次测试以报告记录的 commit SHA、模式、输入源和结果为准。

只在核实报告为 `OFFLINE` 或 `SIMULATION` 后，记录为相应的软件/仿真验证。`LIVE` 是对现有 ROS 图的只读观察，不能算作硬件验收。视觉模型和 P6/P7 仿真通过都不代表真实相机、雷达、底盘或机械臂已通过。

## 查询与快进同步

Mac 在本仓库根目录查询三端状态：

```bash
./tools/device_status.sh
```

只在审核目标 commit 后同步新版源码：

```bash
./tools/sync_devices.sh
```

默认目标是 ThinkBook 和 NUC 的 `~/lunar-robot/code`。同步脚本确认 Git 来源、`main` 分支、工作区干净和远端可快进；条件不符时停止，不覆盖本地修改。脚本不构建、不启动机器人，也不改写历史目录。不要通过 `THINKBOOK_REPO`、`NUC_REPO` 把默认目标改指到以下保护区：

- NUC `/home/tzb/tzb_ws`
- NUC `/home/tzb/Nav2_rea_ws`
- ThinkBook `/home/qibai/robot_p0/tzb_ws`
- ThinkBook `/home/qibai/robot_p0/Nav2_rea_ws`
- 任一设备 `~/lunar-robot/archive/` 或 `~/lunar-robot/lab/`

在设备上确认本地版本：

```bash
git -C ~/lunar-robot/code rev-parse HEAD
git -C ~/lunar-robot/code status --short --branch
```

## NUC 实机部署边界

同步到 NUC 的 `~/lunar-robot/code` 只表示源码已到位，不表示部署已验收或机器人运行。启动任何真实传感器、驱动器或执行器之前，负责人需要审核目标 SHA、设备参数和[`真实机器人安全门槛`](REAL_ROBOT_SAFETY_GATES.md)，再按受控步骤记录实机验收。当前 PR #8 STM32 安全状态机、机械臂产品固件输入、真实传感器/运动验收仍有未完成项。

本项目不使用新版部署脚本写入 NUC 原始 ROS 工作区。不得为了版本统一重置原始目录、历史 P7/P8 基线、模型或设备配置。
