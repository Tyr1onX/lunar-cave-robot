# 开发、验证、同步与部署流程

## 日常开发

1. 在 ThinkBook 的 `~/lunar-cave-robot` 从 `main` 建开发分支。
2. 按 P8 部署说明分别构建 `tzb_ws` 和 `Nav2_rea_ws`；不要合并两个工作空间或在一个终端叠加其 `install/setup.bash`。
3. 只运行仓库提供的离线验证。实体执行器保持禁用；不要因源码同步启动 ROS 节点。
4. 提交并通过 PR 合入 GitHub Private `main`。仓库记录源码版本。

## 查询与安全同步

Mac 需要可用的 SSH 别名 `thinkbook`、`nuc`，以及对 GitHub 私有仓库的 Git 访问权限。在仓库根目录：

```bash
./tools/device_status.sh
./tools/sync_devices.sh
```

同步脚本对缺失的预定目录执行 `git clone --branch main`；对已有目录只接受正确 origin、当前分支为 `main`、工作树干净、没有本地独有提交且远端可快进的情况。同步通过 `fetch` 和 `merge --ff-only` 完成。若本地有改动、分叉、仓库身份不符或远端 main 在操作期间变化，脚本停止并保留现场。它不删除、重置或备份既有工作空间，不运行机器人程序，不连接控制器。

如果检查显示目标目录不是统一 clone 或含修改，应先人工审阅，再将需要保留的改动提交到分支/PR；不要清理旧目录来绕过保护。新的源码版本先只进入统一 Git 工作目录；NUC 的 ROS 安装、SDK、设备私密配置和部署目录保持各自管理。

## ROS 2 离线验证

在每台设备统一 clone 的根目录分别运行：

```bash
./tools/run_offline_validation.sh
```

该入口执行 P7/P8 集成 PTY 离线场景，不会访问真实串口、CAN、相机、雷达或电机。测试结果带有仓库 SHA 保存到用户状态目录，供 `device_status.sh` 展示。不同设备依赖缺失时按错误输出补齐依赖，不得把跳过或未运行标记为通过。

## NUC 部署与回滚

Git 同步只更新源码，不等于将该版本部署到车载运行目录。部署前应由负责人确认目标 SHA、设备专用参数、驱动 SDK 与安全门槛，并沿用 `docs/REAL_ROBOT_SAFETY_GATES.md` 的受控流程。运行程序和执行器需显式人工启动；P9 脚本不会自动启动。

记录部署版本时，由部署负责人在 NUC 写入 `~/.local/state/lunar-cave-robot/deployed-sha`（或 `$XDG_STATE_HOME` 对应位置），内容为实际部署源码 SHA，并保存相应配置/构建记录。回滚应选择已审核的提交 SHA，先确认工作目录干净，再在部署工作流中由负责人执行；不要用强制 reset 覆盖工作区。源码统一 clone 保留 Git 历史，可据状态报告确认从哪个提交回到目标提交。
