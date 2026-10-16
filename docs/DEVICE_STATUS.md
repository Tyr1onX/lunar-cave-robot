# GitHub、ThinkBook 与 NUC

GitHub 私有仓库 `Tyr1onX/lunar-cave-robot` 的 `main` 是正式新版源码。设备上的新版目录关系如下：

| 端 | 正式新版代码 | 用途 |
| --- | --- | --- |
| GitHub | `Tyr1onX/lunar-cave-robot` 的 `main` | 新版源码来源 |
| ThinkBook | `/home/qibai/lunar-robot/code` | 新版开发、构建和离线验证 |
| NUC | `/home/tzb/lunar-robot/code` | 独立新版 checkout、Console 受限测试服务 |

截至 2026-10-16，两台设备的 `code` 都位于 `main` SHA `c2f4a5ffb6fc764713af6040b11cd648b750b3a5`，工作树干净。ThinkBook 已有 `tests/state/`、`archive/baseline/` 与 `archive/p8/`；NUC 另有 `tests/config/`、`tests/results/`、`tests/ros-home/`、`tests/state/`、`lab/vision/`、`lab/simulation/` 和历史 `archive/`。具体清单见[工作区布局](WORKSPACE_LAYOUT.md)。

**NUC 原始工程保护区**：`/home/tzb/tzb_ws`、`/home/tzb/Nav2_rea_ws`。ThinkBook 历史原始工作区为 `/home/qibai/robot_p0/tzb_ws`、`/home/qibai/robot_p0/Nav2_rea_ws`。新版同步、构建和测试只指向 `~/lunar-robot/code`，不得设置为这些原始目录或历史目录。

## 查询设备状态

在 Mac 侧项目工作区执行：

```bash
./tools/device_status.sh
```

脚本只读查询 GitHub `main` 与两台设备的新版目录，显示 SHA、分支、脏状态和支持的测试记录。默认 SSH 别名为 `thinkbook`、`nuc`；默认远程仓库路径为 `~/lunar-robot/code`。可用 `THINKBOOK_ALIAS`、`NUC_ALIAS`、`THINKBOOK_REPO`、`NUC_REPO` 覆盖，但不要把仓库路径设置到原始 ROS 工作区、`archive/`、`lab/` 或 `tests/`。

在任一设备上确认版本：

```bash
git -C ~/lunar-robot/code rev-parse HEAD
git -C ~/lunar-robot/code status --short --branch
```

新增测试报告和日志位于 `~/lunar-robot/tests/state/lunar-cave-robot/`；Console 模块测试记录在其 `module-tests/` 子目录。历史记录仍可能保存原路径字符串，它们是迁移前的测试材料，不表示当前服务仍从旧目录运行。当前 NUC 服务单元和 drop-in 副本在 `~/lunar-robot/tests/config/`。

“有源码”“测试通过”和“已经部署”是不同状态。部署 SHA 只有经过负责人实际部署确认后才能记录；OFFLINE/SIMULATION/LIVE 观察均不自动形成实机验收。
