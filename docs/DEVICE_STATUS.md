# 三端设备状态

GitHub 私有仓库 `Tyr1onX/lunar-cave-robot` 的 `main` 是源码版本来源。ThinkBook 用于开发和离线测试；NUC 用于车载部署准备。默认统一工作目录如下：

| 端 | 统一仓库目录 | 既有工作区处理 |
| --- | --- | --- |
| GitHub | `Tyr1onX/lunar-cave-robot`，`main` | 唯一共享源码入口；保持 Private |
| ThinkBook | `~/lunar-cave-robot` | 新建统一 clone；保留 `~/lunar-cave-p8-validation` 及其它旧工作区 |
| NUC | `~/lunar-cave-robot` | 新建统一 clone；保留 `tzb_ws`、`Nav2_rea_ws`、`nav_ws`、`mid360_driver_dev` 等现场 |

## 查询实时状态

从 Mac 上这个仓库的根目录运行：

```bash
./tools/device_status.sh
```

脚本以只读方式读取 GitHub `main` SHA，并通过 SSH 别名 `thinkbook` 和 `nuc` 查询统一目录的 HEAD、分支、未提交文件数、与 GitHub 的提交差异、最近一次脚本测试记录和显式部署 SHA。可用 `THINKBOOK_ALIAS`、`NUC_ALIAS`、`THINKBOOK_REPO`、`NUC_REPO` 覆盖默认值。

设备最近测试记录写在各自 `$XDG_STATE_HOME/lunar-cave-robot/latest-test.yaml`（默认 `$HOME/.local/state/lunar-cave-robot/latest-test.yaml`），包括时间、结果和测试时的 Git SHA。部署版本记录只有在部署负责人确认后才可以手动写入 `deployed-sha`；代码同步脚本不写入它。若状态不可读取，脚本显示 unknown，不推测状态。

## 已知基线与限制

P8 验收报告记录两台设备旧工作区 `~/lunar-cave-p8-validation`、`~/lunar-cave-p8` 的 HEAD 为 `f6005a076997a7e05e079408f7e84e1b6b9bdb8f`，并有 12 个 PTY 离线场景通过记录。此为 P8 的历史验收 SHA；P9 新统一目录的实时状态以脚本输出为准。NUC 另有 MID-360 驱动构建通过记录，未启动雷达采集。

首次同步前，ThinkBook 的 P4f 工作区有未提交变更，NUC `tzb_ws` 的 README 有本地修改；这些目录保持原样。NUC `mid360_driver_dev` 是独立旧分支，包含对 P7/P8 有回退的差异，不纳入统一源码目录。新增统一 clone 后不会改写这些旧工作区。
