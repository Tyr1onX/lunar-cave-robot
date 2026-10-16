# NUC 新人快速入门

NUC 的三个主要机器人项目入口：

- **原始机械臂与视觉**：`/home/tzb/tzb_ws`，原始 `README.md` 有 92 行 ROS 话题与节点接口说明；只读参考。
- **原始导航与雷达**：`/home/tzb/Nav2_rea_ws`，根目录无 README；只读参考。
- **我们自己的新版**：`/home/tzb/lunar-robot`，日常新版正式代码在 `code/`。

这三个入口之外仍有厂商 SDK、历史辅助数据等目录，未认定所有文件均属于我们的新版，禁止未经确认移动或删除。首次使用可通过已授权的 SSH 访问 `/home/tzb/README.md`。该总入口文档和 `/home/tzb/lunar-robot/README.md` 已部署到 NUC，未修改原始两个工程。

---

## 我们自己的新版项目

这是 NUC 上新版机器人项目的唯一日常入口：`/home/tzb/lunar-robot/`。

**学姐第一次接触 NUC，先看本页；不需要理解 P1～P9 的开发阶段编号。**

## 四类目录

| 目录 | 中文含义 | 如何使用 |
| --- | --- | --- |
| `code/` | **正式代码** | 开发、查看 Git 版本、运行经过批准的离线测试 |
| `tests/` | **测试资料** | 查看 GUI 测试状态、日志、报告、服务配置 |
| `lab/` | **实验内容** | `vision/` 为视觉/模型实验；`simulation/` 为仿真实验 |
| `archive/` | **历史档案** | 保留早期版本、P8 验收和迁移前旧目录；不要当成正式运行目录 |

```text
/home/tzb/
├── tzb_ws/                   ← 原始机械臂、视觉（只读）
├── Nav2_rea_ws/              ← 原始导航、雷达（只读）
└── lunar-robot/              ← 我们自己的新版
    ├── code/                 ← 正式代码
    │   ├── tzb_ws/           ← 新版机械臂、视觉工作空间
    │   ├── Nav2_rea_ws/      ← 新版导航、雷达工作空间
    │   ├── firmware/         ← STM32 固件源码
    │   ├── tools/            ← Mac GUI、Agent、离线测试脚本
    │   └── docs/             ← 版本和技术文档
    ├── tests/                ← 报告、日志、运行状态
    ├── lab/                  ← 视觉和仿真实验
    │   ├── vision/
    │   └── simulation/
    └── archive/              ← 历史副本
        ├── baseline/
        ├── p8/
        └── old-copies/       ← 迁移前五个目录，已统一归档
```

## 日常安全查看（只读）

```bash
cd /home/tzb/lunar-robot/code

# 我们正在使用哪个源码版本？
git status --short --branch
git log -1 --oneline

# 查看 GUI 后端服务状态；不会启动机器人
systemctl --user is-active lunar-console-agent.service
systemctl --user is-active lunar-console-module-tests.service

# 查看 README 和已有的技术文档
less README.md
ls docs
```

当前目录是 GitHub 私有仓库 `Tyr1onX/lunar-cave-robot` 的一个 checkout。**GitHub main 是新版正式源码来源，但当前 NUC 代码同步到 main 不等于已经完成实机部署。**

## Mac GUI 能做什么？

Mac 上的 **Lunar Robot Console** 已连接 NUC 的两个本机服务：

- Console Agent `127.0.0.1:8765`：只读查看系统和 ROS 状态。
- 模块测试 `127.0.0.1:8769`：仅允许运行白名单内的 OFFLINE、SIMULATION 和 LIVE 只读巡检；可以在 GUI 点击「一键执行」「停止测试」。

模块测试报告位于：`/home/tzb/lunar-robot/tests/state/lunar-cave-robot/module-tests/`。

**不要把「GUI 可以测试」理解为「真实机器人已经可以一键启动」。** 实体相机、MID-360、底盘和机械臂的综合实机启动仍未验收。

## 历史副本和原版

- `archive/old-copies/README.md` 提供旧路径与归档位置对照；五个目录整体归档，保留 Git 历史。
- `archive/baseline/` 和 `archive/p8/` 保存更早的基线、构建及验收材料。
- 原版 `/home/tzb/tzb_ws/`、`/home/tzb/Nav2_rea_ws/` **不属于本项目新版目录**，不要修改。
- 原版的 `tzb_ws/README.md` 记录 ROS 接口；原版 `Nav2_rea_ws/` 根目录没有 README。

更多开发资料见 `docs/WORKSPACE_LAYOUT.md` 与 `docs/WORKSPACE_HANDOFF.md`（与本页同属 PR #28，合并前不一定在设备当前的 `code/docs/` 下）。

## 一句话记忆

**看旧版去原始两个工作空间；做我们的新版去 `lunar-robot/code`；找日志去 `tests`；找历史去 `archive`。**
