# Lunar Cave Robot｜月球洞穴机器人

基于 **Ubuntu 22.04 / ROS 2 Humble** 的移动机器人与机械臂协作工程。仓库包含导航、底盘串口控制、RGB-D 视觉、机械臂规划与抓放，以及自主采样任务协调代码。

> **当前默认版本：`main`（2026-10-12 整合至 P8 软件基线）。** P0～P8 的现有 ROS 2 主线代码已统一归档；[`p8/nuc-integration-ready`](https://github.com/Tyr1onX/lunar-cave-robot/tree/p8/nuc-integration-ready) 保留为阶段开发记录。
>
> **验收边界：** P7/P8 在 ThinkBook 与 NUC 上通过了真实 ROS 2 节点配合模拟数据、虚拟串口（PTY）的任务级离线验收；**尚未通过实机运动、传感器数据、STM32 控制或实体安全验收**。不得将软件离线通过视为真实机器人可安全运行。

## 项目结构

| 路径 | 内容 |
| --- | --- |
| [`Nav2_rea_ws/`](Nav2_rea_ws/) | 底盘导航、Nav2、雷达/定位、P5b 串口桥接、P6 自主任务协调 |
| [`tzb_ws/`](tzb_ws/) | RGB-D 视觉、YOLO 检测、目标位姿估计、机械臂模型、规划与安全串口驱动 |
| [`tools/p7_offline/`](tools/p7_offline/) | 不接触真实串口的 ROS 2 + PTY 端到端验收入口与场景测试 |
| [`tools/p6_offline/`](tools/p6_offline/) | P6 任务逻辑测试与离线验证工具 |
| [`firmware/`](firmware/) | 机械臂与六轮底盘 STM32 固件审计、接口摘要和授权状态；**不包含固件工程源码** |
| [`docs/`](docs/) | 架构、各阶段验收、安全约束、部署及发布审计 |

核心链路（**P7 离线验收**）：

```text
P6 自主任务协调
  ├─ Nav2 规划 / 控制 → 速度平滑 → P5b 底盘串口桥 → PTY 模拟底盘
  │                                                    └─ 模拟里程计 / TF / 扫描反馈
  └─ 目标与抓放任务 → P3b 机械臂规划 / P4f 轨迹管理
                     → P4e 主机安全驱动 → PTY 模拟 STM32 → 模拟关节反馈
```

实际硬件所需的相机、MID-360、定位、外参、真实 MCU 应答及制动/急停**不在这条离线通过的闭环内**。完整模块、话题和 TF 说明见[系统架构与交接指南](docs/ARCHITECTURE.md)；其中原始模块说明形成于早期快照，P7/P8 的集成、运行结果及安全边界以本页下面链接的最新验收报告为准。

## 当前进展

| 阶段 | 已完成并记录的范围 | 仍未验证 |
| --- | --- | --- |
| P0～P2b | ROS 2 源码恢复、机械臂规划与合成抓放、RGB-D 数据检查/回放工具 | 真实相机、标定与抓取 |
| P3～P4f | TF/时间戳、URDF、串口协议、关节映射与 ROS 主机安全/轨迹生命周期的离线测试 | 真实 STM32 停止确认、机械安全 |
| P5a～P5b | 底盘源码审计、Nav2 到 14 字节串口帧的主机侧控制链和 PTY 检查 | 控制器反馈、里程计、真实刹停 |
| P6～P7 | ROS 2 任务协调、Nav2 / 机械臂 / 两条模拟串口的完整离线闭环 | 真实任务、运动控制和感知 |
| P8 | ThinkBook 与 NUC 上 12 个 P7 离线场景分别通过；NUC 上 MID-360 驱动编译通过；增加底盘主机串口失联检测 | MID-360 实际点云/IMU、整机实机联调 |

详细证据：[P7 ROS 2 验收](docs/p7/ros2-runtime-acceptance.md) · [P8 验收报告](docs/P8_VALIDATION_REPORT.md) · [P8 部署记录](docs/P8_DEPLOYMENT.md)。

## 2026-10-12 ThinkBook 源码归档核对

ThinkBook 上的 P0～P8 工作区、历史分支和可访问项目归档已与最新 `main` 对照。两个 ROS 2 工作空间在当前主线中保留；没有发现可安全加入且未被 P8 替代的 ROS 2 源码。P4f 工作区的三处独有代码差异会移除 P8 的安全/超时保护，因此未回灌。旧 `nav_ws.zip`、`serial_t.zip` 版本已被当前导航和串口实现替代，来源/许可证据不足的额外内容没有复制。

本轮确认的底盘固件输入包具有完整 STM32H723/CubeMX/CMake/Keil 工程骨架，但缺少项目级许可和团队内部共享授权，且本机没有 STM32 工具链，未上传也未编译。机械臂固件只可核对到以往审计记录；本次 ThinkBook 候选路径没有找到原始包，源码共享授权同样未确认。具体来源、排除项和验证边界见[发布审计](docs/RELEASE_AUDIT.md)及 [firmware/](firmware/)说明。**因此当前仓库具备 ROS 2 源码交接基础，但还不是包含两套获准固件源码的完整源码交接包。**

## 快速接手：先跑安全离线流程

环境：**Ubuntu 22.04、ROS 2 Humble、colcon** 及对应 ROS 系统包（含 Nav2、机械臂规划相关依赖）。完整依赖和独立构建步骤见 [P8 部署文档](docs/P8_DEPLOYMENT.md)。首次构建须按错误信息与 `rosdep` 检查缺失依赖；**不要为了通过构建从原机器复制未授权的 SDK 二进制或模型权重**。

1. 克隆默认主分支：

   ```bash
   git clone https://github.com/Tyr1onX/lunar-cave-robot.git
   cd lunar-cave-robot
   ```

2. 运行**不连接实体设备**的 P7 正常场景（默认先分别构建所需 ROS 包）：

   ```bash
   P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator ./tools/p7_offline/run.sh
   ```

   启动器强制 `ROS_LOCALHOST_ONLY=1`、独立 ROS Domain，并自行创建、验证 `/dev/pts/*` 模拟串口；拒绝外部指定的串口路径。须看到 `PASS serial isolation: only runtime-created PTYs are configured`。这里的 `enable-offline-simulator` **只是对模拟控制器的显式使能，不是实体机器人使能命令**。

3. 需要完整故障场景验收时运行：

   ```bash
   ./tools/p7_offline/validate_scenarios.sh
   ```

   脚本包括正常任务、导航失败、目标丢失、机械臂故障及串口/桥接异常，共 12 个离线场景。历史验收结果详见 [P8 验收报告](docs/P8_VALIDATION_REPORT.md)；新环境仍应独立复测，不能预设结果。

**工作空间隔离：** `tzb_ws/` 与 `Nav2_rea_ws/` 保持独立构建和运行，仍存在重名 `rm_serial_msgs` 包；不要在同一终端随意叠加两个 `install/setup.bash`，也不要把两套源码合并为同一个 colcon 工作空间。P7 启动器已按不同进程隔离所需 overlay。

## 接手工作建议

阅读顺序：**本 README → [架构与模块入口](docs/ARCHITECTURE.md) → [P8 部署与验收](docs/P8_DEPLOYMENT.md) → 自己负责模块的 launch、配置和源码**。

每个模块接手时，至少明确负责人、源码入口、输入/输出 Topic、运行依赖、已验证命令、未完成项和测试结果。修复或新增能力时在独立分支提交 PR，说明**实际执行的测试**；没有执行的验证不要写成通过。实机事项须由设备及安全负责人确认，不能直接套用离线 launch。

最优先的后续工作：确认两套 STM32 固件及协议的发布/接手权限；补齐真实硬件停止确认、急停/制动与机械限位验收；随后分阶段完成 MID-360 静态点云、相机及手眼标定、定位/里程计、受控运动验证。具体准入要求见[实体机器人安全门槛](docs/REAL_ROBOT_SAFETY_GATES.md)和[MID-360 静态验收](docs/MID360_STATIC_ACCEPTANCE.md)。

## 代码来源与公开发布

本仓库目前为**内部私有工程**，不是已获准公开分发的完整开源发布：

- 机械臂和底盘 STM32 固件源码、模型权重、原始 bag / 私有地图、授权受限 SDK 二进制及运行日志**未纳入版本库**，需分别向权利人取得授权和资产。
- 工程包含不同来源及许可证的组件；各第三方原始许可与通知应保留，**不得擅自将整个工程统一授权为 MIT 或其他许可证**。
- 对外设为 Public 前，仍需完成团队代码发布许可、第三方许可证义务、文件及 Git 历史敏感信息审计。

详见[发布审计](docs/RELEASE_AUDIT.md)和[第三方代码许可清单](THIRD_PARTY_NOTICES.md)。如仅需邀请学长审阅，先通过 GitHub 私有仓库协作者权限共享即可。
