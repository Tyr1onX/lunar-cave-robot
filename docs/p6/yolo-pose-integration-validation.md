# YOLO → 位姿估计 → P6 隔离集成验收记录

验收日期：2026-10-18

验证环境：ThinkBook，Ubuntu 22.04，ROS 2 Humble

源码基线：`ccf761972819603eb8f0a3f76e965b608eeaa68e`（PR #49 合并后的 `main`），加本 PR 工作区改动

## 隔离与构建

- 从仓库工作区复制 `p6_autonomous_mission`、`yolo_node`、`pose_estimator` 源码到 ThinkBook 的临时目录 `/tmp`，构建输出也留在该目录。
- 三个 ROS 包在 Humble 下独立构建成功。ROS 场景每次使用单独 Domain，并设置 `ROS_LOCALHOST_ONLY=1`。
- RGB、CameraInfo、PointCloud2、目标检测图片和任务结果均由测试节点生成，默认发布频率 1 Hz（代码限制为 0.2–2 Hz）。OpenVINO 权重从测试主机的外部目录只读引用；未复制进仓库或测试归档。
- Launch 使用 CPU 推理、最高 2 Hz 合成输入、P6 cave simulator 与合成任务反馈。不包含相机、雷达、串口、底盘、机械臂或执行器驱动。
- 未修改 ThinkBook 原始 ROS 工作区和 NUC 服务；没有启动真实硬件。

## 组合场景

| 场景 | 实际观察 | 结果 |
|---|---|---|
| 正常接口链路 | YOLO 发布门控阈值 `0.1`；合成目标最高分 `0.38`，门控打开；收到检测和位姿样本；P6 记录 `TARGET_CONFIRMED`、`ARM_REQUEST_SENT` 与 `SAMPLE_PICK_CONFIRMED`。 | 通过。组合测试在合成抓取反馈确认后结束；该记录不代表本轮完整返航与卸载通过。 |
| 无目标 | 空白 RGB 输入；候选阈值 `0.0001`、发布门控 `0.1`；检测与位姿输出均为 0；P6 因 `DETECTING_TARGET` 阶段超时中止。 | 通过。 |
| 低置信度 | 有目标合成图；候选阈值 `0.0001`、发布门控 `0.5`；实际最高分 `0.38`，`publish=False`，检测与位姿输出均为 0；P6 在目标确认阶段超时中止。 | 通过。 |
| 深度超时 | RGB/YOLO 保持输出，点云发布器停止；记录到 50 条检测消息、0 个点云样本和 0 个位姿；P6 在目标确认阶段超时中止。 | 通过。 |
| 任务取消 | 合成目标进入位姿与 P6 抓取阶段；记录 `ARM_CANCEL_SENT`、匹配任务的 `ARM_CANCEL_CONFIRMED`、合成失败反馈及 `ABORTED`。确认内容仅表示 ROS 上游发布者已静止。 | 通过。 |
| 深度恢复 | 一次隔离运行在 P6 进入 `DETECTING_TARGET` 后暂停深度输入；点云在约 5.18 秒后恢复，随后观察到位姿和 `SAMPLE_PICK_CONFIRMED`。 | 该次恢复通过。另一次复测在 Nav2 到达目标前以状态 6 中止，未进入恢复检查阶段，故当前只记录一次恢复通过。 |

在测试门控 `0.0005` 的一次无目标试跑中，模型对纯色空白图仍产生弱候选并发布，导致 P6 接受了目标。该现象促使无目标测试使用当前 Launch 默认发布门控 `0.1`，同时保留低置信度场景对门控行为的独立断言。它说明极低阈值下的合成负样本可能有弱误检；不能据此推断现场误检率。

## P6 完整闭环复测

在合成 Launch 中增加 `--complete-mission` 后，正常/深度恢复场景必须等到 P6 终态，并核对目标导航、目标站停车、检测、抓取结果、返航导航、返航停车、卸载结果及最终 `SUCCEEDED` 事件。此前 smoke 默认在抓取确认后结束；该检查点不会再被解释为完整任务通过。

复测发现并修正了两项隔离测试配置问题：所有 ROS 夹具进程必须使用同一套 ROS Humble 感知虚拟环境（其中包含 `rclpy`、OpenVINO、Open3D、NumPy 和 OpenCV）；输入发布器依赖缺失时，smoke 现在会从 Launch 日志尽早失败。系统 Python 缺依赖而产生的运行不计入产品验收。

ThinkBook ROS 2 Humble、`ROS_LOCALHOST_ONLY=1`，真实 OpenVINO YOLO/位姿节点、合成 RGB-D、P6、Nav2 cave simulator 和模拟机械臂/卸载反馈组成隔离流程。测试阶段未启动传感器、串口、真实机械臂、底盘或执行器；模型通过外部只读路径加载，未复制到仓库或日志目录。

基线运行在目标站 `(10.0, 6.5)` 处返回 Nav2 Action 状态 6。P6 失败事件记录当前位置 `(8.87, 5.83)`，附近静态障碍范围为 `x=8.8..9.35, y=5.1..5.8`；规划器随后报告起点位于 lethal cost cell，Nav2 恢复行为也遇到过期 TF 查询，无法取得当前 footprint。生命周期节点已全部 active，故障来自局部路径与目标站相对障碍的净空不足，不是模块未启动。提高离线全局/局部 costmap 的 inflation radius 至 `0.60 m` 后，目标导航成功，但原目标站仍使返航起点落在膨胀边缘。修正仅适用于合成组合 Launch：其目标站覆盖为 `(10.5, 6.5)`；P6 默认目标值保持 `(10.0, 6.5)`。

当前有效完整闭环运行（Domain 226）最终状态为 `SUCCEEDED`，事件顺序包括：

`NAVIGATING_TO_TARGET → CONFIRMING_TARGET_STOP → DETECTING_TARGET → ARM_PICK → NAVIGATING_HOME → CONFIRMING_HOME_STOP → ARM_UNLOAD → SUCCEEDED`

记录到 43 条检测消息、129 个检测项、40 个位姿、43 个合成点云样本；YOLO 最高置信度 `0.38`、发布门控打开。目标和返航两次 Nav2 Action 均返回 `SUCCEEDED`，各自都有到达与停车证据；P6 分别记录 pick/unload 的成功反馈和 `BASE_UNLOAD_CONFIRMED`。空控制路径、控制器进度失败、控制/规划周期未达标和 lethal 起点的计数均为 0。可复现结果目录：ThinkBook `/tmp/lunar-p6-yolo-pose-20261018-ccf7619/results/full-closure-safe-target/normal-dq04gjyv/`，包含 `summary.json`、`mission-events.jsonl` 和 `combined-launch.log`。

完整闭环重复验证尚未稳定：Domain 227 和 Domain 228 两次复跑均完成目标导航、目标识别和模拟抓取，但返航 Action 状态为 6，P6 终止为 `ABORTED`。Domain 227 的返航记录了 151 次空控制路径、28 次 controller patience exceeded、5 次控制周期未达标和 2 次规划周期未达标；没有记录 lethal 起点或无效全局路径。Domain 228 记录了 56 次空控制路径、28 次 patience exceeded、2 次控制周期及 2 次规划周期未达标，并出现 11 次 TF 外推、11 次 footprint 获取失败和 11 次恢复行为失败。

Domain 228 的首条 TF 证据发生于 `1792320499.173`：恢复行为查询时间为 `1792320488.908819`，但 TF 缓存最早数据为 `1792320489.163407`，查询时点落后于可用缓存约 10.3 秒。Humble 的 `FootprintSubscriber` 会用 footprint 消息头时间查询 `base_link` 到 footprint frame 的变换；因此当前日志与足迹时间戳过旧相符，但还没有同一时段 `/local_costmap/published_footprint` 原始消息时间和发布频率，不能据此断定源头。下一次隔离复测需同步采集 footprint 消息头时间、costmap 更新日志、TF/里程计时间、控制器局部路径及 costmap 快照，确认哪个生产者停止更新后再针对性修改。Nav2 Humble 的 DWB 实现会在局部变换路径为空时抛出 `Resulting plan has 0 poses in it`；本次返航还未证明是 P6 任务状态机错误。

复现目录：Domain 227 `/tmp/lunar-p6-yolo-pose-20261018-ccf7619/results/full-closure-repeat/normal-axwjod4o/`；Domain 228 `/tmp/lunar-p6-yolo-pose-20261018-ccf7619/results/full-closure-repeat-idle/normal-uu_mrp3c/`。两份目录均包含 `summary.json`、`mission-events.jsonl` 和 `combined-launch.log`。当前完整闭环验收**未通过重复性要求**：虽然 Domain 226 有一次完整 `SUCCEEDED`，两次返航复跑均失败，不应合并为已稳定的闭环。最终成功标准仍为 P6 `SUCCEEDED` 及匹配的目标导航、抓取、返航导航、停车、卸载事件，不以视觉或模拟抓取单项代替。

为下一次隔离复测增加了 `runtime-observations.jsonl` 采集：记录 `/odom`、`/tf` 中 `odom→base_link`、`/local_costmap/published_footprint` 的源时间戳与接收间隔，并自动发现 `nav_msgs/Path` 类型的导航路径 Topic，记录路径长度、坐标范围和时间。`summary.json` 汇总每项数据的样本数、最大接收间隔、最大源时间年龄及非单调时间戳数。该诊断仅添加只读订阅，不发 ROS 命令、不调用 Nav2 服务、不写工作区配置。

可复现完整流程命令需在 ThinkBook 上 source ROS Humble 与隔离构建的三个包，并给三个子进程指向同一 ROS-enabled Python 环境；本次固定输入频率为 `0.5 Hz`，每次使用未占用的 Domain：

```bash
P6_SYNTHETIC_PYTHON=/path/to/ros-humble-python \
YOLO_NODE_PYTHON=/path/to/ros-humble-python \
POSE_ESTIMATOR_PYTHON=/path/to/ros-humble-python \
python3 tools/perception/p6_yolo_pose_integration_smoke.py \
  --scenario normal \
  --complete-mission \
  --require-clean-nav2 \
  --rate-hz 0.5 \
  --model-dir /path/to/blue_block_model \
  --domain-id 229 \
  --results-dir /tmp/lunar-p6-full-closure
```

不同轮次应使用未占用的 Domain 和独立结果目录。当前 ThinkBook 在另一项隔离 P2 Gazebo/YOLO 仿真持续占用多核 CPU，故本次修改后未并发启动新的 ROS 场景，避免影响独立验证任务并污染控制周期测量。此限制下尚未针对最新诊断采集器重跑取消、超时和深度恢复完整场景；PR #52 的既有场景记录不替代当前提交后的复测。

## Console 观测范围

源码检查确认，当前 Agent Graph 能展示 YOLO、位姿估计及 P6 节点和 Topic 名称，视觉接口可读取 RGB 与 YOLO 标注图像。固定 Topic 遥测不包含 `yolo_node/DetectionArray`、`/pose_estimator/object_pose` 或 `/p6/mission_state`、`/p6/mission_events` 消息正文。因此 Console 尚不能在页面中检查实际检测置信度、位姿样本或任务阶段事件；本轮 smoke 测试由独立 ROS 订阅端读取这些数据。当前检查为源码审计，未完成本机 GUI 实际启动验收。

## 结论边界

已验证的是 ROS 2 Humble 中的合成数据与实际 YOLO、位姿估计、P6、Nav2 软件节点组合；不能据此证明相机标定、真实 RGB-D 同步、现场 TF、实体雷达/定位、机械臂规划与执行、底盘运动、STM32 收到或执行命令，以及急停或安全互锁状态。P6 测试关闭了实体状态接口的必需检查，并使用合成任务反馈。完整自主任务闭环仍需真实传感器和执行端逐项受控验收。
