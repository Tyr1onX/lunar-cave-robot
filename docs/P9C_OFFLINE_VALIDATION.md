# P9-C 离线稳定性验证

日期：2026-10-13。软件分支基于 `main` 的 `ea3f64bd37d0372ad878adaf5a33b1dc73bea38c`。ThinkBook 测试使用 P9-C 源树提交 `667699dc13e31dcd5060715ab6a2d0ce30223536`，并应用下表中的导航配置和测试脚本内容。测试仅使用 `ROS_LOCALHOST_ONLY=1`、隔离 ROS domain、PTY 与合成传感器/机械臂数据；没有连接控制器、相机、雷达或电机。

## 故障分析与修正

旧 P7 全套第二轮在 `normal` 场景遇到 DWB `Resulting plan has 0 poses in it`，导航两次失败后按既有断言中止。离线 Nav2 配置随后将 DWB 局部代价地图从 4 m 扩展为 6 m，并把 `BaseObstacle.scale` 从 `0.02` 调为 `0.2`。静态测试固定检查这三个值。没有更改目标、容差、重试次数、超时或验收断言。

另一次 `chassis_bridge_stop` 故障发生在场景启动阶段：Nav2 lifecycle manager 等待 `/bt_navigator/change_state` 超时。ThinkBook ROS 2 Humble 的 CLI 不识别原脚本设置的 `ROS2CLI_DISABLE_DAEMON`；`ros2 action info`、`ros2 topic info` 和 `tf2_echo` 因而留下每个隔离域的常驻 daemon。P7 runner 现于退出时只关闭本场景的 ROS domain daemon。独立桥接停止重跑通过；后续两轮套件运行后未发现测试域的 daemon 遗留。

## P7 完整场景复验

两轮均从完整构建开始，12 个场景全数通过。故障场景保持原有 fault-injection、链路和取消断言。

| 场景 | 期望终态 | 第一轮秒 | 第二轮秒 | 额外证据 |
| --- | --- | ---: | ---: | --- |
| `normal` | `SUCCEEDED` | 100.440 | 93.853 | 取样、返航、卸载均完成 |
| `nav_failure` | `ABORTED` | 73.428 | 73.775 | 两次 Nav2 失败后中止 |
| `target_loss` | `ABORTED` | 52.071 | 51.860 | 到达后目标缺失，未发送机械臂任务 |
| `grasp_failure` | `ABORTED` | 37.850 | 37.106 | 取消与上游静默确认 |
| `communication_loss` | `ABORTED` | 40.860 | 41.609 | 丢弃结果后取消并确认 |
| `duplicate_trigger` | `SUCCEEDED` | 110.546 | 96.076 | 只接受一个任务，重复触发被忽略 |
| `arm_feedback_stop` | `ABORTED` | 37.660 | 36.709 | `ARM_PICK` 注入故障并确认取消 |
| `arm_disconnect` | `ABORTED` | 39.061 | 39.121 | `ARM_PICK` 注入断连并确认取消 |
| `driver_restart` | `ABORTED` | 39.777 | 37.995 | 重启后保持 DISARMED，无自动使能 |
| `chassis_disconnect` | `ABORTED` | 2.405 | 2.351 | 主机检测延迟 0.017 s / 0.012 s |
| `chassis_disconnect_return` | `ABORTED` | 49.710 | 50.022 | 抓取确认后返程断连，检测延迟 0.012 s / 0.019 s |
| `chassis_bridge_stop` | `ABORTED` | 3.432 | 3.317 | 主机心跳过期，检测延迟 0.517 s / 0.501 s |

- 第一轮：`/tmp/p9c-p7-final-r1/20261012T172837Z-645699`，ROS domain 40–51，`P7_E2E_SCENARIOS=PASS cases=12`。
- 第二轮：`/tmp/p9c-p7-final-r2/20261012T174315Z-654772`，ROS domain 60–71，`P7_E2E_SCENARIOS=PASS cases=12`。

## P4f / P6 回归

ThinkBook 命令：`P4F_RESULTS_DIR=/tmp/p4f-p9c-final P4F_P6_DOMAIN_ID=83 P4F_P4E_DOMAIN_ID=84 bash tools/p4f_offline/validate.sh`。退出码 0。

- 两个独立 ROS 工作空间构建通过：P6/Nav2 1 个包、机械臂规划/驱动 6 个包、P4e PTY 驱动 2 个包。
- 真实 ROS 2 `rm_serial_driver` 的 19 项 PTY 故障测试通过。100 Hz 定时器 median 10.000 ms、p99 10.133 ms、最大 10.303 ms；PTY TX median 9.980 ms、p99 10.641 ms、最大 10.962 ms。
- P6 逻辑及静态检查 10/10 通过。真实 planner-result/cancel race 通过：规划结果在取消结果处理前产生，取消确认后的命令流保持静默，高 epoch 后续任务接管通过。
- P6 六场景均通过：`normal` 与 `duplicate_trigger` 为 `SUCCEEDED`；`nav_failure`、`target_loss`、`grasp_failure`、`communication_loss` 为 `ABORTED`。

曾用旧输出目录 `/tmp/p4f-ros2-acceptance` 和 ROS domain 98 进行一次回归，cancel-race monitor 报告历史任务 `f1e047...` 缺少确认，而同次当前事件日志记录的是任务 `b6bfa...`；目录中也留有旧的 `assertions.log`。具体为何复用路径时产生该观察尚未确定。改用新的结果目录及 domain 83/84 后 race 和整套回归通过。

## 测试源码 SHA-256

```text
P7 run.sh: c25f202b776986af1f5255bece68159f8a825a8b5238bcf1a9ab5f9f7b2caa55
P7 validate_scenarios.sh: 4e1eecb4ffabb157fa91b1933c88bef5061fa4a89396ba13998309e9ba569758
P7 mission_control.py: cbf8245cf2ba3d22afa790a124ca33dcc81717bddac1879360abecc601f9fcd6
P6 mission_coordinator.py: 90a43596618266b71c9e958265c4310bdac9cb9c3c17fb1791e93d2a8018a626
P6 nav2_offline.yaml: 05a19bd7e58446f00ab0e75c167633dfbab669fa9c123913f703df47a5398f41
P6 test_p6_static_contract.py: 749eb7fbd0c814317283d06b28ab9e934e52d1bbd60ac6b07bd51fdd9bcd9484
```

该记录只表示 ThinkBook 离线仿真和 PTY 回归结果。没有测试实体停止距离、机械臂承载/防坠、真实传感器数据或实体控制器固件。
