# P4f 机械臂轨迹生命周期与防重放

## ROS 命令上下文

P6 分阶段任务请求使用 task_id|phase|epoch，取消使用 task_id|epoch，结果使用 task_id|phase|outcome|epoch。P6 为每个 arm 阶段分配随机任务 ID 和由系统纳秒时钟推进的 64 位 epoch；epoch 必须递增，超前超过 60 秒的值会被拒绝。

每条 P6 阶段关节命令仍送往 P3b 模拟反馈 topic，并同步发布到 P4e 默认输入 `/tzb_catch/arm_joint_command`，两份消息包含相同的原子上下文。非 P6 阶段不向 P4e 镜像 topic 转发。P4e 订阅默认值和 P6 镜像默认值一致。

每条 tzb_catch 关节命令的 sensor_msgs/JointState.header.frame_id 携带原子命令上下文：

    p4f:<task_id>:<epoch>:<sequence>

任务 ID、epoch、序号与关节目标在同一 ROS 消息内，不依赖时间戳推断新旧。序号从 1 开始，在一个任务 epoch 内严格递增；ROS 时间戳仍用于年龄和未来时间校验。JointTrajectory.header.frame_id 也携带相同格式的上下文。该字段是本项目的 P4f 控制元数据，接入其他 JointState 消费者时应忽略或保留此字段，不应把它解释成机器人 TF frame。

P6 arm request、result、cancel 和 cancel acknowledgement topics：

| Topic | 内容 |
| --- | --- |
| /p6/arm_task/request | task_id|pick_or_unload|epoch |
| /p6/arm_task/result | task_id|stage|success_or_failure|epoch |
| /p6/arm_task/gripper_command | task_id|epoch|sequence|open_or_close_value |
| /p6/arm_task/cancel | task_id|epoch |
| /p6/arm_task/cancel_ack | task_id|epoch|UPSTREAM_QUIESCED |
| /serial/arm_task/cancel_ack | task_id|epoch|ROS_HOST_CANCELLED_STM32_UNCONFIRMED |

P6 拒绝任务 ID、阶段或 epoch 不匹配的结果。抓取节点在执行中的取消后使当前上下文失效，检查所有后续命令发布，停止工作线程的规划/执行循环，并在工作线程确认不再发布后清除回放轨迹、TCP 轨迹和冻结目标缓存，再发布 UPSTREAM_QUIESCED。取消与规划结果并发时，返回的规划结果只有在当前任务仍有效时才能执行。

机械爪命令也由 `/p6/arm_task/gripper_command` 发送为 `task_id|epoch|sequence|value`，与关节轨迹共用单调序号。P4e 在 P4f epoch 高水位建立后拒绝未标记的旧 Int8 夹爪命令；旧夹爪消息不能绕过轨迹代际屏障。

## P4e 驱动防重放

P4e 驱动只接受带有效 P4f 元数据的新命令。对旧 epoch、已取消 epoch、重复/倒退序号和不是当前活动任务的同 epoch 命令，驱动拒绝更新关节目标。第一次接受新 epoch 前，将 epoch 高水位写入 ~/.ros/lunar_cave_arm_trajectory_epoch；P6 抓取节点另将任务 epoch 高水位写入 ~/.ros/lunar_cave_p6_task_epoch。reset、enable、串口重连和 ROS 驱动进程重启都不降低高水位。重启后驱动保持 DISARMED，仍须人工 reset/enable；旧 epoch 即使时间戳被更新也无法重新接管。

取消 topic 能在第一条轨迹尚未到达时安装持久取消屏障。驱动取消后清除主机目标与夹爪命令，并在状态里报告任务、epoch 和最后序号。序号、epoch 与任务 ID 都只存在于 ROS 侧，未改变 STM32 帧。

## 取消确认的边界与 46 字节协议

UPSTREAM_QUIESCED 表示 tzb_catch 工作线程不会再发布该任务的命令。P4e 的 ROS_HOST_CANCELLED_STM32_UNCONFIRMED 表示驱动已使 ROS 主机目标失效并拒绝该任务的后续命令；它不代表机械臂或 STM32 已停止。驱动仍按既有 46 字节布局发送帧，没有修改固件或电机使能、保持扭矩、急停策略。

46 字节协议不携带任务 ID、轨迹 epoch、ROS 命令序号或控制器队列取消确认。驱动无法撤回已经交给串口或 STM32 的字节，也无法确认控制器是否还在执行已接收的目标。协议中的使能位清零不等同于电机急停，不能证明制动距离、保持力或无重力坠落风险。要完成物理侧闭环，需要 STM32 固件提供可确认的任务/epoch 代际、重复序号拒绝、命令队列取消以及停止状态反馈；本任务没有修改固件。

## 离线验证

ThinkBook Ubuntu 22.04 / ROS 2 Humble 上执行：

    ./tools/p4f_offline/validate.sh

该入口在 P6 的 Nav2 与机械臂两个工作空间分别构建，不叠加 overlay；P4e PTY 驱动测试使用单独 install/p4f。P4e 测试只创建 PTY 和字节协议 peer；P6 启动只使用 P3b 模拟反馈和合成视觉，不包含 serial driver。两个运行域均设置 ROS_LOCALHOST_ONLY=1。

P4e PTY 验收覆盖正常 46 字节命令、租约超时、异常时间戳/关节值、重复序号、旧 epoch、执行中取消、较新 epoch 接管、反馈故障、串口 PTY 断连/重连、驱动进程重启和人工恢复。P6 六场景覆盖正常任务、Nav2 失败、目标丢失、抓取取消/失败、arm 通信超时和重复触发。具体运行结果记录在 docs/p4f/ros2-runtime-acceptance.md。
