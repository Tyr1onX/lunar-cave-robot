# P3 模块健康监测审计

审计时间：2026-10-17 09:12 CST（源码基线 `3d0edece1482d0875f0d27c45f823b0b6660dff1`）

本记录分别描述源码配置、Agent 当前进程/ROS 观测和可据此作出的有限结论。源码中的 Topic、launch 参数和预期频率不表示模块当前应启动，也不证明设备已连接。

## Gemini 336L 相机

`tzb_ws/src/pose_estimator/config/camera_switch.yaml` 将 `orbbec_gemini336l` 设为默认驱动，配置 640×480、30 FPS。对应 launch 包含 `orbbec_camera/gemini_330_series.launch.py`，启用 RGB、深度、深度配准和彩色点云，并把默认输出归一为 `/camera/color/image_raw`、`/camera/depth/image_rect_raw` 与 `/camera/depth/color/points`。这些值是该 launch 配置的请求值；未从设备运行时读取最终协商分辨率或帧率。

当前 Agent 已经只读订阅 RGB、深度和 YOLO 标注图像 Topic。它记录发布者数量、类型、接收帧率、NUC 单调时钟帧龄、ROS Header 时间、接收时间及接收间隔。相机 RGB/深度帧超过 1.5 秒标为过期；该阈值用于新鲜度，不代表 30 FPS 性能门槛。Header 时间与 Agent 主机时间的差值依赖时钟同步；源码中的 `time_domain=global` 与 `enable_sync_host_time=true` 参数没有在当前 launch 调用中显式传递，因此不据此宣称时钟已同步。

本次 NUC 只读快照显示 Agent 用户服务运行、`ROS_DOMAIN_ID=187`、`ROS_LOCALHOST_ONLY=1`。没有配置预期 Domain、预期节点或预期模块。ROS graph 成功且年龄为 184 ms，但节点和 Topic 列表为空；RGB、深度及 YOLO 流均显示 0 个发布者，帧率、帧龄和 Header 时间为空。`/dev/video*` 没有设备节点，`lsusb` 未观察到 Orbbec/Gemini 或 Livox 标识。工作区源码存在、进程扫描完整，但新版工程及两个原始工作区均未发现关联进程。以上只表明本次观测没有相机数据和 Agent UID 范围内的关联进程，不判定设备故障或预期未满足。

## 模块能力矩阵

| 模块 | 源码与可见接口 | 已有只读证据 | 当前可安全判定范围 | 仍缺少的证据 |
| --- | --- | --- | --- | --- |
| Orbbec Gemini 336L | `camera_switch.yaml`、`camera_switch.launch.py`；RGB/深度 `sensor_msgs/msg/Image`，CameraInfo 与对齐 `sensor_msgs/msg/PointCloud2` | Agent 对 RGB/深度测帧新鲜度、接收 FPS、帧龄和时间戳；Graph 可见 Topic 名称/类型 | 在模块预期明确、进程扫描完整、Graph 新鲜且工作区与 Agent 同 Domain 时，可判断 RGB/深度流是否有发布及是否超时 | 没有相机 USB/驱动状态订阅；当前没有实际帧。配置 30 FPS 不能作为未经验收的健康阈值；镜头、深度精度、标定与传感器实体健康无法据 ROS 发布证明 |
| YOLO | `tzb_ws/src/yolo_node/launch/yolo_node.launch.py`；输入 `/camera/color/image_raw`，候选输出 `yolo/detections`、`yolo/annotated_image` | Agent 对标注图像流观测；源码显示 `max_confidence` 必须超过 `publish_conf_threshold`，并满足 `publish_hold_seconds` 后才发布 | 输入流和节点进程可分层显示；有输出仅证明数据发布 | 置信度门控可正常抑制输出；空输出不是故障证据。识别准确率、模型质量、目标覆盖率没有运行时真值。当前仅 Agent 观测标注图像，未读取检测阵列 |
| Livox MID-360 | `msg_MID360_launch.py` 默认请求 10 Hz、`xfer_format=1` 自定义点云；候选 `/livox/lidar`、`/livox/imu` | Agent Graph 仅报告同 Domain 中名称和类型；现有固定 LIVE 巡检定义包含 IMU 低带宽频率检查，但本次没有执行 | 可确认节点/Topic 被发现；未来可评估经审核的 IMU 状态流 | 不订阅高带宽点云；当前 Agent 未测量 MID-360 数据时序或传感器状态。自定义点云频率与实际网络/设备输出待实测 |
| Nav2 | 原始导航 launch 声明 `controller_server`、`smoother_server`、`planner_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`、`velocity_smoother` 生命周期节点；定位/地图 launch 另含 AMCL/map server | Agent Graph 与 `/proc` 工作区进程归属 | 可确认节点进程和同 Domain Graph 名称可见性 | Graph 存在不等于 lifecycle `active`。尚未查询 lifecycle 状态，也没有已确认的当前模式节点名单；导航质量、定位质量与路径安全不可由此确认 |
| 机械臂 | `rm_serial_driver` 发布 `/serial/arm_joint_state`（JointState）及 `/serial/arm_safety/status`（String）；规划器另可发布可视化 `/joint_states` | 现有固定 LIVE 巡检定义检查安全状态/关节反馈 Topic | 具备状态 Topic 可见性时可报告反馈是否可读；反馈消息只表明软件数据链路 | `/joint_states` 可能是可视化或合成数据。未验证反馈源身份、时效语义、关节位置合理性与驱动器/机械臂实体状态；不把命令 Topic 当作状态 |
| 底盘 | `chassis_serial_bridge` 发布 `/chassis/serial_link_status`（String）；订阅 `/cmd_vel` | 现有固定 LIVE 巡检定义检查串口桥状态 Topic | `DISCONNECTED` 或 `HOST_WRITE_OK` 可描述主机侧串口路径 | 源码明确指出 `HOST_WRITE_OK` 仅表示操作系统接受完整串口帧，无 STM32 确认语义；不代表控制器、电机、车轮或急停健康。Agent 不订阅速度命令 Topic |

## 运行预期与判定边界

Agent 的模块 `run_expectation` 默认为 `unconfigured`。源码包存在、process scanner 未发现进程或 Graph 中没有候选 Topic 均不自动构成故障。新增的可选 `LUNAR_CONSOLE_EXPECTED_MODULES` 接受 `workspace_id:module_id` 项；只有明确列出且通过 ID 校验的模块才被标为 `confirmed`。任意无效项会使整份配置失效，所有模块改为 `unknown`，不部分应用清单。没有设置该变量的 Agent 输出保持兼容：字段仍为 `unconfigured`，旧版 Console 会忽略新增字段。

模块级故障规则仍需同时满足：当前 Agent 可读、ROS graph 新鲜、目标工作区进程扫描完整、运行预期已确认，以及相关进程与 Agent graph Domain 关系可验证。Domain 不匹配、Graph 过期、来源/进程证据不完整或相互矛盾时，下游状态保持未知。`ROS_DOMAIN_ID`、图像/点云发布、串口主机写入成功和硬件声明运行模式均不能替代实体硬件状态反馈。

本轮没有修改 `tzb_ws` 或 `Nav2_rea_ws`，未执行 ROS launch、传感器驱动、LIVE 模块巡检、运动控制或执行器操作。
