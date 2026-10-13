# Lunar Robot Console：V0.2b 远程准备与 V0.3 最小方案

审计日期：2026-10-13；基线 `main@67fee8e8d77f8efdc40c64162824acc8af112853`。关联 [Epic #12](https://github.com/Tyr1onX/lunar-cave-robot/issues/12) / [V0.2b #19](https://github.com/Tyr1onX/lunar-cave-robot/issues/19)。本文件只记录已观测事实与后续方案，不代表实体设备验收。

## 1. V0.2b：代码契约与 NUC 事实

- Console 在 `tools/mac-console/` 使用 SwiftUI；只读 Agent 在 `tools/console-agent/`，固定监听 `127.0.0.1:8765`，Mac 通过 SSH 转发访问。图像订阅固定为 `/camera/color/image_raw`、`/camera/depth/image_rect_raw`、`/yolo/annotated_image`；只接受 `sensor_msgs/msg/Image`。V0.2a 的合成 ROS 链路不代表真实设备。
- `tzb_ws/src/pose_estimator/config/camera_switch.yaml` 当前选用 Orbbec Gemini 336L、640×480、30 FPS **配置目标**，不是实机测量。对应 `camera_switch.launch.py` 调用 Orbbec `gemini_330_series.launch.py`，按已存在的驱动参数开启 RGB/深度、注册到 COLOR，重映射深度和点云话题。对照当前 vendored launch，相关参数确实已声明。
- `tzb_ws/src/yolo_node/` 的 ROS 节点订阅 RGB，使用 OpenVINO 模型并输出 `yolo/detections`、`yolo/annotated_image`。标注图存在不等于识别准确率达标。
- NUC `tzb-NUC14RVS-B`：Ubuntu 22.04、ROS 2 Humble；SSH 可用。2026-10-13 检查 USB 列表未见 Orbbec、`/dev/video*` 不存在，ROS 图内没有相机/YOLO 节点或数据 Topic，仅 `/parameter_events`、`/rosout`。无可证实的真实相机输入。
- NUC 原仓库仍为 `6a48dc5`（只读检查、工作树干净），不是最新 `main`；`~/tzb_ws` 未构建，不能直接在它上面 source。**没有更新 NUC 生产仓库或启动传感器。**
- NUC `~/OrbbecSDK` 是 **v1.10.37**；ROS wrapper `orbbec_camera` 为 **v2.9.3**。在隔离试构建误指向 v1 SDK 时产生大量 `undefined reference`，实测 `ob_device_list_get_count` 等符号在 v1 动态库缺失；不是 CMake 路径没找到。
- 已发现并复用 **现有** `~/lunar-field-lab-p9i`：其 `vendor/orbbec-sdk-v2.9.3` 提供 v2.9.3 SDK，`build/tzb/install` 已有 `orbbec_camera`、`yolo_node`、`pose_estimator`。在 `source /opt/ros/humble/setup.bash` 和该独立 overlay 后，三个包可发现，Orbbec `--show-args` 和 camera_switch `--show-args` 成功；`liborbbec_camera.so` 的动态依赖解析无缺失。**这是软件准备，不是相机启动测试；不要同时叠加其它机器人工作空间。**
- YOLO 外部模型实际在 `~/yoloe_for_openvino/yoloe-26l-seg_openvino_model`，包含完整 XML/BIN/metadata/embeddings；不是仓库内仅含少量文本的 `best_openvino_model/blue_block`。系统 OpenVINO 2025.4.1、Ultralytics 8.4.164，模型 metadata `segment/YOLOESegment26/end2end=true`。从最新 main 取出的 `verify_yoloe_openvino.py` **离线推理通过**（合成图 6 个低阈值候选），`offline_yolo_ros_smoke.py` 在随机非零 ROS Domain + localhost-only 下 **YOLO ROS 输入 → DetectionArray + 标注图通过**；观测一次软件链路耗时约 563 ms，不代表真实 30 FPS 或识别精度。
- 离线 ROS 测试结束后检查无 YOLO/相机残余进程。未启动 Orbbec 驱动、雷达、底盘、机械臂或固件控制。

### 现场最少操作（需现场负责人批准后执行传感器启动）

1. **接入并辨认相机**：确认真实铭牌/型号、供电与 USB 3.x 数据线；`lsusb` 核实设备，不能仅用 `/dev/video*` 判定 Orbbec 的 libusb 相机是否存在。
2. **独立 ROS 环境开启单独相机**：新终端仅 source `/opt/ros/humble/setup.bash` + `~/lunar-field-lab-p9i/build/tzb/install/setup.bash`，设置所有测试终端相同的**独立非生产 `ROS_DOMAIN_ID`** 以及 `ROS_LOCALHOST_ONLY=1`；在批准后使用 `ros2 launch pose_estimator camera_switch.launch.py`。先检查 RGB、注册深度、camera_info 的 publisher、type、encoding、分辨率、时戳和稳定帧率；若启动失败留存原始日志，不修改设备参数盲试。
3. **验证 YOLO**：指定 `TZB_OPENVINO_MODEL=$HOME/yoloe_for_openvino/yoloe-26l-seg_openvino_model`；仅在 RGB 确认有效后启动 `ros2 launch yolo_node yolo_node.launch.py`，检查 DetectionArray 和 `/yolo/annotated_image`。离线低阈值合成候选不等于真实目标检测合格。
4. **只读 GUI 验收及记录**：同一测试 ROS Domain 启动已审查的 Console Agent，Mac 通过固定 SSH tunnel 打开测试驾驶舱；分别记录真实 RGB、深度、YOLO 来源、NUC 接收 FPS、Mac 图像更新 FPS、PNG 往返、断流/恢复、截图或 Bag、仓库/模型版本、运行时间及异常。深度灰度为逐帧归一化预览，另验证真实单位、对齐/遮挡、距离误差。

未满足设备与场地安全条件时止步于第 1 步；全程不打开实体运动控制。实机通过后再关闭 #19。

## 2. V0.3：先复用，不造新底层

| 现有来源 | 源码实情 | V0.3 复用决定 |
| --- | --- | --- |
| `Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/cave_simulator.py` | **已有** 2D 差速数学模型，接收 `geometry_msgs/Twist`，产生 `/odom`、`/map`、`/scan` 和 TF；0.25s 命令过期归零；地图/雷达均合成，未证明真实底盘碰撞物理。 | WASD **只发给独立 ROS Domain 内这个模拟器**，用其 `/odom` 作为唯一运动状态与轨迹来源。禁止再实现一套 Mac 侧物理积分器。 |
| `firmware/chassis/User/APP/kinematics.c` | 六轮两侧三轮差速/滑移转向模型；侧滑影响真实转向，不能直接用理想差速模型代替实车反馈。 | 作为解释/校核依据，不在 GUI 复制固件算法，不连接 CAN/STM32。 |
| `tzb_ws/src/tzb_description/urdf/tzb_description.urdf` + `lite_motion_planner/config/tzb_description.srdf` | URDF 11 links/10 joints（6 臂旋转、2 爪旋转、2 fixed）；全部引用 STL 存在；SRDF `arm` 链为 base_link→J6_link；相机固定在 J3。**未包括实际六轮底盘完整运动几何**，手眼/安装标定尚未验收。 | 3D 先显示已有机械臂 URDF/关节状态，底盘只以明确标记的示意外形表示；绝不把它当完整实机数字孪生。 |
| `engineer_0520_description.urdf` | 另一历史机型（6 转动关节、2 爪平移关节），模型来源与现机对应尚不明确。 | 不混用、不自动切换；除非证明匹配真实构型。 |
| `tools/mac-console` + `tools/console-agent` | 只读诊断/三路图像及 SSH 回环接口已通过 V0.2a。 | 保留为 UI 和真实感知只读通道，不为 WASD 增加真实设备写入口。 |
| NUC ROS 2 Humble / RViz2 / rosbag2 | 已安装；ROS 原生 TF、Odometry、Image 及记录工具可用。 | 利用现成 ROS 消息契约和离线回放，不重新发明 TF、IK 或 ROS Bag 格式。 |

### 一条最小架构路径

```text
Mac SwiftUI 测试驾驶舱
  ├─ 真实观测（只读） ── SSH / 已有 Console Agent ── 生产 ROS 图
  ├─ 离线回放（只读） ── ROS Bag / 已保存测试证据（明确 REPLAY）
  └─ 模拟底盘（唯一允许 WASD 的模式）
      └─ 独立模拟会话代理（仅 SSH 回环、固定指令和端口、非生产 ROS Domain）
          └─ 现有 P6 cave_simulator
              ├─ 私有模拟命令 Topic（覆盖默认 /cmd_vel）
              └─ /odom、/map、/scan、TF → Mac 位置/朝向/轨迹
```

**安全不变量：** Simulation/Replay/Live 必须是互斥、显式模式，并在画面持续显示来源；任何模式切换不应隐式接通执行器。模拟输入出口与只读生产 Agent **物理上分离的进程和 ROS Domain**，固定接口，无通用 Topic 选择器/任意 shell；键盘释放、窗口失焦、会话断开立刻停发，模拟器自身 0.25 秒超时归零。**不允许仿真代理写入生产 ROS Domain 或真实 /cmd_vel**。

### V0.3 分三个可验收阶段

1. **V0.3a — WASD + 地图轨迹（优先）**：沿用现有 SwiftUI 驾驶舱，用系统 Canvas 画出 `/map`、`/odom` 位姿/航向和有上限的历史轨迹。唯一的输入对接是独立模拟会话内的受限 Twist；先通过模拟器/断连的自动化测试，再开放 WASD。验证焦点丢失、按键松开、超时停止与 ROS Domain 隔离。无需 3D 引擎。
2. **V0.3b — 真实 URDF 3D 姿态**：优先使用成熟 **Foxglove 3D Viewer** 的 URDF/STL、TF、JointState 可视化，或同样现成的 RViz2，在隔离环境验证模型；作为独立视图/窗口，不假定 Foxglove 可以直接嵌入 SwiftUI。先确认模型坐标/关节/安装版本，再考虑是否有价值做 App 内集成。Foxglove 官方文档：https://docs.foxglove.dev/docs/visualization/panels/3d
3. **V0.3c — 单模块 Test Session**：在现有视觉模块和模拟底盘上建立单一“选择模块 → 前置条件 → 输入/输出 → 结果 → 证据”流程，最小记录时间、mode/source、模块、代码/模型版本、必要指标、通过/失败及证据路径。不预建插件系统、通用远程启动器或重复的历史数据库。

**顺序依据：** 第一阶段成本最低、反馈最直观、能证明“游戏式驾驶舱”的核心闭环；第二阶段复用成熟渲染器，避免重复造 URDF/TF；第三阶段在两项真实使用流程出现后收敛统一测试记录。V0.3 不阻塞 #19 的真实视觉实机验收，二者并行。

## 尚未验证的边界

- NUC 安装的独立 P9-I 构建并非生产部署；未实际接入相机，不声称驱动采集、真实 FPS、注册深度精度或现场 YOLO 准确率。
- P6 模拟器是数学差速模型，不是六轮接触/打滑/碰撞动力学数字孪生；是否存在场地地图与真实底盘尺寸的对应关系尚待确认。
- 已有 URDF 表征机械臂，不代表完整六轮车或已标定的相机安装/TCP；3D 联动先以 SIM/REPLAY 数据为准。
- 真实设备命令通道、物理急停、授权仲裁不属于 V0.3，必须另行安全验收。
