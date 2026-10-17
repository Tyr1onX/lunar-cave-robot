import Foundation

enum TopologyEvidenceGrade: String, Equatable {
    case offlineExercise = "历史离线软件验证"
    case sourceWiring = "源码 / Launch 连线"
    case openInterface = "接口待核实"
}

enum TopologySignal: String, Equatable {
    case normal
    case abnormal
    case unknown
    case notRunning
    case processNotFound
    case processObserved
    case topicObserved
    case topicNotObserved

    var label: String {
        switch self {
        case .normal: return "正常"
        case .abnormal: return "异常"
        case .unknown: return "未知"
        case .notRunning: return "未运行"
        case .processNotFound: return "未发现关联进程"
        case .processObserved: return "已发现进程 · 健康未确认"
        case .topicObserved: return "Topic 名称已观察"
        case .topicNotObserved: return "当前未观察到"
        }
    }
}

enum TopologyRuntimeProbe: Equatable {
    case topicName
    case transformTree
    case actionEndpoint
    case serialProtocol
    case openInterface
    case none
}

struct TopologyModuleBinding: Equatable {
    let workspaceID: String
    let moduleID: String
}

struct TopologyNodeDefinition: Identifiable, Equatable {
    let id: String
    let title: String
    let groupID: String
    let summary: String
    let workspaceModules: [TopologyModuleBinding]
    let healthEventIDs: [String]
    let resourceHints: [String]
    let sourcePaths: [String]
    let sourceNote: String
    let isExternalHardware: Bool

    init(
        id: String,
        title: String,
        groupID: String,
        summary: String,
        workspaceModules: [TopologyModuleBinding] = [],
        healthEventIDs: [String] = [],
        resourceHints: [String] = [],
        sourcePaths: [String] = [],
        sourceNote: String,
        isExternalHardware: Bool = false
    ) {
        self.id = id
        self.title = title
        self.groupID = groupID
        self.summary = summary
        self.workspaceModules = workspaceModules
        self.healthEventIDs = healthEventIDs
        self.resourceHints = resourceHints
        self.sourcePaths = sourcePaths
        self.sourceNote = sourceNote
        self.isExternalHardware = isExternalHardware
    }
}

struct TopologyEdgeDefinition: Identifiable, Equatable {
    let id: String
    let sourceNodeID: String
    let destinationNodeID: String
    let label: String
    let topics: [String]
    let probe: TopologyRuntimeProbe
    let evidence: TopologyEvidenceGrade
    let sourcePaths: [String]
    let sourceNote: String
}

struct TopologyGroupDefinition: Identifiable, Equatable {
    let id: String
    let title: String
    let summary: String
    let nodeIDs: [String]
}

struct TopologyFlowDefinition: Identifiable, Equatable {
    let id: String
    let title: String
    let summary: String
    let nodeIDs: [String]
    let edgeIDs: [String]
}

enum RobotTopologyCatalog {
    private static func modules(_ pairs: [(String, String)]) -> [TopologyModuleBinding] {
        pairs.map { TopologyModuleBinding(workspaceID: $0.0, moduleID: $0.1) }
    }

    static let groups: [TopologyGroupDefinition] = [
        .init(id: "system", title: "Agent 与 ROS 运行图", summary: "监测入口、Domain 和实时 Graph", nodeIDs: ["agent", "ros-domain", "ros-graph", "expected-nodes"]),
        .init(id: "perception", title: "视觉感知", summary: "彩色 / 深度图像、检测与目标位姿", nodeIDs: ["camera", "yolo", "arm2nav", "pose-estimator"]),
        .init(id: "localization", title: "雷达定位", summary: "MID-360、Point-LIO、地形过滤、扫描转换与地图", nodeIDs: ["lidar-stack", "mid360", "point-lio", "terrain", "scan-conversion", "localization-map"]),
        .init(id: "navigation", title: "Nav2 导航", summary: "地图、扫描、TF 与速度输出", nodeIDs: ["nav2"]),
        .init(id: "coordination", title: "任务协调", summary: "当前 main 尚无已接入的任务协调器", nodeIDs: ["mission-coordinator"]),
        .init(id: "arm", title: "机械臂", summary: "运动规划、旧版 arm 串口驱动与状态反馈", nodeIDs: ["arm-stack", "arm-planner", "arm-serial", "arm-controller", "arm-safety-status"]),
        .init(id: "chassis", title: "底盘", summary: "新版 Nav2 串口桥、STM32 与主机侧链路状态", nodeIDs: ["chassis-bridge", "chassis-controller", "chassis-link-status"]),
        .init(id: "safety", title: "安全反馈", summary: "可见的主机 / ROS 证据与未接入的实体反馈", nodeIDs: ["physical-estop"]),
    ]

    static let nodes: [TopologyNodeDefinition] = [
        .init(id: "agent", title: "NUC 只读 Agent", groupID: "system", summary: "快照、工作区清单和 ROS Graph 只读观测。", healthEventIDs: ["agent-connection"], resourceHints: ["/health", "/v1/snapshot"], sourcePaths: ["tools/console-agent/console_agent/server.py", "tools/mac-console/Sources/LunarRobotConsole/ConsoleModel.swift"], sourceNote: "当前 Agent 快照与 ROS Graph 状态由现有监测逻辑提供。"),
        .init(id: "ros-domain", title: "ROS Domain", groupID: "system", summary: "将 Graph 实际 Domain 与已配置预期分开展示。", healthEventIDs: ["ros-domain"], resourceHints: ["ROS_DOMAIN_ID"], sourcePaths: ["tools/console-agent/console_agent/collectors.py", "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift"], sourceNote: "预期 Domain 未配置时不推断 Domain 错误；无效或不匹配配置会阻断下游实时归属。"),
        .init(id: "ros-graph", title: "ROS Graph", groupID: "system", summary: "当前 Domain 的节点与 Topic 名称；不包含发布者到订阅者关系或 TF 树。", healthEventIDs: ["ros-graph-freshness"], resourceHints: ["节点", "Topic"], sourcePaths: ["tools/console-agent/console_agent/collectors.py", "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift"], sourceNote: "Agent 的 Graph 快照只提供节点和 Topic 名称；名称出现不等于端到端数据流正常。"),
        .init(id: "expected-nodes", title: "预期节点集合", groupID: "system", summary: "只在当前运行模式的节点名单显式配置后比较缺失节点。", healthEventIDs: ["expected-ros-nodes"], resourceHints: ["ROS 节点"], sourcePaths: ["tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift"], sourceNote: "没有配置预期节点集合时，未发现节点不会作为故障告警。"),

        .init(id: "camera", title: "Gemini 336L 相机", groupID: "perception", summary: "彩色、深度、彩色对齐点云与 CameraInfo。", workspaceModules: modules([("new-engineering", "camera"), ("original-arm-vision", "camera")]), healthEventIDs: ["module-camera", "vision-rgb", "vision-depth"], resourceHints: ["/camera/color/image_raw", "/camera/depth/image_rect_raw", "/camera/depth/color/points", "/camera/color/camera_info"], sourcePaths: ["tzb_ws/src/pose_estimator/launch/camera_switch.launch.py", "tzb_ws/src/pose_estimator/config/camera_switch.yaml", "tzb_ws/src/pose_estimator/launch/perception_bringup.launch.py"], sourceNote: "同名 Topic 可能来自不同工作区；Topic 名称本身不确认指定相机来源。"),
        .init(id: "yolo", title: "YOLO 检测", groupID: "perception", summary: "订阅 RGB 图像，输出检测和标注图像。", workspaceModules: modules([("new-engineering", "yolo"), ("original-arm-vision", "yolo")]), healthEventIDs: ["vision-yolo"], resourceHints: ["/camera/color/image_raw", "/yolo/detections", "/yolo/annotated_image"], sourcePaths: ["tzb_ws/src/yolo_node/launch/yolo_node.launch.py", "tzb_ws/src/yolo_node/yolo_node/yolo_detector.py"], sourceNote: "源码含置信度发布门控；没有检测结果不能单独证明故障。"),
        .init(id: "arm2nav", title: "视觉导航建议", groupID: "perception", summary: "读取检测、点云和相机信息，发布 Arm2Nav 状态 / 建议。", resourceHints: ["/yolo/detections", "/camera/depth/color/points", "/camera/color/camera_info", "/camera/color/image_raw", "/arm2nav"], sourcePaths: ["tzb_ws/src/arm2nav/arm2nav/node.py"], sourceNote: "仓库有输入与状态 Topic 源码；没有已确认连接到任务协调或机械臂运动规划的主线。"),
        .init(id: "pose-estimator", title: "目标位姿估计", groupID: "perception", summary: "从点云、CameraInfo 和检测结果估计目标位姿，并发布 Pose / TF。", workspaceModules: modules([("new-engineering", "pose"), ("original-arm-vision", "pose")]), resourceHints: ["/camera/depth/color/points", "/camera/color/camera_info", "/yolo/detections", "/pose_estimator/object_pose", "/tf"], sourcePaths: ["tzb_ws/src/pose_estimator/pose_estimator/pose_estimation_node.py"], sourceNote: "TF 发布位于源码路径；Agent 不采集 TF frame tree，因此运行时 TF 连接保持未知。"),

        .init(id: "mid360", title: "Livox MID-360", groupID: "localization", summary: "单雷达 Launch 配置和 Point-LIO 输入候选。", resourceHints: ["/livox/lidar", "/imu/data", "frame livox_frame"], sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2/launch/mid360_static.launch.py", "Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml"], sourceNote: "Launch 需要设备配置与 broadcast code；Graph / 进程名不能确认设备上电或真实数据质量。Console 不启动设备。", isExternalHardware: true),
        .init(id: "lidar-stack", title: "雷达与定位软件模块", groupID: "localization", summary: "Agent 汇总的 livox / Point-LIO 等工作区模块进程证据。", workspaceModules: modules([("new-engineering", "lidar"), ("original-navigation", "lidar")]), sourcePaths: ["tools/console-agent/console_agent/workspaces.py", "Nav2_rea_ws/src/localization/point_lio/"], sourceNote: "Agent 模块聚合多个包；已发现进程不代表 MID-360、Point-LIO 或整条链路分别正常。"),
        .init(id: "point-lio", title: "Point-LIO 定位", groupID: "localization", summary: "消费 Livox 点云 / IMU，输出里程计、注册点云和 TF。", resourceHints: ["/livox/lidar", "/imu/data", "/odometry", "/cloud_registered", "/tf", "odom_init → base_frame"], sourcePaths: ["Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp", "Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml"], sourceNote: "配置和源码支持这些接口；Agent Graph 不提供具体 TF frame 变换内容。"),
        .init(id: "terrain", title: "地形 / 障碍过滤", groupID: "localization", summary: "Terrain analysis 生成导航侧点云候选。", resourceHints: ["/terrain_map"], sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/navigation/terrain_analysis/launch/terrain_analysis_plus.launch.py"], sourceNote: "属于源码和 Launch 连线；实际运行及数据质量由实时模块接口确认。"),
        .init(id: "scan-conversion", title: "点云转 LaserScan", groupID: "localization", summary: "将 terrain map 点云 remap 为 Nav2 输入 /scan。", resourceHints: ["/terrain_map", "/scan", "target_frame base_frame"], sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"], sourceNote: "该 Launch 明确设置 cloud_in=/terrain_map、scan=/scan；是否启动取决于选用的 Launch。"),
        .init(id: "localization-map", title: "SLAM Toolbox 地图", groupID: "localization", summary: "选定的单雷达 Launch 中由 slam_toolbox 发布 /map。", resourceHints: ["/map", "map → odom_init"], sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"], sourceNote: "该 Launch 定义 slam_toolbox 在线地图和静态 map → odom_init；Agent 不读取地图内容或 TF frame pair。"),

        .init(id: "nav2", title: "Nav2 导航", groupID: "navigation", summary: "消费地图、扫描和 TF；速度经 smoother 输出 /cmd_vel。", workspaceModules: modules([("new-engineering", "navigation"), ("original-navigation", "navigation")]), resourceHints: ["/map", "/scan", "/tf", "/cmd_vel_nav", "/cmd_vel"], sourcePaths: ["Nav2_rea_ws/src/navigation/rm_navigation/launch/navigation_launch.py", "Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"], sourceNote: "Graph Topic 名称不显示 Nav2 Action 状态或 publisher/subscriber 关系。"),
        .init(id: "mission-coordinator", title: "任务协调接口（未接入）", groupID: "coordination", summary: "main 有离线验收文档与脚本，但没有已纳入 main 的 ROS 任务协调节点。", sourcePaths: ["docs/p6/README.md", "tools/p6_offline/"], sourceNote: "P6 文档将协调器描述为离线闭环；不能把它展示为当前部署的新旧工作区运行模块。"),

        .init(id: "arm-stack", title: "机械臂软件模块（规划 / 串口）", groupID: "arm", summary: "Agent 的 arm 模块合并 lite_motion_planner 与 rm_serial_driver 两类包。", workspaceModules: modules([("new-engineering", "arm"), ("original-arm-vision", "arm")]), sourcePaths: ["tools/console-agent/console_agent/workspaces.py", "tzb_ws/src/lite_motion_planner/", "tzb_ws/src/serial/driver/package.xml"], sourceNote: "进程扫描只能报告合并模块是否发现进程，不能据此单独确认规划器或 rm_serial_driver。"),
        .init(id: "arm-planner", title: "机械臂运动规划", groupID: "arm", summary: "lite_motion_planner、MoveIt 与 tzb_catch 源码。", resourceHints: ["/tzb_catch/arm_joint_command", "/serial/arm_joint_state", "/p6/arm_task/request"], sourcePaths: ["tzb_ws/src/lite_motion_planner/", "tzb_ws/src/serial/driver/include/rm_serial_driver/node.hpp", "docs/P8_VALIDATION_REPORT.md"], sourceNote: "离线流程与 ROS 主机串口桥有源码证据；Agent 的合并 arm 模块不能区分本节点运行状态。"),
        .init(id: "arm-serial", title: "旧版 rm_serial_driver", groupID: "arm", summary: "机械臂命令 / 反馈主机桥；不是 Nav2_rea_ws 的新版底盘桥。", resourceHints: ["/tzb_catch/arm_joint_command", "/serial/arm_joint_state", "/serial/arm_safety/status"], sourcePaths: ["tzb_ws/src/serial/driver/package.xml", "tzb_ws/src/serial/driver/src/node.cpp", "tzb_ws/src/serial/driver/src/packet.cpp"], sourceNote: "状态 Topic 由主机驱动发布；不能单独证明 MCU 安全输入、所有关节反馈或电机健康。"),
        .init(id: "arm-controller", title: "机械臂 STM32", groupID: "arm", summary: "串口协议目标控制器；不由 ROS Graph 证明实际连接或动作。", sourcePaths: ["firmware/arm/README.md", "docs/P9E_PR8_INTEGRATION_REVIEW.md"], sourceNote: "固件静态接口与安全状态机有源码依据；当前运行硬件、串口归属和物理响应未知。", isExternalHardware: true),
        .init(id: "arm-safety-status", title: "机械臂安全状态 Topic", groupID: "arm", summary: "主机发布的 /serial/arm_safety/status。", resourceHints: ["/serial/arm_safety/status"], sourcePaths: ["tzb_ws/src/serial/driver/src/node.cpp", "docs/P9E_PR8_INTEGRATION_REVIEW.md"], sourceNote: "Agent 只发现 Topic 名称，不读取该 Topic 的消息值；状态内容和实体急停仍不可由当前 Graph 证明。"),

        .init(id: "chassis-bridge", title: "chassis_serial_bridge", groupID: "chassis", summary: "Nav2_rea_ws 的底盘专用 14 字节串口发送桥。", workspaceModules: modules([("new-engineering", "chassis"), ("original-navigation", "chassis")]), resourceHints: ["/cmd_vel", "/chassis/serial_link_status", "USART1 · 115200 · 14 B"], sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/package.xml", "Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp", "Nav2_rea_ws/src/drivers/serial/driver/launch/serial.launch.py"], sourceNote: "HOST_WRITE_OK 表示本机 TTY 接受写入；该协议没有 STM32 acknowledgement。"),
        .init(id: "chassis-controller", title: "底盘 STM32", groupID: "chassis", summary: "USART1 运动帧接收端；仓库协议不提供 ROS 可读确认。", workspaceModules: modules([("new-engineering", "chassis"), ("original-navigation", "chassis")]), sourcePaths: ["firmware/chassis/底盘串口通信协议.md", "firmware/chassis/README.md"], sourceNote: "14 字节 A5 / 3×float32 little-endian / 4A 协议有静态依据；设备是否运行、是否接收与驱动器反馈未知。", isExternalHardware: true),
        .init(id: "chassis-link-status", title: "底盘主机串口状态", groupID: "chassis", summary: "主机侧 /chassis/serial_link_status，仅表示本地串口打开 / 写入 / 断开。", workspaceModules: modules([("new-engineering", "chassis"), ("original-navigation", "chassis")]), resourceHints: ["/chassis/serial_link_status"], sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp"], sourceNote: "状态语义明确声明没有 STM32 acknowledgement；不可用作底盘硬件健康结论。"),

        .init(id: "physical-estop", title: "实体急停 / 硬件安全输入", groupID: "safety", summary: "当前 Agent 没有接入可核实的实体 E-stop 输入。", healthEventIDs: [], sourcePaths: ["tools/console-agent/console_agent/collectors.py", "docs/P9E_PR8_INTEGRATION_REVIEW.md"], sourceNote: "系统安全快照把实体急停和硬件健康保持为 unknown。", isExternalHardware: true),
    ]

    static let edges: [TopologyEdgeDefinition] = [
        .init(id: "camera-yolo", sourceNodeID: "camera", destinationNodeID: "yolo", label: "RGB 图像", topics: ["/camera/color/image_raw"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/pose_estimator/launch/perception_bringup.launch.py", "tzb_ws/src/yolo_node/launch/yolo_node.launch.py"], sourceNote: "YOLO 输入默认使用 RGB image topic。"),
        .init(id: "camera-pose", sourceNodeID: "camera", destinationNodeID: "pose-estimator", label: "深度点云 / CameraInfo", topics: ["/camera/depth/color/points", "/camera/color/camera_info"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/pose_estimator/config/params.yaml", "tzb_ws/src/pose_estimator/pose_estimator/pose_estimation_node.py"], sourceNote: "pose_estimator 默认订阅点云与相机内参。"),
        .init(id: "yolo-arm2nav", sourceNodeID: "yolo", destinationNodeID: "arm2nav", label: "目标检测", topics: ["/yolo/detections"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/arm2nav/arm2nav/node.py", "tzb_ws/src/yolo_node/launch/yolo_node.launch.py"], sourceNote: "输入接口在源码中匹配；未确认与任务协调器的运行时链路。"),
        .init(id: "yolo-pose", sourceNodeID: "yolo", destinationNodeID: "pose-estimator", label: "检测结果", topics: ["/yolo/detections"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/pose_estimator/pose_estimator/pose_estimation_node.py"], sourceNote: "检测订阅由 pose_estimator 参数定义。"),
        .init(id: "arm2nav-coordinator", sourceNodeID: "arm2nav", destinationNodeID: "mission-coordinator", label: "建议 → 任务协调", topics: ["/arm2nav"], probe: .none, evidence: .openInterface, sourcePaths: ["tzb_ws/src/arm2nav/arm2nav/node.py", "docs/p6/README.md"], sourceNote: "仓库源码展示 Arm2Nav 状态和离线协调器；main 中没有两者连接的 ROS 消费节点。"),
        .init(id: "pose-coordinator", sourceNodeID: "pose-estimator", destinationNodeID: "mission-coordinator", label: "目标位姿 → 任务", topics: ["/pose_estimator/object_pose"], probe: .none, evidence: .openInterface, sourcePaths: ["tzb_ws/src/pose_estimator/pose_estimator/pose_estimation_node.py", "docs/p6/README.md"], sourceNote: "位姿输出存在源码，但 main 中没有已接入的任务协调器消费该输出。"),

        .init(id: "mid360-pointlio", sourceNodeID: "mid360", destinationNodeID: "point-lio", label: "点云 / IMU", topics: ["/livox/lidar", "/imu/data"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml", "Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp"], sourceNote: "Point-LIO 配置声明 livox/lidar 和 imu/data 输入。"),
        .init(id: "pointlio-terrain", sourceNodeID: "point-lio", destinationNodeID: "terrain", label: "注册点云 / 里程计", topics: ["/cloud_registered", "/odometry"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp", "Nav2_rea_ws/src/navigation/terrain_analysis/src/terrain_analysis_plus.cpp"], sourceNote: "Point-LIO 发布 cloud_registered / odometry；terrain_analysis_plus 订阅同名相对接口。Agent 不提供端点归属。"),
        .init(id: "terrain-scan", sourceNodeID: "terrain", destinationNodeID: "scan-conversion", label: "障碍点云", topics: ["/terrain_map"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"], sourceNote: "选定的单 MID-360 Launch 把 cloud_in 映射到 /terrain_map。"),
        .init(id: "scan-nav2", sourceNodeID: "scan-conversion", destinationNodeID: "nav2", label: "LaserScan", topics: ["/scan"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/navigation/rm_navigation/"], sourceNote: "Launch 将 scan remap 为 /scan；Nav2 runtime publisher/subscriber 关系不在 Agent payload 中。"),
        .init(id: "map-nav2", sourceNodeID: "localization-map", destinationNodeID: "nav2", label: "占据栅格地图", topics: ["/map"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"], sourceNote: "该单雷达 Launch 由 slam_toolbox 发布在线地图并交给 Nav2。"),
        .init(id: "tf-navigation", sourceNodeID: "point-lio", destinationNodeID: "nav2", label: "TF：map → odom_init → base_frame", topics: ["/tf", "/tf_static"], probe: .transformTree, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp"], sourceNote: "Launch / source define TF publishers; Agent does not read TF frame pairs, so live transform status stays unknown."),
        .init(id: "nav2-chassis", sourceNodeID: "nav2", destinationNodeID: "chassis-bridge", label: "平滑速度", topics: ["/cmd_vel"], probe: .topicName, evidence: .offlineExercise, sourcePaths: ["Nav2_rea_ws/src/navigation/rm_navigation/launch/navigation_launch.py", "docs/p5b/chassis-ros2-integration.md"], sourceNote: "Velocity smoother remaps cmd_vel_smoothed to /cmd_vel; prior PTY validation covered the host software path, not robot hardware."),
        .init(id: "chassis-bridge-mcu", sourceNodeID: "chassis-bridge", destinationNodeID: "chassis-controller", label: "USART1 · 115200 · 14 B", topics: [], probe: .serialProtocol, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp", "firmware/chassis/底盘串口通信协议.md"], sourceNote: "主机发送 A5 + 3 个 little-endian float32 + 4A；没有 STM32 ACK。"),

        .init(id: "mission-nav2", sourceNodeID: "mission-coordinator", destinationNodeID: "nav2", label: "NavigateToPose Action", topics: ["/navigate_to_pose"], probe: .actionEndpoint, evidence: .openInterface, sourcePaths: ["docs/p6/README.md", "docs/p6/ros2-runtime-acceptance.md"], sourceNote: "P6 离线验收记录过 Action 交互，但主线没有纳入任务协调器；Agent Graph 也不采集 ROS Action endpoint。"),
        .init(id: "mission-arm", sourceNodeID: "mission-coordinator", destinationNodeID: "arm-planner", label: "取样 / 卸载任务", topics: ["/p6/arm_task/request", "/p6/arm_task/result"], probe: .openInterface, evidence: .openInterface, sourcePaths: ["docs/p6/README.md", "docs/p6/ros2-runtime-acceptance.md"], sourceNote: "任务接口存在于离线验收记录，不代表 main 中集成或实体运行。"),
        .init(id: "arm-planner-driver", sourceNodeID: "arm-planner", destinationNodeID: "arm-serial", label: "机械臂命令 / 状态", topics: ["/tzb_catch/arm_joint_command", "/serial/arm_joint_state"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/node.cpp", "tzb_ws/src/lite_motion_planner/"], sourceNote: "主机驱动订阅命令并发布 JointState；具体运动规划集成需以当前 launch / 配置为准。"),
        .init(id: "arm-driver-mcu", sourceNodeID: "arm-serial", destinationNodeID: "arm-controller", label: "机械臂串口协议", topics: [], probe: .serialProtocol, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/packet.cpp", "firmware/arm/others/src/app/Host.cpp", "docs/P9E_PR8_INTEGRATION_REVIEW.md"], sourceNote: "主机与 STM32 的串口协议和安全源码可审查；当前设备连接和 STM32 下游停止确认未由 Agent 验证。"),
        .init(id: "arm-driver-safety", sourceNodeID: "arm-serial", destinationNodeID: "arm-safety-status", label: "安全状态文本 Topic", topics: ["/serial/arm_safety/status"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/node.cpp"], sourceNote: "当前 Agent 只观察 Topic 名称，不订阅或读取消息内容。"),
        .init(id: "chassis-status", sourceNodeID: "chassis-bridge", destinationNodeID: "chassis-link-status", label: "主机串口状态", topics: ["/chassis/serial_link_status"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp"], sourceNote: "仅是 host serial open/write 状态，没有 STM32 acknowledgement 语义。"),
        .init(id: "estop-unconnected", sourceNodeID: "physical-estop", destinationNodeID: "agent", label: "实体急停输入", topics: [], probe: .none, evidence: .openInterface, sourcePaths: ["tools/console-agent/console_agent/collectors.py"], sourceNote: "没有接入可验证的物理急停输入。"),
    ]

    static let flows: [TopologyFlowDefinition] = [
        .init(id: "system-flow", title: "监测入口与 ROS 运行图", summary: "NUC 只读 Agent → Domain → Graph；节点预期只在显式配置时比较", nodeIDs: ["agent", "ros-domain", "ros-graph", "expected-nodes"], edgeIDs: []),
        .init(id: "visual-flow", title: "视觉感知与目标理解", summary: "图像 → 检测 / 位姿；任务入口在 main 中尚未接通", nodeIDs: ["camera", "yolo", "arm2nav", "pose-estimator", "mission-coordinator"], edgeIDs: ["camera-yolo", "camera-pose", "yolo-arm2nav", "yolo-pose", "arm2nav-coordinator", "pose-coordinator"]),
        .init(id: "navigation-flow", title: "雷达定位与底盘导航", summary: "MID-360 → Point-LIO → 地图 / 扫描 → Nav2 → 主机串口桥 → STM32", nodeIDs: ["lidar-stack", "mid360", "point-lio", "terrain", "scan-conversion", "localization-map", "nav2", "chassis-bridge", "chassis-controller"], edgeIDs: ["mid360-pointlio", "pointlio-terrain", "terrain-scan", "scan-nav2", "map-nav2", "tf-navigation", "nav2-chassis", "chassis-bridge-mcu"]),
        .init(id: "arm-flow", title: "任务与机械臂", summary: "任务协调器当前未纳入 main；驱动、协议与设备状态分开展示", nodeIDs: ["mission-coordinator", "arm-stack", "arm-planner", "arm-serial", "arm-controller", "arm-safety-status"], edgeIDs: ["mission-arm", "mission-nav2", "arm-planner-driver", "arm-driver-mcu", "arm-driver-safety"]),
        .init(id: "safety-flow", title: "安全反馈与限制", summary: "主机链路状态不等于 MCU / 电机健康；实体急停输入未接入", nodeIDs: ["arm-safety-status", "chassis-link-status", "physical-estop", "agent"], edgeIDs: ["chassis-status", "estop-unconnected"]),
    ]

    private static let nodeByID = Dictionary(uniqueKeysWithValues: nodes.map { ($0.id, $0) })
    private static let edgeByID = Dictionary(uniqueKeysWithValues: edges.map { ($0.id, $0) })

    static func node(_ id: String) -> TopologyNodeDefinition? { nodeByID[id] }
    static func edge(_ id: String) -> TopologyEdgeDefinition? { edgeByID[id] }

    static func downstreamNodeIDs(from startID: String) -> Set<String> {
        var visited = Set<String>()
        var pending = [startID]
        while let current = pending.popLast() {
            for edge in edges where edge.sourceNodeID == current && edge.evidence != .openInterface {
                if visited.insert(edge.destinationNodeID).inserted {
                    pending.append(edge.destinationNodeID)
                }
            }
        }
        visited.remove(startID)
        return visited
    }
}

struct TopologyInteractionState: Equatable {
    private(set) var expandedFlowID: String?
    private(set) var selectedNodeID: String?

    mutating func toggleFlow(_ id: String) {
        expandedFlowID = expandedFlowID == id ? nil : id
        selectedNodeID = nil
    }

    mutating func selectNode(_ nodeID: String, in flowID: String) {
        expandedFlowID = flowID
        selectedNodeID = nodeID
    }

    mutating func clearSelection() {
        selectedNodeID = nil
    }
}

struct TopologyRuntimeContext {
    let agentOnline: Bool
    let snapshot: ConsoleSnapshot?
    let displayedWorkspaceInventory: WorkspaceInventoryReading?
    let workspaceFreshness: WorkspaceSnapshotFreshness
    let vision: VisionReading?
    let healthEvents: [HealthEvent]

    var liveInventory: WorkspaceInventoryReading? {
        guard agentOnline, workspaceFreshness == .live else { return nil }
        return snapshot?.workspaces
    }

    var graphBlockReason: String? {
        guard agentOnline, snapshot != nil else { return "Agent 快照当前不可用；ROS 运行状态未知。" }
        guard let snapshot else { return "Agent 快照当前不可用；ROS 运行状态未知。" }
        if let error = snapshot.ros.expectedDomainError { return "预期 ROS Domain 配置无效：\(error)" }
        if let expected = snapshot.ros.expectedDomainId, snapshot.ros.domainId != expected {
            return "ROS Domain 不匹配；下游运行状态保持未知。"
        }
        guard snapshot.ros.domainId != nil else { return "当前快照没有实际 ROS Domain 证据；下游运行状态保持未知。" }
        guard snapshot.ros.status == "ok" else { return snapshot.ros.reason ?? "ROS Graph 状态未知。" }
        guard let age = snapshot.ros.graphAgeMs, age >= 0, age <= HealthEvaluator.rosGraphTimeoutMs else {
            return "ROS Graph 数据过期或缺少时间证据；下游运行状态保持未知。"
        }
        return nil
    }
}

struct TopologyElementStatus: Equatable {
    let signal: TopologySignal
    let note: String
    let directEvents: [HealthEvent]
    let possiblyAffectedDownstream: Bool
}

struct TopologyEvaluation {
    let nodeStatuses: [String: TopologyElementStatus]
    let edgeStatuses: [String: TopologyElementStatus]
    let possiblyAffectedNodeIDs: Set<String>

    func nodeStatus(_ id: String) -> TopologyElementStatus {
        nodeStatuses[id] ?? TopologyElementStatus(signal: .unknown, note: "未找到拓扑节点定义。", directEvents: [], possiblyAffectedDownstream: false)
    }

    func edgeStatus(_ id: String) -> TopologyElementStatus {
        edgeStatuses[id] ?? TopologyElementStatus(signal: .unknown, note: "未找到连接定义。", directEvents: [], possiblyAffectedDownstream: false)
    }
}

enum TopologyOverallStatus {
    static func signal(for evaluation: TopologyEvaluation) -> TopologySignal {
        let signals = evaluation.nodeStatuses.values.map(\.signal) + evaluation.edgeStatuses.values.map(\.signal)
        if signals.contains(.abnormal) { return .abnormal }
        return signals.allSatisfy { $0 == .normal } ? .normal : .unknown
    }
}

enum TopologyStatusResolver {
    static func evaluate(_ context: TopologyRuntimeContext) -> TopologyEvaluation {
        let directAbnormalNodes = RobotTopologyCatalog.nodes.compactMap { node -> String? in
            guard !["agent", "ros-domain", "ros-graph", "expected-nodes"].contains(node.id),
                  context.graphBlockReason == nil else { return nil }
            let events = directEvents(for: node, context: context)
            return events.contains(where: { $0.state == .abnormal }) ? node.id : nil
        }
        let downstream = directAbnormalNodes.reduce(into: Set<String>()) { result, nodeID in
            result.formUnion(RobotTopologyCatalog.downstreamNodeIDs(from: nodeID))
        }
        let directAbnormalNodeSet = Set(directAbnormalNodes)

        var nodes: [String: TopologyElementStatus] = [:]
        for node in RobotTopologyCatalog.nodes {
            nodes[node.id] = nodeStatus(node, context: context, downstream: downstream)
        }

        var edges: [String: TopologyElementStatus] = [:]
        for edge in RobotTopologyCatalog.edges {
            let possiblyAffected = directAbnormalNodeSet.contains(edge.sourceNodeID) ||
                directAbnormalNodeSet.contains(edge.destinationNodeID) ||
                downstream.contains(edge.sourceNodeID) || downstream.contains(edge.destinationNodeID)
            edges[edge.id] = edgeStatus(edge, context: context, possiblyAffected: possiblyAffected)
        }
        return TopologyEvaluation(nodeStatuses: nodes, edgeStatuses: edges, possiblyAffectedNodeIDs: downstream)
    }

    private static func directEvents(for node: TopologyNodeDefinition, context: TopologyRuntimeContext) -> [HealthEvent] {
        context.healthEvents.filter { node.healthEventIDs.contains($0.id) }
    }

    private static func nodeStatus(
        _ node: TopologyNodeDefinition,
        context: TopologyRuntimeContext,
        downstream: Set<String>
    ) -> TopologyElementStatus {
        let events = directEvents(for: node, context: context)

        if ["ros-graph", "ros-domain", "expected-nodes"].contains(node.id) {
            if let event = events.first {
                return TopologyElementStatus(signal: signal(for: event.state), note: event.observation, directEvents: events, possiblyAffectedDownstream: false)
            }
            return TopologyElementStatus(signal: .unknown, note: "尚无当前 ROS 运行图诊断事件。", directEvents: [], possiblyAffectedDownstream: false)
        }

        if node.id == "agent" {
            if let event = events.first {
                return TopologyElementStatus(signal: signal(for: event.state), note: event.observation, directEvents: events, possiblyAffectedDownstream: false)
            }
            return TopologyElementStatus(signal: context.agentOnline ? .normal : .unknown, note: context.agentOnline ? "只读 Agent 当前在线。" : "Agent 连接状态未知。", directEvents: events, possiblyAffectedDownstream: false)
        }

        if !["ros-domain", "ros-graph", "expected-nodes"].contains(node.id),
           let blocked = context.graphBlockReason {
            return TopologyElementStatus(signal: .unknown, note: blocked, directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
        }

        if node.id == "camera" {
            if let streamEvent = events.first(where: { ["vision-rgb", "vision-depth"].contains($0.id) && $0.state == .abnormal }) {
                return TopologyElementStatus(signal: .abnormal, note: "相机图像数据流异常；工作区相机来源仍需单独确认。\n\(streamEvent.observation)", directEvents: events, possiblyAffectedDownstream: false)
            }
            guard let moduleEvent = events.first(where: { $0.id == "module-camera" }) else {
                return TopologyElementStatus(signal: .unknown, note: "图像数据可能存在，但相机模块来源尚未确认。", directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
            }
            return TopologyElementStatus(signal: signal(for: moduleEvent.state), note: moduleEvent.observation, directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
        }

        if events.contains(where: { $0.state == .abnormal }) {
            return TopologyElementStatus(signal: .abnormal, note: "直接异常来自现有健康规则；下游仅标记可能受影响。", directEvents: events, possiblyAffectedDownstream: false)
        }

        if node.isExternalHardware {
            return TopologyElementStatus(signal: .unknown, note: "Agent / ROS Graph 不提供该实体硬件的状态或确认反馈。", directEvents: events, possiblyAffectedDownstream: false)
        }

        if let event = events.first(where: { $0.state == .unknown }) {
            return TopologyElementStatus(signal: .unknown, note: event.observation, directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
        }
        if let event = events.first(where: { $0.state == .normal }) {
            return TopologyElementStatus(signal: .normal, note: event.observation, directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
        }
        if let event = events.first(where: { $0.state == .notRunning }) {
            return TopologyElementStatus(signal: .processNotFound, note: event.observation, directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
        }

        guard let inventory = context.liveInventory,
              inventory.status == "ok",
              inventory.processScan?.status == "ok" else {
            return TopologyElementStatus(signal: .unknown, note: "没有当前完整的工作区与进程扫描证据。", directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
        }

        let modules = node.workspaceModules.flatMap { binding in
            inventory.entries.compactMap { workspace -> (WorkspaceReading, WorkspaceModuleReading)? in
                guard workspace.id == binding.workspaceID,
                      let module = workspace.modules?.first(where: { $0.id == binding.moduleID }) else { return nil }
                return (workspace, module)
            }
        }
        guard !modules.isEmpty else {
            let note = node.groupID == "coordination" ? node.sourceNote : "当前工作区快照没有该模块的可归属证据。"
            return TopologyElementStatus(signal: .unknown, note: note, directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
        }
        if modules.contains(where: { $0.1.runtimeStatus == "running" }) {
            return TopologyElementStatus(signal: .processObserved, note: "Agent UID 扫描发现关联模块进程；此处不推断接口或硬件健康。", directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
        }
        let completeAbsence = modules.allSatisfy {
            $0.0.sourceStatus == "present" && $0.1.sourceStatus == "present" && $0.1.runtimeStatus == "not_running"
        }
        if completeAbsence {
            return TopologyElementStatus(signal: .processNotFound, note: "完整 Agent UID 进程扫描未发现该模块关联进程；运行预期未配置时此状态不表示故障。", directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
        }
        return TopologyElementStatus(signal: .unknown, note: "模块来源或进程证据不足。", directEvents: events, possiblyAffectedDownstream: downstream.contains(node.id))
    }

    private static func edgeStatus(_ edge: TopologyEdgeDefinition, context: TopologyRuntimeContext, possiblyAffected: Bool) -> TopologyElementStatus {
        guard let graphBlock = context.graphBlockReason else {
            return liveEdgeStatus(edge, context: context, possiblyAffected: possiblyAffected)
        }
        return TopologyElementStatus(signal: .unknown, note: graphBlock, directEvents: [], possiblyAffectedDownstream: possiblyAffected)
    }

    private static func liveEdgeStatus(_ edge: TopologyEdgeDefinition, context: TopologyRuntimeContext, possiblyAffected: Bool) -> TopologyElementStatus {
        guard let snapshot = context.snapshot else {
            return TopologyElementStatus(signal: .unknown, note: "没有当前 ROS Graph 快照。", directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        if edge.probe == .openInterface || edge.probe == .none || edge.evidence == .openInterface {
            return TopologyElementStatus(signal: .unknown, note: edge.sourceNote, directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        if edge.probe == .transformTree {
            let topicNames = Set(snapshot.ros.topics.map(\.name))
            let topicSeen = edge.topics.filter { topicNames.contains($0) }
            let suffix = topicSeen.isEmpty ? "Graph 未发现 TF Topic 名称。" : "Graph 发现 \(topicSeen.joined(separator: "、")) 名称，但不提供 frame pair / TF 树。"
            return TopologyElementStatus(signal: .unknown, note: suffix, directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        if edge.probe == .actionEndpoint || edge.probe == .serialProtocol {
            return TopologyElementStatus(signal: .unknown, note: "当前 Agent 不采集 ROS Action endpoint 或底层串口确认。\(edge.sourceNote)", directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        let topicNames = Set(snapshot.ros.topics.map(\.name))
        let observed = edge.topics.filter { topicNames.contains($0) }
        if !observed.isEmpty {
            return TopologyElementStatus(signal: .topicObserved, note: "当前 ROS Graph 发现 Topic 名称 \(observed.joined(separator: "、"))；Graph 不提供该 Topic 的发布 / 订阅两端归属或消息流状态。", directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        return TopologyElementStatus(signal: .topicNotObserved, note: "当前新鲜 ROS Graph 未列出 \(edge.topics.joined(separator: "、"))；除非现有健康规则确认运行预期，否则不作为故障。", directEvents: [], possiblyAffectedDownstream: possiblyAffected)
    }

    private static func signal(for state: HealthState) -> TopologySignal {
        switch state {
        case .normal: return .normal
        case .abnormal: return .abnormal
        case .notRunning: return .notRunning
        case .unknown: return .unknown
        }
    }
}
