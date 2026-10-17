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
    case topicsAllVisible
    case topicsPartiallyVisible
    case topicsNoneVisible

    var label: String {
        switch self {
        case .normal: return "正常"
        case .abnormal: return "异常"
        case .unknown: return "未知"
        case .notRunning: return "未运行"
        case .processNotFound: return "未发现关联进程"
        case .processObserved: return "已发现进程 · 健康未确认"
        case .topicsAllVisible: return "Topic 名称全部可见"
        case .topicsPartiallyVisible: return "Topic 名称部分可见"
        case .topicsNoneVisible: return "Topic 名称均未见"
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

struct TopologySubsystemDefinition: Identifiable, Equatable {
    let id: String
    let title: String
    let summary: String
    let symbol: String
    let flowID: String
    let statusNodeIDs: [String]
}

struct TopologyOverviewEdgeDefinition: Identifiable, Equatable {
    let id: String
    let sourceSubsystemID: String
    let destinationSubsystemID: String
    let label: String
    let supportingEdgeIDs: [String]
    let evidence: TopologyEvidenceGrade
}

struct TopologyCanvasLayout: Equatable {
    let size: CGSize
    let nodeCenters: [String: CGPoint]
}

enum RobotTopologyLayout {
    static func overview(availableWidth: CGFloat) -> TopologyCanvasLayout {
        let width = max(1_260, availableWidth)
        let height: CGFloat = 810
        return TopologyCanvasLayout(
            size: CGSize(width: width, height: height),
            nodeCenters: [
                "perception": CGPoint(x: width * 0.11, y: height * 0.29),
                "localization": CGPoint(x: width * 0.11, y: height * 0.73),
                "coordination": CGPoint(x: width * 0.36, y: height * 0.51),
                "navigation": CGPoint(x: width * 0.59, y: height * 0.73),
                "arm": CGPoint(x: width * 0.79, y: height * 0.28),
                "chassis": CGPoint(x: width * 0.79, y: height * 0.76),
                "safety": CGPoint(x: width * 0.90, y: height * 0.51),
            ]
        )
    }

    static func detail(_ flow: TopologyFlowDefinition) -> TopologyCanvasLayout {
        let nodeIDs = flow.nodeIDs
        let nodeSet = Set(nodeIDs)
        let edges = flow.edgeIDs.compactMap(RobotTopologyCatalog.edge).filter {
            nodeSet.contains($0.sourceNodeID) && nodeSet.contains($0.destinationNodeID)
        }
        var ranks = Dictionary(uniqueKeysWithValues: nodeIDs.map { ($0, 0) })
        for _ in nodeIDs.indices {
            var changed = false
            for edge in edges {
                let candidate = (ranks[edge.sourceNodeID] ?? 0) + 1
                if candidate > (ranks[edge.destinationNodeID] ?? 0) {
                    ranks[edge.destinationNodeID] = candidate
                    changed = true
                }
            }
            if !changed { break }
        }

        let maxRank = ranks.values.max() ?? 0
        let columns = (0...maxRank).map { rank in nodeIDs.filter { ranks[$0] == rank } }
        let maxRows = columns.map(\.count).max() ?? 1
        let columnSpacing: CGFloat = 320
        let width = max(1_080, CGFloat(maxRank + 1) * columnSpacing + 130)
        let height = max(440, CGFloat(maxRows) * 145 + 150)
        var centers: [String: CGPoint] = [:]
        for (columnIndex, column) in columns.enumerated() {
            for (rowIndex, nodeID) in column.enumerated() {
                centers[nodeID] = CGPoint(
                    x: 120 + CGFloat(columnIndex) * columnSpacing,
                    y: 105 + CGFloat(rowIndex) * 145
                )
            }
        }
        return TopologyCanvasLayout(size: CGSize(width: width, height: height), nodeCenters: centers)
    }
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
        .init(id: "coordination", title: "任务协调", summary: "main 含 P6 协调器源码并通过离线 ROS 2 验证；实体主启动流程集成待确认", nodeIDs: ["mission-coordinator"]),
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
        .init(id: "mission-coordinator", title: "P6 任务协调器", groupID: "coordination", summary: "源码已在 main；离线 ROS 2/Nav2 闭环已验证，实体主启动流程集成状态待确认。", sourcePaths: ["Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py", "Nav2_rea_ws/src/p6_autonomous_mission/launch/p6_autonomous_mission.launch.py", "docs/p6/ros2-runtime-acceptance.md", "docs/p7/ros2-runtime-acceptance.md"], sourceNote: "P6 协调器、启动文件和离线 ROS 2 运行验收记录均在 main。验收启动包含隔离的 Nav2 与 cave simulator；它不证明实体主启动流程、真实传感器或执行器已接入。"),

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
        .init(id: "arm2nav-coordinator", sourceNodeID: "arm2nav", destinationNodeID: "mission-coordinator", label: "建议 → 任务协调", topics: ["/arm2nav"], probe: .topicName, evidence: .openInterface, sourcePaths: ["tzb_ws/src/arm2nav/arm2nav/node.py", "Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py"], sourceNote: "两端源码均存在，但未确认实体主启动流程把 Arm2Nav 输出接入 P6 协调器；当前 Agent 不提供 Topic 两端归属。"),
        .init(id: "pose-coordinator", sourceNodeID: "pose-estimator", destinationNodeID: "mission-coordinator", label: "目标位姿 → 任务", topics: ["/pose_estimator/object_pose"], probe: .topicName, evidence: .openInterface, sourcePaths: ["tzb_ws/src/pose_estimator/pose_estimator/pose_estimation_node.py", "Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py"], sourceNote: "位姿发布端与 P6 任务输入源码均存在；未确认实体主启动流程接线及运行时端点归属。"),

        .init(id: "mid360-pointlio", sourceNodeID: "mid360", destinationNodeID: "point-lio", label: "点云 / IMU", topics: ["/livox/lidar", "/imu/data"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml", "Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp"], sourceNote: "Point-LIO 配置声明 livox/lidar 和 imu/data 输入。"),
        .init(id: "pointlio-terrain", sourceNodeID: "point-lio", destinationNodeID: "terrain", label: "注册点云 / 里程计", topics: ["/cloud_registered", "/odometry"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp", "Nav2_rea_ws/src/navigation/terrain_analysis/src/terrain_analysis_plus.cpp"], sourceNote: "Point-LIO 发布 cloud_registered / odometry；terrain_analysis_plus 订阅同名相对接口。Agent 不提供端点归属。"),
        .init(id: "terrain-scan", sourceNodeID: "terrain", destinationNodeID: "scan-conversion", label: "障碍点云", topics: ["/terrain_map"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"], sourceNote: "选定的单 MID-360 Launch 把 cloud_in 映射到 /terrain_map。"),
        .init(id: "scan-nav2", sourceNodeID: "scan-conversion", destinationNodeID: "nav2", label: "LaserScan", topics: ["/scan"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/navigation/rm_navigation/"], sourceNote: "Launch 将 scan remap 为 /scan；Nav2 runtime publisher/subscriber 关系不在 Agent payload 中。"),
        .init(id: "map-nav2", sourceNodeID: "localization-map", destinationNodeID: "nav2", label: "占据栅格地图", topics: ["/map"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"], sourceNote: "该单雷达 Launch 由 slam_toolbox 发布在线地图并交给 Nav2。"),
        .init(id: "tf-navigation", sourceNodeID: "point-lio", destinationNodeID: "nav2", label: "TF：map → odom_init → base_frame", topics: ["/tf", "/tf_static"], probe: .transformTree, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp"], sourceNote: "Launch / source define TF publishers; Agent does not read TF frame pairs, so live transform status stays unknown."),
        .init(id: "nav2-chassis", sourceNodeID: "nav2", destinationNodeID: "chassis-bridge", label: "平滑速度", topics: ["/cmd_vel"], probe: .topicName, evidence: .offlineExercise, sourcePaths: ["Nav2_rea_ws/src/navigation/rm_navigation/launch/navigation_launch.py", "docs/p5b/chassis-ros2-integration.md"], sourceNote: "Velocity smoother remaps cmd_vel_smoothed to /cmd_vel; prior PTY validation covered the host software path, not robot hardware."),
        .init(id: "chassis-bridge-mcu", sourceNodeID: "chassis-bridge", destinationNodeID: "chassis-controller", label: "USART1 · 115200 · 14 B", topics: [], probe: .serialProtocol, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp", "firmware/chassis/底盘串口通信协议.md"], sourceNote: "主机发送 A5 + 3 个 little-endian float32 + 4A；没有 STM32 ACK。"),

        .init(id: "mission-nav2", sourceNodeID: "mission-coordinator", destinationNodeID: "nav2", label: "NavigateToPose Action", topics: ["/navigate_to_pose"], probe: .actionEndpoint, evidence: .offlineExercise, sourcePaths: ["Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py", "Nav2_rea_ws/src/p6_autonomous_mission/launch/p6_autonomous_mission.launch.py", "docs/p6/ros2-runtime-acceptance.md"], sourceNote: "main 有 ActionClient 源码并在 P6 隔离离线 ROS 2 验收中运行；Agent 不读取 Action endpoint，实体主启动流程集成仍待确认。"),
        .init(id: "mission-arm", sourceNodeID: "mission-coordinator", destinationNodeID: "arm-planner", label: "P6 机械臂任务接口", topics: ["/p6/arm_task/request", "/p6/arm_task/result"], probe: .topicName, evidence: .offlineExercise, sourcePaths: ["Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py", "docs/p6/ros2-runtime-acceptance.md", "docs/p7/ros2-runtime-acceptance.md"], sourceNote: "任务请求 / 结果接口在 main 的 P6 源码和离线 ROS 2 验收中存在；这不证明实体主启动流程已连接到机械臂执行器。"),
        .init(id: "arm-planner-driver", sourceNodeID: "arm-planner", destinationNodeID: "arm-serial", label: "机械臂命令 / 状态", topics: ["/tzb_catch/arm_joint_command", "/serial/arm_joint_state"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/node.cpp", "tzb_ws/src/lite_motion_planner/"], sourceNote: "主机驱动订阅命令并发布 JointState；具体运动规划集成需以当前 launch / 配置为准。"),
        .init(id: "arm-driver-mcu", sourceNodeID: "arm-serial", destinationNodeID: "arm-controller", label: "机械臂串口协议", topics: [], probe: .serialProtocol, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/packet.cpp", "firmware/arm/others/src/app/Host.cpp", "docs/P9E_PR8_INTEGRATION_REVIEW.md"], sourceNote: "主机与 STM32 的串口协议和安全源码可审查；当前设备连接和 STM32 下游停止确认未由 Agent 验证。"),
        .init(id: "arm-driver-safety", sourceNodeID: "arm-serial", destinationNodeID: "arm-safety-status", label: "安全状态文本 Topic", topics: ["/serial/arm_safety/status"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/node.cpp"], sourceNote: "当前 Agent 只观察 Topic 名称，不订阅或读取消息内容。"),
        .init(id: "chassis-status", sourceNodeID: "chassis-bridge", destinationNodeID: "chassis-link-status", label: "主机串口状态", topics: ["/chassis/serial_link_status"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp"], sourceNote: "仅是 host serial open/write 状态，没有 STM32 acknowledgement 语义。"),
        .init(id: "estop-unconnected", sourceNodeID: "physical-estop", destinationNodeID: "agent", label: "实体急停输入", topics: [], probe: .none, evidence: .openInterface, sourcePaths: ["tools/console-agent/console_agent/collectors.py"], sourceNote: "没有接入可验证的物理急停输入。"),
    ]

    static let subsystems: [TopologySubsystemDefinition] = [
        .init(id: "system", title: "只读监测入口", summary: "Agent、ROS Domain、Graph 与运行预期", symbol: "waveform.path.ecg", flowID: "system-flow", statusNodeIDs: ["agent", "ros-domain", "ros-graph", "expected-nodes"]),
        .init(id: "perception", title: "视觉感知", summary: "RGB-D 输入分支至检测与位姿估计", symbol: "camera.viewfinder", flowID: "visual-flow", statusNodeIDs: ["camera", "yolo", "arm2nav", "pose-estimator"]),
        .init(id: "localization", title: "雷达定位", summary: "MID-360 点云 / IMU → Point-LIO → 地图与扫描", symbol: "dot.radiowaves.left.and.right", flowID: "localization-flow", statusNodeIDs: ["lidar-stack", "mid360", "point-lio", "terrain", "scan-conversion", "localization-map"]),
        .init(id: "coordination", title: "任务协调", summary: "P6 源码和离线验证存在，实体主启动集成待确认", symbol: "point.3.connected.trianglepath.dotted", flowID: "coordination-flow", statusNodeIDs: ["mission-coordinator"]),
        .init(id: "navigation", title: "Nav2 导航", summary: "地图、LaserScan、TF 与速度输出", symbol: "location.north.line", flowID: "navigation-flow", statusNodeIDs: ["nav2"]),
        .init(id: "arm", title: "机械臂", summary: "规划、旧版串口驱动与反馈接口", symbol: "move.3d", flowID: "arm-flow", statusNodeIDs: ["arm-stack", "arm-planner", "arm-serial", "arm-controller", "arm-safety-status"]),
        .init(id: "chassis", title: "底盘", summary: "Nav2 速度、chassis_serial_bridge 与主机状态", symbol: "steeringwheel", flowID: "chassis-flow", statusNodeIDs: ["chassis-bridge", "chassis-controller", "chassis-link-status"]),
        .init(id: "safety", title: "安全反馈", summary: "显示主机状态与未接入的实体安全输入", symbol: "checkmark.shield", flowID: "safety-flow", statusNodeIDs: ["arm-safety-status", "chassis-link-status", "physical-estop"]),
    ]

    static let overviewEdges: [TopologyOverviewEdgeDefinition] = [
        .init(id: "perception-coordination", sourceSubsystemID: "perception", destinationSubsystemID: "coordination", label: "目标建议 / 位姿 · 主启动接线待确认", supportingEdgeIDs: ["arm2nav-coordinator", "pose-coordinator"], evidence: .openInterface),
        .init(id: "localization-navigation", sourceSubsystemID: "localization", destinationSubsystemID: "navigation", label: "扫描 / 地图 / TF", supportingEdgeIDs: ["scan-nav2", "map-nav2", "tf-navigation"], evidence: .sourceWiring),
        .init(id: "coordination-navigation", sourceSubsystemID: "coordination", destinationSubsystemID: "navigation", label: "NavigateToPose · P6 离线验证", supportingEdgeIDs: ["mission-nav2"], evidence: .offlineExercise),
        .init(id: "coordination-arm", sourceSubsystemID: "coordination", destinationSubsystemID: "arm", label: "机械臂任务 · P6 离线接口", supportingEdgeIDs: ["mission-arm"], evidence: .offlineExercise),
        .init(id: "navigation-chassis", sourceSubsystemID: "navigation", destinationSubsystemID: "chassis", label: "/cmd_vel · PTY 软件验证", supportingEdgeIDs: ["nav2-chassis"], evidence: .offlineExercise),
        .init(id: "arm-safety", sourceSubsystemID: "arm", destinationSubsystemID: "safety", label: "主机安全状态 Topic", supportingEdgeIDs: ["arm-driver-safety"], evidence: .sourceWiring),
        .init(id: "chassis-safety", sourceSubsystemID: "chassis", destinationSubsystemID: "safety", label: "主机串口状态 Topic", supportingEdgeIDs: ["chassis-status"], evidence: .sourceWiring),
    ]

    static let flows: [TopologyFlowDefinition] = [
        .init(id: "system-flow", title: "只读监测入口", summary: "NUC Agent 快照、ROS Domain、ROS Graph 与显式预期配置", nodeIDs: ["agent", "ros-domain", "ros-graph", "expected-nodes"], edgeIDs: []),
        .init(id: "visual-flow", title: "视觉感知内部链路", summary: "相机分支到检测、Arm2Nav 与位姿估计；外发任务边在整机总览标明为待确认", nodeIDs: ["camera", "yolo", "arm2nav", "pose-estimator"], edgeIDs: ["camera-yolo", "camera-pose", "yolo-arm2nav", "yolo-pose"]),
        .init(id: "localization-flow", title: "雷达定位内部链路", summary: "MID-360 → Point-LIO → 地形过滤、扫描转换与地图", nodeIDs: ["lidar-stack", "mid360", "point-lio", "terrain", "scan-conversion", "localization-map"], edgeIDs: ["mid360-pointlio", "pointlio-terrain", "terrain-scan"]),
        .init(id: "coordination-flow", title: "任务协调链路", summary: "main 已包含 P6 源码并通过隔离离线 ROS 2 验证；实体主启动流程集成待确认", nodeIDs: ["mission-coordinator", "nav2", "arm-planner"], edgeIDs: ["mission-nav2", "mission-arm"]),
        .init(id: "navigation-flow", title: "Nav2 输入与输出", summary: "地图 / 扫描 / TF 汇入 Nav2，速度输出到主机底盘桥", nodeIDs: ["localization-map", "scan-conversion", "point-lio", "nav2", "chassis-bridge"], edgeIDs: ["map-nav2", "scan-nav2", "tf-navigation", "nav2-chassis"]),
        .init(id: "arm-flow", title: "机械臂软件与反馈", summary: "规划和串口主机桥；控制器响应与实体反馈保持未知", nodeIDs: ["mission-coordinator", "arm-stack", "arm-planner", "arm-serial", "arm-controller", "arm-safety-status"], edgeIDs: ["mission-arm", "arm-planner-driver", "arm-driver-mcu", "arm-driver-safety"]),
        .init(id: "chassis-flow", title: "底盘速度与状态链路", summary: "Nav2 → chassis_serial_bridge → STM32；主机状态不等于控制器确认", nodeIDs: ["nav2", "chassis-bridge", "chassis-controller", "chassis-link-status"], edgeIDs: ["nav2-chassis", "chassis-bridge-mcu", "chassis-status"]),
        .init(id: "safety-flow", title: "安全反馈与未接入输入", summary: "主机状态 Topic、STM32 确认边界与未接入的实体急停", nodeIDs: ["arm-serial", "arm-safety-status", "chassis-bridge", "chassis-link-status", "physical-estop"], edgeIDs: ["arm-driver-safety", "chassis-status", "estop-unconnected"]),
    ]

    private static let nodeByID = Dictionary(uniqueKeysWithValues: nodes.map { ($0.id, $0) })
    private static let edgeByID = Dictionary(uniqueKeysWithValues: edges.map { ($0.id, $0) })

    static func node(_ id: String) -> TopologyNodeDefinition? { nodeByID[id] }
    static func edge(_ id: String) -> TopologyEdgeDefinition? { edgeByID[id] }
    static func subsystem(_ id: String) -> TopologySubsystemDefinition? { subsystems.first { $0.id == id } }
    static func flow(_ id: String) -> TopologyFlowDefinition? { flows.first { $0.id == id } }

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
    private(set) var selectedSubsystemID: String?
    private(set) var selectedNodeID: String?

    mutating func selectSubsystem(_ id: String) {
        selectedSubsystemID = id
        selectedNodeID = nil
    }

    mutating func selectNode(_ nodeID: String, in subsystemID: String) {
        selectedSubsystemID = subsystemID
        selectedNodeID = nodeID
    }

    mutating func showOverview() {
        selectedSubsystemID = nil
        selectedNodeID = nil
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
        guard !edge.topics.isEmpty else {
            return TopologyElementStatus(signal: .unknown, note: "连接没有可观察的 Topic 名称；仅保留源码 / 协议证据。", directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        let topicNames = Set(snapshot.ros.topics.map(\.name))
        let observed = edge.topics.filter { topicNames.contains($0) }
        let signal: TopologySignal
        if observed.count == edge.topics.count {
            signal = .topicsAllVisible
        } else if observed.isEmpty {
            signal = .topicsNoneVisible
        } else {
            signal = .topicsPartiallyVisible
        }
        let missing = edge.topics.filter { !topicNames.contains($0) }
        let note = "ROS Graph 可见 \(observed.count)/\(edge.topics.count) 个 Topic 名称" +
            (observed.isEmpty ? "。" : "：\(observed.joined(separator: "、"))。") +
            (missing.isEmpty ? "" : "未见：\(missing.joined(separator: "、"))。") +
            "名称可见不证明发布 / 订阅两端归属或消息实际流动。"
        return TopologyElementStatus(signal: signal, note: note, directEvents: [], possiblyAffectedDownstream: possiblyAffected)
    }

    static func overviewEdgeStatus(
        _ overviewEdge: TopologyOverviewEdgeDefinition,
        context: TopologyRuntimeContext,
        evaluation: TopologyEvaluation
    ) -> TopologyElementStatus {
        let supporting = overviewEdge.supportingEdgeIDs.compactMap(RobotTopologyCatalog.edge)
        let possiblyAffected = supporting.contains { evaluation.edgeStatus($0.id).possiblyAffectedDownstream }
        guard !supporting.isEmpty else {
            return TopologyElementStatus(signal: .unknown, note: "尚无具体连接证据。", directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        if let blocked = context.graphBlockReason {
            return TopologyElementStatus(signal: .unknown, note: blocked, directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        if supporting.contains(where: { $0.evidence == .openInterface || $0.probe == .openInterface }) {
            let notes = supporting.filter { $0.evidence == .openInterface || $0.probe == .openInterface }
                .map(\.sourceNote).joined(separator: " ")
            return TopologyElementStatus(signal: .unknown, note: notes, directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }

        let topicEdges = supporting.filter { $0.probe == .topicName }
        guard !topicEdges.isEmpty, let snapshot = context.snapshot else {
            let notes = supporting.map(\.sourceNote).joined(separator: " ")
            return TopologyElementStatus(signal: .unknown, note: notes, directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        let expectedTopics = Array(Set(topicEdges.flatMap(\.topics))).sorted()
        guard !expectedTopics.isEmpty else {
            return TopologyElementStatus(signal: .unknown, note: "该连接没有可观测的 Topic 名称。", directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        let observedNames = Set(snapshot.ros.topics.map(\.name))
        let visible = expectedTopics.filter { observedNames.contains($0) }
        let signal: TopologySignal
        if visible.count == expectedTopics.count {
            signal = .topicsAllVisible
        } else if visible.isEmpty {
            signal = .topicsNoneVisible
        } else {
            signal = .topicsPartiallyVisible
        }
        let invisible = expectedTopics.filter { !observedNames.contains($0) }
        var note = "ROS Graph 可见 \(visible.count)/\(expectedTopics.count) 个 Topic 名称" +
            (visible.isEmpty ? "。" : "：\(visible.joined(separator: "、"))。") +
            (invisible.isEmpty ? "" : "未见：\(invisible.joined(separator: "、"))。") +
            "名称可见不证明连接两端或消息流正常。"
        if supporting.contains(where: { [.transformTree, .actionEndpoint, .serialProtocol].contains($0.probe) }) {
            note += "该汇总连接还包含 Agent 未采集的 TF / Action / 串口反馈证据，因此整体保持未知。"
            return TopologyElementStatus(signal: .unknown, note: note, directEvents: [], possiblyAffectedDownstream: possiblyAffected)
        }
        return TopologyElementStatus(signal: signal, note: note, directEvents: [], possiblyAffectedDownstream: possiblyAffected)
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
