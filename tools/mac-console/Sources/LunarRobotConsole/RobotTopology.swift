import Foundation

enum TopologyEvidenceGrade: String, Equatable {
    case offlineExercise = "历史离线软件验证"
    case sourceWiring = "源码 / Launch 连线"
    case conditionalSourceWiring = "可选模式源码连线"
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
    case dataObserved
    case dataPartiallyObserved
    case dataStale
    case dataNotObserved

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
        case .dataObserved: return "Agent 已收到数据"
        case .dataPartiallyObserved: return "部分数据已收到"
        case .dataStale: return "数据流暂未更新"
        case .dataNotObserved: return "尚未收到数据"
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
    var isFeedback = false
}

struct TopologySubsystemDefinition: Identifiable, Equatable {
    let id: String
    let title: String
    let summary: String
    let symbol: String
    let statusNodeIDs: [String]
}

struct TopologyProjectedNode: Identifiable, Equatable {
    let id: String
    let subsystemID: String
    let moduleNodeID: String?
    let memberNodeIDs: [String]

    var isSubsystem: Bool { moduleNodeID == nil }
}

struct TopologyProjectedEdge: Identifiable, Equatable {
    let id: String
    let sourceID: String
    let destinationID: String
    let evidence: TopologyEvidenceGrade
    let isFeedback: Bool
}

struct TopologyGraphProjection: Equatable {
    let nodes: [TopologyProjectedNode]
    let edges: [TopologyProjectedEdge]
}

enum RobotTopologyCatalog {
    private static func modules(_ pairs: [(String, String)]) -> [TopologyModuleBinding] {
        pairs.map { TopologyModuleBinding(workspaceID: $0.0, moduleID: $0.1) }
    }

    static let nodes: [TopologyNodeDefinition] = [
        .init(id: "agent", title: "NUC 只读 Agent", groupID: "system", summary: "快照、工作区清单和 ROS Graph 只读观测。", healthEventIDs: ["agent-connection"], resourceHints: ["/health", "/v1/snapshot"], sourcePaths: ["tools/console-agent/console_agent/server.py", "tools/mac-console/Sources/LunarRobotConsole/ConsoleModel.swift"], sourceNote: "当前 Agent 快照与 ROS Graph 状态由现有监测逻辑提供。"),
        .init(id: "ros-domain", title: "ROS Domain", groupID: "system", summary: "将 Graph 实际 Domain 与已配置预期分开展示。", healthEventIDs: ["ros-domain"], resourceHints: ["ROS_DOMAIN_ID"], sourcePaths: ["tools/console-agent/console_agent/collectors.py", "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift"], sourceNote: "预期 Domain 未配置时不推断 Domain 错误；无效或不匹配配置会阻断下游实时归属。"),
        .init(id: "ros-graph", title: "ROS Graph", groupID: "system", summary: "当前 Domain 的节点与 Topic 名称；不包含发布者到订阅者关系或 TF 树。", healthEventIDs: ["ros-graph-freshness"], resourceHints: ["节点", "Topic"], sourcePaths: ["tools/console-agent/console_agent/collectors.py", "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift"], sourceNote: "Agent 的 Graph 快照只提供节点和 Topic 名称；名称出现不等于端到端数据流正常。"),
        .init(id: "expected-nodes", title: "预期节点集合", groupID: "system", summary: "只在当前运行模式的节点名单显式配置后比较缺失节点。", healthEventIDs: ["expected-ros-nodes"], resourceHints: ["ROS 节点"], sourcePaths: ["tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift"], sourceNote: "没有配置预期节点集合时，未发现节点不会作为故障告警。"),

        .init(id: "camera", title: "Gemini 336L 相机", groupID: "perception", summary: "彩色、深度、彩色对齐点云与 CameraInfo。", workspaceModules: modules([("new-engineering", "camera"), ("original-arm-vision", "camera")]), healthEventIDs: ["module-camera", "vision-rgb", "vision-depth"], resourceHints: ["/camera/color/image_raw", "/camera/depth/image_rect_raw", "/camera/depth/color/points", "/camera/color/camera_info"], sourcePaths: ["tzb_ws/src/pose_estimator/launch/camera_switch.launch.py", "tzb_ws/src/pose_estimator/config/camera_switch.yaml", "tzb_ws/src/pose_estimator/launch/perception_bringup.launch.py"], sourceNote: "同名 Topic 可能来自不同工作区；Topic 名称本身不确认指定相机来源。"),
        .init(id: "yolo", title: "YOLO 检测", groupID: "perception", summary: "订阅 RGB 图像，输出检测和标注图像。", workspaceModules: modules([("new-engineering", "yolo"), ("original-arm-vision", "yolo")]), healthEventIDs: ["vision-yolo"], resourceHints: ["/camera/color/image_raw", "/yolo/detections", "/yolo/annotated_image"], sourcePaths: ["tzb_ws/src/yolo_node/launch/yolo_node.launch.py", "tzb_ws/src/yolo_node/yolo_node/yolo_detector.py"], sourceNote: "源码含置信度发布门控；没有检测结果不能单独证明故障。"),
        .init(id: "arm2nav", title: "视觉导航建议", groupID: "perception", summary: "读取检测、点云和相机信息，发布 Arm2Nav 状态 / 建议。", resourceHints: ["/yolo/detections", "/camera/depth/color/points", "/camera/color/camera_info", "/camera/color/image_raw", "/arm2nav"], sourcePaths: ["tzb_ws/src/arm2nav/arm2nav/node.py"], sourceNote: "仓库有输入与状态 Topic 源码；没有已确认连接到任务协调或机械臂运动规划的主线。"),
        .init(id: "pose-estimator", title: "目标位姿估计", groupID: "perception", summary: "从点云、CameraInfo 和检测结果估计目标位姿，并发布 Pose / TF。", workspaceModules: modules([("new-engineering", "pose"), ("original-arm-vision", "pose")]), resourceHints: ["/camera/depth/color/points", "/camera/color/camera_info", "/yolo/detections", "/pose_estimator/object_pose", "/tf"], sourcePaths: ["tzb_ws/src/pose_estimator/pose_estimator/pose_estimation_node.py"], sourceNote: "TF 发布位于源码路径；Agent 不采集 TF frame tree，因此运行时 TF 连接保持未知。"),

        .init(id: "mid360", title: "Livox MID-360", groupID: "localization", summary: "单雷达 Launch 的点云与原始 IMU 来源。", resourceHints: ["/livox/lidar", "/livox/imu", "frame livox_frame"], sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2/launch/msg_MID360_launch.py", "Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml"], sourceNote: "Launch 需要设备配置与 broadcast code；Graph / 进程名不能确认设备上电或真实数据质量。Console 不启动设备。", isExternalHardware: true),
        .init(id: "lidar-stack", title: "雷达与定位软件模块", groupID: "localization", summary: "Agent 汇总的 livox / Point-LIO 等工作区模块进程证据。", workspaceModules: modules([("new-engineering", "lidar"), ("original-navigation", "lidar")]), sourcePaths: ["tools/console-agent/console_agent/workspaces.py", "Nav2_rea_ws/src/localization/point_lio/"], sourceNote: "Agent 模块聚合多个包；已发现进程不代表 MID-360、Point-LIO 或整条链路分别正常。"),
        .init(id: "imu-transform", title: "IMU 坐标变换", groupID: "localization", summary: "将 MID-360 原始 IMU 从 /livox/imu 变换为 /livox/imu_aft_tf。", resourceHints: ["/livox/imu", "/livox/imu_aft_tf"], sourcePaths: ["Nav2_rea_ws/src/preprocess/imu_tools/launch/imu_transform.launch.py", "Nav2_rea_ws/src/preprocess/imu_tools/config/imu_transform.yaml"], sourceNote: "单雷达 Launch 启动 imu_transform；Topic 名称只表示 Graph 可见，不证明消息已处理或送达。"),
        .init(id: "imu-filter", title: "IMU 互补滤波", groupID: "localization", summary: "接收变换后的 IMU，输出 Point-LIO 配置使用的 /imu/data。", resourceHints: ["/livox/imu_aft_tf", "/imu/data"], sourcePaths: ["Nav2_rea_ws/src/preprocess/imu_tools/launch/imu_complementary_filter.launch.py", "Nav2_rea_ws/src/preprocess/imu_tools/imu_complementary_filter/complementary_filter_ros.cpp"], sourceNote: "Launch 将输入重映射为 /livox/imu_aft_tf；输出 /imu/data 与 Point-LIO 配置匹配。"),
        .init(id: "point-lio", title: "Point-LIO 定位", groupID: "localization", summary: "消费 Livox 点云 / IMU，输出里程计、注册点云和 TF。", resourceHints: ["/livox/lidar", "/imu/data", "/odometry", "/cloud_registered", "/tf", "odom_init → base_frame"], sourcePaths: ["Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp", "Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml"], sourceNote: "配置和源码支持这些接口；Agent Graph 不提供具体 TF frame 变换内容。"),
        .init(id: "terrain", title: "地形 / 障碍过滤", groupID: "localization", summary: "Terrain analysis 生成导航侧点云候选。", resourceHints: ["/terrain_map"], sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/navigation/terrain_analysis/launch/terrain_analysis_plus.launch.py"], sourceNote: "属于源码和 Launch 连线；实际运行及数据质量由实时模块接口确认。"),
        .init(id: "scan-conversion", title: "点云转 LaserScan", groupID: "localization", summary: "将 /terrain_map 点云转换为 SLAM Toolbox 使用的虚拟 /scan。", resourceHints: ["/terrain_map", "/scan", "target_frame base_frame"], sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"], sourceNote: "所选单雷达 Launch 将 cloud_in=/terrain_map、scan=/scan；Nav2 在该 Launch 中通过 /terrain_map 接收障碍点云。"),
        .init(id: "slam-toolbox", title: "SLAM Toolbox", groupID: "localization", summary: "订阅 LaserScan /scan 并生成在线占据栅格 /map。", resourceHints: ["scan_topic /scan", "OccupancyGrid /map"], sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/bringup/config/slam_toolbox_single_lidar.yaml"], sourceNote: "所选 Launch 启动 sync_slam_toolbox_node；配置使用 /scan、map_frame=map、odom_frame=odom_init。transform_publish_period=0，由独立静态 TF 节点发布 map → odom_init。"),
        .init(id: "map-odom-tf", title: "发布地图与里程计坐标变换", groupID: "localization", summary: "单雷达在线建图 Launch 中独立发布 map → odom_init 静态 TF。", resourceHints: ["/tf_static", "map → odom_init"], sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/bringup/config/slam_toolbox_single_lidar.yaml"], sourceNote: "Launch 创建 online_map_to_odom_init_tf；SLAM Toolbox 的 transform_publish_period 为 0，因此该静态节点是 map → odom_init 的唯一发布者。"),
        .init(id: "localization-map", title: "在线地图 Topic /map", groupID: "localization", summary: "SLAM Toolbox 发布、Nav2 静态图层消费的 OccupancyGrid。", resourceHints: ["/map", "nav_msgs/msg/OccupancyGrid"], sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/bringup/config/slam_toolbox_single_lidar.yaml", "Nav2_rea_ws/src/bringup/config/nav2_params.yaml"], sourceNote: "该节点表示 /map 数据接口，不是另一个进程。单雷达 Launch 不启动预制 map_server；当前 Agent 不读取地图内容或发布 / 订阅端点。"),

        .init(id: "nav2", title: "Nav2 导航", groupID: "navigation", summary: "消费 /map 与 /terrain_map 点云及 TF；速度经 smoother 输出 /cmd_vel。", workspaceModules: modules([("new-engineering", "navigation"), ("original-navigation", "navigation")]), resourceHints: ["/map", "/terrain_map", "/tf", "/cmd_vel_nav", "/cmd_vel"], sourcePaths: ["Nav2_rea_ws/src/navigation/rm_navigation/launch/navigation_launch.py", "Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/bringup/config/nav2_params.yaml"], sourceNote: "单雷达 Launch 使用 bringup/config/nav2_params.yaml：静态图层消费 /map，STVL 点云层消费 /terrain_map；/scan 输入用于 SLAM Toolbox。Graph Topic 名称不显示 Nav2 Action 状态或 publisher/subscriber 关系。"),
        .init(id: "mission-coordinator", title: "P6 任务协调器", groupID: "coordination", summary: "源码已在 main；离线 ROS 2/Nav2 闭环已验证，实体主启动流程集成状态待确认。", sourcePaths: ["Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py", "Nav2_rea_ws/src/p6_autonomous_mission/launch/p6_autonomous_mission.launch.py", "docs/p6/ros2-runtime-acceptance.md", "docs/p7/ros2-runtime-acceptance.md"], sourceNote: "P6 协调器、启动文件和离线 ROS 2 运行验收记录均在 main。验收启动包含隔离的 Nav2 与 cave simulator；它不证明实体主启动流程、真实传感器或执行器已接入。"),

        .init(id: "arm-stack", title: "机械臂软件模块（规划 / 串口）", groupID: "arm", summary: "Agent 的 arm 模块合并 lite_motion_planner 与 rm_serial_driver 两类包。", workspaceModules: modules([("new-engineering", "arm"), ("original-arm-vision", "arm")]), sourcePaths: ["tools/console-agent/console_agent/workspaces.py", "tzb_ws/src/lite_motion_planner/", "tzb_ws/src/serial/driver/package.xml"], sourceNote: "进程扫描只能报告合并模块是否发现进程，不能据此单独确认规划器或 rm_serial_driver。"),
        .init(id: "arm-planner", title: "机械臂运动规划", groupID: "arm", summary: "lite_motion_planner、MoveIt 与 tzb_catch 源码。", resourceHints: ["/tzb_catch/arm_joint_command", "/serial/arm_joint_state", "/p6/arm_task/request"], sourcePaths: ["tzb_ws/src/lite_motion_planner/", "tzb_ws/src/serial/driver/include/rm_serial_driver/node.hpp", "docs/P8_VALIDATION_REPORT.md"], sourceNote: "离线流程与 ROS 主机串口桥有源码证据；Agent 的合并 arm 模块不能区分本节点运行状态。"),
        .init(id: "arm-serial", title: "旧版 rm_serial_driver", groupID: "arm", summary: "机械臂命令 / 反馈主机桥；不是 Nav2_rea_ws 的新版底盘桥。", resourceHints: ["/tzb_catch/arm_joint_command", "/serial/arm_joint_state", "/serial/arm_safety/status"], sourcePaths: ["tzb_ws/src/serial/driver/package.xml", "tzb_ws/src/serial/driver/src/node.cpp", "tzb_ws/src/serial/driver/src/packet.cpp"], sourceNote: "状态 Topic 由主机驱动发布；不能单独证明 MCU 安全输入、所有关节反馈或电机健康。"),
        .init(id: "arm-controller", title: "机械臂 STM32", groupID: "arm", summary: "串口协议目标控制器；不由 ROS Graph 证明实际连接或动作。", sourcePaths: ["firmware/arm/README.md", "docs/P9E_PR8_INTEGRATION_REVIEW.md"], sourceNote: "固件静态接口与安全状态机有源码依据；当前运行硬件、串口归属和物理响应未知。", isExternalHardware: true),
        .init(id: "arm-safety-status", title: "机械臂安全状态 Topic", groupID: "arm", summary: "主机发布的 /serial/arm_safety/status。", resourceHints: ["/serial/arm_safety/status"], sourcePaths: ["tzb_ws/src/serial/driver/src/node.cpp", "docs/P9E_PR8_INTEGRATION_REVIEW.md"], sourceNote: "Agent 只发现 Topic 名称，不读取该 Topic 的消息值；状态内容和实体急停仍不可由当前 Graph 证明。"),

        .init(id: "chassis-bridge", title: "chassis_serial_bridge", groupID: "chassis", summary: "Nav2_rea_ws 的底盘专用 14 字节串口发送桥。", workspaceModules: modules([("new-engineering", "chassis"), ("original-navigation", "chassis")]), resourceHints: ["/cmd_vel", "/chassis/serial_link_status", "USART1 · 115200 · 14 B"], sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/package.xml", "Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp", "Nav2_rea_ws/src/drivers/serial/driver/launch/serial.launch.py"], sourceNote: "HOST_WRITE_OK 表示本机 TTY 接受写入；该协议没有 STM32 acknowledgement。"),
        .init(id: "chassis-controller", title: "底盘 STM32", groupID: "chassis", summary: "USART1 运动帧接收端；仓库协议不提供 ROS 可读确认。", workspaceModules: modules([("new-engineering", "chassis"), ("original-navigation", "chassis")]), sourcePaths: ["firmware/chassis/底盘串口通信协议.md", "firmware/chassis/README.md"], sourceNote: "14 字节 A5 / 3×float32 little-endian / 4A 协议有静态依据；设备是否运行、是否接收与驱动器反馈未知。", isExternalHardware: true),
        .init(id: "chassis-link-status", title: "底盘主机串口状态", groupID: "chassis", summary: "主机侧 /chassis/serial_link_status，仅表示本地串口打开 / 写入 / 断开。", workspaceModules: modules([("new-engineering", "chassis"), ("original-navigation", "chassis")]), resourceHints: ["/chassis/serial_link_status"], sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp"], sourceNote: "状态语义明确声明没有 STM32 acknowledgement；不可用作底盘硬件健康结论。"),

        .init(id: "physical-estop", title: "实体急停 / 硬件安全输入", groupID: "safety", summary: "当前 Agent 没有接入可核实的实体 E-stop 输入。", healthEventIDs: [], sourcePaths: ["tools/console-agent/console_agent/collectors.py", "docs/P9E_PR8_INTEGRATION_REVIEW.md"], sourceNote: "当前没有从该实体输入到 Agent 或 ROS 的已验证接口；拓扑将其作为未接入的独立边界展示，不绘制连接线。", isExternalHardware: true),
    ]

    static let edges: [TopologyEdgeDefinition] = [
        .init(id: "camera-yolo", sourceNodeID: "camera", destinationNodeID: "yolo", label: "RGB 图像", topics: ["/camera/color/image_raw"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/pose_estimator/launch/perception_bringup.launch.py", "tzb_ws/src/yolo_node/launch/yolo_node.launch.py"], sourceNote: "YOLO 输入默认使用 RGB image topic。"),
        .init(id: "camera-pose", sourceNodeID: "camera", destinationNodeID: "pose-estimator", label: "深度点云 / CameraInfo", topics: ["/camera/depth/color/points", "/camera/color/camera_info"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/pose_estimator/config/params.yaml", "tzb_ws/src/pose_estimator/pose_estimator/pose_estimation_node.py"], sourceNote: "pose_estimator 默认订阅点云与相机内参。"),
        .init(id: "yolo-arm2nav", sourceNodeID: "yolo", destinationNodeID: "arm2nav", label: "目标检测", topics: ["/yolo/detections"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/arm2nav/arm2nav/node.py", "tzb_ws/src/yolo_node/launch/yolo_node.launch.py"], sourceNote: "输入接口在源码中匹配；未确认与任务协调器的运行时链路。"),
        .init(id: "yolo-pose", sourceNodeID: "yolo", destinationNodeID: "pose-estimator", label: "检测结果", topics: ["/yolo/detections"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/pose_estimator/pose_estimator/pose_estimation_node.py"], sourceNote: "检测订阅由 pose_estimator 参数定义。"),
        .init(id: "pose-arm-planner", sourceNodeID: "pose-estimator", destinationNodeID: "arm-planner", label: "目标位姿", topics: ["/pose_estimator/object_pose"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/pose_estimator/pose_estimator/pose_estimation_node.py", "tzb_ws/src/lite_motion_planner/src/tzb_catch.cpp", "tzb_ws/src/lite_motion_planner/config/tzb_planner.yaml"], sourceNote: "位姿估计默认发布 /pose_estimator/object_pose，机械臂规划节点默认订阅同一 PoseStamped Topic；实际运行端点与硬件执行仍未确认。"),
        .init(id: "arm2nav-arm-planner", sourceNodeID: "arm2nav", destinationNodeID: "arm-planner", label: "可选目标 / 触发状态", topics: ["/arm2nav", "/arm2nav/from_arm"], probe: .topicName, evidence: .conditionalSourceWiring, sourcePaths: ["tzb_ws/src/arm2nav/config/arm2nav.yaml", "tzb_ws/src/lite_motion_planner/src/tzb_catch.cpp", "tzb_ws/src/lite_motion_planner/config/tzb_planner.yaml", "tzb_ws/src/lite_motion_planner/launch/bring_launch.launch.py"], sourceNote: "两端源码和可选订阅接口存在；bring_launch 默认关闭 use_arm2nav，规划配置默认 sentry_mode=false，因此此路径不是默认运行连接。"),

        .init(id: "mid360-pointlio", sourceNodeID: "mid360", destinationNodeID: "point-lio", label: "Livox 点云", topics: ["/livox/lidar"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2/launch/msg_MID360_launch.py", "Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml", "Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp"], sourceNote: "Point-LIO lid_topic 配置为 livox/lidar；IMU 由独立预处理链路输入。"),
        .init(id: "mid360-imu-transform", sourceNodeID: "mid360", destinationNodeID: "imu-transform", label: "原始 IMU", topics: ["/livox/imu"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2/src/lddc.cpp", "Nav2_rea_ws/src/preprocess/imu_tools/config/imu_transform.yaml"], sourceNote: "单 MID-360 驱动发布 livox/imu；imu_transform 配置将其设为 sub_topic。"),
        .init(id: "imu-transform-filter", sourceNodeID: "imu-transform", destinationNodeID: "imu-filter", label: "变换后 IMU", topics: ["/livox/imu_aft_tf"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/preprocess/imu_tools/config/imu_transform.yaml", "Nav2_rea_ws/src/preprocess/imu_tools/launch/imu_complementary_filter.launch.py"], sourceNote: "imu_transform 的 pub_topic 与互补滤波器重映射后的输入相同。"),
        .init(id: "imu-filter-pointlio", sourceNodeID: "imu-filter", destinationNodeID: "point-lio", label: "滤波 IMU", topics: ["/imu/data"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/preprocess/imu_tools/imu_complementary_filter/complementary_filter_ros.cpp", "Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml"], sourceNote: "互补滤波器发布 imu/data；Point-LIO imu_topic 配置为 imu/data。"),
        .init(id: "pointlio-terrain", sourceNodeID: "point-lio", destinationNodeID: "terrain", label: "注册点云 / 里程计", topics: ["/cloud_registered", "/odometry"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp", "Nav2_rea_ws/src/navigation/terrain_analysis/src/terrain_analysis_plus.cpp"], sourceNote: "Point-LIO 发布 cloud_registered / odometry；terrain_analysis_plus 订阅同名相对接口。Agent 不提供端点归属。"),
        .init(id: "terrain-scan", sourceNodeID: "terrain", destinationNodeID: "scan-conversion", label: "障碍点云", topics: ["/terrain_map"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"], sourceNote: "选定的单 MID-360 Launch 把 cloud_in 映射到 /terrain_map。"),
        .init(id: "scan-slam-toolbox", sourceNodeID: "scan-conversion", destinationNodeID: "slam-toolbox", label: "LaserScan 输入", topics: ["/scan"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/bringup/config/slam_toolbox_single_lidar.yaml"], sourceNote: "Launch remap 将 scan 发布为 /scan；SLAM Toolbox 配置订阅 /scan。Topic 可见不证明消息传输。"),
        .init(id: "slam-map", sourceNodeID: "slam-toolbox", destinationNodeID: "localization-map", label: "在线 OccupancyGrid", topics: ["/map"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/bringup/config/slam_toolbox_single_lidar.yaml"], sourceNote: "所选 Launch 指定 slam_toolbox 为在线 /map 发布者；不启动预制 map_server。"),
        .init(id: "terrain-nav2", sourceNodeID: "terrain", destinationNodeID: "nav2", label: "点云障碍输入", topics: ["/terrain_map"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/bringup/config/nav2_params.yaml"], sourceNote: "所选单雷达 Launch 的 Nav2 STVL 点云层使用 /terrain_map；不会把 /scan 作为该 Launch 的 Nav2 障碍输入。"),
        .init(id: "map-nav2", sourceNodeID: "localization-map", destinationNodeID: "nav2", label: "静态图层 / OccupancyGrid", topics: ["/map"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/bringup/config/nav2_params.yaml"], sourceNote: "单雷达 Launch 使用在线 slam_toolbox 地图；Nav2 global_costmap 的静态图层消费 /map。"),
        .init(id: "pointlio-tf-navigation", sourceNodeID: "point-lio", destinationNodeID: "nav2", label: "定位 TF", topics: ["/tf"], probe: .transformTree, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp", "Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml", "Nav2_rea_ws/src/navigation/rm_navigation/launch/navigation_launch.py"], sourceNote: "Point-LIO 发布 odom_init → base_frame 动态变换，Nav2 通过 TF 使用定位结果；Agent 不读取 frame pair，实时变换保持未知。"),
        .init(id: "map-tf-navigation", sourceNodeID: "map-odom-tf", destinationNodeID: "nav2", label: "地图原点 TF", topics: ["/tf_static"], probe: .transformTree, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", "Nav2_rea_ws/src/navigation/rm_navigation/launch/navigation_launch.py"], sourceNote: "单雷达 Launch 的独立静态节点发布 map → odom_init；Nav2 所需 TF 树的实时连通性未由 Agent 采集。"),
        .init(id: "nav2-chassis", sourceNodeID: "nav2", destinationNodeID: "chassis-bridge", label: "平滑速度", topics: ["/cmd_vel"], probe: .topicName, evidence: .offlineExercise, sourcePaths: ["Nav2_rea_ws/src/navigation/rm_navigation/launch/navigation_launch.py", "docs/p5b/chassis-ros2-integration.md"], sourceNote: "Velocity smoother remaps cmd_vel_smoothed to /cmd_vel; prior PTY validation covered the host software path, not robot hardware."),
        .init(id: "mission-chassis-stop", sourceNodeID: "mission-coordinator", destinationNodeID: "chassis-bridge", label: "任务中止零速请求", topics: ["/cmd_vel"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py", "Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp"], sourceNote: "协调器在中止流程中发布零速度 Twist，底盘桥订阅 /cmd_vel；这是源码路径证据，不证明运行时发布端归属或实体停止。"),
        .init(id: "chassis-bridge-mcu", sourceNodeID: "chassis-bridge", destinationNodeID: "chassis-controller", label: "USART1 · 115200 · 14 B", topics: [], probe: .serialProtocol, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp", "firmware/chassis/底盘串口通信协议.md"], sourceNote: "主机发送 A5 + 3 个 little-endian float32 + 4A；没有 STM32 ACK。"),

        .init(id: "mission-nav2", sourceNodeID: "mission-coordinator", destinationNodeID: "nav2", label: "导航目标 Action", topics: ["/navigate_to_pose"], probe: .actionEndpoint, evidence: .offlineExercise, sourcePaths: ["Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py", "Nav2_rea_ws/src/p6_autonomous_mission/launch/p6_autonomous_mission.launch.py", "docs/p6/ros2-runtime-acceptance.md"], sourceNote: "协调器创建 /navigate_to_pose ActionClient，并在隔离离线 ROS 2 验收中运行；Agent 不采集 Action endpoint，实体主启动流程集成仍待确认。"),
        .init(id: "nav2-mission-feedback", sourceNodeID: "nav2", destinationNodeID: "mission-coordinator", label: "导航结果与反馈", topics: ["/navigate_to_pose"], probe: .actionEndpoint, evidence: .offlineExercise, sourcePaths: ["Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py", "docs/p6/ros2-runtime-acceptance.md"], sourceNote: "NavigateToPose Action 结果回传给协调器；Agent 不采集 Action feedback/result，实体主流程未确认。", isFeedback: true),
        .init(id: "mission-arm-request", sourceNodeID: "mission-coordinator", destinationNodeID: "arm-planner", label: "机械臂任务请求", topics: ["/p6/arm_task/request", "/p6/arm_task/cancel"], probe: .topicName, evidence: .offlineExercise, sourcePaths: ["Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py", "tzb_ws/src/lite_motion_planner/src/tzb_catch.cpp", "docs/p6/ros2-runtime-acceptance.md"], sourceNote: "协调器发布请求 / cancel，P6 staged-tasks 规划节点订阅相同接口；离线验收不证明实体机械臂已连接。"),
        .init(id: "arm-mission-feedback", sourceNodeID: "arm-planner", destinationNodeID: "mission-coordinator", label: "机械臂任务结果", topics: ["/p6/arm_task/result", "/p6/arm_task/cancel_ack"], probe: .topicName, evidence: .offlineExercise, sourcePaths: ["Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py", "tzb_ws/src/lite_motion_planner/src/tzb_catch.cpp", "docs/p6/ros2-runtime-acceptance.md"], sourceNote: "规划节点发布 task result / cancel acknowledgement，协调器订阅；只证明源码与隔离离线接口。", isFeedback: true),
        .init(id: "arm-planner-driver", sourceNodeID: "arm-planner", destinationNodeID: "arm-serial", label: "关节命令", topics: ["/tzb_catch/arm_joint_command"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/lite_motion_planner/src/tzb_catch.cpp", "tzb_ws/src/lite_motion_planner/config/tzb_planner.yaml", "tzb_ws/src/serial/driver/src/node.cpp", "tzb_ws/src/serial/driver/config/serial_params.yaml"], sourceNote: "规划器发布 /tzb_catch/arm_joint_command，旧版 rm_serial_driver 订阅同一 JointState Topic。"),
        .init(id: "arm-serial-feedback", sourceNodeID: "arm-serial", destinationNodeID: "arm-planner", label: "关节状态反馈", topics: ["/serial/arm_joint_state"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/node.cpp", "tzb_ws/src/lite_motion_planner/src/tzb_catch.cpp", "tzb_ws/src/lite_motion_planner/config/tzb_planner.yaml"], sourceNote: "旧版 rm_serial_driver 发布 JointState，规划器订阅该 Topic；Topic 名称不证明实体关节反馈有效。", isFeedback: true),
        .init(id: "arm-serial-mcu-command", sourceNodeID: "arm-serial", destinationNodeID: "arm-controller", label: "机械臂串口命令帧", topics: [], probe: .serialProtocol, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/packet.cpp", "firmware/arm/others/src/app/Host.cpp", "docs/P9E_PR8_INTEGRATION_REVIEW.md"], sourceNote: "主机与 STM32 命令帧协议有源码依据；当前串口设备连接与控制器接收未知。"),
        .init(id: "arm-mcu-serial-feedback", sourceNodeID: "arm-controller", destinationNodeID: "arm-serial", label: "STM32 关节反馈帧", topics: [], probe: .serialProtocol, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/node.cpp", "tzb_ws/src/serial/driver/src/packet.cpp", "firmware/arm/others/src/app/Host.cpp"], sourceNote: "驱动解析控制器反馈并发布 ROS 状态；Agent 不读取串口字节或确认物理响应。", isFeedback: true),
        .init(id: "arm-driver-safety", sourceNodeID: "arm-serial", destinationNodeID: "arm-safety-status", label: "安全状态文本 Topic", topics: ["/serial/arm_safety/status"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/node.cpp"], sourceNote: "当前 Agent 只观察 Topic 名称，不订阅或读取消息内容。"),
        .init(id: "arm-safety-mission", sourceNodeID: "arm-safety-status", destinationNodeID: "mission-coordinator", label: "机械臂安全状态反馈", topics: ["/serial/arm_safety/status"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["tzb_ws/src/serial/driver/src/node.cpp", "Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py"], sourceNote: "机械臂驱动发布安全状态文本，P6 协调器订阅该 Topic；Agent 仅看名称，不读取安全状态内容。", isFeedback: true),
        .init(id: "chassis-status", sourceNodeID: "chassis-bridge", destinationNodeID: "chassis-link-status", label: "主机串口状态", topics: ["/chassis/serial_link_status"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp"], sourceNote: "仅是 host serial open/write 状态，没有 STM32 acknowledgement 语义。"),
        .init(id: "chassis-status-mission", sourceNodeID: "chassis-link-status", destinationNodeID: "mission-coordinator", label: "底盘主机链路状态反馈", topics: ["/chassis/serial_link_status"], probe: .topicName, evidence: .sourceWiring, sourcePaths: ["Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp", "Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py"], sourceNote: "底盘桥发布主机串口状态，P6 协调器可订阅；其值不代表 STM32 已接收或底盘运动正常。", isFeedback: true),
    ]

    static let subsystems: [TopologySubsystemDefinition] = [
        .init(id: "perception", title: "视觉感知", summary: "相机、目标检测与位姿估计", symbol: "camera.viewfinder", statusNodeIDs: ["camera", "yolo", "arm2nav", "pose-estimator"]),
        .init(id: "localization", title: "雷达定位与建图", summary: "MID-360 点云 / IMU、定位、地形过滤、扫描与在线地图", symbol: "dot.radiowaves.left.and.right", statusNodeIDs: ["lidar-stack", "mid360", "imu-transform", "imu-filter", "point-lio", "terrain", "scan-conversion", "slam-toolbox", "map-odom-tf", "localization-map"]),
        .init(id: "coordination", title: "任务协调", summary: "P6 源码与隔离离线验证存在；实体主启动集成待确认", symbol: "point.3.connected.trianglepath.dotted", statusNodeIDs: ["mission-coordinator"]),
        .init(id: "navigation", title: "自动导航", summary: "地图、障碍点云、定位 TF、目标 Action 与速度输出", symbol: "location.north.line", statusNodeIDs: ["nav2"]),
        .init(id: "arm", title: "机械臂", summary: "目标位姿、任务规划、主机串口桥、STM32 与状态反馈", symbol: "move.3d", statusNodeIDs: ["arm-stack", "arm-planner", "arm-serial", "arm-controller", "arm-safety-status"]),
        .init(id: "chassis", title: "底盘", summary: "Nav2 / 任务中止速度、主机串口桥与 STM32", symbol: "steeringwheel", statusNodeIDs: ["chassis-bridge", "chassis-controller", "chassis-link-status"]),
        .init(id: "safety", title: "未接入安全输入", summary: "实体急停输入与当前监测边界", symbol: "checkmark.shield", statusNodeIDs: ["physical-estop"]),
    ]

    private static let nodeByID = Dictionary(uniqueKeysWithValues: nodes.map { ($0.id, $0) })
    private static let edgeByID = Dictionary(uniqueKeysWithValues: edges.map { ($0.id, $0) })

    static func node(_ id: String) -> TopologyNodeDefinition? { nodeByID[id] }
    static func edge(_ id: String) -> TopologyEdgeDefinition? { edgeByID[id] }
    static func subsystem(_ id: String) -> TopologySubsystemDefinition? { subsystems.first { $0.id == id } }

    static var subsystemIDs: [String] { subsystems.map(\.id) }

    static func project(expandedSubsystemIDs: Set<String>) -> TopologyGraphProjection {
        let robotSubsystemIDs = Set(subsystems.map(\.id))
        var projectedNodes: [TopologyProjectedNode] = []
        var endpointByNodeID: [String: String] = [:]

        for subsystem in subsystems {
            let members = nodes.filter { $0.groupID == subsystem.id }.map(\.id)
            if expandedSubsystemIDs.contains(subsystem.id) {
                for nodeID in members {
                    endpointByNodeID[nodeID] = nodeID
                    projectedNodes.append(TopologyProjectedNode(
                        id: nodeID,
                        subsystemID: subsystem.id,
                        moduleNodeID: nodeID,
                        memberNodeIDs: [nodeID]
                    ))
                }
            } else {
                endpointByNodeID.merge(Dictionary(uniqueKeysWithValues: members.map { ($0, subsystem.id) })) { current, _ in current }
                projectedNodes.append(TopologyProjectedNode(
                    id: subsystem.id,
                    subsystemID: subsystem.id,
                    moduleNodeID: nil,
                    memberNodeIDs: members
                ))
            }
        }

        let projectedEdges = edges.compactMap { edge -> TopologyProjectedEdge? in
            guard let source = nodeByID[edge.sourceNodeID],
                  let destination = nodeByID[edge.destinationNodeID],
                  robotSubsystemIDs.contains(source.groupID),
                  robotSubsystemIDs.contains(destination.groupID),
                  let projectedSource = endpointByNodeID[edge.sourceNodeID],
                  let projectedDestination = endpointByNodeID[edge.destinationNodeID],
                  projectedSource != projectedDestination else { return nil }
            return TopologyProjectedEdge(
                id: edge.id,
                sourceID: projectedSource,
                destinationID: projectedDestination,
                evidence: edge.evidence,
                isFeedback: edge.isFeedback
            )
        }
        return TopologyGraphProjection(nodes: projectedNodes, edges: projectedEdges)
    }

    static func downstreamNodeIDs(from startID: String) -> Set<String> {
        var visited = Set<String>()
        var pending = [startID]
        while let current = pending.popLast() {
            for edge in edges where edge.sourceNodeID == current &&
                edge.evidence != .openInterface && edge.evidence != .conditionalSourceWiring {
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
    private(set) var expandedSubsystemIDs: Set<String> = []
    private(set) var selectedNodeID: String?

    mutating func toggleSubsystem(_ id: String) {
        if expandedSubsystemIDs.contains(id) {
            expandedSubsystemIDs.remove(id)
            if let selectedNodeID,
               RobotTopologyCatalog.node(selectedNodeID)?.groupID == id {
                self.selectedNodeID = nil
            }
        } else {
            expandedSubsystemIDs.insert(id)
        }
    }

    mutating func selectNode(_ nodeID: String) {
        if let subsystemID = RobotTopologyCatalog.node(nodeID)?.groupID,
           RobotTopologyCatalog.subsystem(subsystemID) != nil {
            expandedSubsystemIDs.insert(subsystemID)
        }
        selectedNodeID = nodeID
    }

    mutating func collapseAll() {
        expandedSubsystemIDs.removeAll()
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
    let linkTelemetry: LinkTelemetryReading?
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

    var linkTelemetryBlockReason: String? {
        guard let linkTelemetry else { return nil }
        guard linkTelemetry.status == "ok", linkTelemetry.graphStatus == "ok" else {
            return linkTelemetry.graphReason ?? "实时链路遥测或 ROS Graph 证据不可用。"
        }
        guard let snapshotDomain = snapshot?.ros.domainId,
              linkTelemetry.domainId == snapshotDomain else {
            return "链路遥测 ROS Domain 与当前快照不一致；Topic 与工作区归属保持未知。"
        }
        if linkTelemetry.domainMatchesExpectation == false {
            return "预期 ROS Domain 不匹配；下游链路归属保持未知。"
        }
        if let error = linkTelemetry.expectedDomainError {
            return "预期 ROS Domain 配置无效：\(error)"
        }
        guard let age = linkTelemetry.graphAgeMs, age >= 0, age <= HealthEvaluator.rosGraphTimeoutMs else {
            return "链路遥测所依赖的 ROS Graph 数据过期或缺少时间证据。"
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
        let staleStreamDownstream = RobotTopologyCatalog.edges.reduce(into: Set<String>()) { result, edge in
            guard streamState(for: edge, context: context) == .stale else { return }
            result.insert(edge.destinationNodeID)
            result.formUnion(RobotTopologyCatalog.downstreamNodeIDs(from: edge.destinationNodeID))
        }
        let allPossiblyAffected = downstream.union(staleStreamDownstream)
        let directAbnormalNodeSet = Set(directAbnormalNodes)

        var nodes: [String: TopologyElementStatus] = [:]
        for node in RobotTopologyCatalog.nodes {
            nodes[node.id] = nodeStatus(node, context: context, downstream: allPossiblyAffected)
        }

        var edges: [String: TopologyElementStatus] = [:]
        for edge in RobotTopologyCatalog.edges {
            let edgePossiblyAffected = directAbnormalNodeSet.contains(edge.sourceNodeID) ||
                directAbnormalNodeSet.contains(edge.destinationNodeID) ||
                allPossiblyAffected.contains(edge.sourceNodeID) || allPossiblyAffected.contains(edge.destinationNodeID) ||
                streamState(for: edge, context: context) == .stale
            edges[edge.id] = edgeStatus(edge, context: context, possiblyAffected: edgePossiblyAffected)
        }
        return TopologyEvaluation(nodeStatuses: nodes, edgeStatuses: edges, possiblyAffectedNodeIDs: allPossiblyAffected)
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
        if let streamSignal = telemetrySignal(for: edge, context: context) {
            return telemetryEdgeStatus(edge, context: context, possiblyAffected: possiblyAffected, signal: streamSignal)
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

    private enum StreamEvidenceState: Equatable {
        case streaming
        case stale
        case notObserved
        case unknown
    }

    private static func streamState(for edge: TopologyEdgeDefinition, context: TopologyRuntimeContext) -> StreamEvidenceState? {
        guard edge.probe == .topicName,
              !edge.topics.isEmpty,
              context.linkTelemetryBlockReason == nil,
              let telemetry = context.linkTelemetry else { return nil }
        let streams = edge.topics.compactMap { telemetry.stream(topic: $0) }
        guard !streams.isEmpty else { return nil }
        if streams.contains(where: { $0.status == "stale" }) { return .stale }
        if streams.allSatisfy({ $0.status == "streaming" }) { return .streaming }
        if streams.contains(where: { $0.status == "streaming" }) { return .notObserved }
        if streams.allSatisfy({ ["waiting", "unavailable"].contains($0.status) }) { return .notObserved }
        return .unknown
    }

    private static func telemetrySignal(for edge: TopologyEdgeDefinition, context: TopologyRuntimeContext) -> TopologySignal? {
        guard context.linkTelemetry != nil else { return nil }
        if context.linkTelemetryBlockReason != nil {
            return .unknown
        }
        let streams = edge.topics.compactMap { context.linkTelemetry?.stream(topic: $0) }
        guard !streams.isEmpty else { return nil }
        if streams.allSatisfy({ $0.status == "streaming" }) { return .dataObserved }
        if streams.contains(where: { $0.status == "stale" }) {
            return streams.contains(where: { $0.status == "streaming" }) ? .dataPartiallyObserved : .dataStale
        }
        if streams.contains(where: { $0.status == "streaming" }) { return .dataPartiallyObserved }
        if streams.allSatisfy({ ["waiting", "unavailable"].contains($0.status) }) { return .dataNotObserved }
        return .unknown
    }

    private static func telemetryEdgeStatus(
        _ edge: TopologyEdgeDefinition,
        context: TopologyRuntimeContext,
        possiblyAffected: Bool,
        signal: TopologySignal
    ) -> TopologyElementStatus {
        if let blocked = context.linkTelemetryBlockReason {
            return TopologyElementStatus(signal: .unknown, note: blocked, directEvents: [], possiblyAffectedDownstream: false)
        }
        let telemetry = context.linkTelemetry
        let streams = edge.topics.compactMap { telemetry?.stream(topic: $0) }
        let details = streams.map { stream in
            let freshness = stream.ageMs.map { "\($0) ms" } ?? "无接收时间"
            let rate = stream.receiveRateHz.map { String(format: "%.2f Hz", $0) } ?? "频率待采样"
            let endpoints = stream.publisherEndpoints.map { endpoint in
                endpoint.namespace == "/" ? endpoint.name : "\(endpoint.namespace)/\(endpoint.name)"
            }
            let source = endpoints.isEmpty ? "发布端点未见" : "发布端点：\(endpoints.joined(separator: "、"))"
            return "\(stream.topic)：\(stream.status)，\(rate)，最近接收 \(freshness)，类型 \(stream.topicType ?? "未知")，\(source)"
        }.joined(separator: "\n")
        let note = details + "\n工作区归属未确认；Agent 收到数据不证明下游消费者已处理，也不证明实体硬件健康。数据时效只描述接收端；跨时钟时间差需确认时钟同步。"
        return TopologyElementStatus(
            signal: signal,
            note: note,
            directEvents: [],
            possiblyAffectedDownstream: possiblyAffected || signal == .dataStale
        )
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

enum LinkTelemetryDiagnostics {
    static func streams(for nodeID: String, telemetry: LinkTelemetryReading?) -> [LinkTelemetryStreamReading] {
        guard let telemetry, let node = RobotTopologyCatalog.node(nodeID) else { return [] }
        let incidentTopics = RobotTopologyCatalog.edges
            .filter { $0.sourceNodeID == nodeID || $0.destinationNodeID == nodeID }
            .flatMap(\.topics)
        let topicHints = node.resourceHints.filter { $0.hasPrefix("/") }
        let topics = Set(incidentTopics + topicHints)
        return telemetry.streams.filter { topics.contains($0.topic) }
    }

    static func pipelineSummary(nodeID: String, telemetry: LinkTelemetryReading?, blockedReason: String?) -> String? {
        guard ["point-lio", "slam-toolbox"].contains(nodeID) else { return nil }
        guard let telemetry else { return "实时链路遥测不可用；当前 Agent 版本或连接未提供消息接收证据。" }
        if let blockedReason { return "实时链路对照保持未知：\(blockedReason)" }

        let inputs: [String]
        let outputs: [String]
        switch nodeID {
        case "point-lio":
            inputs = ["/livox/lidar", "/imu/data"]
            outputs = ["/odometry", "/cloud_registered"]
        default:
            inputs = ["/scan"]
            outputs = ["/map"]
        }
        let inputStreams = inputs.compactMap { telemetry.stream(topic: $0) }
        let outputStreams = outputs.compactMap { telemetry.stream(topic: $0) }
        guard inputStreams.count == inputs.count, outputStreams.count == outputs.count else {
            return "链路对照数据不完整；不能定位处理原因或确认模块运行状态。"
        }

        let allInputsFresh = inputStreams.allSatisfy { $0.status == "streaming" }
        let anyInputObserved = inputStreams.contains { ["streaming", "stale"].contains($0.status) }
        let allOutputsFresh = outputStreams.allSatisfy { $0.status == "streaming" }
        let anyOutputStale = outputStreams.contains { $0.status == "stale" }
        let noOutputObserved = outputStreams.allSatisfy { ["waiting", "unavailable"].contains($0.status) }

        if allInputsFresh && !allOutputsFresh && (anyOutputStale || noOutputObserved) {
            return "处理链路线索：输入 Topic 已由 Agent 收到，输出 Topic 暂未收到或已过期。可能存在处理链路间断；工作区来源与运行预期未确认，因此不判定模块故障。\n建议查看对应模块日志、输入/输出类型和 TF 证据。"
        }
        if !anyInputObserved && !allOutputsFresh {
            return "输入数据尚未观测到，输出状态不能用于区分模块未运行与上游数据缺失。运行预期未确认，模块状态保持未知。"
        }
        if !allInputsFresh && allOutputsFresh {
            return "部分输入未更新，但输出最近仍有 Agent 接收记录；数据可能处于恢复或跨源状态，不能据此定位根因。"
        }
        if allInputsFresh && allOutputsFresh {
            return "输入和输出 Topic 均有近期 Agent 接收记录；这不确认指定工作区归属、算法质量、消费者处理或实体硬件健康。"
        }
        return "输入或输出证据不足；模块运行预期未确认，处理状态保持未知。"
    }
}
