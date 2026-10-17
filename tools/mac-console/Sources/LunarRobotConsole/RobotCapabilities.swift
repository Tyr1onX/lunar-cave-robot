import Foundation

struct RobotCapabilityStepDefinition: Identifiable, Equatable {
    let id: String
    let title: String
    let input: String
    let process: String
    let output: String
    let nodeIDs: [String]
    let edgeIDs: [String]
    let secondaryEdgeIDs: [String]
    let note: String

    init(
        id: String,
        title: String,
        input: String,
        process: String,
        output: String,
        nodeIDs: [String],
        edgeIDs: [String] = [],
        secondaryEdgeIDs: [String] = [],
        note: String = ""
    ) {
        self.id = id
        self.title = title
        self.input = input
        self.process = process
        self.output = output
        self.nodeIDs = nodeIDs
        self.edgeIDs = edgeIDs
        self.secondaryEdgeIDs = secondaryEdgeIDs
        self.note = note
    }
}

struct RobotCapabilityDefinition: Identifiable, Equatable {
    let id: String
    let title: String
    let subtitle: String
    let symbol: String
    let steps: [RobotCapabilityStepDefinition]
    let feedbackEdgeIDs: [String]
    let boundaryNodeIDs: [String]

    var defaultNodeIDs: [String] {
        orderedUnique(steps.flatMap(\.nodeIDs))
    }

    var defaultEdgeIDs: [String] {
        orderedUnique(steps.flatMap(\.edgeIDs)).filter { edgeID in
            guard let edge = RobotTopologyCatalog.edge(edgeID) else { return false }
            return !edge.isFeedback && edge.evidence != .conditionalSourceWiring && edge.evidence != .openInterface
        }
    }

    private func orderedUnique(_ values: [String]) -> [String] {
        var seen = Set<String>()
        return values.filter { seen.insert($0).inserted }
    }
}

enum RobotCapabilityCatalog {
    static let capabilities: [RobotCapabilityDefinition] = [
        .init(
            id: "observe-and-grasp",
            title: "观察并抓取目标",
            subtitle: "从图像与深度信息，到机械臂动作请求",
            symbol: "viewfinder",
            steps: [
                .init(
                    id: "capture-target",
                    title: "采集目标图像",
                    input: "相机视野中的环境",
                    process: "相机采集彩色图像、深度和相机参数",
                    output: "彩色图像、深度点云与相机内参",
                    nodeIDs: ["camera"],
                    note: "源码与 Launch 配置了 Gemini 336L 接口；Topic 名称本身不能确认相机数据来自哪个工作区。"
                ),
                .init(
                    id: "recognize-target",
                    title: "识别目标",
                    input: "彩色图像",
                    process: "识别模型根据置信度条件筛选候选目标",
                    output: "符合置信度条件的目标候选",
                    nodeIDs: ["yolo"],
                    edgeIDs: ["camera-yolo"],
                    note: "检测结果发布带有置信度门控。没有发布检测结果可能表示当前图像中没有满足条件的目标，不能单独判定为故障。"
                ),
                .init(
                    id: "estimate-target-pose",
                    title: "计算目标位置",
                    input: "目标检测结果，以及相机深度点云和内参",
                    process: "将图像中的目标与深度信息转换为机器人坐标中的目标位姿",
                    output: "目标在机器人坐标系中的位置",
                    nodeIDs: ["pose-estimator"],
                    edgeIDs: ["camera-pose", "yolo-pose"],
                    note: "位姿估计同时接收相机深度与 YOLO 检测；Console 当前不读取 TF frame tree，运行时坐标变换仍需在详情中核实。"
                ),
                .init(
                    id: "plan-arm-motion",
                    title: "规划机械臂动作",
                    input: "目标在机器人坐标系中的位置",
                    process: "运动规划程序将目标位置转换为关节动作",
                    output: "机械臂关节动作请求",
                    nodeIDs: ["arm-planner"],
                    edgeIDs: ["pose-arm-planner"],
                    secondaryEdgeIDs: ["arm2nav-arm-planner", "mission-arm-request"],
                    note: "规划器存在源码和离线验证依据。视觉建议输入为可选配置；任务协调器发起的请求有离线接口依据，实体主流程接入状态待确认。"
                ),
                .init(
                    id: "send-arm-request",
                    title: "发送机械臂动作请求",
                    input: "规划后的关节动作请求",
                    process: "主机通信程序将动作请求转换为控制协议",
                    output: "控制器协议帧；设备响应尚未被监测接口确认",
                    nodeIDs: ["arm-serial", "arm-controller"],
                    edgeIDs: ["arm-planner-driver", "arm-serial-mcu-command"],
                    note: "该流程描述源码接口，不证明串口当前已连接、STM32 已接收或机械臂已经运动。"
                ),
            ],
            feedbackEdgeIDs: ["arm-serial-feedback", "arm-mcu-serial-feedback"],
            boundaryNodeIDs: []
        ),
        .init(
            id: "localize-and-drive",
            title: "定位并自主行驶",
            subtitle: "从雷达与惯性测量，到导航速度请求",
            symbol: "location.north.line",
            steps: [
                .init(
                    id: "collect-lidar-imu",
                    title: "采集雷达与惯性数据",
                    input: "周围环境与传感器运动测量",
                    process: "雷达驱动发布点云，惯性测量经过坐标转换与滤波",
                    output: "环境点云与整理后的惯性测量",
                    nodeIDs: ["mid360", "imu-transform", "imu-filter"],
                    edgeIDs: ["mid360-imu-transform", "imu-transform-filter"],
                    note: "选定的单雷达 Launch 中点云直接送入 Point-LIO，IMU 则经两个预处理节点后输入。该页面不会启动或检查真实传感器电源状态。"
                ),
                .init(
                    id: "estimate-robot-motion",
                    title: "估计机器人位置与移动",
                    input: "雷达点云与整理后的惯性测量",
                    process: "位置估计程序融合测量，计算机器人的移动轨迹",
                    output: "机器人位置、处理后的点云与坐标关系",
                    nodeIDs: ["point-lio"],
                    edgeIDs: ["mid360-pointlio", "imu-filter-pointlio"],
                    note: "源码与配置定义这些输入输出；Topic 可见不证明 Point-LIO 正在处理数据，也不证明定位精度。"
                ),
                .init(
                    id: "prepare-map-input",
                    title: "整理障碍和扫描数据",
                    input: "处理后的环境点云与机器人位置",
                    process: "过滤障碍点云，再转换为平面扫描数据",
                    output: "导航障碍数据与地图生成所需的扫描信息",
                    nodeIDs: ["terrain", "scan-conversion"],
                    edgeIDs: ["pointlio-terrain", "terrain-scan"],
                    note: "在单雷达 Launch 中，/terrain_map 另有一路直接供 Nav2 的点云图层使用；这条支路不会经过 /scan。"
                ),
                .init(
                    id: "build-map",
                    title: "生成在线地图",
                    input: "平面扫描信息",
                    process: "地图程序持续更新占据栅格，并发布地图坐标关系",
                    output: "在线地图与地图坐标关系",
                    nodeIDs: ["slam-toolbox", "map-odom-tf", "localization-map"],
                    edgeIDs: ["scan-slam-toolbox", "slam-map"],
                    note: "仅反映单雷达 Launch 的地图路径；Agent 只读 Graph 不读取地图内容或 TF 连通性。"
                ),
                .init(
                    id: "plan-route",
                    title: "规划导航路线",
                    input: "在线地图、障碍数据和机器人位置",
                    process: "导航规划程序依据地图和障碍规划行驶路线",
                    output: "平滑后的底盘速度请求",
                    nodeIDs: ["nav2"],
                    edgeIDs: ["terrain-nav2", "map-nav2", "pointlio-tf-navigation", "map-tf-navigation"],
                    note: "所选 Launch 中 Nav2 的静态图层使用 /map，障碍点云图层使用 /terrain_map；/scan 输入用于 SLAM。Agent 不采集导航 Action 的运行结果。"
                ),
                .init(
                    id: "send-chassis-request",
                    title: "发送底盘速度请求",
                    input: "导航规划输出的速度请求",
                    process: "底盘通信程序将速度编码为串口协议",
                    output: "主机串口发送请求；控制板是否接收尚未确认",
                    nodeIDs: ["chassis-bridge", "chassis-controller"],
                    edgeIDs: ["nav2-chassis", "chassis-bridge-mcu"],
                    secondaryEdgeIDs: ["chassis-status"],
                    note: "主机侧写入状态不代表底盘控制板已接收或机器人已移动。Console 不发送速度请求。"
                ),
            ],
            feedbackEdgeIDs: [],
            boundaryNodeIDs: []
        ),
        .init(
            id: "task-and-safety",
            title: "任务与安全管理",
            subtitle: "了解任务协调接口与当前安全观测边界",
            symbol: "checkmark.shield",
            steps: [
                .init(
                    id: "coordinate-tasks",
                    title: "协调导航与机械臂任务",
                    input: "任务阶段与软件模块状态",
                    process: "任务程序按阶段协调导航和机械臂请求",
                    output: "导航目标请求与机械臂任务请求",
                    nodeIDs: ["mission-coordinator"],
                    edgeIDs: ["mission-nav2", "mission-arm-request"],
                    note: "协调器源码和隔离离线 ROS 2 / Nav2 验证存在；实体机器人主启动流程是否集成仍待确认。"
                ),
                .init(
                    id: "safety-observation-boundary",
                    title: "确认安全状态可见范围",
                    input: "ROS Graph 名称、Agent 快照及已公开的主机链路状态",
                    process: "Console 展示可读的软件状态线索，不订阅实体急停或控制器确认接口",
                    output: "可见范围与未接入的实体安全输入边界",
                    nodeIDs: ["physical-estop"],
                    note: "实体急停输入没有已验证的 ROS / Agent 接口。Topic 名称、主机串口状态和 ROS 通信均不能代替硬件安全确认。"
                ),
            ],
            feedbackEdgeIDs: [
                "nav2-mission-feedback", "arm-mission-feedback", "arm-safety-mission", "chassis-status-mission"
            ],
            boundaryNodeIDs: ["physical-estop"]
        ),
    ]

    static func capability(_ id: String) -> RobotCapabilityDefinition? {
        capabilities.first { $0.id == id }
    }

    static func step(capabilityID: String, stepID: String) -> RobotCapabilityStepDefinition? {
        capability(capabilityID)?.steps.first { $0.id == stepID }
    }
}

enum RobotCapabilityObservationHint: Equatable {
    case directAnomaly
    case staleData
    case processNotFound
    case evidencePending
    case observedWithoutHealthClaim
    case noDirectAnomalyEvidence

    var title: String {
        switch self {
        case .directAnomaly: return "检测到直接异常"
        case .staleData: return "数据暂未更新"
        case .processNotFound: return "未发现关联进程"
        case .evidencePending: return "运行状态待确认"
        case .observedWithoutHealthClaim: return "已有接口观测，健康未确认"
        case .noDirectAnomalyEvidence: return "当前观测未发现直接异常"
        }
    }
}

enum RobotCapabilityStatusSummary {
    static func hint(for step: RobotCapabilityStepDefinition, evaluation: TopologyEvaluation) -> RobotCapabilityObservationHint? {
        let nodeStatuses = step.nodeIDs.map { evaluation.nodeStatus($0) }
        let edgeStatuses = step.edgeIDs.map { evaluation.edgeStatus($0) }
        if (nodeStatuses + edgeStatuses).contains(where: { $0.signal == .abnormal }) { return .directAnomaly }
        if edgeStatuses.contains(where: { $0.signal == .dataStale }) { return .staleData }
        if nodeStatuses.contains(where: { $0.signal == .processNotFound }) { return .processNotFound }
        if (nodeStatuses + edgeStatuses).contains(where: {
            [.unknown, .notRunning, .topicsNoneVisible, .dataNotObserved].contains($0.signal)
        }) { return .evidencePending }
        if (nodeStatuses + edgeStatuses).contains(where: {
            [.processObserved, .topicsAllVisible, .topicsPartiallyVisible, .dataObserved, .dataPartiallyObserved].contains($0.signal)
        }) { return .observedWithoutHealthClaim }
        return nil
    }

    static func landingHint(for evaluation: TopologyEvaluation) -> RobotCapabilityObservationHint {
        let nodes = Array(evaluation.nodeStatuses.values)
        let edges = Array(evaluation.edgeStatuses.values)
        if (nodes + edges).contains(where: { $0.signal == .abnormal }) { return .directAnomaly }
        if edges.contains(where: { $0.signal == .dataStale }) { return .staleData }
        if (nodes + edges).contains(where: {
            [.unknown, .notRunning, .topicsNoneVisible, .dataNotObserved].contains($0.signal)
        }) { return .evidencePending }
        if nodes.contains(where: { $0.signal == .processNotFound }) { return .processNotFound }
        return .noDirectAnomalyEvidence
    }
}
