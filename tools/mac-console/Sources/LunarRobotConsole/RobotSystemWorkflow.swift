import Foundation

struct RobotWorkflowStepDefinition: Identifiable, Equatable {
    let id: String
    let title: String
    let input: String
    let process: String
    let output: String
    let nodeIDs: [String]
    let incomingEdgeIDs: [String]
    let note: String
}

struct RobotWorkflowLaneDefinition: Identifiable, Equatable {
    let id: String
    let title: String
    let summary: String
    let steps: [RobotWorkflowStepDefinition]
}

struct RobotWorkflowPhaseDefinition: Identifiable, Equatable {
    let id: String
    let number: String
    let title: String
    let summary: String
    let lanes: [RobotWorkflowLaneDefinition]
    let internalEdgeIDs: [String]

    var nodeIDs: [String] {
        orderedUnique(lanes.flatMap { $0.steps.flatMap(\.nodeIDs) })
    }

    var stepIDs: [String] { lanes.flatMap { $0.steps.map(\.id) } }

    private func orderedUnique(_ values: [String]) -> [String] {
        var seen = Set<String>()
        return values.filter { seen.insert($0).inserted }
    }
}

struct RobotWorkflowInterfaceGroup: Identifiable, Equatable {
    let id: String
    let sourcePhaseID: String
    let destinationPhaseID: String
    let title: String
    let edgeIDs: [String]
    let isFeedback: Bool
}

enum RobotSystemWorkflowCatalog {
    static let phases: [RobotWorkflowPhaseDefinition] = [
        .init(
            id: "perception",
            number: "01",
            title: "感知与目标识别",
            summary: "视觉目标位姿与雷达定位 / 地图并行形成，为不同规划路径提供输入。",
            lanes: [
                .init(id: "vision", title: "视觉与目标", summary: "图像 → 检测 → 目标位姿", steps: [
                    .init(id: "camera-input", title: "采集图像与深度", input: "相机视野中的场景", process: "相机按所选配置提供 RGB、深度和相机参数。", output: "RGB 图像、深度点云、CameraInfo", nodeIDs: ["camera"], incomingEdgeIDs: [], note: "所选配置默认 Gemini 336L，也可切换 D435i；Topic 名称不能确认实际设备来源。"),
                    .init(id: "target-detection", title: "识别目标", input: "RGB 图像", process: "YOLO 按模型结果和置信度门控筛选候选。", output: "检测结果与标注图像（满足门控时）", nodeIDs: ["yolo"], incomingEdgeIDs: ["camera-yolo"], note: "门控条件未满足时不发布检测结果是允许状态，不能仅凭空 Topic 判故障。"),
                    .init(id: "target-pose", title: "计算目标位置", input: "检测结果、深度点云与相机内参", process: "位姿估计将检测框与深度信息转换为机器人坐标中的目标位姿。", output: "目标 PoseStamped 与源码定义的 TF", nodeIDs: ["pose-estimator"], incomingEdgeIDs: ["camera-pose", "yolo-pose"], note: "Agent 不采集 TF frame tree，运行时坐标连通性不能由 Topic 名称确认。"),
                    .init(id: "optional-visual-suggestion", title: "可选视觉导航建议", input: "检测结果与相机数据", process: "arm2nav 可生成建议状态。", output: "可选建议接口；默认流程未启用", nodeIDs: ["arm2nav"], incomingEdgeIDs: ["yolo-arm2nav"], note: "bring_launch 默认 use_arm2nav=false，planner 默认 sentry_mode=false。",),
                ]),
                .init(id: "localization", title: "雷达定位与地图", summary: "MID-360 点云 / IMU → Point-LIO → 障碍数据与在线地图", steps: [
                    .init(id: "sensor-inputs", title: "采集雷达与惯性数据", input: "MID-360 的周围环境测量与运动测量", process: "点云保留为 Point-LIO 直接输入；IMU 单独经过坐标变换和互补滤波。", output: "/livox/lidar 与整理后的 /imu/data", nodeIDs: ["mid360", "imu-transform", "imu-filter"], incomingEdgeIDs: ["mid360-imu-transform", "imu-transform-filter"], note: "该卡片中点云与 IMU 是并行支路，不是串行经过同一处理链。Topic 可见不能证明设备供电或测量质量。"),
                    .init(id: "localization-estimate", title: "估计位置与移动", input: "/livox/lidar 与 /imu/data", process: "Point-LIO 融合点云和惯性测量。", output: "/odometry、/cloud_registered 与源码定义的定位 TF", nodeIDs: ["point-lio"], incomingEdgeIDs: ["mid360-pointlio", "imu-filter-pointlio"], note: "Agent 不检查点云质量、里程计精度或 TF frame pair。"),
                    .init(id: "terrain-filter", title: "整理地形与障碍", input: "注册点云与里程计", process: "地形分析过滤导航侧点云。", output: "/terrain_map", nodeIDs: ["terrain"], incomingEdgeIDs: ["pointlio-terrain"], note: "同一 /terrain_map 还会直接提供给 Nav2 点云障碍层。"),
                    .init(id: "scan-generation", title: "生成地图扫描输入", input: "/terrain_map", process: "点云转换节点生成平面 LaserScan。", output: "/scan", nodeIDs: ["scan-conversion"], incomingEdgeIDs: ["terrain-scan"], note: "该 Launch 中 /scan 是 SLAM Toolbox 的输入，不是 Nav2 的障碍点云输入。"),
                    .init(id: "map-generation", title: "生成在线地图", input: "/scan", process: "SLAM Toolbox 更新占据栅格；独立静态变换节点提供 map → odom_init。", output: "/map 与 map → odom_init 源码接口", nodeIDs: ["slam-toolbox", "localization-map", "map-odom-tf"], incomingEdgeIDs: ["scan-slam-toolbox", "slam-map"], note: "单雷达 Launch 不启动预制 map_server；当前 Agent 不读取地图内容或实时 TF 连接。"),
                ])
            ],
            internalEdgeIDs: []
        ),
        .init(
            id: "planning",
            number: "02",
            title: "规划与决策",
            summary: "导航规划与机械臂规划是两条不同路径；P6 协调器通过软件接口请求两侧任务。",
            lanes: [
                .init(id: "navigation-planning", title: "雷达导航", summary: "地图、障碍点云与定位关系 → Nav2 路线 / 速度请求", steps: [
                    .init(id: "route-planning", title: "规划行驶路线", input: "/map、/terrain_map、定位 TF", process: "Nav2 使用静态地图层、点云障碍层和定位关系计算导航路径。", output: "经速度平滑后的 /cmd_vel 请求", nodeIDs: ["nav2"], incomingEdgeIDs: ["terrain-nav2", "map-nav2", "pointlio-tf-navigation", "map-tf-navigation"], note: "Topic 名称不表示导航 Action 已成功；Console 不发送导航目标。")
                ]),
                .init(id: "arm-planning", title: "视觉抓取规划", summary: "目标位姿 → 机械臂关节轨迹 / 命令请求", steps: [
                    .init(id: "arm-motion-planning", title: "规划机械臂动作", input: "/pose_estimator/object_pose；可选视觉建议或任务请求", process: "IK、碰撞约束与路径规划计算关节轨迹。", output: "/tzb_catch/arm_joint_command", nodeIDs: ["arm-planner"], incomingEdgeIDs: ["pose-arm-planner", "arm2nav-arm-planner", "mission-arm-request"], note: "arm2nav 和 P6 staged-task 都依赖可选配置；IK/RRTConnect/Ruckig 是源码算法，不等同于实体执行能力。")
                ]),
                .init(id: "task-coordination", title: "协调任务阶段", summary: "导航 Action 与机械臂任务接口（隔离离线验证）", steps: [
                    .init(id: "coordinate-mission", title: "协调导航与机械臂任务", input: "任务阶段、Nav2 Action 与已订阅的软件状态接口", process: "P6 协调器在离线集成路径中安排导航、机械臂请求和取消。", output: "/navigate_to_pose、/p6/arm_task/request 与取消请求", nodeIDs: ["mission-coordinator"], incomingEdgeIDs: ["mission-nav2"], note: "P6 Launch 使用 ROS_LOCALHOST_ONLY、仿真与合成手臂后端默认值；实体主启动集成尚未确认。")
                ])
            ],
            internalEdgeIDs: ["nav2-mission-feedback", "mission-arm-request", "arm-mission-feedback"]
        ),
        .init(
            id: "control",
            number: "03",
            title: "运动控制",
            summary: "主机桥接程序把软件请求编码为通信帧；Console 仅展示接口，不执行或发送控制。",
            lanes: [
                .init(id: "arm-host-bridge", title: "机械臂主机桥", summary: "关节轨迹与串口 / 状态接口", steps: [
                    .init(id: "arm-serial-bridge", title: "转换机械臂通信帧", input: "/tzb_catch/arm_joint_command 与控制器返回帧", process: "旧版 rm_serial_driver 解析 / 编码串口协议，并发布主机侧关节与安全状态 Topic。", output: "串口帧、/serial/arm_joint_state、/serial/arm_safety/status", nodeIDs: ["arm-serial", "arm-safety-status"], incomingEdgeIDs: ["arm-planner-driver", "arm-driver-safety", "arm-serial-feedback", "arm-mcu-serial-feedback"], note: "协议源码不能证明当前串口连接、固件版本或物理反馈有效。")
                ]),
                .init(id: "chassis-host-bridge", title: "底盘主机桥", summary: "速度请求与主机串口状态", steps: [
                    .init(id: "chassis-serial-bridge", title: "转换底盘串口帧", input: "/cmd_vel；P6 中止时可发布零速请求", process: "chassis_serial_bridge 编码 14 字节 USART1 帧并报告本机串口状态。", output: "串口帧与 /chassis/serial_link_status", nodeIDs: ["chassis-bridge", "chassis-link-status"], incomingEdgeIDs: ["nav2-chassis", "mission-chassis-stop", "chassis-status", "chassis-status-mission"], note: "主机写入状态不代表 STM32 已接收、底盘已移动或停止。")
                ])
            ],
            internalEdgeIDs: ["arm-driver-safety", "chassis-status"]
        ),
        .init(
            id: "execution",
            number: "04",
            title: "执行与反馈",
            summary: "控制器与安全输入只显示已定义接口边界；软件反馈和实体响应分别说明。",
            lanes: [
                .init(id: "arm-execution", title: "机械臂控制器", summary: "协议响应由主机驱动接收；不代表当前已连接", steps: [
                    .init(id: "arm-controller-endpoint", title: "机械臂控制板接口", input: "主机串口命令帧", process: "协议定义控制帧与控制器响应帧。", output: "控制器响应帧接口；当前物理连接未确认", nodeIDs: ["arm-controller"], incomingEdgeIDs: ["arm-serial-mcu-command"], note: "协议静态定义不证明已部署固件、串口归属、关节响应或电机状态。")
                ]),
                .init(id: "chassis-execution", title: "底盘控制器", summary: "现协议无 STM32 确认回包", steps: [
                    .init(id: "chassis-controller-endpoint", title: "底盘控制板接口", input: "14 字节 USART1 速度帧", process: "仓库协议定义 A5 + 3×little-endian float32 + 4A。", output: "控制板接收状态与实际运动未确认", nodeIDs: ["chassis-controller"], incomingEdgeIDs: ["chassis-bridge-mcu"], note: "该协议没有控制板 ACK；Console 不发送帧，也不推断实体响应。")
                ]),
                .init(id: "safety-boundary", title: "安全反馈边界", summary: "实体急停没有已验证的软件输入", steps: [
                    .init(id: "estop-boundary", title: "实体急停接口待接入", input: "实体急停 / 硬件安全输入", process: "当前 Agent 与 ROS 监测接口未读取该输入。", output: "无可验证连接，保持独立边界", nodeIDs: ["physical-estop"], incomingEdgeIDs: [], note: "ROS 通信与主机状态不能替代实体急停确认。")
                ])
            ],
            internalEdgeIDs: []
        )
    ]

    static let interfaces: [RobotWorkflowInterfaceGroup] = [
        .init(id: "perception-to-planning", sourcePhaseID: "perception", destinationPhaseID: "planning", title: "感知结果进入规划", edgeIDs: ["pose-arm-planner", "terrain-nav2", "map-nav2", "pointlio-tf-navigation", "map-tf-navigation", "arm2nav-arm-planner"], isFeedback: false),
        .init(id: "planning-to-control", sourcePhaseID: "planning", destinationPhaseID: "control", title: "规划请求进入主机桥", edgeIDs: ["nav2-chassis", "mission-chassis-stop", "arm-planner-driver"], isFeedback: false),
        .init(id: "control-to-execution", sourcePhaseID: "control", destinationPhaseID: "execution", title: "主机桥到控制器接口", edgeIDs: ["arm-serial-mcu-command", "chassis-bridge-mcu"], isFeedback: false),
        .init(id: "execution-to-control", sourcePhaseID: "execution", destinationPhaseID: "control", title: "控制器响应接口", edgeIDs: ["arm-mcu-serial-feedback"], isFeedback: true),
        .init(id: "control-to-planning-feedback", sourcePhaseID: "control", destinationPhaseID: "planning", title: "主机软件状态反馈", edgeIDs: ["arm-serial-feedback", "arm-safety-mission", "chassis-status-mission"], isFeedback: true),
    ]

    static func phase(_ id: String) -> RobotWorkflowPhaseDefinition? { phases.first { $0.id == id } }
    static func step(_ id: String) -> RobotWorkflowStepDefinition? { phases.lazy.flatMap(\.lanes).flatMap(\.steps).first { $0.id == id } }
    static func interface(_ id: String) -> RobotWorkflowInterfaceGroup? { interfaces.first { $0.id == id } }

    static var referencedEdgeIDs: [String] {
        phases.flatMap { phase in
            phase.lanes.flatMap { lane in lane.steps.flatMap(\.incomingEdgeIDs) } + phase.internalEdgeIDs
        } + interfaces.flatMap(\.edgeIDs)
    }
}
