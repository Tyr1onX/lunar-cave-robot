import SwiftUI

struct RobotTopologyView: View {
    @ObservedObject var model: ConsoleModel
    @State private var interaction = TopologyInteractionState()

    private var context: TopologyRuntimeContext {
        TopologyRuntimeContext(
            agentOnline: model.agentOnline,
            snapshot: model.snapshot,
            displayedWorkspaceInventory: model.workspaceInventorySnapshot,
            workspaceFreshness: model.workspaceSnapshotFreshness,
            vision: model.vision,
            linkTelemetry: model.linkTelemetry,
            healthEvents: model.healthEvents
        )
    }

    private var evaluation: TopologyEvaluation { TopologyStatusResolver.evaluate(context) }
    private var selectedSubsystem: TopologySubsystemDefinition? {
        interaction.selectedSubsystemID.flatMap(RobotTopologyCatalog.subsystem)
    }
    private var selectedFlow: TopologyFlowDefinition? {
        selectedSubsystem.flatMap { RobotTopologyCatalog.flow($0.flowID) }
    }
    private var selectedNode: TopologyNodeDefinition? {
        interaction.selectedNodeID.flatMap(RobotTopologyCatalog.node)
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            pageHeading

            if let selectedSubsystem, let selectedFlow {
                subsystemHeading(selectedSubsystem)
                detailContent(selectedFlow, subsystemID: selectedSubsystem.id)
            } else {
                overview
            }
        }
        .accessibilityIdentifier("robot-topology.view")
    }

    private var pageHeading: some View {
        HStack(alignment: .firstTextBaseline, spacing: 12) {
            Text("机器人整机链路")
                .font(.system(size: 24, weight: .semibold, design: .rounded))
            Text("从环境感知到执行机构与反馈")
                .font(.system(size: 11))
                .foregroundStyle(Palette.muted)
            Spacer()
        }
        .accessibilityIdentifier("topology.page-heading")
    }

    private var overview: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack(spacing: 7) {
                Text("整机总览").font(.system(size: 12, weight: .medium))
                Text("·").foregroundStyle(Palette.muted)
                Text("选择子系统查看内部链路").font(.system(size: 10)).foregroundStyle(Palette.muted)
                Spacer()
                evidenceLegend
            }

            GeometryReader { geometry in
                let layout = RobotTopologyPresentationLayout.overview(width: geometry.size.width)
                TopologyVerticalDiagram(
                    layout: layout,
                    nodes: overviewNodes,
                    links: overviewLinks,
                    routeSkippedEdges: false,
                    accessibilityID: "topology.overview.canvas"
                )
                .frame(width: layout.size.width, height: layout.size.height, alignment: .topLeading)
            }
            .frame(height: RobotTopologyPresentationLayout.overviewHeight)
        }
    }

    private var overviewNodes: [TopologyExperienceNode] {
        RobotTopologyPresentationLayout.overviewRows.flatMap { $0 }.compactMap { id in
            guard let subsystem = RobotTopologyCatalog.subsystem(id) else { return nil }
            let status = subsystemStatus(subsystem)
            return TopologyExperienceNode(
                id: subsystem.id,
                title: TopologyFriendlyNames.subsystem(subsystem.id),
                status: status,
                selected: false,
                isSubsystem: true,
                action: { interaction.selectSubsystem(subsystem.id) }
            )
        }
    }

    private var overviewLinks: [TopologyExperienceLink] {
        RobotTopologyCatalog.overviewEdges.compactMap { edge in
            guard RobotTopologyCatalog.subsystem(edge.sourceSubsystemID) != nil,
                  RobotTopologyCatalog.subsystem(edge.destinationSubsystemID) != nil else { return nil }
            return TopologyExperienceLink(
                id: edge.id,
                sourceID: edge.sourceSubsystemID,
                destinationID: edge.destinationSubsystemID,
                evidence: edge.evidence,
                status: TopologyStatusResolver.overviewEdgeStatus(edge, context: context, evaluation: evaluation)
            )
        }
    }

    private var evidenceLegend: some View {
        HStack(spacing: 12) {
            HStack(spacing: 5) {
                Capsule().fill(Palette.muted).frame(width: 17, height: 2)
                Text("源码关系").foregroundStyle(Palette.muted)
            }
            HStack(spacing: 5) {
                Capsule().fill(Palette.warning).frame(width: 17, height: 2)
                Text("待确认").foregroundStyle(Palette.muted)
            }
            legendStatus(Palette.accent, title: "正常")
            legendStatus(Palette.danger, title: "异常")
            legendStatus(Palette.warning, title: "未知")
        }
        .font(.system(size: 9))
        .accessibilityIdentifier("topology.evidence-legend")
    }

    private func legendStatus(_ color: Color, title: String) -> some View {
        HStack(spacing: 4) {
            Circle().fill(color).frame(width: 5, height: 5)
            Text(title).foregroundStyle(Palette.muted)
        }
    }

    private func subsystemHeading(_ subsystem: TopologySubsystemDefinition) -> some View {
        HStack(spacing: 10) {
            Button {
                interaction.showOverview()
            } label: {
                Image(systemName: "chevron.left")
                    .font(.system(size: 11, weight: .semibold))
                    .foregroundStyle(Palette.muted)
                    .frame(width: 28, height: 28)
                    .contentShape(Rectangle())
            }
            .buttonStyle(.plain)
            .help("返回整机总览")
            .accessibilityIdentifier("topology.breadcrumb.overview")

            Image(systemName: subsystem.symbol)
                .font(.system(size: 15, weight: .medium))
                .foregroundStyle(Palette.accent)
            Text(TopologyFriendlyNames.subsystem(subsystem.id))
                .font(.system(size: 18, weight: .semibold, design: .rounded))
            TopologyExperienceStatus(status: subsystemStatus(subsystem), compact: true)
            Spacer()
        }
        .accessibilityIdentifier("topology.detail.\(subsystem.id)")
    }

    @ViewBuilder
    private func detailContent(_ flow: TopologyFlowDefinition, subsystemID: String) -> some View {
        let graph = detailDiagram(flow, subsystemID: subsystemID)
        if let selectedNode {
            GeometryReader { geometry in
                if TopologyInspectorPlacement.usesTrailingPane(width: geometry.size.width) {
                    HStack(alignment: .top, spacing: 20) {
                        graph.frame(maxWidth: .infinity, alignment: .top)
                        nodeEvidence(selectedNode)
                            .frame(width: 370, alignment: .topLeading)
                    }
                    .frame(maxWidth: .infinity, alignment: .topLeading)
                } else {
                    VStack(alignment: .leading, spacing: 18) {
                        graph
                        nodeEvidence(selectedNode)
                    }
                    .frame(maxWidth: .infinity, alignment: .topLeading)
                }
            }
            .frame(height: max(detailHeight(flow), selectedNodeEvidenceMinimumHeight))
        } else {
            graph
        }
    }

    private var selectedNodeEvidenceMinimumHeight: CGFloat { 510 }

    private func detailDiagram(_ flow: TopologyFlowDefinition, subsystemID: String) -> some View {
        let flowNodes = Set(flow.nodeIDs)
        let nodes = flow.nodeIDs.compactMap { id -> TopologyExperienceNode? in
            guard RobotTopologyCatalog.node(id) != nil else { return nil }
            return TopologyExperienceNode(
                id: id,
                title: TopologyFriendlyNames.node(id),
                status: evaluation.nodeStatus(id),
                selected: interaction.selectedNodeID == id,
                isSubsystem: false,
                action: { interaction.selectNode(id, in: subsystemID) }
            )
        }
        let links = flow.edgeIDs.compactMap { id -> TopologyExperienceLink? in
            guard let edge = RobotTopologyCatalog.edge(id),
                  flowNodes.contains(edge.sourceNodeID),
                  flowNodes.contains(edge.destinationNodeID) else { return nil }
            return TopologyExperienceLink(
                id: id,
                sourceID: edge.sourceNodeID,
                destinationID: edge.destinationNodeID,
                evidence: edge.evidence,
                status: evaluation.edgeStatus(id)
            )
        }
        let layoutHeight = detailHeight(flow)

        return VStack(alignment: .leading, spacing: 8) {
            Text(flow.title)
                .font(.system(size: 11, weight: .medium))
                .foregroundStyle(Palette.muted)
            GeometryReader { geometry in
                let layout = RobotTopologyPresentationLayout.detail(flow, width: geometry.size.width)
                TopologyVerticalDiagram(
                    layout: layout,
                    nodes: nodes,
                    links: links,
                    routeSkippedEdges: true,
                    accessibilityID: "topology.detail.canvas.\(flow.id)"
                )
                .frame(width: layout.size.width, height: layout.size.height, alignment: .topLeading)
            }
            .frame(height: layoutHeight)
        }
        .accessibilityIdentifier("topology.diagram.\(flow.id)")
    }

    private func detailHeight(_ flow: TopologyFlowDefinition) -> CGFloat {
        RobotTopologyPresentationLayout.detailHeight(nodeCount: flow.nodeIDs.count)
    }

    private func nodeEvidence(_ node: TopologyNodeDefinition) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            Text(TopologyFriendlyNames.node(node.id))
                .font(.system(size: 14, weight: .semibold, design: .rounded))
            TopologyNodeEvidencePanel(
                node: node,
                status: evaluation.nodeStatus(node.id),
                context: context,
                linkTelemetryMessage: model.linkTelemetryMessage,
                onClose: { interaction.clearSelection() }
            )
        }
        .accessibilityIdentifier("topology.node-evidence")
    }

    private func subsystemStatus(_ subsystem: TopologySubsystemDefinition) -> TopologyElementStatus {
        let statuses = subsystem.statusNodeIDs.map { evaluation.nodeStatus($0) }
        let events = statuses.flatMap(\.directEvents)
        let affected = statuses.contains(where: \.possiblyAffectedDownstream)
        if let abnormal = statuses.first(where: { $0.signal == .abnormal }) {
            return TopologyElementStatus(signal: .abnormal, note: abnormal.note, directEvents: events, possiblyAffectedDownstream: false)
        }
        if let staleStream = RobotTopologyCatalog.flow(subsystem.flowID)?.edgeIDs
            .compactMap({ evaluation.edgeStatus($0) })
            .first(where: { $0.signal == .dataStale }) {
            return TopologyElementStatus(
                signal: .dataStale,
                note: "直接观测到数据流暂未更新；原因和工作区来源未知，不判定模块故障。\n\(staleStream.note)",
                directEvents: events,
                possiblyAffectedDownstream: true
            )
        }
        if statuses.contains(where: { $0.signal == .unknown }) {
            return TopologyElementStatus(signal: .unknown, note: "子系统存在未知或未确认来源的组件状态。", directEvents: events, possiblyAffectedDownstream: affected)
        }
        if statuses.allSatisfy({ $0.signal == .normal }) {
            return TopologyElementStatus(signal: .normal, note: "现有监测项均返回正常；不据此推断实体硬件健康。", directEvents: events, possiblyAffectedDownstream: affected)
        }
        if statuses.contains(where: { $0.signal == .processNotFound }) {
            return TopologyElementStatus(signal: .processNotFound, note: "未发现部分关联进程；没有明确运行预期时不视为故障。", directEvents: events, possiblyAffectedDownstream: affected)
        }
        if statuses.contains(where: { $0.signal == .processObserved }) {
            return TopologyElementStatus(signal: .processObserved, note: "发现部分关联进程，模块接口与硬件健康尚未确认。", directEvents: events, possiblyAffectedDownstream: affected)
        }
        return TopologyElementStatus(signal: .unknown, note: "子系统证据不足。", directEvents: events, possiblyAffectedDownstream: affected)
    }
}

enum TopologyFriendlyNames {
    static func subsystem(_ id: String) -> String {
        switch id {
        case "perception": return "视觉感知"
        case "localization": return "雷达定位与建图"
        case "coordination": return "任务协调"
        case "navigation": return "自动导航"
        case "arm": return "机械臂"
        case "chassis": return "底盘运动"
        case "safety": return "安全反馈"
        default: return RobotTopologyCatalog.subsystem(id)?.title ?? id
        }
    }

    static func node(_ id: String) -> String {
        switch id {
        case "camera": return "采集彩色与深度图像"
        case "yolo": return "识别目标"
        case "arm2nav": return "生成导航建议"
        case "pose-estimator": return "估计目标位置"
        case "lidar-stack": return "雷达与定位进程"
        case "mid360": return "采集雷达点云与 IMU"
        case "imu-transform": return "转换 IMU 坐标"
        case "imu-filter": return "融合 IMU 数据"
        case "point-lio": return "判断机器人位置"
        case "terrain": return "分析地形与障碍"
        case "scan-conversion": return "生成扫描数据"
        case "slam-toolbox": return "生成地图"
        case "localization-map": return "发布在线地图"
        case "nav2": return "自动规划导航路线"
        case "mission-coordinator": return "协调导航与机械臂任务"
        case "arm-stack": return "机械臂软件进程"
        case "arm-planner": return "规划机械臂动作"
        case "arm-serial": return "连接机械臂控制板"
        case "arm-controller": return "机械臂控制板"
        case "arm-safety-status": return "机械臂主机安全状态"
        case "chassis-bridge": return "发送底盘指令"
        case "chassis-controller": return "底盘控制板"
        case "chassis-link-status": return "底盘主机串口状态"
        case "physical-estop": return "实体急停反馈"
        case "agent": return "只读 Agent"
        case "ros-domain": return "ROS 通信域"
        case "ros-graph": return "ROS 节点与数据目录"
        case "expected-nodes": return "预期节点集合"
        default: return RobotTopologyCatalog.node(id)?.title ?? id
        }
    }
}

enum TopologyInspectorPlacement {
    static func usesTrailingPane(width: CGFloat) -> Bool { width >= 1_050 }
}

struct TopologyVerticalCanvasLayout: Equatable {
    let size: CGSize
    let nodeFrames: [String: CGRect]
}

enum RobotTopologyPresentationLayout {
    static let overviewRows = [
        ["perception", "localization"],
        ["coordination"],
        ["arm", "navigation"],
        ["chassis"],
        ["safety"],
    ]

    static let overviewHeight: CGFloat = 450
    private static let cardHeight: CGFloat = 58
    private static let verticalGap: CGFloat = 36

    static func overview(width: CGFloat) -> TopologyVerticalCanvasLayout {
        let safeWidth = max(320, width)
        let cardWidth = min(390, max(160, (safeWidth - 48) / 2))
        let originY: [String: CGFloat] = [
            "perception": 14, "localization": 14,
            "coordination": 106,
            "arm": 198, "navigation": 198,
            "chassis": 290,
            "safety": 382,
        ]
        let centerX: [String: CGFloat] = [
            "perception": safeWidth * 0.28,
            "localization": safeWidth * 0.72,
            "coordination": safeWidth * 0.28,
            "arm": safeWidth * 0.28,
            "navigation": safeWidth * 0.72,
            "chassis": safeWidth * 0.72,
            "safety": safeWidth * 0.50,
        ]
        let frames = Dictionary(uniqueKeysWithValues: overviewRows.flatMap { $0 }.compactMap { id -> (String, CGRect)? in
            guard let x = centerX[id], let y = originY[id] else { return nil }
            return (id, CGRect(x: x - cardWidth / 2, y: y, width: cardWidth, height: cardHeight))
        })
        return TopologyVerticalCanvasLayout(size: CGSize(width: safeWidth, height: overviewHeight), nodeFrames: frames)
    }

    static func detail(_ flow: TopologyFlowDefinition, width: CGFloat) -> TopologyVerticalCanvasLayout {
        let nodeIDs = flow.nodeIDs
        let nodeSet = Set(nodeIDs)
        let edges = flow.edgeIDs.compactMap(RobotTopologyCatalog.edge).filter {
            nodeSet.contains($0.sourceNodeID) && nodeSet.contains($0.destinationNodeID)
        }
        let order = topologicalOrder(nodeIDs: nodeIDs, edges: edges)
        let rows = order.map { [$0] }
        let height = detailHeight(nodeCount: rows.count)
        return makeLayout(rows: rows, width: max(320, width), height: height, maxColumns: 1, cardHeight: cardHeight, verticalGap: verticalGap, maxCardWidth: 570)
    }

    static func detailHeight(nodeCount: Int) -> CGFloat {
        max(260, CGFloat(max(1, nodeCount)) * (cardHeight + verticalGap) + 10)
    }

    private static func topologicalOrder(nodeIDs: [String], edges: [TopologyEdgeDefinition]) -> [String] {
        let orderIndex = Dictionary(uniqueKeysWithValues: nodeIDs.enumerated().map { ($0.element, $0.offset) })
        var remaining = Set(nodeIDs)
        var result: [String] = []
        while !remaining.isEmpty {
            let ready = remaining.filter { nodeID in
                !edges.contains { $0.destinationNodeID == nodeID && remaining.contains($0.sourceNodeID) }
            }.sorted { (orderIndex[$0] ?? 0) < (orderIndex[$1] ?? 0) }
            guard !ready.isEmpty else {
                result.append(contentsOf: remaining.sorted { (orderIndex[$0] ?? 0) < (orderIndex[$1] ?? 0) })
                break
            }
            result.append(contentsOf: ready)
            remaining.subtract(ready)
        }
        return result
    }

    private static func makeLayout(
        rows: [[String]],
        width: CGFloat,
        height: CGFloat,
        maxColumns: Int,
        cardHeight: CGFloat,
        verticalGap: CGFloat,
        maxCardWidth: CGFloat
    ) -> TopologyVerticalCanvasLayout {
        let horizontalInset: CGFloat = 16
        let columnGap: CGFloat = 18
        var frames: [String: CGRect] = [:]
        let contentRows = rows.filter { !$0.isEmpty }
        let totalRowsHeight = CGFloat(contentRows.count) * cardHeight + CGFloat(max(0, contentRows.count - 1)) * verticalGap
        let topInset = max(10, (height - totalRowsHeight) / 2)

        for (rowIndex, row) in contentRows.enumerated() {
            let columns = min(maxColumns, row.count)
            let usableWidth = max(220, width - horizontalInset * 2 - CGFloat(max(0, columns - 1)) * columnGap)
            let cardWidth = min(maxCardWidth, usableWidth / CGFloat(max(1, columns)))
            let rowWidth = CGFloat(columns) * cardWidth + CGFloat(max(0, columns - 1)) * columnGap
            let firstX = (width - rowWidth) / 2
            let y = topInset + CGFloat(rowIndex) * (cardHeight + verticalGap)
            for (column, id) in row.enumerated() where column < columns {
                frames[id] = CGRect(x: firstX + CGFloat(column) * (cardWidth + columnGap), y: y, width: cardWidth, height: cardHeight)
            }
        }
        return TopologyVerticalCanvasLayout(size: CGSize(width: width, height: height), nodeFrames: frames)
    }
}

private struct TopologyExperienceNode {
    let id: String
    let title: String
    let status: TopologyElementStatus
    let selected: Bool
    let isSubsystem: Bool
    let action: () -> Void
}

private struct TopologyExperienceLink: Identifiable {
    let id: String
    let sourceID: String
    let destinationID: String
    let evidence: TopologyEvidenceGrade
    let status: TopologyElementStatus
}

private struct TopologyVerticalDiagram: View {
    let layout: TopologyVerticalCanvasLayout
    let nodes: [TopologyExperienceNode]
    let links: [TopologyExperienceLink]
    let routeSkippedEdges: Bool
    let accessibilityID: String

    var body: some View {
        ZStack(alignment: .topLeading) {
            Canvas { context, _ in
                for (index, link) in links.enumerated() {
                    guard let source = layout.nodeFrames[link.sourceID],
                          let destination = layout.nodeFrames[link.destinationID] else { continue }
                    draw(link, index: index, from: source, to: destination, in: &context)
                }
            }

            ForEach(nodes, id: \.id) { node in
                if let frame = layout.nodeFrames[node.id] {
                    TopologyExperienceNodeButton(node: node)
                        .frame(width: frame.width, height: frame.height)
                        .position(x: frame.midX, y: frame.midY)
                }
            }
        }
        .frame(width: layout.size.width, height: layout.size.height, alignment: .topLeading)
        .accessibilityIdentifier(accessibilityID)
    }

    private func draw(_ link: TopologyExperienceLink, index: Int, from source: CGRect, to destination: CGRect, in context: inout GraphicsContext) {
        let color = topologyColor(link.status.signal)
        var path = Path()
        let start = CGPoint(x: source.midX, y: source.maxY)
        let end = CGPoint(x: destination.midX, y: destination.minY)
        path.move(to: start)

        if !routeSkippedEdges || destination.minY - source.maxY <= 76 {
            path.addCurve(
                to: end,
                control1: CGPoint(x: start.x, y: start.y + 12),
                control2: CGPoint(x: end.x, y: end.y - 12)
            )
        } else {
            let laneInset = CGFloat(index % 3 + 1) * 8
            let laneX = max(4, source.minX - laneInset)
            let firstBend = CGPoint(x: laneX, y: start.y + 13)
            let lastBend = CGPoint(x: laneX, y: end.y - 13)
            path.addCurve(to: firstBend, control1: CGPoint(x: start.x, y: start.y + 5), control2: CGPoint(x: laneX, y: start.y + 6))
            path.addLine(to: lastBend)
            path.addCurve(to: end, control1: CGPoint(x: laneX, y: end.y - 5), control2: CGPoint(x: end.x, y: end.y - 7))
        }

        let dash: [CGFloat]
        switch link.evidence {
        case .sourceWiring: dash = []
        case .offlineExercise: dash = [5, 4]
        case .openInterface: dash = [2, 4]
        }
        context.stroke(path, with: .color(color.opacity(0.9)), style: StrokeStyle(lineWidth: link.status.signal == .abnormal ? 2.6 : 1.5, lineCap: .round, dash: dash))

        let arrow = Path { arrow in
            arrow.move(to: end)
            arrow.addLine(to: CGPoint(x: end.x - 4.5, y: end.y - 7))
            arrow.addLine(to: CGPoint(x: end.x + 4.5, y: end.y - 7))
            arrow.closeSubpath()
        }
        context.fill(arrow, with: .color(color))
    }
}

private struct TopologyExperienceNodeButton: View {
    let node: TopologyExperienceNode

    var body: some View {
        Button(action: node.action) {
            HStack(spacing: 11) {
                Image(systemName: node.isSubsystem ? subsystemSymbol(node.id) : nodeSymbol(node.id))
                    .font(.system(size: 13, weight: .medium))
                    .foregroundStyle(node.status.signal == .abnormal ? Palette.danger : Palette.accent)
                    .frame(width: 20)
                Text(node.title)
                    .font(.system(size: node.isSubsystem ? 12 : 11, weight: .semibold))
                    .foregroundStyle(.white)
                    .lineLimit(2)
                    .multilineTextAlignment(.leading)
                Spacer(minLength: 4)
                TopologyExperienceStatus(status: node.status, compact: false)
            }
            .padding(.horizontal, 13)
            .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .leading)
            .background(node.selected ? Palette.accent.opacity(0.13) : Palette.panel.opacity(0.88), in: RoundedRectangle(cornerRadius: 12))
            .overlay(alignment: .leading) {
                if node.status.signal == .abnormal {
                    Capsule().fill(Palette.danger).frame(width: 3, height: 28).padding(.leading, 2)
                } else if node.selected {
                    Capsule().fill(Palette.accent).frame(width: 3, height: 28).padding(.leading, 2)
                }
            }
        }
        .buttonStyle(.plain)
        .help(node.status.note)
        .accessibilityIdentifier(node.isSubsystem ? "topology.subsystem.\(node.id)" : "topology.node.\(node.id)")
    }

    private func subsystemSymbol(_ id: String) -> String {
        RobotTopologyCatalog.subsystem(id)?.symbol ?? "circle"
    }

    private func nodeSymbol(_ id: String) -> String {
        if RobotTopologyCatalog.node(id)?.isExternalHardware == true { return "cpu" }
        switch id {
        case "camera": return "camera"
        case "yolo", "pose-estimator", "arm2nav": return "viewfinder"
        case "mid360": return "dot.radiowaves.left.and.right"
        case "nav2", "mission-coordinator": return "point.3.connected.trianglepath.dotted"
        case "arm-controller", "chassis-controller": return "cpu"
        default: return "circle.grid.2x2"
        }
    }
}

private struct TopologyExperienceStatus: View {
    let status: TopologyElementStatus
    let compact: Bool

    var body: some View {
        HStack(spacing: 5) {
            Circle().fill(topologyColor(status.signal)).frame(width: 6, height: 6)
            Text(shortLabel(status.signal))
                .font(.system(size: compact ? 9 : 8, weight: .medium))
                .foregroundStyle(topologyColor(status.signal))
                .lineLimit(1)
            if status.possiblyAffectedDownstream {
                Text("可能受影响")
                    .font(.system(size: 8))
                    .foregroundStyle(Palette.warning)
                    .lineLimit(1)
            }
        }
        .accessibilityLabel(status.note)
    }

    private func shortLabel(_ signal: TopologySignal) -> String {
        switch signal {
        case .normal: return "正常"
        case .abnormal: return "异常"
        case .unknown: return "未知"
        case .notRunning: return "未运行"
        case .processNotFound: return "未发现进程"
        case .processObserved: return "已发现进程"
        case .topicsAllVisible: return "Topic 可见"
        case .topicsPartiallyVisible: return "部分可见"
        case .topicsNoneVisible: return "Topic 未见"
        case .dataObserved: return "有数据"
        case .dataPartiallyObserved: return "部分数据"
        case .dataStale: return "数据过期"
        case .dataNotObserved: return "尚无数据"
        }
    }
}

private func topologyColor(_ signal: TopologySignal) -> Color {
    switch signal {
    case .normal: return Palette.accent
    case .abnormal: return Palette.danger
    case .unknown, .notRunning, .processNotFound, .dataPartiallyObserved, .dataStale: return Palette.warning
    case .processObserved, .topicsAllVisible, .topicsPartiallyVisible, .dataObserved: return Palette.blue
    case .topicsNoneVisible, .dataNotObserved: return Palette.muted
    }
}
