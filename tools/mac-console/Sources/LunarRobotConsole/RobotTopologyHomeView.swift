import SwiftUI

struct AdvancedRobotTopologyView: View {
    @ObservedObject var model: ConsoleModel
    let onReturn: () -> Void
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
    private var selectedNode: TopologyNodeDefinition? {
        interaction.selectedNodeID.flatMap(RobotTopologyCatalog.node)
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            pageHeading
            HStack(spacing: 8) {
                Text("点击子系统原位展开；点击模块查看证据")
                Text("·").foregroundStyle(Palette.muted)
                evidenceLegend
                Spacer()
                if !interaction.expandedSubsystemIDs.isEmpty {
                    Button("全部收起") { interaction.collapseAll() }
                        .font(.system(size: 10, weight: .medium))
                        .buttonStyle(.plain)
                        .foregroundStyle(Palette.muted)
                        .accessibilityIdentifier("topology.collapse-all")
                }
            }
            .font(.system(size: 10))
            .foregroundStyle(Palette.muted)
            topologyGraph
        }
        .accessibilityIdentifier("robot-topology.view")
    }

    private var pageHeading: some View {
        HStack(alignment: .firstTextBaseline, spacing: 12) {
            Button(action: onReturn) {
                Label("能力首页", systemImage: "chevron.left")
                    .font(.system(size: 10, weight: .medium))
                    .foregroundStyle(Palette.muted)
            }
            .buttonStyle(.plain)
            .accessibilityIdentifier("topology.advanced.return")
            Text("机器人整机链路")
                .font(.system(size: 24, weight: .semibold, design: .rounded))
            Text("从环境感知到执行机构与反馈")
                .font(.system(size: 11))
                .foregroundStyle(Palette.muted)
            Spacer()
        }
        .accessibilityIdentifier("topology.page-heading")
    }

    private var topologyGraph: some View {
        let projection = RobotTopologyCatalog.project(expandedSubsystemIDs: interaction.expandedSubsystemIDs)
        let nodes = experienceNodes(projection)
        let links = experienceLinks(projection)
        let headers = expandedHeaders(projection)
        let graphHeight = RobotTopologyPresentationLayout.graphHeight(expandedSubsystemIDs: interaction.expandedSubsystemIDs)

        return TopologyAdaptiveInspectorLayout(
            expandedSubsystemIDs: interaction.expandedSubsystemIDs,
            hasInspector: selectedNode != nil
        ) {
            TopologyGraphCanvas(
                expandedSubsystemIDs: interaction.expandedSubsystemIDs,
                nodes: nodes,
                expandedHeaders: headers,
                links: links,
                height: graphHeight
            )
            if let selectedNode {
                nodeEvidence(selectedNode)
            }
        }
        .frame(maxWidth: .infinity, alignment: .topLeading)
    }

    private func experienceNodes(_ projection: TopologyGraphProjection) -> [TopologyExperienceNode] {
        projection.nodes.compactMap { projected in
            if projected.isSubsystem {
                guard let subsystem = RobotTopologyCatalog.subsystem(projected.subsystemID) else { return nil }
                return TopologyExperienceNode(
                    id: projected.id,
                    title: TopologyFriendlyNames.subsystem(subsystem.id),
                    status: subsystemStatus(subsystem),
                    selected: false,
                    isSubsystem: true,
                    action: { interaction.toggleSubsystem(subsystem.id) }
                )
            }
            guard let nodeID = projected.moduleNodeID,
                  RobotTopologyCatalog.node(nodeID) != nil else { return nil }
            return TopologyExperienceNode(
                id: projected.id,
                title: TopologyFriendlyNames.node(nodeID),
                status: evaluation.nodeStatus(nodeID),
                selected: interaction.selectedNodeID == nodeID,
                isSubsystem: false,
                action: { interaction.selectNode(nodeID) }
            )
        }
    }

    private func expandedHeaders(_ projection: TopologyGraphProjection) -> [TopologyExperienceSubsystemHeader] {
        interaction.expandedSubsystemIDs.compactMap { id in
            guard let subsystem = RobotTopologyCatalog.subsystem(id),
                  projection.nodes.contains(where: { $0.subsystemID == id }) else { return nil }
            return TopologyExperienceSubsystemHeader(
                id: id,
                title: TopologyFriendlyNames.subsystem(id),
                symbol: subsystem.symbol,
                status: subsystemStatus(subsystem),
                action: { interaction.toggleSubsystem(id) }
            )
        }
    }

    private func experienceLinks(_ projection: TopologyGraphProjection) -> [TopologyExperienceLink] {
        projection.edges.compactMap { projected in
            guard let source = RobotTopologyCatalog.edge(projected.id) else { return nil }
            return TopologyExperienceLink(
                id: projected.id,
                sourceID: projected.sourceID,
                destinationID: projected.destinationID,
                evidence: projected.evidence,
                isFeedback: projected.isFeedback,
                status: evaluation.edgeStatus(source.id)
            )
        }
    }

    private var evidenceLegend: some View {
        HStack(spacing: 8) {
            evidenceLine(title: "源码", evidence: .sourceWiring)
            evidenceLine(title: "离线验证", evidence: .offlineExercise)
            evidenceLine(title: "可选模式", evidence: .conditionalSourceWiring)
            evidenceLine(title: "待核实", evidence: .openInterface)
            legendStatus(Palette.accent, title: "正常")
            legendStatus(Palette.danger, title: "异常")
            legendStatus(Palette.warning, title: "未知")
        }
        .font(.system(size: 9))
        .accessibilityIdentifier("topology.evidence-legend")
    }

    private func evidenceLine(title: String, evidence: TopologyEvidenceGrade) -> some View {
        HStack(spacing: 4) {
            switch evidence {
            case .sourceWiring:
                Capsule().fill(Palette.muted).frame(width: 13, height: 2)
            case .offlineExercise:
                Capsule().stroke(Palette.muted, style: StrokeStyle(lineWidth: 1.5, dash: [4, 3])).frame(width: 13, height: 2)
            case .conditionalSourceWiring:
                Capsule().stroke(Palette.warning, style: StrokeStyle(lineWidth: 1.5, dash: [2, 2])).frame(width: 13, height: 2)
            case .openInterface:
                Capsule().stroke(Palette.muted, style: StrokeStyle(lineWidth: 1.5, dash: [1, 3])).frame(width: 13, height: 2)
            }
            Text(title).foregroundStyle(Palette.muted)
        }
    }

    private func legendStatus(_ color: Color, title: String) -> some View {
        HStack(spacing: 4) {
            Circle().fill(color).frame(width: 5, height: 5)
            Text(title).foregroundStyle(Palette.muted)
        }
    }

    private func nodeEvidence(_ node: TopologyNodeDefinition) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            Text(TopologyFriendlyNames.node(node.id))
                .font(.system(size: 14, weight: .semibold, design: .rounded))
            TopologyNodeEvidencePanel(
                node: node,
                status: evaluation.nodeStatus(node.id),
                evaluation: evaluation,
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
        let subsystemNodeIDs = Set(RobotTopologyCatalog.nodes.filter { $0.groupID == subsystem.id }.map(\.id))
        let incidentEdgeStatuses = RobotTopologyCatalog.edges
            .filter { subsystemNodeIDs.contains($0.sourceNodeID) || subsystemNodeIDs.contains($0.destinationNodeID) }
            .map { evaluation.edgeStatus($0.id) }
        if let staleStream = incidentEdgeStatuses.first(where: { $0.signal == .dataStale }) {
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
        case "safety": return "未接入安全输入"
        default: return RobotTopologyCatalog.subsystem(id)?.title ?? id
        }
    }

    static func node(_ id: String) -> String {
        switch id {
        case "camera": return "采集彩色与深度图像"
        case "yolo": return "识别目标"
        case "arm2nav": return "生成导航建议"
        case "pose-estimator": return "估计目标位置"
        case "lidar-stack": return "雷达软件进程扫描"
        case "mid360": return "采集雷达点云与 IMU"
        case "imu-transform": return "转换 IMU 坐标"
        case "imu-filter": return "融合 IMU 数据"
        case "point-lio": return "判断机器人位置"
        case "terrain": return "分析地形与障碍"
        case "scan-conversion": return "生成扫描数据"
        case "slam-toolbox": return "生成地图"
        case "map-odom-tf": return "发布地图坐标变换"
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
        case "chassis-link-status": return "读取底盘主机串口状态"
        case "physical-estop": return "实体急停输入（未接入）"
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
    let expandedHeaderFrames: [String: CGRect]
}

enum RobotTopologyPresentationLayout {
    private static let subsystemPriority = ["perception", "localization", "coordination", "navigation", "arm", "chassis", "safety"]
    private static let nodeHeight: CGFloat = 54
    private static let groupHeight: CGFloat = 62
    private static let groupHeaderHeight: CGFloat = 28
    private static let moduleGap: CGFloat = 22
    private static let groupGap: CGFloat = 30

    static func graphHeight(expandedSubsystemIDs: Set<String>) -> CGFloat {
        var height: CGFloat = 18
        for subsystemID in orderedSubsystemIDs() {
            if expandedSubsystemIDs.contains(subsystemID) {
                let count = moduleOrder(subsystemID).count
                height += groupHeaderHeight + 11
                height += CGFloat(count) * nodeHeight + CGFloat(max(0, count - 1)) * moduleGap
            } else {
                height += groupHeight
            }
            height += groupGap
        }
        return max(260, height - groupGap + 18)
    }

    static func graph(width: CGFloat, expandedSubsystemIDs: Set<String>) -> TopologyVerticalCanvasLayout {
        let safeWidth = max(1, width)
        let cardWidth = min(600, max(240, safeWidth - 32))
        let x = (safeWidth - cardWidth) / 2
        var nodeFrames: [String: CGRect] = [:]
        var headerFrames: [String: CGRect] = [:]
        var y: CGFloat = 18

        for subsystemID in orderedSubsystemIDs() {
            if expandedSubsystemIDs.contains(subsystemID) {
                headerFrames[subsystemID] = CGRect(x: x, y: y, width: cardWidth, height: groupHeaderHeight)
                y += groupHeaderHeight + 11
                for nodeID in moduleOrder(subsystemID) {
                    nodeFrames[nodeID] = CGRect(x: x, y: y, width: cardWidth, height: nodeHeight)
                    y += nodeHeight + moduleGap
                }
                y -= moduleGap
            } else {
                nodeFrames[subsystemID] = CGRect(x: x, y: y, width: cardWidth, height: groupHeight)
                y += groupHeight
            }
            y += groupGap
        }

        return TopologyVerticalCanvasLayout(
            size: CGSize(width: safeWidth, height: max(260, y - groupGap + 18)),
            nodeFrames: nodeFrames,
            expandedHeaderFrames: headerFrames
        )
    }

    static func orderedSubsystemIDs() -> [String] {
        let ids = RobotTopologyCatalog.subsystemIDs
        let orderIndex = Dictionary(uniqueKeysWithValues: subsystemPriority.enumerated().map { ($0.element, $0.offset) })
        let visibleEdges = RobotTopologyCatalog.edges.filter { edge in
            guard !edge.isFeedback,
                  edge.evidence != .openInterface else { return false }
            guard let source = RobotTopologyCatalog.node(edge.sourceNodeID),
                  let destination = RobotTopologyCatalog.node(edge.destinationNodeID) else { return false }
            return source.groupID != destination.groupID && ids.contains(source.groupID) && ids.contains(destination.groupID)
        }
        var remaining = Set(ids)
        var result: [String] = []
        while !remaining.isEmpty {
            var ready = remaining.filter { id in
                !visibleEdges.contains { edge in
                    guard let destination = RobotTopologyCatalog.node(edge.destinationNodeID),
                          destination.groupID == id,
                          let source = RobotTopologyCatalog.node(edge.sourceNodeID) else { return false }
                    return remaining.contains(source.groupID)
                }
            }.sorted { lhs, rhs in
                if lhs == "safety" { return false }
                if rhs == "safety" { return true }
                return (orderIndex[lhs] ?? 0) < (orderIndex[rhs] ?? 0)
            }
            if remaining.count > 1 {
                ready.removeAll { $0 == "safety" }
            }
            guard !ready.isEmpty else {
                result.append(contentsOf: remaining.sorted { (orderIndex[$0] ?? 0) < (orderIndex[$1] ?? 0) })
                break
            }
            result.append(contentsOf: ready)
            remaining.subtract(ready)
        }
        return result
    }

    static func moduleOrder(_ subsystemID: String) -> [String] {
        let nodeIDs = RobotTopologyCatalog.nodes.filter { $0.groupID == subsystemID }.map(\.id)
        let nodeSet = Set(nodeIDs)
        let edges = RobotTopologyCatalog.edges.filter {
            nodeSet.contains($0.sourceNodeID) && nodeSet.contains($0.destinationNodeID) && !$0.isFeedback
        }
        let orderIndex = Dictionary(uniqueKeysWithValues: nodeIDs.enumerated().map { ($0.element, $0.offset) })
        var remaining = Set(nodeIDs)
        var result: [String] = []
        while !remaining.isEmpty {
            let ready = remaining.filter { id in
                !edges.contains { $0.destinationNodeID == id && remaining.contains($0.sourceNodeID) }
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
}

private struct TopologyExperienceNode {
    let id: String
    let title: String
    let status: TopologyElementStatus
    let selected: Bool
    let isSubsystem: Bool
    let action: () -> Void
}

private struct TopologyExperienceSubsystemHeader: Identifiable {
    let id: String
    let title: String
    let symbol: String
    let status: TopologyElementStatus
    let action: () -> Void
}

private struct TopologyExperienceLink: Identifiable {
    let id: String
    let sourceID: String
    let destinationID: String
    let evidence: TopologyEvidenceGrade
    let isFeedback: Bool
    let status: TopologyElementStatus
}

private struct TopologyGraphCanvas: View {
    let expandedSubsystemIDs: Set<String>
    let nodes: [TopologyExperienceNode]
    let expandedHeaders: [TopologyExperienceSubsystemHeader]
    let links: [TopologyExperienceLink]
    let height: CGFloat

    var body: some View {
        GeometryReader { geometry in
            let layout = RobotTopologyPresentationLayout.graph(
                width: geometry.size.width,
                expandedSubsystemIDs: expandedSubsystemIDs
            )
            TopologyVerticalDiagram(
                layout: layout,
                nodes: nodes,
                expandedHeaders: expandedHeaders,
                links: links,
                accessibilityID: "topology.recursive-graph"
            )
            .frame(width: layout.size.width, height: layout.size.height, alignment: .topLeading)
        }
        .frame(height: height)
    }
}

private struct TopologyAdaptiveInspectorLayout: Layout {
    let expandedSubsystemIDs: Set<String>
    let hasInspector: Bool

    func sizeThatFits(proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) -> CGSize {
        let width = max(320, proposal.width ?? 1_200)
        let graphHeight = RobotTopologyPresentationLayout.graphHeight(expandedSubsystemIDs: expandedSubsystemIDs)
        guard hasInspector, subviews.count > 1 else { return CGSize(width: width, height: graphHeight) }

        if TopologyInspectorPlacement.usesTrailingPane(width: width) {
            let inspectorHeight = subviews[1].sizeThatFits(ProposedViewSize(width: 370, height: nil)).height
            return CGSize(width: width, height: max(graphHeight, inspectorHeight))
        }
        let inspectorHeight = subviews[1].sizeThatFits(ProposedViewSize(width: width, height: nil)).height
        return CGSize(width: width, height: graphHeight + 18 + inspectorHeight)
    }

    func placeSubviews(in bounds: CGRect, proposal: ProposedViewSize, subviews: Subviews, cache: inout ()) {
        guard let graph = subviews.first else { return }
        let width = max(320, bounds.width)
        let graphHeight = RobotTopologyPresentationLayout.graphHeight(expandedSubsystemIDs: expandedSubsystemIDs)
        guard hasInspector, subviews.count > 1 else {
            graph.place(
                at: bounds.origin,
                anchor: .topLeading,
                proposal: ProposedViewSize(width: width, height: graphHeight)
            )
            return
        }

        let inspector = subviews[1]
        if TopologyInspectorPlacement.usesTrailingPane(width: width) {
            let graphWidth = max(320, width - 390)
            graph.place(
                at: bounds.origin,
                anchor: .topLeading,
                proposal: ProposedViewSize(width: graphWidth, height: graphHeight)
            )
            inspector.place(
                at: CGPoint(x: bounds.minX + width - 370, y: bounds.minY),
                anchor: .topLeading,
                proposal: ProposedViewSize(width: 370, height: bounds.height)
            )
        } else {
            graph.place(
                at: bounds.origin,
                anchor: .topLeading,
                proposal: ProposedViewSize(width: width, height: graphHeight)
            )
            inspector.place(
                at: CGPoint(x: bounds.minX, y: bounds.minY + graphHeight + 18),
                anchor: .topLeading,
                proposal: ProposedViewSize(width: width, height: nil)
            )
        }
    }
}

private struct TopologyVerticalDiagram: View {
    let layout: TopologyVerticalCanvasLayout
    let nodes: [TopologyExperienceNode]
    let expandedHeaders: [TopologyExperienceSubsystemHeader]
    let links: [TopologyExperienceLink]
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

            ForEach(expandedHeaders) { header in
                if let frame = layout.expandedHeaderFrames[header.id] {
                    TopologyExperienceSubsystemHeaderButton(header: header)
                        .frame(width: frame.width, height: frame.height)
                        .position(x: frame.midX, y: frame.midY)
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
        let downward = !link.isFeedback && destination.midY > source.midY
        let start = CGPoint(x: source.midX, y: downward ? source.maxY : source.minY)
        let end = CGPoint(x: destination.midX, y: downward ? destination.minY : destination.maxY)
        let verticalDistance = abs(end.y - start.y)
        var path = Path()
        path.move(to: start)

        if downward && verticalDistance <= 82 {
            path.addCurve(
                to: end,
                control1: CGPoint(x: start.x, y: start.y + 12),
                control2: CGPoint(x: end.x, y: end.y - 12)
            )
        } else {
            let laneOffset = CGFloat(index % 4) * 5
            let laneX = downward
                ? max(4, min(source.minX, destination.minX) - 10 - laneOffset)
                : min(layout.size.width - 4, max(source.maxX, destination.maxX) + 10 + laneOffset)
            let sign: CGFloat = downward ? 1 : -1
            let firstBend = CGPoint(x: laneX, y: start.y + sign * 13)
            let lastBend = CGPoint(x: laneX, y: end.y - sign * 13)
            path.addCurve(to: firstBend, control1: CGPoint(x: start.x, y: start.y + sign * 5), control2: CGPoint(x: laneX, y: start.y + sign * 6))
            path.addLine(to: lastBend)
            path.addCurve(to: end, control1: CGPoint(x: laneX, y: end.y - sign * 5), control2: CGPoint(x: end.x, y: end.y - sign * 7))
        }

        let dash: [CGFloat]
        switch link.evidence {
        case .sourceWiring: dash = []
        case .offlineExercise: dash = [5, 4]
        case .conditionalSourceWiring: dash = [3, 3]
        case .openInterface: dash = [2, 4]
        }
        context.stroke(path, with: .color(color.opacity(0.92)), style: StrokeStyle(lineWidth: link.status.signal == .abnormal ? 2.6 : 1.5, lineCap: .round, dash: dash))

        let arrow = Path { arrow in
            arrow.move(to: end)
            let vertical: CGFloat = downward ? -7 : 7
            arrow.addLine(to: CGPoint(x: end.x - 4.5, y: end.y + vertical))
            arrow.addLine(to: CGPoint(x: end.x + 4.5, y: end.y + vertical))
            arrow.closeSubpath()
        }
        context.fill(arrow, with: .color(color))
    }
}

private struct TopologyExperienceSubsystemHeaderButton: View {
    let header: TopologyExperienceSubsystemHeader

    var body: some View {
        Button(action: header.action) {
            HStack(spacing: 9) {
                Image(systemName: header.symbol)
                    .font(.system(size: 12, weight: .medium))
                    .foregroundStyle(Palette.accent)
                Text(header.title)
                    .font(.system(size: 11, weight: .semibold))
                    .foregroundStyle(.white)
                TopologyExperienceStatus(status: header.status, compact: true)
                Spacer()
                Image(systemName: "chevron.up")
                    .font(.system(size: 9, weight: .semibold))
                    .foregroundStyle(Palette.muted)
            }
            .padding(.horizontal, 7)
            .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .leading)
            .contentShape(Rectangle())
        }
        .buttonStyle(.plain)
        .accessibilityIdentifier("topology.subsystem-header.\(header.id)")
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
                if node.isSubsystem {
                    Image(systemName: "chevron.down")
                        .font(.system(size: 9, weight: .semibold))
                        .foregroundStyle(Palette.muted)
                }
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
