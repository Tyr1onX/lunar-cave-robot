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
        VStack(alignment: .leading, spacing: 16) {
            topologyHeader
            if interaction.selectedSubsystemID == nil {
                monitoringRail
                overview
            } else if let selectedSubsystem, let selectedFlow {
                breadcrumb
                subsystemHeading(selectedSubsystem)
                flowDiagram(selectedFlow, subsystemID: selectedSubsystem.id)
                if let selectedNode {
                    nodeEvidence(selectedNode)
                }
            }
        }
        .accessibilityIdentifier("robot-topology.view")
    }

    private var topologyHeader: some View {
        let signals = evaluation.nodeStatuses.values.map(\.signal) + evaluation.edgeStatuses.values.map(\.signal)
        let abnormalCount = signals.filter { $0 == .abnormal }.count
        let unknownCount = signals.filter { $0 == .unknown }.count
        let overall = TopologyOverallStatus.signal(for: evaluation)

        return Panel(title: "机器人整机链路图", subtitle: "按源码与离线证据展示系统结构；实时状态只采用只读 Agent 观测") {
            VStack(alignment: .leading, spacing: 11) {
                HStack(alignment: .center, spacing: 12) {
                    TopologyStatusBadge(signal: overall, compact: false)
                    Text(summaryText(abnormal: abnormalCount, unknown: unknownCount))
                        .font(.system(size: 10)).foregroundStyle(Palette.muted)
                        .fixedSize(horizontal: false, vertical: true)
                    Spacer(minLength: 8)
                    Text("\(rosFreshnessLabel) · 工作区 \(workspaceFreshnessLabel) · Domain \(context.snapshot?.ros.domainId.map(String.init) ?? "未知")")
                        .font(.system(size: 9, design: .monospaced)).foregroundStyle(Palette.muted)
                        .multilineTextAlignment(.trailing)
                }
                HStack(spacing: 12) {
                    legend("实线", color: Palette.blue, text: "源码 / Launch")
                    legend("虚线", color: Palette.warning, text: "离线验证或待核实")
                    legend("蓝色", color: Palette.blue, text: "Agent 收到数据")
                    legend("琥珀", color: Palette.warning, text: "数据未更新 / 未确认")
                    Spacer(minLength: 6)
                    Text("Topic 名称可见不代表消息已传输")
                        .font(.system(size: 9)).foregroundStyle(Palette.muted)
                }
            }
        }
    }

    private var monitoringRail: some View {
        Panel(title: "只读观测通道", subtitle: "监测数据与机器人内部链路分开展示") {
            HStack(spacing: 8) {
                ForEach(["agent", "ros-domain", "ros-graph", "expected-nodes"], id: \.self) { id in
                    if let node = RobotTopologyCatalog.node(id) {
                        Button {
                            interaction.selectSubsystem("system")
                        } label: {
                            HStack(spacing: 7) {
                                Circle().fill(signalColor(evaluation.nodeStatus(id).signal)).frame(width: 7, height: 7)
                                VStack(alignment: .leading, spacing: 2) {
                                    Text(node.title).font(.system(size: 9, weight: .semibold))
                                    Text(evaluation.nodeStatus(id).signal.label).font(.system(size: 8)).foregroundStyle(Palette.muted)
                                }
                                Spacer(minLength: 2)
                                Image(systemName: "arrow.up.right").font(.system(size: 8)).foregroundStyle(Palette.muted)
                            }
                            .padding(.horizontal, 9).padding(.vertical, 7)
                            .background(Palette.panelRaised, in: RoundedRectangle(cornerRadius: 8))
                            .overlay(RoundedRectangle(cornerRadius: 8).stroke(Palette.line, lineWidth: 1))
                        }
                        .buttonStyle(.plain)
                        .accessibilityIdentifier("topology.monitor.\(id)")
                        .help(evaluation.nodeStatus(id).note)
                    }
                }
            }
        }
    }

    private var overview: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("整机总览").font(.system(size: 13, weight: .semibold))
                Spacer()
                Text("选择子系统查看内部链路").font(.system(size: 9)).foregroundStyle(Palette.muted)
            }
            GeometryReader { geometry in
                let layout = RobotTopologyLayout.overview(availableWidth: geometry.size.width)
                TopologyDiagramCanvas(
                    layout: layout,
                    nodes: overviewNodes,
                    links: overviewLinks,
                    accessibilityID: "topology.overview.canvas"
                )
                .frame(height: layout.size.height)
            }
            .frame(height: 810)
            Text("主启动流程与当前设备状态仍需对应证据确认；开放接口以无方向虚线端点表示。")
                .font(.system(size: 9)).foregroundStyle(Palette.muted)
        }
    }

    private var overviewNodes: [TopologyDiagramNode] {
        RobotTopologyCatalog.subsystems.filter { $0.id != "system" }.map { subsystem in
            TopologyDiagramNode(
                id: subsystem.id,
                title: subsystem.title,
                summary: subsystemSummary(subsystem),
                symbol: subsystem.symbol,
                status: subsystemStatus(subsystem),
                selected: false,
                isSubsystem: true,
                onSelect: { interaction.selectSubsystem(subsystem.id) }
            )
        }
    }

    private var overviewLinks: [TopologyDiagramLink] {
        RobotTopologyCatalog.overviewEdges.compactMap { edge in
            guard RobotTopologyCatalog.subsystem(edge.sourceSubsystemID) != nil,
                  RobotTopologyCatalog.subsystem(edge.destinationSubsystemID) != nil else { return nil }
            let status = TopologyStatusResolver.overviewEdgeStatus(edge, context: context, evaluation: evaluation)
            let offset: CGFloat
            switch edge.id {
            case "coordination-navigation": offset = -25
            case "navigation-chassis": offset = 22
            case "arm-safety": offset = -20
            case "chassis-safety": offset = 20
            default: offset = -8
            }
            return TopologyDiagramLink(
                id: edge.id,
                sourceID: edge.sourceSubsystemID,
                destinationID: edge.destinationSubsystemID,
                label: edge.label,
                evidence: edge.evidence,
                status: status,
                labelOffset: offset
            )
        }
    }

    private var breadcrumb: some View {
        HStack(spacing: 7) {
            Button {
                interaction.showOverview()
            } label: {
                Label("整机总览", systemImage: "chevron.left")
                    .font(.system(size: 10, weight: .medium))
            }
            .buttonStyle(.plain)
            .foregroundStyle(Palette.accent)
            .accessibilityIdentifier("topology.breadcrumb.overview")

            if let selectedSubsystem {
                Image(systemName: "chevron.right").font(.system(size: 8)).foregroundStyle(Palette.muted)
                Text(selectedSubsystem.title).font(.system(size: 10, weight: .semibold))
                if let selectedNode {
                    Image(systemName: "chevron.right").font(.system(size: 8)).foregroundStyle(Palette.muted)
                    Text(selectedNode.title).font(.system(size: 10)).foregroundStyle(Palette.muted)
                }
            }
            Spacer()
        }
    }

    private func subsystemHeading(_ subsystem: TopologySubsystemDefinition) -> some View {
        HStack(spacing: 11) {
            Image(systemName: subsystem.symbol)
                .font(.system(size: 17, weight: .medium)).foregroundStyle(Palette.accent)
                .frame(width: 36, height: 36)
                .background(Palette.accent.opacity(0.12), in: RoundedRectangle(cornerRadius: 10))
            VStack(alignment: .leading, spacing: 3) {
                Text(subsystem.title).font(.system(size: 17, weight: .semibold, design: .rounded))
                Text(subsystem.summary).font(.system(size: 10)).foregroundStyle(Palette.muted)
            }
            Spacer()
            TopologyStatusBadge(signal: subsystemStatus(subsystem).signal, compact: false)
        }
        .padding(.vertical, 2)
        .accessibilityIdentifier("topology.detail.\(subsystem.id)")
    }

    private func flowDiagram(_ flow: TopologyFlowDefinition, subsystemID: String) -> some View {
        let layout = RobotTopologyLayout.detail(flow)
        let flowNodes = Set(flow.nodeIDs)
        let nodes = flow.nodeIDs.compactMap { id -> TopologyDiagramNode? in
            guard let node = RobotTopologyCatalog.node(id) else { return nil }
            return TopologyDiagramNode(
                id: node.id,
                title: node.title,
                summary: node.resourceHints.first.map { $0 + (node.resourceHints.count > 1 ? " · …" : "") } ?? node.summary,
                symbol: node.isExternalHardware ? "cpu" : "square.stack.3d.up",
                status: evaluation.nodeStatus(id),
                selected: interaction.selectedNodeID == id,
                isSubsystem: false,
                onSelect: { interaction.selectNode(id, in: subsystemID) }
            )
        }
        let links = flow.edgeIDs.compactMap { id -> TopologyDiagramLink? in
            guard let edge = RobotTopologyCatalog.edge(id),
                  flowNodes.contains(edge.sourceNodeID),
                  flowNodes.contains(edge.destinationNodeID) else { return nil }
            return TopologyDiagramLink(
                id: edge.id,
                sourceID: edge.sourceNodeID,
                destinationID: edge.destinationNodeID,
                label: edge.label + (edge.topics.isEmpty ? "" : " · " + edge.topics.joined(separator: " / ")),
                evidence: edge.evidence,
                status: evaluation.edgeStatus(id),
                labelOffset: linksLabelOffset(for: id)
            )
        }

        return VStack(alignment: .leading, spacing: 9) {
            HStack(spacing: 6) {
                Image(systemName: "arrow.right").font(.system(size: 9, weight: .semibold))
                Text(flow.summary).font(.system(size: 9)).lineLimit(2)
                Spacer()
                Text("\(flowNodes.count) 个节点 · \(links.count) 条连接")
                    .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
            }
            .foregroundStyle(Palette.muted)

            TopologyDiagramCanvas(
                layout: layout,
                nodes: nodes,
                links: links,
                accessibilityID: "topology.detail.canvas.\(flow.id)"
            )
            .frame(height: layout.size.height)
            .accessibilityIdentifier("topology.diagram.\(flow.id)")

            HStack(spacing: 10) {
                legend("箭头", color: Palette.blue, text: "源码方向")
                legend("虚线", color: Palette.warning, text: "待核实 / 离线接口")
                Text("连接状态仅反映当前可取得的证据，不代表端到端数据已送达。")
                    .font(.system(size: 8)).foregroundStyle(Palette.muted)
            }
        }
    }

    private func nodeEvidence(_ node: TopologyNodeDefinition) -> some View {
        TopologyNodeEvidencePanel(
            node: node,
            status: evaluation.nodeStatus(node.id),
            context: context,
            linkTelemetryMessage: model.linkTelemetryMessage,
            onClose: { interaction.clearSelection() }
        )
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

    private func subsystemSummary(_ subsystem: TopologySubsystemDefinition) -> String {
        let state = subsystemStatus(subsystem)
        if state.signal == .abnormal { return "直接异常 · " + state.note }
        if state.possiblyAffectedDownstream { return "可能受影响 · 状态未推断为异常" }
        return subsystem.summary
    }

    private func summaryText(abnormal: Int, unknown: Int) -> String {
        if abnormal > 0 { return "直接异常 \(abnormal) 项 · 下游只标记可能受影响，不推断根因" }
        if unknown > 0 { return "未知 \(unknown) 项 · 证据不足保持未知，不合并为正常" }
        return "未发现当前规则确认的异常；Topic 可见性和进程存在不代表端到端或实体健康"
    }

    private var rosFreshnessLabel: String {
        guard model.agentOnline else { return "Agent 断连" }
        return context.graphBlockReason == nil ? "实时 ROS Graph" : "ROS 运行未知"
    }

    private var workspaceFreshnessLabel: String {
        switch model.workspaceSnapshotFreshness {
        case .live: return "实时"
        case .historical: return "历史快照"
        case .expired: return "过期"
        case .unavailable: return "等待"
        }
    }

    private func linksLabelOffset(for id: String) -> CGFloat {
        switch id {
        case "map-nav2", "mission-arm", "arm-driver-safety", "chassis-status": return -18
        case "tf-navigation", "arm-driver-mcu", "chassis-bridge-mcu": return 20
        default: return -10
        }
    }

    private func legend(_ symbol: String, color: Color, text: String) -> some View {
        HStack(spacing: 5) {
            Text(symbol).font(.system(size: 8, weight: .semibold)).foregroundStyle(color)
            Text(text).font(.system(size: 8)).foregroundStyle(Palette.muted)
        }
    }
}

private struct TopologyDiagramNode {
    let id: String
    let title: String
    let summary: String
    let symbol: String
    let status: TopologyElementStatus
    let selected: Bool
    let isSubsystem: Bool
    let onSelect: () -> Void
}

private struct TopologyDiagramLink: Identifiable {
    let id: String
    let sourceID: String
    let destinationID: String
    let label: String
    let evidence: TopologyEvidenceGrade
    let status: TopologyElementStatus
    let labelOffset: CGFloat
}

private struct TopologyDiagramCanvas: View {
    let layout: TopologyCanvasLayout
    let nodes: [TopologyDiagramNode]
    let links: [TopologyDiagramLink]
    let accessibilityID: String

    private let subsystemSize = CGSize(width: 208, height: 106)
    private let componentSize = CGSize(width: 190, height: 86)

    var body: some View {
        ScrollView(.horizontal, showsIndicators: true) {
            ZStack(alignment: .topLeading) {
                RoundedRectangle(cornerRadius: 14)
                    .fill(Palette.panel.opacity(0.76))
                    .overlay(RoundedRectangle(cornerRadius: 14).stroke(Palette.line, lineWidth: 1))

                Canvas { context, _ in
                    for link in links {
                        guard let sourceCenter = layout.nodeCenters[link.sourceID],
                              let destinationCenter = layout.nodeCenters[link.destinationID] else { continue }
                        let sourceSize = nodes.first(where: { $0.id == link.sourceID }).map { $0.isSubsystem ? subsystemSize : componentSize } ?? componentSize
                        let destinationSize = nodes.first(where: { $0.id == link.destinationID }).map { $0.isSubsystem ? subsystemSize : componentSize } ?? componentSize
                        draw(link, from: sourceCenter, to: destinationCenter, sourceSize: sourceSize, destinationSize: destinationSize, in: &context)
                    }
                }

                ForEach(links) { link in
                    if let source = layout.nodeCenters[link.sourceID],
                       let destination = layout.nodeCenters[link.destinationID] {
                        let sourceWidth = nodes.first(where: { $0.id == link.sourceID }).map { $0.isSubsystem ? subsystemSize.width : componentSize.width } ?? componentSize.width
                        let destinationWidth = nodes.first(where: { $0.id == link.destinationID }).map { $0.isSubsystem ? subsystemSize.width : componentSize.width } ?? componentSize.width
                        let openSpace = abs(destination.x - source.x) - (sourceWidth + destinationWidth) / 2 - 12
                        VStack(spacing: 3) {
                            Text(link.label)
                                .font(.system(size: 8, weight: .medium))
                                .lineLimit(2)
                                .multilineTextAlignment(.center)
                            Text(link.status.signal.label)
                                .font(.system(size: 7, weight: .semibold))
                                .foregroundStyle(signalColor(link.status.signal))
                                .lineLimit(1)
                        }
                        .padding(.horizontal, 5).padding(.vertical, 3)
                        .background(Palette.background.opacity(0.92), in: RoundedRectangle(cornerRadius: 5))
                        .overlay(RoundedRectangle(cornerRadius: 5).stroke(Palette.line, lineWidth: 1))
                        .frame(width: min(148, max(76, openSpace)))
                        .position(
                            x: (source.x + destination.x) / 2,
                            y: (source.y + destination.y) / 2 + link.labelOffset
                        )
                        .help(link.status.note)
                        .accessibilityIdentifier("topology.link.\(link.id)")
                    }
                }

                ForEach(nodes, id: \.id) { node in
                    if let center = layout.nodeCenters[node.id] {
                        TopologyDiagramNodeCard(node: node)
                            .position(center)
                    }
                }
            }
            .frame(width: layout.size.width, height: layout.size.height)
            .accessibilityIdentifier(accessibilityID)
        }
        .frame(height: layout.size.height)
    }

    private func draw(
        _ link: TopologyDiagramLink,
        from source: CGPoint,
        to destination: CGPoint,
        sourceSize: CGSize,
        destinationSize: CGSize,
        in context: inout GraphicsContext
    ) {
        let direction: CGFloat = destination.x >= source.x ? 1 : -1
        let start = CGPoint(x: source.x + direction * (sourceSize.width / 2), y: source.y)
        let end = CGPoint(x: destination.x - direction * (destinationSize.width / 2), y: destination.y)
        let horizontal = end.x - start.x
        let vertical = end.y - start.y
        let control1 = CGPoint(x: start.x + horizontal * 0.42, y: start.y + vertical * 0.16)
        let control2 = CGPoint(x: end.x - horizontal * 0.42, y: end.y - vertical * 0.16)
        var path = Path()
        path.move(to: start)
        path.addCurve(to: end, control1: control1, control2: control2)

        let color = signalColor(link.status.signal)
        let dash: [CGFloat]
        switch link.evidence {
        case .sourceWiring: dash = []
        case .offlineExercise: dash = [7, 4]
        case .openInterface: dash = [2, 5]
        }
        context.stroke(path, with: .color(color.opacity(0.9)), style: StrokeStyle(lineWidth: link.status.signal == .abnormal ? 3 : 1.8, lineCap: .round, dash: dash))

        if link.evidence == .openInterface {
            let marker = CGRect(x: end.x - 4, y: end.y - 4, width: 8, height: 8)
            context.stroke(Path(ellipseIn: marker), with: .color(color), lineWidth: 1.5)
        } else {
            drawArrow(at: end, direction: CGPoint(x: end.x - control2.x, y: end.y - control2.y), color: color, in: &context)
        }
    }

    private func drawArrow(at point: CGPoint, direction: CGPoint, color: Color, in context: inout GraphicsContext) {
        let angle = atan2(direction.y, direction.x)
        let length: CGFloat = 9
        let halfWidth: CGFloat = 4.5
        let base = CGPoint(x: point.x - cos(angle) * length, y: point.y - sin(angle) * length)
        let left = CGPoint(x: base.x + sin(angle) * halfWidth, y: base.y - cos(angle) * halfWidth)
        let right = CGPoint(x: base.x - sin(angle) * halfWidth, y: base.y + cos(angle) * halfWidth)
        var arrow = Path()
        arrow.move(to: point)
        arrow.addLine(to: left)
        arrow.addLine(to: right)
        arrow.closeSubpath()
        context.fill(arrow, with: .color(color))
    }
}

private struct TopologyDiagramNodeCard: View {
    let node: TopologyDiagramNode

    private var width: CGFloat { node.isSubsystem ? 208 : 190 }

    var body: some View {
        Button(action: node.onSelect) {
            VStack(alignment: .leading, spacing: 7) {
                HStack(spacing: 7) {
                    Image(systemName: node.symbol)
                        .font(.system(size: 12, weight: .medium))
                        .foregroundStyle(node.isSubsystem ? Palette.accent : Palette.blue)
                        .frame(width: 19)
                    Text(node.title)
                        .font(.system(size: 10, weight: .semibold))
                        .lineLimit(2)
                        .multilineTextAlignment(.leading)
                    Spacer(minLength: 0)
                    if node.isSubsystem {
                        Image(systemName: "arrow.up.right").font(.system(size: 8, weight: .semibold)).foregroundStyle(Palette.muted)
                    }
                }
                Text(node.summary)
                    .font(.system(size: 8))
                    .foregroundStyle(Palette.muted)
                    .lineLimit(2)
                    .multilineTextAlignment(.leading)
                HStack(spacing: 5) {
                    TopologyStatusBadge(signal: node.status.signal, compact: true)
                    if node.status.possiblyAffectedDownstream {
                        Text("可能受影响")
                            .font(.system(size: 7, weight: .medium))
                            .foregroundStyle(Palette.warning)
                    }
                    Spacer(minLength: 0)
                }
            }
            .padding(9)
            .frame(width: width, height: node.isSubsystem ? 106 : 86, alignment: .leading)
            .background(node.selected ? Palette.accent.opacity(0.15) : Palette.panelRaised, in: RoundedRectangle(cornerRadius: 10))
            .overlay(RoundedRectangle(cornerRadius: 10).stroke(
                node.status.signal == .abnormal ? Palette.danger.opacity(0.8) : (node.selected ? Palette.accent.opacity(0.75) : Palette.line),
                lineWidth: node.status.signal == .abnormal ? 1.7 : 1
            ))
            .contentShape(RoundedRectangle(cornerRadius: 10))
        }
        .buttonStyle(.plain)
        .help(node.status.note)
        .accessibilityIdentifier(node.isSubsystem ? "topology.subsystem.\(node.id)" : "topology.node.\(node.id)")
    }
}

private struct TopologyStatusBadge: View {
    let signal: TopologySignal
    var title: String? = nil
    var compact = true

    var body: some View {
        HStack(spacing: 4) {
            Circle().fill(signalColor(signal)).frame(width: compact ? 5 : 7, height: compact ? 5 : 7)
            Text(title ?? signal.label).font(.system(size: compact ? 8 : 10, weight: .semibold)).lineLimit(1)
        }
        .foregroundStyle(signalColor(signal))
        .padding(.horizontal, compact ? 6 : 9)
        .padding(.vertical, compact ? 4 : 6)
        .background(signalColor(signal).opacity(0.10), in: Capsule())
        .overlay(Capsule().stroke(signalColor(signal).opacity(0.22), lineWidth: 1))
    }
}

private func signalColor(_ signal: TopologySignal) -> Color {
    switch signal {
    case .normal: return Palette.accent
    case .abnormal: return Palette.danger
    case .unknown, .notRunning, .processNotFound: return Palette.warning
    case .processObserved, .topicsAllVisible, .topicsPartiallyVisible: return Palette.blue
    case .topicsNoneVisible: return Palette.muted
    case .dataObserved: return Palette.blue
    case .dataPartiallyObserved, .dataStale: return Palette.warning
    case .dataNotObserved: return Palette.muted
    }
}

private struct TopologyNodeEvidencePanel: View {
    let node: TopologyNodeDefinition
    let status: TopologyElementStatus
    let context: TopologyRuntimeContext
    let linkTelemetryMessage: String?
    let onClose: () -> Void

    private var workspaceEvidence: [(WorkspaceReading, WorkspaceModuleReading)] {
        guard let inventory = context.displayedWorkspaceInventory else { return [] }
        return node.workspaceModules.flatMap { binding in
            inventory.entries.compactMap { workspace -> (WorkspaceReading, WorkspaceModuleReading)? in
                guard workspace.id == binding.workspaceID,
                      let module = workspace.modules?.first(where: { $0.id == binding.moduleID }) else { return nil }
                return (workspace, module)
            }
        }
    }

    var body: some View {
        Panel(title: "\(node.title) · 诊断证据", subtitle: "结构来源与实时观测分开展示") {
            VStack(alignment: .leading, spacing: 14) {
                HStack(alignment: .top, spacing: 10) {
                    TopologyStatusBadge(signal: status.signal, compact: false)
                    Text(status.note).font(.system(size: 10)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                    Spacer(minLength: 0)
                    Button(action: onClose) {
                        Image(systemName: "xmark.circle.fill").foregroundStyle(Palette.muted)
                    }.buttonStyle(.plain).help("关闭节点详情")
                }

                evidenceSection(title: "结构依据", source: node.sourceNote, paths: node.sourcePaths)

                if !node.resourceHints.isEmpty {
                    evidenceSection(title: "Topic / 接口", source: node.resourceHints.joined(separator: " · "), paths: [])
                }

                if node.groupID == "localization" {
                    linkTelemetryEvidence
                }

                if !workspaceEvidence.isEmpty {
                    VStack(alignment: .leading, spacing: 8) {
                        evidenceHeading("工作区观测 · \(workspaceFreshnessLabel)")
                        ForEach(workspaceEvidence.indices, id: \.self) { index in
                            WorkspaceModuleEvidenceRow(workspace: workspaceEvidence[index].0, module: workspaceEvidence[index].1)
                        }
                    }
                } else if !node.workspaceModules.isEmpty {
                    evidenceSection(title: "工作区观测", source: "当前快照没有可归属的工作区模块记录。", paths: [])
                }

                if !status.directEvents.isEmpty {
                    VStack(alignment: .leading, spacing: 8) {
                        evidenceHeading("现有健康规则证据")
                        ForEach(status.directEvents) { event in
                            HealthEventEvidenceRow(event: event)
                        }
                    }
                }

                HStack(alignment: .top, spacing: 12) {
                    detailValue("运行模式", value: runModeSummary)
                    detailValue("ROS Domain", value: context.snapshot?.ros.domainId.map(String.init) ?? "未知")
                    detailValue("Graph 时间", value: context.snapshot?.ros.observedAt ?? "未知")
                    detailValue("观测时效", value: context.agentOnline ? "当前 Agent" : "历史 / 不参与实时判定")
                }
            }
        }
    }

    private var workspaceFreshnessLabel: String {
        switch context.workspaceFreshness {
        case .live: return "实时"
        case .historical: return "历史，不参与实时判定"
        case .expired: return "过期，不参与实时判定"
        case .unavailable: return "不可用"
        }
    }

    private var linkTelemetryEvidence: some View {
        let streams = LinkTelemetryDiagnostics.streams(for: node.id, telemetry: context.linkTelemetry)
        let telemetry = context.linkTelemetry
        return VStack(alignment: .leading, spacing: 8) {
            evidenceHeading("实时数据链路 · Agent 只读接收")
            if let telemetry {
                Text("ROS Domain \(telemetry.domainId.map(String.init) ?? "未知") · Graph \(telemetry.graphObservedAt ?? "时间未知") · Agent 快照 \(telemetry.collectedAt) · 工作区来源未确认")
                    .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
                if let blocked = context.linkTelemetryBlockReason {
                    Text("当前 Domain / Graph 证据阻止链路归属判断：\(blocked)")
                        .font(.system(size: 9, weight: .medium)).foregroundStyle(Palette.warning).fixedSize(horizontal: false, vertical: true)
                }
                if let summary = LinkTelemetryDiagnostics.pipelineSummary(
                    nodeID: node.id,
                    telemetry: telemetry,
                    blockedReason: context.linkTelemetryBlockReason
                ) {
                    Text(summary).font(.system(size: 9)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                }
                if streams.isEmpty {
                    Text("该节点没有当前 Agent 固定采样的 Topic，或该版本未提供遥测记录。")
                        .font(.system(size: 9)).foregroundStyle(Palette.muted)
                } else {
                    ForEach(streams) { stream in
                        LinkTelemetryStreamEvidenceRow(stream: stream, blockedReason: context.linkTelemetryBlockReason, clockNote: telemetry.clockNote)
                    }
                }
            } else {
                Text(context.agentOnline
                     ? (linkTelemetryMessage ?? "当前 Agent 尚未返回实时链路遥测。")
                     : "Agent 当前断连；不使用历史链路数据进行实时判定。")
                    .font(.system(size: 9)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
            }
        }
    }

    private var runModeSummary: String {
        let modes = workspaceEvidence.compactMap { $0.0.runMode }.filter { $0 != "unknown" }
        return modes.isEmpty ? "未知" : Array(Set(modes)).sorted().joined(separator: " / ")
    }

    private func evidenceSection(title: String, source: String, paths: [String]) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            evidenceHeading(title)
            Text(source).font(.system(size: 10)).foregroundStyle(Palette.muted).textSelection(.enabled)
            ForEach(paths, id: \.self) { path in
                Text(path).font(.system(size: 9, design: .monospaced)).foregroundStyle(Palette.blue).textSelection(.enabled)
            }
        }
    }

    private func evidenceHeading(_ title: String) -> some View {
        Text(title).font(.system(size: 10, weight: .semibold)).foregroundStyle(.white)
    }

    private func detailValue(_ title: String, value: String) -> some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(title).font(.system(size: 9, weight: .medium)).foregroundStyle(Palette.muted)
            Text(value).font(.system(size: 9, design: .monospaced)).foregroundStyle(.white).textSelection(.enabled)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

private struct LinkTelemetryStreamEvidenceRow: View {
    let stream: LinkTelemetryStreamReading
    let blockedReason: String?
    let clockNote: String

    private var signal: TopologySignal {
        guard blockedReason == nil else { return .unknown }
        switch stream.status {
        case "streaming": return .dataObserved
        case "stale": return .dataStale
        case "waiting", "unavailable": return .dataNotObserved
        default: return .unknown
        }
    }

    private var recommendation: String {
        switch stream.id {
        case "livox_lidar", "livox_imu": return "查看 MID-360 配置、驱动日志与主机网络证据；Console 不启动设备或驱动。"
        case "imu_transformed": return "查看 imu_transform 节点日志、输入/输出类型与变换配置。"
        case "imu_data": return "查看 IMU 互补滤波节点日志及 /livox/imu_aft_tf 输入证据。"
        case "pointlio_odometry", "cloud_registered": return "对照点云和 /imu/data 输入，再查看 Point-LIO 日志与 TF 证据。"
        case "terrain_map": return "对照 /cloud_registered 和地形分析节点证据。"
        case "scan": return "对照 /terrain_map、点云转 LaserScan 节点及所需 TF。"
        case "map": return "对照 /scan 更新、SLAM Toolbox 日志和地图更新条件。"
        default: return "查看该 Topic 对应的只读运行证据。"
        }
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 5) {
            HStack(spacing: 7) {
                TopologyStatusBadge(signal: signal, compact: true)
                Text(stream.label).font(.system(size: 9, weight: .semibold))
                Spacer()
                Text(stream.receiveRateHz.map { "Agent \(String(format: "%.2f Hz", $0))" } ?? "Agent 接收频率未知")
                    .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.blue)
            }
            Text("\(stream.topic) · \(stream.topicType ?? "消息类型未知") · ROS Domain 接收年龄 \(stream.ageMs.map { "\($0) ms" } ?? "未知") · 超时阈值 \(stream.staleAfterMs) ms")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
            Text("源时间 \(stream.sourceStamp ?? "未知") · Agent 接收 \(stream.receivedAt ?? "未知") · header 对照 \(stream.headerToAgentMs.map { String(format: "%.1f ms", $0) } ?? "未知") · frame \(stream.frameId ?? "未知")")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
            Text("发布者 \(stream.publisherCount.map { String($0) } ?? "未知") · 端点 \(publisherSummary) · 工作区归属：未确认 · 样本尺寸 \(sampleSummary) · 接收间断 \(stream.gapEvents) 次（最近 \(stream.lastGapMs.map { "\($0) ms" } ?? "无记录")）")
                .font(.system(size: 8)).foregroundStyle(Palette.muted).textSelection(.enabled)
            if let reason = blockedReason ?? stream.reason {
                Text(reason).font(.system(size: 8)).foregroundStyle(Palette.warning).fixedSize(horizontal: false, vertical: true)
            }
            Text("排查建议：\(recommendation)").font(.system(size: 8)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
            Text(clockNote).font(.system(size: 7)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
        }
        .padding(8)
        .background(Palette.panelRaised.opacity(0.7), in: RoundedRectangle(cornerRadius: 8))
        .accessibilityIdentifier("topology.telemetry.\(stream.id)")
    }

    private var publisherSummary: String {
        let endpoints = stream.publisherEndpoints.map { endpoint in
            endpoint.namespace == "/" ? endpoint.name : "\(endpoint.namespace)/\(endpoint.name)"
        }
        return endpoints.isEmpty ? "未见" : endpoints.joined(separator: "、")
    }

    private var sampleSummary: String {
        guard let count = stream.sampleCount else { return "未知" }
        return "\(count) \(stream.sampleUnit ?? "")"
    }
}

private struct WorkspaceModuleEvidenceRow: View {
    let workspace: WorkspaceReading
    let module: WorkspaceModuleReading

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack(spacing: 6) {
                Text(workspace.label).font(.system(size: 9, weight: .semibold))
                Text(module.label).font(.system(size: 9)).foregroundStyle(Palette.muted)
                Spacer()
                Text(workspace.runMode ?? "未知模式").font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
            }
            Text("源码：\(module.sourceStatus ?? workspace.sourceStatus ?? "未知") · 进程：\(module.runtimeStatus ?? "未知") · 预期：\(module.runExpectation ?? "旧版 Agent / 未提供") · Graph：\(workspace.graphRelationship ?? "未知")")
                .font(.system(size: 8)).foregroundStyle(Palette.muted).textSelection(.enabled)
            Text("进程观测：\(workspace.runtimeObservedAt ?? "未知") · 预期来源：\(module.runExpectationSource ?? "未提供") · 预期时间：\(module.runExpectationObservedAt ?? "未知")")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
            if let reason = module.runExpectationReason, !reason.isEmpty {
                Text(reason).font(.system(size: 8)).foregroundStyle(Palette.muted).textSelection(.enabled)
            }
            if let topics = module.observedInterfaceTopics, !topics.isEmpty {
                Text("观察到的接口名称：\(topics.joined(separator: "、"))")
                    .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
            }
        }
        .padding(9)
        .background(Palette.panelRaised.opacity(0.7), in: RoundedRectangle(cornerRadius: 8))
    }
}

private struct HealthEventEvidenceRow: View {
    let event: HealthEvent

    var body: some View {
        VStack(alignment: .leading, spacing: 5) {
            HStack(spacing: 7) {
                TopologyStatusBadge(signal: signal(for: event.state), title: event.state.label)
                Text(event.title).font(.system(size: 9, weight: .semibold))
                Text(event.persistenceLabel).font(.system(size: 8)).foregroundStyle(Palette.muted)
                Spacer()
            }
            Text(event.observation).font(.system(size: 9)).foregroundStyle(Palette.muted).textSelection(.enabled)
            Text("规则：\(event.ruleID) · 严重度：\(event.severity.label) · 来源：\(event.source)")
                .font(.system(size: 8)).foregroundStyle(Palette.muted).textSelection(.enabled)
            Text("观测：\(event.observationWindow) · 数据时间：\(event.dataTime) · Domain：\(event.rosDomain) · 模式：\(event.runMode.rawValue)")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
            Text("资源：\(event.resource) · 检查：\(event.suggestedCheck)")
                .font(.system(size: 8)).foregroundStyle(Palette.muted).textSelection(.enabled)
            Text("源码位置：\(event.sourceLocation)")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
        }
        .padding(9)
        .background(Palette.panelRaised.opacity(0.7), in: RoundedRectangle(cornerRadius: 8))
    }

    private func signal(for state: HealthState) -> TopologySignal {
        switch state {
        case .normal: return .normal
        case .abnormal: return .abnormal
        case .notRunning: return .notRunning
        case .unknown: return .unknown
        }
    }
}
