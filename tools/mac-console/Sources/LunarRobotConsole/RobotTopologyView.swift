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
            healthEvents: model.healthEvents
        )
    }

    private var evaluation: TopologyEvaluation { TopologyStatusResolver.evaluate(context) }

    private var selectedNode: TopologyNodeDefinition? {
        interaction.selectedNodeID.flatMap { RobotTopologyCatalog.node($0) }
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            topologyIntroduction
            LazyVStack(alignment: .leading, spacing: 14) {
                ForEach(RobotTopologyCatalog.flows) { flow in
                    TopologyFlowCard(
                        flow: flow,
                        evaluation: evaluation,
                        expanded: interaction.expandedFlowID == flow.id,
                        selectedNodeID: interaction.selectedNodeID,
                        onToggle: { interaction.toggleFlow(flow.id) },
                        onSelectNode: { nodeID in interaction.selectNode(nodeID, in: flow.id) }
                    )
                }
            }

            if let selectedNode {
                TopologyNodeEvidencePanel(
                    node: selectedNode,
                    status: evaluation.nodeStatus(selectedNode.id),
                    context: context,
                    onClose: { interaction.clearSelection() }
                )
                .accessibilityIdentifier("topology.node-evidence")
            } else {
                Panel(title: "节点诊断证据", subtitle: "选择拓扑节点查看来源、实时观测和状态规则") {
                    UnknownBlock(text: "拓扑结构来自仓库源码；运行状态只使用当前 Agent 与 ROS 证据。")
                }
            }
        }
        .accessibilityIdentifier("robot-topology.view")
    }

    private var topologyIntroduction: some View {
        let statuses = evaluation.nodeStatuses.values.map(\.signal) + evaluation.edgeStatuses.values.map(\.signal)
        let abnormalCount = evaluation.nodeStatuses.values.filter { $0.signal == .abnormal }.count +
            evaluation.edgeStatuses.values.filter { $0.signal == .abnormal }.count
        let unknownCount = statuses.filter { $0 == .unknown }.count
        let missingProcessCount = statuses.filter { $0 == .processNotFound }.count
        let observedProcessCount = statuses.filter { $0 == .processObserved }.count
        let overall = TopologyOverallStatus.signal(for: evaluation)

        return Panel(title: "机器人系统拓扑", subtitle: "源码说明模块如何连接；状态只反映可验证的实时观测") {
            VStack(alignment: .leading, spacing: 13) {
                HStack(alignment: .center, spacing: 13) {
                    TopologyStatusBadge(signal: overall, compact: false)
                    Text(summaryText(abnormal: abnormalCount, unknown: unknownCount, missing: missingProcessCount, observed: observedProcessCount))
                        .font(.system(size: 11)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                    Spacer(minLength: 0)
                    Text("\(rosFreshnessLabel) · 工作区 \(workspaceFreshnessLabel) · Domain \(context.snapshot?.ros.domainId.map(String.init) ?? "未知")")
                        .font(.system(size: 10, design: .monospaced)).foregroundStyle(Palette.muted).multilineTextAlignment(.trailing)
                }
                HStack(spacing: 7) {
                    legendDot(Palette.accent, text: "已确认正常")
                    legendDot(Palette.danger, text: "直接异常")
                    legendDot(Palette.warning, text: "未知 / 未发现关联进程")
                    Spacer(minLength: 0)
                    Text("连接线证据：源码 / Launch · Graph Topic 名称 · 开放接口")
                        .font(.system(size: 9)).foregroundStyle(Palette.muted)
                }
            }
        }
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

    private func summaryText(abnormal: Int, unknown: Int, missing: Int, observed: Int) -> String {
        if abnormal > 0 {
            return "现有健康规则确认 \(abnormal) 项异常。下游标记只表示可能受影响，不代表根因。"
        }
        var pieces: [String] = []
        if unknown > 0 { pieces.append("\(unknown) 项未知") }
        if missing > 0 { pieces.append("\(missing) 项未发现关联进程") }
        if statusesContainNotRunning { pieces.append("有模块未运行") }
        if observed > 0 { pieces.append("\(observed) 项已发现进程，健康未确认") }
        return pieces.isEmpty ? "当前可见检查项正常。" : pieces.joined(separator: " · ") + "。未知状态不会合并为绿色正常。"
    }

    private var statusesContainNotRunning: Bool {
        evaluation.nodeStatuses.values.contains { $0.signal == .notRunning }
    }

    private func legendDot(_ color: Color, text: String) -> some View {
        HStack(spacing: 5) {
            Circle().fill(color).frame(width: 6, height: 6)
            Text(text).font(.system(size: 9)).foregroundStyle(Palette.muted)
        }
    }
}

private struct TopologyFlowCard: View {
    let flow: TopologyFlowDefinition
    let evaluation: TopologyEvaluation
    let expanded: Bool
    let selectedNodeID: String?
    let onToggle: () -> Void
    let onSelectNode: (String) -> Void

    private var flowNodes: [TopologyNodeDefinition] {
        flow.nodeIDs.compactMap { RobotTopologyCatalog.node($0) }
    }

    private var abnormalCount: Int {
        flowNodes.filter { evaluation.nodeStatus($0.id).signal == .abnormal }.count
    }

    private var unknownCount: Int {
        flowNodes.filter { evaluation.nodeStatus($0.id).signal == .unknown }.count
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Button(action: onToggle) {
                HStack(spacing: 12) {
                    Image(systemName: expanded ? "chevron.down.circle.fill" : "chevron.right.circle")
                        .font(.system(size: 17)).foregroundStyle(Palette.accent)
                    VStack(alignment: .leading, spacing: 4) {
                        Text(flow.title).font(.system(size: 13, weight: .semibold))
                        Text(flow.summary).font(.system(size: 10)).foregroundStyle(Palette.muted).lineLimit(2)
                    }
                    Spacer(minLength: 8)
                    if abnormalCount > 0 {
                        TopologyStatusBadge(signal: .abnormal, title: "\(abnormalCount) 项异常")
                    } else if unknownCount > 0 {
                        TopologyStatusBadge(signal: .unknown, title: "\(unknownCount) 项未知")
                    }
                    Text("\(flow.edgeIDs.count) 条连接").font(.system(size: 9)).foregroundStyle(Palette.muted)
                }
                .contentShape(Rectangle())
            }
            .buttonStyle(.plain)
            .accessibilityIdentifier("topology.flow.\(flow.id)")

            if expanded {
                VStack(alignment: .leading, spacing: 10) {
                    Text("模块").font(.system(size: 9, weight: .semibold)).foregroundStyle(Palette.muted)
                    ScrollView(.horizontal, showsIndicators: false) {
                        HStack(spacing: 8) {
                            ForEach(flowNodes) { node in
                                TopologyNodeButton(
                                    node: node,
                                    status: evaluation.nodeStatus(node.id),
                                    selected: selectedNodeID == node.id,
                                    onSelect: { onSelectNode(node.id) }
                                )
                            }
                        }
                    }

                    if flow.edgeIDs.isEmpty {
                        UnknownBlock(text: "当前模块尚无已建模的连接。")
                    } else {
                        Text("已建模连接").font(.system(size: 9, weight: .semibold)).foregroundStyle(Palette.muted)
                        VStack(spacing: 8) {
                            ForEach(flow.edgeIDs, id: \.self) { edgeID in
                                if let edge = RobotTopologyCatalog.edge(edgeID),
                                   let source = RobotTopologyCatalog.node(edge.sourceNodeID),
                                   let destination = RobotTopologyCatalog.node(edge.destinationNodeID) {
                                    TopologyEdgeRow(
                                        edge: edge,
                                        source: source,
                                        destination: destination,
                                        evaluation: evaluation,
                                        selectedNodeID: selectedNodeID,
                                        onSelectNode: onSelectNode
                                    )
                                }
                            }
                        }
                    }
                }
                .transition(.opacity)
            }
        }
        .padding(15)
        .background(Palette.panel, in: RoundedRectangle(cornerRadius: 13))
        .overlay(RoundedRectangle(cornerRadius: 13).stroke(Palette.line, lineWidth: 1))
    }
}

private struct TopologyEdgeRow: View {
    let edge: TopologyEdgeDefinition
    let source: TopologyNodeDefinition
    let destination: TopologyNodeDefinition
    let evaluation: TopologyEvaluation
    let selectedNodeID: String?
    let onSelectNode: (String) -> Void

    private var edgeStatus: TopologyElementStatus { evaluation.edgeStatus(edge.id) }

    var body: some View {
        HStack(spacing: 8) {
            TopologyNodeButton(
                node: source,
                status: evaluation.nodeStatus(source.id),
                selected: selectedNodeID == source.id,
                onSelect: { onSelectNode(source.id) }
            )
            connection
            TopologyNodeButton(
                node: destination,
                status: evaluation.nodeStatus(destination.id),
                selected: selectedNodeID == destination.id,
                onSelect: { onSelectNode(destination.id) }
            )
        }
    }

    private var connection: some View {
        VStack(spacing: 5) {
            HStack(spacing: 4) {
                Rectangle().fill(signalColor(edgeStatus.signal).opacity(0.75)).frame(height: 1.5)
                Image(systemName: "arrowtriangle.right.fill").font(.system(size: 7)).foregroundStyle(signalColor(edgeStatus.signal))
            }
            Text(edge.label).font(.system(size: 9, weight: .medium)).foregroundStyle(Palette.muted).lineLimit(1)
            Text(edge.evidence.rawValue).font(.system(size: 7)).foregroundStyle(Palette.muted).lineLimit(1)
            TopologyStatusBadge(signal: edgeStatus.signal, compact: true)
            if edgeStatus.possiblyAffectedDownstream {
                Text("可能受影响").font(.system(size: 8, weight: .medium)).foregroundStyle(Palette.warning)
            }
        }
        .frame(minWidth: 110, maxWidth: .infinity)
        .accessibilityIdentifier("topology.edge.\(edge.id)")
        .help(edgeStatus.note)
    }
}

private struct TopologyNodeButton: View {
    let node: TopologyNodeDefinition
    let status: TopologyElementStatus
    let selected: Bool
    let onSelect: () -> Void

    var body: some View {
        Button(action: onSelect) {
            VStack(alignment: .leading, spacing: 7) {
                HStack(alignment: .top, spacing: 6) {
                    Circle().fill(signalColor(status.signal)).frame(width: 7, height: 7).padding(.top, 4)
                    Text(node.title).font(.system(size: 10, weight: .semibold)).lineLimit(2).multilineTextAlignment(.leading)
                    Spacer(minLength: 0)
                    if status.possiblyAffectedDownstream {
                        Image(systemName: "arrow.down.right.circle.fill").font(.system(size: 10)).foregroundStyle(Palette.warning)
                            .help("存在上游异常；此节点仅标记可能受影响，状态没有被推断为异常。")
                    }
                }
                Text(status.signal.label).font(.system(size: 8, weight: .medium)).foregroundStyle(signalColor(status.signal)).lineLimit(1)
            }
            .frame(width: 152)
            .frame(minHeight: 55, alignment: .leading)
            .padding(.horizontal, 10).padding(.vertical, 8)
            .background(selected ? Palette.accent.opacity(0.13) : Palette.panelRaised, in: RoundedRectangle(cornerRadius: 10))
            .overlay(RoundedRectangle(cornerRadius: 10).stroke(selected ? Palette.accent.opacity(0.7) : Palette.line, lineWidth: 1))
            .contentShape(RoundedRectangle(cornerRadius: 10))
        }
        .buttonStyle(.plain)
        .accessibilityIdentifier("topology.node.\(node.id)")
        .help(status.note)
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

private struct TopologyNodeEvidencePanel: View {
    let node: TopologyNodeDefinition
    let status: TopologyElementStatus
    let context: TopologyRuntimeContext
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

private func signalColor(_ signal: TopologySignal) -> Color {
    switch signal {
    case .normal: return Palette.accent
    case .abnormal: return Palette.danger
    case .unknown, .notRunning, .processNotFound: return Palette.warning
    case .processObserved: return Palette.blue
    case .topicObserved: return Palette.blue
    case .topicNotObserved: return Palette.muted
    }
}
