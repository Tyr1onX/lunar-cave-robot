import SwiftUI

struct RobotTopologyView: View {
    @ObservedObject var model: ConsoleModel
    @State private var expandedPhaseIDs: Set<String> = []
    @State private var selectedStepID: String?
    @State private var selectedNodeID: String?
    @State private var showingCapabilityView = false
    @State private var showingAdvancedTopology = false

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
    private var selectedStep: RobotWorkflowStepDefinition? {
        selectedStepID.flatMap(RobotSystemWorkflowCatalog.step)
    }
    private var selectedNode: TopologyNodeDefinition? {
        selectedNodeID.flatMap(RobotTopologyCatalog.node)
    }

    var body: some View {
        Group {
            if showingAdvancedTopology {
                AdvancedRobotTopologyView(model: model) {
                    showingAdvancedTopology = false
                }
            } else if showingCapabilityView {
                RobotCapabilityTopologyView(model: model) {
                    showingCapabilityView = false
                }
            } else {
                workflowHome
            }
        }
        .frame(maxWidth: 1_560, alignment: .topLeading)
        .frame(maxWidth: .infinity, alignment: .top)
        .accessibilityIdentifier("robot-workflow.view")
    }

    private var workflowHome: some View {
        VStack(alignment: .leading, spacing: 18) {
            pageHeading

            if let selectedStep {
                ViewThatFits(in: .horizontal) {
                    HStack(alignment: .top, spacing: 18) {
                        stageFlow
                            .frame(minWidth: 1_060, maxWidth: .infinity, alignment: .topLeading)
                        detailColumn(for: selectedStep)
                            .frame(width: 360, alignment: .topLeading)
                    }
                    VStack(alignment: .leading, spacing: 18) {
                        stageFlow
                        detailColumn(for: selectedStep)
                    }
                }
            } else {
                stageFlow
            }
        }
        .accessibilityIdentifier("robot-workflow.home")
    }

    private var pageHeading: some View {
        HStack(alignment: .center, spacing: 12) {
            VStack(alignment: .leading, spacing: 5) {
                Text("机器人工作流程")
                    .font(.system(size: 28, weight: .semibold, design: .rounded))
                    .accessibilityIdentifier("robot-workflow.heading")
                Text("从环境感知到控制器接口与软件反馈。阶段关系按源码、Launch 和离线验证分别标注。")
                    .font(.system(size: 11))
                    .foregroundStyle(Palette.muted)
            }
            Spacer(minLength: 8)
            menuLinks
        }
    }

    private var menuLinks: some View {
        HStack(spacing: 12) {
            Button {
                showingCapabilityView = true
                clearSelection()
            } label: {
                Label("按能力查看", systemImage: "rectangle.3.group")
            }
            .accessibilityIdentifier("robot-workflow.capability-view")

            Button {
                showingAdvancedTopology = true
                clearSelection()
            } label: {
                Label("工程拓扑", systemImage: "point.3.connected.trianglepath.dotted")
            }
            .accessibilityIdentifier("robot-workflow.advanced-view")

            Button {
                model.section = .diagnostics
            } label: {
                Label("系统诊断", systemImage: "waveform.path.ecg")
            }
            .accessibilityIdentifier("robot-workflow.diagnostics")
        }
        .font(.system(size: 10, weight: .medium))
        .foregroundStyle(Palette.muted)
        .buttonStyle(.plain)
        .labelStyle(.titleAndIcon)
    }

    @ViewBuilder
    private var stageFlow: some View {
        ViewThatFits(in: .horizontal) {
            HStack(alignment: .top, spacing: 10) {
                ForEach(Array(RobotSystemWorkflowCatalog.phases.enumerated()), id: \.element.id) { index, phase in
                    WorkflowPhaseCard(
                        phase: phase,
                        expanded: expandedPhaseIDs.contains(phase.id),
                        selectedStepID: selectedStepID,
                        evaluation: evaluation,
                        onToggle: { toggle(phase) },
                        onSelectStep: { select($0) }
                    )
                    .frame(minWidth: 244, maxWidth: .infinity, alignment: .topLeading)
                    .accessibilityIdentifier("robot-workflow.phase.\(phase.id)")

                    if index < RobotSystemWorkflowCatalog.phases.count - 1 {
                        WorkflowPhaseArrow()
                            .frame(width: 16)
                            .padding(.top, 23)
                            .accessibilityIdentifier("robot-workflow.phase-arrow.\(phase.id)")
                    }
                }
            }
            .frame(minWidth: 1_040, alignment: .topLeading)

            VStack(alignment: .leading, spacing: 10) {
                ForEach(Array(RobotSystemWorkflowCatalog.phases.enumerated()), id: \.element.id) { index, phase in
                    WorkflowPhaseCard(
                        phase: phase,
                        expanded: expandedPhaseIDs.contains(phase.id),
                        selectedStepID: selectedStepID,
                        evaluation: evaluation,
                        onToggle: { toggle(phase) },
                        onSelectStep: { select($0) }
                    )
                    .accessibilityIdentifier("robot-workflow.phase.\(phase.id)")
                    if index < RobotSystemWorkflowCatalog.phases.count - 1 {
                        WorkflowPhaseArrow(vertical: true)
                            .frame(height: 13)
                            .frame(maxWidth: .infinity)
                            .accessibilityIdentifier("robot-workflow.phase-arrow.\(phase.id)")
                    }
                }
            }
            .frame(maxWidth: .infinity, alignment: .topLeading)
        }
        .accessibilityIdentifier("robot-workflow.stage-flow")
    }

    private func detailColumn(for step: RobotWorkflowStepDefinition) -> some View {
        VStack(alignment: .leading, spacing: 12) {
            WorkflowStepInspector(
                step: step,
                evaluation: evaluation,
                onSelectNode: { selectedNodeID = $0 }
            )
            if let selectedNode {
                TopologyNodeEvidencePanel(
                    node: selectedNode,
                    status: evaluation.nodeStatus(selectedNode.id),
                    evaluation: evaluation,
                    context: context,
                    linkTelemetryMessage: model.linkTelemetryMessage,
                    onClose: { selectedNodeID = nil }
                )
            }
        }
        .accessibilityIdentifier("robot-workflow.detail-column")
    }

    private func toggle(_ phase: RobotWorkflowPhaseDefinition) {
        if expandedPhaseIDs.contains(phase.id) {
            expandedPhaseIDs.remove(phase.id)
            if let selectedStep, phase.stepIDs.contains(selectedStep.id) {
                selectedStepID = nil
                selectedNodeID = nil
            }
        } else {
            expandedPhaseIDs.insert(phase.id)
        }
    }

    private func select(_ stepID: String) {
        selectedStepID = stepID
        selectedNodeID = nil
        if let phase = RobotSystemWorkflowCatalog.phases.first(where: { $0.stepIDs.contains(stepID) }) {
            expandedPhaseIDs.insert(phase.id)
        }
    }

    private func clearSelection() {
        selectedStepID = nil
        selectedNodeID = nil
    }
}

private struct WorkflowPhaseCard: View {
    let phase: RobotWorkflowPhaseDefinition
    let expanded: Bool
    let selectedStepID: String?
    let evaluation: TopologyEvaluation
    let onToggle: () -> Void
    let onSelectStep: (String) -> Void

    private var actionableSignal: TopologySignal? {
        let edgeIDs = phase.lanes.flatMap { $0.steps.flatMap(\.incomingEdgeIDs) } + phase.internalEdgeIDs +
            RobotSystemWorkflowCatalog.interfaces.filter { $0.sourcePhaseID == phase.id || $0.destinationPhaseID == phase.id }.flatMap(\.edgeIDs)
        let statuses = phase.nodeIDs.map { evaluation.nodeStatus($0) } + edgeIDs.map { evaluation.edgeStatus($0) }
        if statuses.contains(where: { $0.signal == .abnormal }) { return .abnormal }
        if statuses.contains(where: { $0.signal == .dataStale }) { return .dataStale }
        return nil
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            Button(action: onToggle) {
                VStack(alignment: .leading, spacing: 9) {
                    HStack(alignment: .firstTextBaseline, spacing: 8) {
                        Text(phase.number)
                            .font(.system(size: 11, weight: .semibold, design: .monospaced))
                            .foregroundStyle(Palette.accent)
                        Text(phase.title)
                            .font(.system(size: 15, weight: .semibold, design: .rounded))
                            .foregroundStyle(.white)
                            .fixedSize(horizontal: false, vertical: true)
                        Spacer(minLength: 3)
                        if let actionableSignal {
                            HStack(spacing: 5) {
                                Circle().fill(actionableSignal == .abnormal ? Palette.danger : Palette.warning).frame(width: 6, height: 6)
                                Text(actionableSignal == .abnormal ? "异常" : "数据延迟")
                                    .font(.system(size: 9, weight: .medium))
                                    .foregroundStyle(actionableSignal == .abnormal ? Palette.danger : Palette.warning)
                            }
                            .accessibilityIdentifier("robot-workflow.phase-actionable.\(phase.id)")
                        }
                        Image(systemName: expanded ? "chevron.down" : "chevron.right")
                            .font(.system(size: 9, weight: .semibold))
                            .foregroundStyle(Palette.muted)
                    }
                    Text(phase.summary)
                        .font(.system(size: 10))
                        .foregroundStyle(Palette.muted)
                        .multilineTextAlignment(.leading)
                        .fixedSize(horizontal: false, vertical: true)
                    if !expanded {
                        VStack(alignment: .leading, spacing: 5) {
                            ForEach(phase.lanes) { lane in
                                HStack(alignment: .top, spacing: 6) {
                                    Circle().fill(Palette.accent.opacity(0.55)).frame(width: 4, height: 4).padding(.top, 4)
                                    Text("\(lane.title)：\(lane.summary)")
                                        .font(.system(size: 9))
                                        .foregroundStyle(Palette.muted.opacity(0.92))
                                        .fixedSize(horizontal: false, vertical: true)
                                }
                            }
                        }
                        .padding(.top, 1)
                    }
                }
                .contentShape(Rectangle())
            }
            .buttonStyle(.plain)
            .accessibilityIdentifier("robot-workflow.phase-toggle.\(phase.id)")

            if expanded {
                Rectangle().fill(Palette.line).frame(height: 1)
                ForEach(phase.lanes) { lane in
                    laneView(lane)
                }
                if !phase.internalEdgeIDs.isEmpty {
                    WorkflowInterfaceList(title: "阶段内协同接口", edgeIDs: phase.internalEdgeIDs, evaluation: evaluation)
                }
                let outgoingGroups = RobotSystemWorkflowCatalog.interfaces.filter { $0.sourcePhaseID == phase.id }
                ForEach(outgoingGroups) { group in
                    WorkflowInterfaceList(
                        title: group.title,
                        edgeIDs: group.edgeIDs,
                        evaluation: evaluation,
                        isFeedback: group.isFeedback
                    )
                }
                if phase.id == "execution" {
                    Text("底盘当前协议没有控制板确认回包；实体急停接口未接入监测。")
                        .font(.system(size: 9))
                        .foregroundStyle(Palette.muted)
                        .fixedSize(horizontal: false, vertical: true)
                        .accessibilityIdentifier("robot-workflow.execution-boundary-note")
                }
            }
        }
        .padding(13)
        .frame(maxWidth: .infinity, alignment: .topLeading)
        .background(Palette.panel, in: RoundedRectangle(cornerRadius: 15, style: .continuous))
        .overlay(RoundedRectangle(cornerRadius: 15, style: .continuous).stroke(expanded ? Palette.accent.opacity(0.28) : Palette.line, lineWidth: 1))
    }

    private func laneView(_ lane: RobotWorkflowLaneDefinition) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            VStack(alignment: .leading, spacing: 3) {
                Text(lane.title)
                    .font(.system(size: 10, weight: .semibold))
                    .foregroundStyle(.white)
                Text(lane.summary)
                    .font(.system(size: 8))
                    .foregroundStyle(Palette.muted)
                    .fixedSize(horizontal: false, vertical: true)
            }
            ForEach(Array(lane.steps.enumerated()), id: \.element.id) { index, step in
                WorkflowStepRow(
                    step: step,
                    selected: step.id == selectedStepID,
                    evaluation: evaluation,
                    action: { onSelectStep(step.id) }
                )
                if index < lane.steps.count - 1 {
                    WorkflowStepArrow()
                }
            }
        }
        .padding(.top, 2)
        .accessibilityIdentifier("robot-workflow.lane.\(lane.id)")
    }
}

private struct WorkflowStepRow: View {
    let step: RobotWorkflowStepDefinition
    let selected: Bool
    let evaluation: TopologyEvaluation
    let action: () -> Void

    private var actionableSignal: TopologySignal? {
        let signals = step.nodeIDs.map { evaluation.nodeStatus($0).signal } + step.incomingEdgeIDs.map { evaluation.edgeStatus($0).signal }
        if signals.contains(.abnormal) { return .abnormal }
        if signals.contains(.dataStale) { return .dataStale }
        return nil
    }

    var body: some View {
        Button(action: action) {
            HStack(alignment: .center, spacing: 8) {
                RoundedRectangle(cornerRadius: 2)
                    .fill(actionableSignal.map { $0 == .abnormal ? Palette.danger : Palette.warning } ?? Palette.accent.opacity(selected ? 0.9 : 0.4))
                    .frame(width: 3, height: 26)
                VStack(alignment: .leading, spacing: 3) {
                    Text(step.title)
                        .font(.system(size: 10, weight: .medium))
                        .foregroundStyle(.white)
                        .fixedSize(horizontal: false, vertical: true)
                    Text(step.output)
                        .font(.system(size: 8))
                        .foregroundStyle(Palette.muted)
                        .lineLimit(2)
                        .fixedSize(horizontal: false, vertical: true)
                }
                Spacer(minLength: 2)
                if let actionableSignal {
                    Image(systemName: actionableSignal == .abnormal ? "exclamationmark.circle.fill" : "clock.fill")
                        .font(.system(size: 9))
                        .foregroundStyle(actionableSignal == .abnormal ? Palette.danger : Palette.warning)
                        .accessibilityIdentifier("robot-workflow.step-actionable.\(step.id)")
                }
            }
            .padding(.horizontal, 8)
            .padding(.vertical, 7)
            .frame(maxWidth: .infinity, alignment: .leading)
            .background(selected ? Palette.panelRaised : Color.white.opacity(0.025), in: RoundedRectangle(cornerRadius: 8, style: .continuous))
            .overlay(RoundedRectangle(cornerRadius: 8, style: .continuous).stroke(selected ? Palette.accent.opacity(0.45) : Palette.line.opacity(0.6), lineWidth: 1))
            .contentShape(RoundedRectangle(cornerRadius: 8, style: .continuous))
        }
        .buttonStyle(.plain)
        .accessibilityIdentifier("robot-workflow.step.\(step.id)")
    }
}

private struct WorkflowStepInspector: View {
    let step: RobotWorkflowStepDefinition
    let evaluation: TopologyEvaluation
    let onSelectNode: (String) -> Void

    private var relatedEdges: [TopologyEdgeDefinition] {
        step.incomingEdgeIDs.compactMap(RobotTopologyCatalog.edge)
    }

    private var actionableSignal: TopologySignal? {
        let signals = step.nodeIDs.map { evaluation.nodeStatus($0).signal } + step.incomingEdgeIDs.map { evaluation.edgeStatus($0).signal }
        if signals.contains(.abnormal) { return .abnormal }
        if signals.contains(.dataStale) { return .dataStale }
        return nil
    }

    var body: some View {
        Panel(title: "\(step.title) · 工作原理", subtitle: "先读输入、处理和输出，再查看技术依据") {
            VStack(alignment: .leading, spacing: 13) {
                if let actionableSignal {
                    Label(actionableSignal == .abnormal ? "存在直接异常证据" : "存在数据延迟证据", systemImage: "exclamationmark.circle.fill")
                        .font(.system(size: 10, weight: .medium))
                        .foregroundStyle(actionableSignal == .abnormal ? Palette.danger : Palette.warning)
                        .accessibilityIdentifier("robot-workflow.step-inspector.alert")
                }
                WorkflowIOField(title: "输入", value: step.input)
                WorkflowIOField(title: "处理", value: step.process)
                WorkflowIOField(title: "输出", value: step.output)
                if !step.note.isEmpty {
                    Text(step.note)
                        .font(.system(size: 10))
                        .foregroundStyle(Palette.muted)
                        .fixedSize(horizontal: false, vertical: true)
                }
                if !step.incomingEdgeIDs.isEmpty {
                    VStack(alignment: .leading, spacing: 7) {
                        Text("接口与连接依据")
                            .font(.system(size: 9, weight: .semibold))
                            .foregroundStyle(Palette.muted)
                        ForEach(relatedEdges) { edge in
                            VStack(alignment: .leading, spacing: 3) {
                                Text("\(TopologyFriendlyNames.node(edge.sourceNodeID)) → \(TopologyFriendlyNames.node(edge.destinationNodeID)) · \(edge.label)")
                                    .font(.system(size: 9, weight: .medium))
                                    .foregroundStyle(.white)
                                    .fixedSize(horizontal: false, vertical: true)
                                Text("\(edge.evidence.rawValue)\(edge.topics.isEmpty ? "" : " · " + edge.topics.joined(separator: " / "))")
                                    .font(.system(size: 8, design: .monospaced))
                                    .foregroundStyle(Palette.muted)
                                    .fixedSize(horizontal: false, vertical: true)
                            }
                            .padding(.vertical, 2)
                        }
                    }
                }
                VStack(alignment: .leading, spacing: 6) {
                    Text("查看技术节点与实时证据")
                        .font(.system(size: 9, weight: .semibold))
                        .foregroundStyle(Palette.muted)
                    ForEach(step.nodeIDs, id: \.self) { nodeID in
                        Button {
                            onSelectNode(nodeID)
                        } label: {
                            HStack(spacing: 6) {
                                Image(systemName: "chevron.right")
                                    .font(.system(size: 8, weight: .semibold))
                                    .foregroundStyle(Palette.accent)
                                Text(TopologyFriendlyNames.node(nodeID))
                                    .font(.system(size: 10, weight: .medium))
                                    .foregroundStyle(.white)
                                Spacer()
                                if let definition = RobotTopologyCatalog.node(nodeID) {
                                    Text(definition.groupID)
                                        .font(.system(size: 8, design: .monospaced))
                                        .foregroundStyle(Palette.muted)
                                }
                            }
                            .padding(.vertical, 4)
                            .contentShape(Rectangle())
                        }
                        .buttonStyle(.plain)
                        .accessibilityIdentifier("robot-workflow.step-node.\(nodeID)")
                    }
                }
            }
        }
        .accessibilityIdentifier("robot-workflow.step-inspector.\(step.id)")
    }
}

private struct WorkflowInterfaceList: View {
    let title: String
    let edgeIDs: [String]
    let evaluation: TopologyEvaluation
    var isFeedback = false

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack(spacing: 5) {
                Image(systemName: isFeedback ? "arrow.uturn.backward" : "arrow.right")
                    .font(.system(size: 8, weight: .semibold))
                Text(title).font(.system(size: 9, weight: .semibold))
            }
            .foregroundStyle(isFeedback ? Palette.blue : Palette.muted)
            ForEach(edgeIDs, id: \.self) { id in
                if let edge = RobotTopologyCatalog.edge(id) {
                    HStack(alignment: .top, spacing: 5) {
                        Image(systemName: "arrow.right")
                            .font(.system(size: 7, weight: .semibold))
                            .foregroundStyle(edge.evidence == .conditionalSourceWiring ? Palette.warning : Palette.accent.opacity(0.8))
                            .padding(.top, 2)
                        VStack(alignment: .leading, spacing: 2) {
                            Text("\(TopologyFriendlyNames.node(edge.sourceNodeID)) → \(TopologyFriendlyNames.node(edge.destinationNodeID)) · \(edge.label)")
                                .font(.system(size: 8, weight: .medium))
                                .foregroundStyle(.white.opacity(0.9))
                                .fixedSize(horizontal: false, vertical: true)
                            HStack(alignment: .firstTextBaseline, spacing: 4) {
                                Text(edge.evidence.rawValue)
                                    .foregroundStyle(edge.evidence == .conditionalSourceWiring ? Palette.warning : Palette.muted)
                                if !edge.topics.isEmpty {
                                    Text("· \(edge.topics.joined(separator: " / "))").foregroundStyle(Palette.muted)
                                }
                            }
                            .font(.system(size: 7, design: .monospaced))
                            .fixedSize(horizontal: false, vertical: true)
                            if evaluation.edgeStatus(id).signal == .abnormal || evaluation.edgeStatus(id).signal == .dataStale {
                                Text(evaluation.edgeStatus(id).note)
                                    .font(.system(size: 8))
                                    .foregroundStyle(evaluation.edgeStatus(id).signal == .abnormal ? Palette.danger : Palette.warning)
                                    .fixedSize(horizontal: false, vertical: true)
                            }
                        }
                    }
                    .accessibilityIdentifier("robot-workflow.interface.\(id)")
                }
            }
        }
        .padding(.top, 7)
        .overlay(alignment: .top) { Rectangle().fill(Palette.line).frame(height: 1) }
    }
}

private struct WorkflowIOField: View {
    let title: String
    let value: String

    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(title)
                .font(.system(size: 8, weight: .semibold))
                .foregroundStyle(Palette.accent)
            Text(value)
                .font(.system(size: 10))
                .foregroundStyle(.white.opacity(0.9))
                .fixedSize(horizontal: false, vertical: true)
        }
        .accessibilityIdentifier("robot-workflow.io.\(title)")
    }
}

private struct WorkflowStepArrow: View {
    var body: some View {
        HStack(spacing: 0) {
            Rectangle().fill(Palette.accent.opacity(0.35)).frame(width: 1, height: 10)
            Image(systemName: "chevron.down")
                .font(.system(size: 7, weight: .semibold))
                .foregroundStyle(Palette.accent.opacity(0.7))
                .offset(x: -3)
        }
        .frame(height: 8)
        .accessibilityHidden(true)
    }
}

private struct WorkflowPhaseArrow: View {
    var vertical = false

    var body: some View {
        Image(systemName: vertical ? "arrow.down" : "arrow.right")
            .font(.system(size: vertical ? 11 : 12, weight: .medium))
            .foregroundStyle(Palette.accent.opacity(0.7))
            .frame(maxWidth: .infinity, alignment: .top)
            .accessibilityHidden(true)
    }
}
