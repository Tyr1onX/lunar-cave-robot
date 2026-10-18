import SwiftUI

struct RobotCapabilityTopologyView: View {
    @ObservedObject var model: ConsoleModel
    let onReturnToWorkflow: () -> Void
    @State private var selectedCapabilityID: String?
    @State private var selectedStepID: String?
    @State private var selectedNodeID: String?
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
    private var selectedCapability: RobotCapabilityDefinition? {
        selectedCapabilityID.flatMap(RobotCapabilityCatalog.capability)
    }
    private var selectedStep: RobotCapabilityStepDefinition? {
        guard let selectedCapabilityID, let selectedStepID else { return nil }
        return RobotCapabilityCatalog.step(capabilityID: selectedCapabilityID, stepID: selectedStepID)
    }
    private var selectedNode: TopologyNodeDefinition? {
        selectedNodeID.flatMap(RobotTopologyCatalog.node)
    }

    var body: some View {
        Group {
            if showingAdvancedTopology {
                AdvancedRobotTopologyView(model: model) {
                    showingAdvancedTopology = false
                    selectedCapabilityID = nil
                    selectedStepID = nil
                    selectedNodeID = nil
                }
            } else if let selectedCapability {
                capabilityPage(selectedCapability)
            } else {
                landingPage
            }
        }
        .frame(maxWidth: 1_180, alignment: .topLeading)
        .frame(maxWidth: .infinity, alignment: .top)
        .accessibilityIdentifier("robot-capability.view")
    }

    private var landingPage: some View {
        VStack(alignment: .leading, spacing: 26) {
            VStack(alignment: .leading, spacing: 8) {
                Text("机器人能力")
                    .font(.system(size: 29, weight: .semibold, design: .rounded))
                    .accessibilityIdentifier("capability.home.heading")
                Text("从机器人能完成的任务出发，逐步了解每个环节。")
                    .font(.system(size: 13))
                    .foregroundStyle(Palette.muted)
            }

            CapabilityObservationSummary(hint: RobotCapabilityStatusSummary.landingHint(for: evaluation))

            ViewThatFits(in: .horizontal) {
                HStack(alignment: .top, spacing: 14) {
                    capabilityEntries(minimumWidth: 285)
                }
                VStack(spacing: 12) {
                    capabilityEntries(minimumWidth: nil)
                }
            }

            HStack(spacing: 14) {
                Button(action: onReturnToWorkflow) {
                    Label("工作流程首页", systemImage: "arrow.uturn.backward")
                        .font(.system(size: 11, weight: .medium))
                        .foregroundStyle(Palette.muted)
                }
                .buttonStyle(.plain)
                .accessibilityIdentifier("capability.return-to-workflow")

                Button {
                    showingAdvancedTopology = true
                } label: {
                    Label("整机拓扑（高级视图）", systemImage: "point.3.connected.trianglepath.dotted")
                        .font(.system(size: 11, weight: .medium))
                        .foregroundStyle(Palette.muted)
                }
                .buttonStyle(.plain)
                .accessibilityIdentifier("capability.advanced-topology")

                Button {
                    model.section = .diagnostics
                } label: {
                    Label("系统诊断", systemImage: "waveform.path.ecg")
                        .font(.system(size: 11, weight: .medium))
                        .foregroundStyle(Palette.muted)
                }
                .buttonStyle(.plain)
            }
            .padding(.top, 2)
        }
        .accessibilityIdentifier("capability.home")
    }

    @ViewBuilder
    private func capabilityEntries(minimumWidth: CGFloat?) -> some View {
        ForEach(RobotCapabilityCatalog.capabilities) { capability in
            Button {
                selectedCapabilityID = capability.id
                selectedStepID = nil
                selectedNodeID = nil
            } label: {
                CapabilityEntryCard(capability: capability, hint: capabilityHint(capability))
            }
            .buttonStyle(.plain)
            .frame(minWidth: minimumWidth, maxWidth: .infinity, alignment: .leading)
            .accessibilityIdentifier("capability.entry.\(capability.id)")
        }
    }

    private func capabilityHint(_ capability: RobotCapabilityDefinition) -> RobotCapabilityObservationHint? {
        let hints = capability.steps.compactMap { RobotCapabilityStatusSummary.hint(for: $0, evaluation: evaluation) }
        if hints.contains(.directAnomaly) { return .directAnomaly }
        if hints.contains(.staleData) { return .staleData }
        return nil
    }

    private func capabilityPage(_ capability: RobotCapabilityDefinition) -> some View {
        VStack(alignment: .leading, spacing: 20) {
            HStack(alignment: .center, spacing: 14) {
                Button {
                    selectedCapabilityID = nil
                    selectedStepID = nil
                    selectedNodeID = nil
                } label: {
                    Label("所有能力", systemImage: "chevron.left")
                        .font(.system(size: 11, weight: .medium))
                        .foregroundStyle(Palette.muted)
                }
                .buttonStyle(.plain)
                .accessibilityIdentifier("capability.flow.back")

                Rectangle().fill(Palette.line).frame(width: 1, height: 18)
                Image(systemName: capability.symbol)
                    .font(.system(size: 19, weight: .medium))
                    .foregroundStyle(Palette.accent)
                VStack(alignment: .leading, spacing: 3) {
                    Text(capability.title)
                        .font(.system(size: 23, weight: .semibold, design: .rounded))
                        .accessibilityIdentifier("capability.flow.heading")
                    Text(capability.subtitle)
                        .font(.system(size: 11))
                        .foregroundStyle(Palette.muted)
                }
                Spacer()
            }

            CapabilityObservationSummary(hint: capabilitySummary(capability))

            if selectedStep == nil {
                workflowColumn(capability)
                    .frame(maxWidth: 1_120, alignment: .topLeading)
            } else {
                ViewThatFits(in: .horizontal) {
                    HStack(alignment: .top, spacing: 24) {
                        workflowColumn(capability)
                            .frame(minWidth: 500, maxWidth: 690, alignment: .topLeading)
                        inspector(capability)
                            .frame(width: 360, alignment: .topLeading)
                    }
                    VStack(alignment: .leading, spacing: 20) {
                        workflowColumn(capability)
                            .frame(maxWidth: 850, alignment: .topLeading)
                        inspector(capability)
                            .frame(maxWidth: 850, alignment: .topLeading)
                    }
                }
            }
        }
        .accessibilityIdentifier("capability.flow.\(capability.id)")
    }

    private func capabilitySummary(_ capability: RobotCapabilityDefinition) -> RobotCapabilityObservationHint {
        let hints = capability.steps.compactMap { RobotCapabilityStatusSummary.hint(for: $0, evaluation: evaluation) }
        if hints.contains(.directAnomaly) { return .directAnomaly }
        if hints.contains(.staleData) { return .staleData }
        if hints.contains(.evidencePending) { return .evidencePending }
        if hints.contains(.processNotFound) { return .processNotFound }
        if hints.contains(.observedWithoutHealthClaim) { return .observedWithoutHealthClaim }
        return .noDirectAnomalyEvidence
    }

    private func workflowColumn(_ capability: RobotCapabilityDefinition) -> some View {
        VStack(alignment: .leading, spacing: 0) {
            ForEach(Array(capability.steps.enumerated()), id: \.element.id) { index, step in
                CapabilityStepCard(
                    step: step,
                    index: index + 1,
                    selected: selectedStepID == step.id,
                    hint: RobotCapabilityStatusSummary.hint(for: step, evaluation: evaluation),
                    action: {
                        selectedStepID = step.id
                        selectedNodeID = nil
                    }
                )
                if index < capability.steps.count - 1,
                   RobotCapabilityFlowRelations.hasDefaultConnection(from: step, to: capability.steps[index + 1]) {
                    CapabilityFlowConnector()
                } else if index < capability.steps.count - 1 {
                    CapabilityFlowBreak()
                }
            }
        }
        .accessibilityIdentifier("capability.workflow.\(capability.id)")
    }

    @ViewBuilder
    private func inspector(_ capability: RobotCapabilityDefinition) -> some View {
        if let selectedNode {
            VStack(alignment: .leading, spacing: 9) {
                Button {
                    selectedNodeID = nil
                } label: {
                    Label("返回步骤说明", systemImage: "chevron.left")
                        .font(.system(size: 10, weight: .medium))
                        .foregroundStyle(Palette.muted)
                }
                .buttonStyle(.plain)
                .accessibilityIdentifier("capability.inspector.return-to-step")
                TopologyNodeEvidencePanel(
                    node: selectedNode,
                    status: evaluation.nodeStatus(selectedNode.id),
                    evaluation: evaluation,
                    context: context,
                    linkTelemetryMessage: model.linkTelemetryMessage,
                    pipelineDiagnosis: model.pipelineDiagnosis,
                    onClose: { selectedNodeID = nil },
                    onOpenPipelineDiagnostics: { model.section = .diagnostics }
                )
            }
        } else if let step = selectedStep {
            CapabilityStepInspector(
                capability: capability,
                step: step,
                evaluation: evaluation,
                onSelectNode: { selectedNodeID = $0 }
            )
        }
    }
}

enum RobotCapabilityFlowRelations {
    static func hasDefaultConnection(from sourceStep: RobotCapabilityStepDefinition, to destinationStep: RobotCapabilityStepDefinition) -> Bool {
        RobotTopologyCatalog.edges.contains { edge in
            sourceStep.nodeIDs.contains(edge.sourceNodeID) &&
                destinationStep.nodeIDs.contains(edge.destinationNodeID) &&
                !edge.isFeedback &&
                edge.evidence != .openInterface &&
                edge.evidence != .conditionalSourceWiring
        }
    }
}

private struct CapabilityEntryCard: View {
    let capability: RobotCapabilityDefinition
    let hint: RobotCapabilityObservationHint?

    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            HStack(alignment: .top) {
                Image(systemName: capability.symbol)
                    .font(.system(size: 21, weight: .medium))
                    .foregroundStyle(Palette.accent)
                    .frame(width: 42, height: 42)
                    .background(Palette.accent.opacity(0.12), in: RoundedRectangle(cornerRadius: 13))
                Spacer()
                Image(systemName: "arrow.up.right")
                    .font(.system(size: 11, weight: .medium))
                    .foregroundStyle(Palette.muted)
            }
            VStack(alignment: .leading, spacing: 7) {
                Text(capability.title)
                    .font(.system(size: 17, weight: .semibold, design: .rounded))
                    .foregroundStyle(.white)
                Text(capability.subtitle)
                    .font(.system(size: 11))
                    .foregroundStyle(Palette.muted)
                    .fixedSize(horizontal: false, vertical: true)
            }
            if let hint, hint == .directAnomaly || hint == .staleData {
                HStack(spacing: 6) {
                    Circle().fill(hint == .directAnomaly ? Palette.danger : Palette.warning).frame(width: 6, height: 6)
                    Text(hint.title)
                        .font(.system(size: 10, weight: .medium))
                        .foregroundStyle(hint == .directAnomaly ? Palette.danger : Palette.warning)
                }
            }
        }
        .padding(19)
        .frame(maxWidth: .infinity, minHeight: 164, alignment: .topLeading)
        .background(Palette.panel, in: RoundedRectangle(cornerRadius: 17, style: .continuous))
        .contentShape(RoundedRectangle(cornerRadius: 17, style: .continuous))
    }
}

private struct CapabilityObservationSummary: View {
    let hint: RobotCapabilityObservationHint

    private var color: Color {
        switch hint {
        case .directAnomaly: return Palette.danger
        case .staleData, .evidencePending: return Palette.warning
        case .processNotFound: return Palette.muted
        case .observedWithoutHealthClaim: return Palette.blue
        case .noDirectAnomalyEvidence: return Palette.muted
        }
    }

    var body: some View {
        HStack(spacing: 8) {
            Circle().fill(color).frame(width: 6, height: 6)
            Text(hint.title)
                .font(.system(size: 10, weight: .medium))
                .foregroundStyle(color)
            Text("·")
                .foregroundStyle(Palette.muted)
            Text("接口与结构证据不代表实体硬件健康")
                .font(.system(size: 10))
                .foregroundStyle(Palette.muted)
            Spacer(minLength: 0)
        }
        .padding(.vertical, 2)
        .accessibilityIdentifier("capability.observation-summary")
    }
}

private struct CapabilityStepCard: View {
    let step: RobotCapabilityStepDefinition
    let index: Int
    let selected: Bool
    let hint: RobotCapabilityObservationHint?
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            VStack(alignment: .leading, spacing: 13) {
                HStack(alignment: .firstTextBaseline, spacing: 10) {
                    Text(String(format: "%02d", index))
                        .font(.system(size: 10, weight: .medium, design: .monospaced))
                        .foregroundStyle(Palette.muted)
                    Text(step.title)
                        .font(.system(size: 14, weight: .semibold, design: .rounded))
                        .foregroundStyle(.white)
                    Spacer(minLength: 6)
                    if let hint, hint == .directAnomaly || hint == .staleData {
                        Text(hint.title)
                            .font(.system(size: 9, weight: .medium))
                            .foregroundStyle(hint == .directAnomaly ? Palette.danger : Palette.warning)
                    }
                    Image(systemName: "chevron.right")
                        .font(.system(size: 9, weight: .semibold))
                        .foregroundStyle(Palette.muted.opacity(0.8))
                }

                ViewThatFits(in: .horizontal) {
                    HStack(alignment: .top, spacing: 12) {
                        CapabilityIOField(title: "输入", value: step.input)
                            .frame(maxWidth: .infinity, alignment: .leading)
                        CapabilityIOField(title: "处理", value: step.process)
                            .frame(maxWidth: .infinity, alignment: .leading)
                        CapabilityIOField(title: "输出", value: step.output)
                            .frame(maxWidth: .infinity, alignment: .leading)
                    }
                    VStack(alignment: .leading, spacing: 9) {
                        CapabilityIOField(title: "输入", value: step.input)
                        CapabilityIOField(title: "处理", value: step.process)
                        CapabilityIOField(title: "输出", value: step.output)
                    }
                }
            }
            .padding(.horizontal, 15)
            .padding(.vertical, 14)
            .frame(maxWidth: .infinity, alignment: .leading)
            .background(selected ? Palette.panelRaised : Palette.panel.opacity(0.68), in: RoundedRectangle(cornerRadius: 13, style: .continuous))
            .overlay {
                RoundedRectangle(cornerRadius: 13, style: .continuous)
                    .stroke(selected ? Palette.accent.opacity(0.45) : Palette.line.opacity(0.42), lineWidth: 1)
            }
            .contentShape(RoundedRectangle(cornerRadius: 13, style: .continuous))
        }
        .buttonStyle(.plain)
        .accessibilityIdentifier("capability.flow.step.\(step.id)")
    }
}

private struct CapabilityIOField: View {
    let title: String
    let value: String

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title)
                .font(.system(size: 8, weight: .semibold))
                .foregroundStyle(Palette.muted)
            Text(value)
                .font(.system(size: 10))
                .foregroundStyle(.white.opacity(0.87))
                .fixedSize(horizontal: false, vertical: true)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

private struct CapabilityFlowConnector: View {
    var body: some View {
        VStack(spacing: 0) {
            Rectangle().fill(Palette.accent.opacity(0.34)).frame(width: 1, height: 12)
            Image(systemName: "chevron.down")
                .font(.system(size: 8, weight: .semibold))
                .foregroundStyle(Palette.accent.opacity(0.62))
                .frame(height: 12)
            Rectangle().fill(Palette.accent.opacity(0.34)).frame(width: 1, height: 12)
        }
        .frame(maxWidth: .infinity)
        .accessibilityHidden(true)
    }
}

private struct CapabilityFlowBreak: View {
    var body: some View {
        HStack(spacing: 10) {
            Rectangle().fill(Palette.line).frame(height: 1)
            Text("无已验证的系统连接")
                .font(.system(size: 8, weight: .medium))
                .foregroundStyle(Palette.muted)
                .fixedSize()
            Rectangle().fill(Palette.line).frame(height: 1)
        }
        .padding(.horizontal, 12)
        .frame(height: 26)
        .accessibilityIdentifier("capability.workflow.unconnected-boundary")
    }
}

private struct CapabilityStepInspector: View {
    let capability: RobotCapabilityDefinition
    let step: RobotCapabilityStepDefinition
    let evaluation: TopologyEvaluation
    let onSelectNode: (String) -> Void
    @State private var showingAdditionalConnections = false

    private var relatedNodeIDs: [String] {
        let edgeIDs = step.edgeIDs + (showingAdditionalConnections ? step.secondaryEdgeIDs + capability.feedbackEdgeIDs : [])
        var result = step.nodeIDs
        for edgeID in edgeIDs {
            guard let edge = RobotTopologyCatalog.edge(edgeID) else { continue }
            result.append(edge.sourceNodeID)
            result.append(edge.destinationNodeID)
        }
        var seen = Set<String>()
        return result.filter { seen.insert($0).inserted && RobotTopologyCatalog.node($0) != nil }
    }

    private var visibleEdgeIDs: [String] {
        var seen = Set<String>()
        return (step.edgeIDs + (showingAdditionalConnections ? step.secondaryEdgeIDs + capability.feedbackEdgeIDs : []))
            .filter { seen.insert($0).inserted }
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            VStack(alignment: .leading, spacing: 5) {
                Text("步骤说明")
                    .font(.system(size: 9, weight: .semibold))
                    .foregroundStyle(Palette.muted)
                Text(step.title)
                    .font(.system(size: 17, weight: .semibold, design: .rounded))
                    .foregroundStyle(.white)
                if let hint = RobotCapabilityStatusSummary.hint(for: step, evaluation: evaluation) {
                    HStack(spacing: 6) {
                        Circle().fill(capabilityHintColor(hint)).frame(width: 6, height: 6)
                        Text(hint.title).font(.system(size: 9, weight: .medium))
                            .foregroundStyle(capabilityHintColor(hint))
                    }
                }
            }

            VStack(alignment: .leading, spacing: 10) {
                CapabilityIOField(title: "输入", value: step.input)
                CapabilityIOField(title: "处理", value: step.process)
                CapabilityIOField(title: "输出", value: step.output)
            }

            if !step.note.isEmpty {
                Text(step.note)
                    .font(.system(size: 10))
                    .foregroundStyle(Palette.muted)
                    .fixedSize(horizontal: false, vertical: true)
            }

            VStack(alignment: .leading, spacing: 7) {
                Text("继续查看内部模块")
                    .font(.system(size: 9, weight: .semibold))
                    .foregroundStyle(Palette.muted)
                ForEach(relatedNodeIDs, id: \.self) { nodeID in
                    Button {
                        onSelectNode(nodeID)
                    } label: {
                        HStack(spacing: 7) {
                            Image(systemName: "chevron.right")
                                .font(.system(size: 8, weight: .semibold))
                                .foregroundStyle(Palette.accent)
                            Text(TopologyFriendlyNames.node(nodeID))
                                .font(.system(size: 10, weight: .medium))
                                .foregroundStyle(.white)
                            Spacer(minLength: 0)
                        }
                        .padding(.vertical, 4)
                        .contentShape(Rectangle())
                    }
                    .buttonStyle(.plain)
                    .accessibilityIdentifier("capability.inspector.node.\(nodeID)")
                }
            }

            if !visibleEdgeIDs.isEmpty {
                VStack(alignment: .leading, spacing: 8) {
                    Text("连接依据")
                        .font(.system(size: 9, weight: .semibold))
                        .foregroundStyle(Palette.muted)
                    ForEach(visibleEdgeIDs, id: \.self) { edgeID in
                        if let edge = RobotTopologyCatalog.edge(edgeID) {
                            CapabilityEdgeEvidenceRow(edge: edge, status: evaluation.edgeStatus(edgeID))
                        }
                    }
                }
            }

            let hiddenCount = step.secondaryEdgeIDs.count + capability.feedbackEdgeIDs.count
            if hiddenCount > 0 {
                Button {
                    showingAdditionalConnections.toggle()
                } label: {
                    Label(
                        showingAdditionalConnections ? "收起反馈与次要接口" : "查看反馈与次要接口（\(hiddenCount)）",
                        systemImage: showingAdditionalConnections ? "minus.circle" : "plus.circle"
                    )
                    .font(.system(size: 9, weight: .medium))
                    .foregroundStyle(Palette.muted)
                }
                .buttonStyle(.plain)
                .accessibilityIdentifier("capability.inspector.additional-connections")
            }
        }
        .padding(15)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Palette.panel, in: RoundedRectangle(cornerRadius: 15, style: .continuous))
        .accessibilityIdentifier("capability.step-inspector")
    }
}

private func capabilityHintColor(_ hint: RobotCapabilityObservationHint) -> Color {
    switch hint {
    case .directAnomaly: return Palette.danger
    case .staleData, .evidencePending: return Palette.warning
    case .processNotFound, .noDirectAnomalyEvidence: return Palette.muted
    case .observedWithoutHealthClaim: return Palette.blue
    }
}

private struct CapabilityEdgeEvidenceRow: View {
    let edge: TopologyEdgeDefinition
    let status: TopologyElementStatus

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack(alignment: .firstTextBaseline, spacing: 6) {
                Text("\(TopologyFriendlyNames.node(edge.sourceNodeID)) → \(TopologyFriendlyNames.node(edge.destinationNodeID))")
                    .font(.system(size: 9, weight: .medium))
                    .foregroundStyle(.white.opacity(0.9))
                    .fixedSize(horizontal: false, vertical: true)
                Spacer(minLength: 0)
            }
            Text("\(edge.evidence.rawValue) · \(edge.topics.isEmpty ? edge.label : edge.topics.joined(separator: " / "))")
                .font(.system(size: 8, design: .monospaced))
                .foregroundStyle(Palette.muted)
                .fixedSize(horizontal: false, vertical: true)
            Text(status.note)
                .font(.system(size: 8))
                .foregroundStyle(Palette.muted)
                .fixedSize(horizontal: false, vertical: true)
        }
        .padding(.vertical, 6)
        .overlay(alignment: .bottom) { Rectangle().fill(Palette.line).frame(height: 1) }
    }
}
