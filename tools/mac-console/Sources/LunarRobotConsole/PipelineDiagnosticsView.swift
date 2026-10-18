import AppKit
import SwiftUI

struct PipelineDiagnosticsPanel: View {
    @ObservedObject var model: ConsoleModel
    @State private var showingExpectationEditor = false
    @State private var exportMessage: String?

    private var diagnosis: PipelineDiagnosis { model.pipelineDiagnosis }
    private var selectedStage: PipelineStageDiagnosis? {
        let id = model.selectedPipelineStageID ?? diagnosis.findingStageID ?? .radar
        return diagnosis.stage(id)
    }

    var body: some View {
        Panel(title: "雷达导航链路定位", subtitle: "MID-360 → Point-LIO → SLAM → Nav2 · 只读证据，不代表实体硬件已验证") {
            VStack(alignment: .leading, spacing: 13) {
                HStack(alignment: .top, spacing: 12) {
                    VStack(alignment: .leading, spacing: 5) {
                        Text(diagnosis.summary)
                            .font(.system(size: 11, weight: .medium))
                            .foregroundStyle(diagnosis.findingStageID == nil ? Palette.muted : diagnosisColor(for: diagnosis.stage(diagnosis.findingStageID!)))
                            .fixedSize(horizontal: false, vertical: true)
                            .accessibilityIdentifier("pipeline-diagnostics.summary")
                        Text("场景：\(diagnosis.scenario.label) · 来源预期：\(diagnosis.scenario.workspaceRoot ?? "未配置") · Domain：实际 \(diagnosis.actualDomainID.map(String.init) ?? "未知") / 预期 \(diagnosis.expectedDomainID.map(String.init) ?? "未配置") · 观测：\(diagnosis.observedAt)")
                            .font(.system(size: 8, design: .monospaced))
                            .foregroundStyle(Palette.muted)
                            .textSelection(.enabled)
                    }
                    Spacer(minLength: 4)
                    Button {
                        showingExpectationEditor = true
                    } label: {
                        Label("运行预期", systemImage: "slider.horizontal.3")
                    }
                    .buttonStyle(.bordered)
                    .controlSize(.small)
                    .accessibilityIdentifier("pipeline-diagnostics.expectation")
                    Button(action: exportReport) {
                        Label("保存报告", systemImage: "square.and.arrow.down")
                    }
                    .buttonStyle(.bordered)
                    .controlSize(.small)
                    .accessibilityIdentifier("pipeline-diagnostics.export")
                }

                if let finding = diagnosis.firstFinding {
                    HStack(alignment: .top, spacing: 9) {
                        Image(systemName: finding.confidence == .confirmedExpectationMismatch ? "exclamationmark.triangle.fill" : "questionmark.circle.fill")
                            .foregroundStyle(finding.confidence == .confirmedExpectationMismatch ? Palette.danger : Palette.warning)
                        VStack(alignment: .leading, spacing: 4) {
                            Text("首个证据缺口 · \(finding.title)")
                                .font(.system(size: 11, weight: .semibold))
                                .foregroundStyle(.white)
                            Text(finding.observation)
                                .font(.system(size: 9))
                                .foregroundStyle(Palette.muted)
                                .fixedSize(horizontal: false, vertical: true)
                            Text("置信级别：\(finding.confidence.label) · 首次观测：\(finding.firstObservedAt) · \(finding.persistenceLabel)")
                                .font(.system(size: 8, design: .monospaced))
                                .foregroundStyle(Palette.muted)
                                .textSelection(.enabled)
                        }
                        Spacer(minLength: 0)
                    }
                    .padding(10)
                    .background(diagnosisColor(for: diagnosis.stage(finding.stageID)).opacity(0.08), in: RoundedRectangle(cornerRadius: 9))
                    .overlay(RoundedRectangle(cornerRadius: 9).stroke(diagnosisColor(for: diagnosis.stage(finding.stageID)).opacity(0.25), lineWidth: 1))
                    .accessibilityIdentifier("pipeline-diagnostics.first-finding")
                }

                ViewThatFits(in: .horizontal) {
                    HStack(alignment: .top, spacing: 14) {
                        stageList.frame(width: 270, alignment: .topLeading)
                        if let selectedStage {
                            PipelineStageInspector(stage: selectedStage, diagnosis: diagnosis)
                                .frame(maxWidth: .infinity, alignment: .topLeading)
                        }
                    }
                    VStack(alignment: .leading, spacing: 14) {
                        stageList
                        if let selectedStage {
                            PipelineStageInspector(stage: selectedStage, diagnosis: diagnosis)
                        }
                    }
                }

                Text("Agent 只订阅固定 Topic 并读取 ROS Graph 发布端点；不读取订阅端点、TF 树、Nav2 Action 结果或点云 / 地图内容。Header 到 Agent 延迟受主机时钟同步影响；同名 Topic 的工作区来源仍未知。")
                    .font(.system(size: 8))
                    .foregroundStyle(Palette.muted)
                    .fixedSize(horizontal: false, vertical: true)
                    .accessibilityIdentifier("pipeline-diagnostics.boundary")
                if let exportMessage {
                    Text(exportMessage).font(.system(size: 9)).foregroundStyle(Palette.muted)
                }
            }
        }
        .sheet(isPresented: $showingExpectationEditor) {
            PipelineExpectationEditor(expectation: model.pipelineExpectation) { expectation in
                model.updatePipelineExpectation(expectation)
                if let stageID = model.pipelineDiagnosis.findingStageID { model.selectPipelineStage(stageID) }
                showingExpectationEditor = false
            } onCancel: {
                showingExpectationEditor = false
            }
        }
    }

    private var stageList: some View {
        VStack(alignment: .leading, spacing: 5) {
            ForEach(Array(PipelineStageID.allCases.enumerated()), id: \.element) { index, stageID in
                if let stage = diagnosis.stage(stageID) {
                    Button {
                        model.selectPipelineStage(stageID)
                    } label: {
                        HStack(alignment: .top, spacing: 8) {
                            VStack(spacing: 3) {
                                Circle()
                                    .fill(diagnosisColor(for: stage))
                                    .frame(width: 8, height: 8)
                                    .padding(.top, 2)
                                if index < PipelineStageID.allCases.count - 1 {
                                    Rectangle().fill(Palette.line).frame(width: 1, height: 13)
                                }
                            }
                            VStack(alignment: .leading, spacing: 3) {
                                HStack {
                                    Text(stageID.title).font(.system(size: 10, weight: .semibold)).foregroundStyle(.white)
                                    Spacer(minLength: 4)
                                    if stage.state == .confirmedExpectationMismatch || stage.state == .suspectedGap {
                                        Text(stage.state == .confirmedExpectationMismatch ? "预期偏差" : "疑似")
                                            .font(.system(size: 8, weight: .semibold))
                                            .foregroundStyle(diagnosisColor(for: stage))
                                    }
                                }
                                Text(stage.summary)
                                    .font(.system(size: 8))
                                    .foregroundStyle(Palette.muted)
                                    .lineLimit(2)
                                    .multilineTextAlignment(.leading)
                                    .fixedSize(horizontal: false, vertical: true)
                            }
                        }
                        .padding(.horizontal, 8)
                        .padding(.vertical, 7)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .background(model.selectedPipelineStageID == stageID || (model.selectedPipelineStageID == nil && diagnosis.findingStageID == stageID) ? Palette.panelRaised : .clear, in: RoundedRectangle(cornerRadius: 8))
                        .contentShape(Rectangle())
                    }
                    .buttonStyle(.plain)
                    .accessibilityIdentifier("pipeline-diagnostics.stage.\(stageID.rawValue)")
                }
            }
        }
        .accessibilityIdentifier("pipeline-diagnostics.stages")
    }

    private func diagnosisColor(for stage: PipelineStageDiagnosis?) -> Color {
        guard let stage else { return Palette.muted }
        switch stage.state {
        case .confirmedExpectationMismatch: return Palette.danger
        case .suspectedGap, .possiblyAffected: return Palette.warning
        case .observed: return Palette.accent
        case .notConfigured, .unknown: return Palette.muted
        }
    }

    private func exportReport() {
        let panel = NSSavePanel()
        panel.title = "保存只读链路诊断报告"
        panel.nameFieldStringValue = PipelineReportFilename.make(for: .now)
        panel.canCreateDirectories = true
        guard panel.runModal() == .OK, let url = panel.url else { return }
        do {
            try reportText().write(to: url, atomically: true, encoding: .utf8)
            exportMessage = "报告已保存：\(url.path)"
        } catch {
            exportMessage = "报告保存失败：\(error.localizedDescription)"
        }
    }

    private func reportText() -> String {
        var lines = [
            "# Lunar Robot Console · 只读链路诊断报告",
            "",
            "- 场景：\(diagnosis.scenario.label)",
            "- 工作区预期：\(diagnosis.scenario.workspaceRoot ?? "未配置")",
            "- ROS Domain：实际 \(diagnosis.actualDomainID.map(String.init) ?? "未知")；预期 \(diagnosis.expectedDomainID.map(String.init) ?? "未配置")",
            "- 观测时间：\(diagnosis.observedAt)",
            "- 摘要：\(diagnosis.summary)",
            ""
        ]
        if let finding = diagnosis.firstFinding {
            lines += [
                "## 首个证据缺口",
                "",
                "- 环节：\(finding.title)",
                "- 置信级别：\(finding.confidence.label)",
                "- 首次观测：\(finding.firstObservedAt)",
                "- 持续性：\(finding.persistenceLabel)",
                "- 观测：\(finding.observation)",
                "- 相关接口：\(finding.evidenceTopics.joined(separator: ", "))",
                ""
            ]
        }
        for stage in diagnosis.stages {
            lines += ["## \(stage.id.title)", "", "- 状态：\(stage.state.label)", "- 证据级别：\(stage.confidence.label)", "- 说明：\(stage.summary)", "- 观测时间：\(stage.observedAt)", "- 最近数据时间：\(stage.dataTime)", ""]
            for stream in stage.inputEvidence + stage.outputEvidence {
                let latency = stream.headerToAgentMs.map { String(format: "%.1f ms", $0) } ?? "未知"
                lines.append("- \(stream.role) `\(stream.topic)`：\(stream.oneLineEvidence)；数据时间 \(stream.receivedAt ?? "未知")；ROS Header \(stream.sourceStamp ?? "未知")；Header 到 Agent \(latency)；工作区归属 \(stream.workspaceAttribution)；发布端点 \(stream.publisherEndpoints.joined(separator: ", "))")
            }
            for node in stage.nodeEvidence {
                lines.append("- 预期节点 `\(node.name)`：\(node.visible.map { $0 ? "已发现" : "未发现" } ?? "未知")；Graph 时间 \(node.observedAt ?? "未知")")
            }
            if !stage.graphOnlyTopics.isEmpty { lines.append("- 仅 Graph 名称可见的接口：\(stage.graphOnlyTopics.joined(separator: ", "))；不代表消息流动或 TF 内容有效。") }
            if !stage.possibleCauses.isEmpty { lines.append("- 可能原因：\(stage.possibleCauses.joined(separator: "；"))。这些是排查方向，不是已确认根因。") }
            lines.append("- 源码依据：\(stage.sourcePaths.joined(separator: "; "))")
            if !stage.logs.isEmpty {
                lines += ["", "### 相关只读日志"]
                lines += stage.logs.map { "- [\($0.time)] [\($0.severity)] \($0.source)：\($0.message)" }
            }
            lines.append("")
        }
        lines += ["## 观测边界", "", "Agent 不读取订阅端点、TF 树、Nav2 Action 结果或点云 / 地图内容。Topic 发布者与数据到达 Agent 不证明指定工作区、消费者处理、算法质量或实体硬件健康。Header 时间差需以时钟同步为前提。", ""]
        return lines.joined(separator: "\n")
    }
}

private struct PipelineStageInspector: View {
    let stage: PipelineStageDiagnosis
    let diagnosis: PipelineDiagnosis

    private var upstreamText: String {
        guard let index = PipelineStageID.allCases.firstIndex(of: stage.id), index > 0 else { return "起始环节" }
        let upstream = PipelineStageID.allCases[index - 1]
        let result = diagnosis.stage(upstream)
        return "\(upstream.title)：\(result?.state.label ?? "未知")"
    }

    private var downstreamText: String {
        guard let index = PipelineStageID.allCases.firstIndex(of: stage.id), index < PipelineStageID.allCases.count - 1 else { return "无后续试点环节" }
        return PipelineStageID.allCases.suffix(from: index + 1).map { id in
            "\(id.title)：\(diagnosis.stage(id)?.state.label ?? "未知")"
        }.joined(separator: "；")
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            HStack(alignment: .firstTextBaseline) {
                Text(stage.id.title).font(.system(size: 12, weight: .semibold, design: .rounded))
                Spacer()
                Text(stage.state.label).font(.system(size: 9, weight: .medium)).foregroundStyle(color)
            }
            Text(stage.summary).font(.system(size: 9)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
            Text("证据级别 · \(stage.confidence.label)    上游 · \(upstreamText)")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
            Text("下游 · \(downstreamText)")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true).textSelection(.enabled)
            HStack(spacing: 12) {
                Text("观测 \(stage.observedAt)")
                Text("最近数据 \(stage.dataTime)")
            }
            .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)

            if !stage.nodeEvidence.isEmpty {
                evidenceSection("预期节点") {
                    ForEach(stage.nodeEvidence) { node in
                        HStack(spacing: 6) {
                            Image(systemName: node.visible == true ? "checkmark.circle" : node.visible == false ? "xmark.circle" : "questionmark.circle")
                                .foregroundStyle(node.visible == false ? Palette.danger : Palette.muted)
                            Text(node.name).font(.system(size: 8, design: .monospaced))
                            Spacer()
                            Text(node.visible.map { $0 ? "Graph 已发现" : "Graph 未发现" } ?? "Graph 未确认")
                                .font(.system(size: 8)).foregroundStyle(Palette.muted)
                        }
                    }
                }
            }

            let streams = stage.inputEvidence + stage.outputEvidence
            if !streams.isEmpty {
                evidenceSection("Topic 实际接收证据") {
                    ForEach(streams) { stream in
                        VStack(alignment: .leading, spacing: 3) {
                            HStack {
                                Text("\(stream.role) · \(stream.topic)").font(.system(size: 8, weight: .medium, design: .monospaced))
                                Spacer()
                                Text(stream.status).font(.system(size: 8, weight: .semibold)).foregroundStyle(stream.isFresh ? Palette.accent : stream.hasGapEvidence ? Palette.warning : Palette.muted)
                            }
                            Text("\(stream.oneLineEvidence) · 接收 \(stream.receivedAt ?? "未知") · Header \(stream.sourceStamp ?? "未知")")
                                .font(.system(size: 7, design: .monospaced)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true).textSelection(.enabled)
                            Text("Header→Agent \(stream.headerToAgentMs.map { String(format: "%.1f ms", $0) } ?? "未知") · 端点 \(stream.publisherEndpoints.isEmpty ? "未提供" : stream.publisherEndpoints.joined(separator: ", ")) · 工作区归属 \(stream.workspaceAttribution)")
                                .font(.system(size: 7, design: .monospaced)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true).textSelection(.enabled)
                            if let reason = stream.reason {
                                Text(reason).font(.system(size: 7)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                            }
                        }
                        .padding(.vertical, 3)
                    }
                }
            }

            if !stage.graphOnlyTopics.isEmpty {
                evidenceSection("仅 Graph 名称可见") {
                    Text("\(stage.graphOnlyTopics.joined(separator: "、"))。Agent 未采集其消息频率、延迟、TF 内容或消费端状态。")
                        .font(.system(size: 8)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                }
            }

            evidenceSection("可能原因 · 尚未确认") {
                ForEach(stage.possibleCauses, id: \.self) { cause in
                    HStack(alignment: .top, spacing: 5) {
                        Text("·").foregroundStyle(Palette.warning)
                        Text(cause).font(.system(size: 8)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                    }
                }
            }

            evidenceSection("相关日志 · 只读") {
                if stage.logs.isEmpty {
                    Text("当前 Agent 日志没有匹配到此环节的 ROS / Topic 关键词。日志缺失不表示模块正常。")
                        .font(.system(size: 8)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                } else {
                    ForEach(stage.logs) { log in
                        VStack(alignment: .leading, spacing: 2) {
                            Text("[\(log.time)] [\(log.severity)] \(log.source)")
                                .font(.system(size: 7, design: .monospaced)).foregroundStyle(Palette.muted)
                            Text(log.message).font(.system(size: 8, design: .monospaced)).foregroundStyle(.white.opacity(0.82)).textSelection(.enabled).fixedSize(horizontal: false, vertical: true)
                        }
                        .padding(.vertical, 2)
                    }
                }
            }

            evidenceSection("源码与 Launch") {
                ForEach(stage.sourcePaths, id: \.self) { path in
                    Text(path).font(.system(size: 7, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled).fixedSize(horizontal: false, vertical: true)
                }
            }

            Text("Topic 接收只能证明 Agent 在当前 Domain 收到消息；订阅者是否接收、节点是否处理成功以及实体硬件状态均需独立证据。")
                .font(.system(size: 8)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
        }
        .padding(12)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Palette.panelRaised.opacity(0.5), in: RoundedRectangle(cornerRadius: 10))
        .overlay(RoundedRectangle(cornerRadius: 10).stroke(Palette.line, lineWidth: 1))
        .accessibilityIdentifier("pipeline-diagnostics.inspector.\(stage.id.rawValue)")
    }

    private var color: Color {
        switch stage.state {
        case .confirmedExpectationMismatch: return Palette.danger
        case .suspectedGap, .possiblyAffected: return Palette.warning
        case .observed: return Palette.accent
        case .notConfigured, .unknown: return Palette.muted
        }
    }

    @ViewBuilder
    private func evidenceSection<Content: View>(_ title: String, @ViewBuilder content: () -> Content) -> some View {
        VStack(alignment: .leading, spacing: 5) {
            Text(title).font(.system(size: 8, weight: .semibold)).foregroundStyle(.white.opacity(0.84))
            content()
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

private struct PipelineExpectationEditor: View {
    let onSave: (PipelineRunExpectation) -> Void
    let onCancel: () -> Void
    @State private var scenario: PipelineScenario
    @State private var domainText: String
    @State private var expectedNodesText: String
    @State private var navigationGoalActive: Bool
    @State private var validationMessage: String?

    init(expectation: PipelineRunExpectation, onSave: @escaping (PipelineRunExpectation) -> Void, onCancel: @escaping () -> Void) {
        self.onSave = onSave
        self.onCancel = onCancel
        _scenario = State(initialValue: expectation.scenario)
        _domainText = State(initialValue: expectation.expectedDomainID.map(String.init) ?? "")
        _expectedNodesText = State(initialValue: Self.serializedNodes(expectation.expectedNodesByStage))
        _navigationGoalActive = State(initialValue: expectation.navigationGoalActive)
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 15) {
            Text("配置只读运行预期").font(.system(size: 18, weight: .semibold, design: .rounded))
            Text("设置只保存在 Mac Console 本地，不写入 Agent、ROS 环境或工作区。未配置时不会把模块缺失判为异常。")
                .font(.system(size: 10)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)

            Picker("测试 / 运行场景", selection: $scenario) {
                ForEach(PipelineScenario.allCases) { item in
                    Text(item.label).tag(item)
                }
            }
            .accessibilityIdentifier("pipeline-expectation.scenario")

            TextField("预期 ROS Domain（0–232）", text: $domainText)
                .textFieldStyle(.roundedBorder)
                .disabled(!scenario.isConfigured)
                .accessibilityIdentifier("pipeline-expectation.domain")

            VStack(alignment: .leading, spacing: 5) {
                Text("精确预期节点（可选；每行 阶段=/节点名）")
                    .font(.system(size: 10, weight: .medium))
                Text("例如：radar=/livox_lidar_publisher。只有显式列出的节点才用于缺失判断；空白时不检查节点是否退出。")
                    .font(.system(size: 8)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                TextEditor(text: $expectedNodesText)
                    .font(.system(size: 9, design: .monospaced))
                    .frame(height: 82)
                    .scrollContentBackground(.hidden)
                    .background(Palette.panelRaised, in: RoundedRectangle(cornerRadius: 7))
                    .disabled(!scenario.isConfigured)
                    .accessibilityIdentifier("pipeline-expectation.nodes")
            }

            Toggle("当前有 Nav2 导航目标", isOn: $navigationGoalActive)
                .font(.system(size: 10))
                .disabled(!scenario.isConfigured)
                .accessibilityIdentifier("pipeline-expectation.nav-goal")
            Text("未启用该选项时，不要求 /cmd_vel 持续发布；没有导航目标不作为异常。")
                .font(.system(size: 8)).foregroundStyle(Palette.muted)

            if let validationMessage {
                Text(validationMessage).font(.system(size: 9)).foregroundStyle(Palette.warning)
            }

            HStack {
                Spacer()
                Button("取消", action: onCancel).keyboardShortcut(.cancelAction)
                Button("保存设置") { save() }
                    .keyboardShortcut(.defaultAction)
                    .accessibilityIdentifier("pipeline-expectation.save")
            }
        }
        .padding(22)
        .frame(minWidth: 560, minHeight: 420)
    }

    private func save() {
        let trimmedDomain = domainText.trimmingCharacters(in: .whitespacesAndNewlines)
        let domain: Int?
        if trimmedDomain.isEmpty {
            domain = nil
        } else if let value = Int(trimmedDomain) {
            domain = value
        } else {
            validationMessage = "Domain 需填写 0 到 232 的整数，或留空。"
            return
        }
        let nodes = parseNodes(expectedNodesText)
        let expectation = PipelineRunExpectation(
            scenario: scenario,
            expectedDomainID: domain,
            expectedNodesByStage: nodes,
            navigationGoalActive: scenario.isConfigured && navigationGoalActive
        )
        if scenario.isConfigured, domain == nil {
            validationMessage = "已保存场景；Domain 未填写时整段链路保持未知，不会产生下游缺失告警。"
            onSave(expectation)
            return
        }
        if let error = expectation.configurationError {
            validationMessage = error
            return
        }
        onSave(expectation)
    }

    private func parseNodes(_ text: String) -> [PipelineStageID: [String]] {
        var values: [PipelineStageID: [String]] = [:]
        for line in text.split(whereSeparator: \.isNewline) {
            let pair = line.split(separator: "=", maxSplits: 1).map { $0.trimmingCharacters(in: .whitespacesAndNewlines) }
            guard pair.count == 2,
                  let stage = PipelineStageID(rawValue: pair[0]),
                  !pair[1].isEmpty else { continue }
            values[stage, default: []].append(contentsOf: pair[1].split(separator: ",").map { $0.trimmingCharacters(in: .whitespacesAndNewlines) })
        }
        return values
    }

    private static func serializedNodes(_ nodes: [PipelineStageID: [String]]) -> String {
        PipelineStageID.allCases.flatMap { stage in
            (nodes[stage] ?? []).map { "\(stage.rawValue)=\($0)" }
        }.joined(separator: "\n")
    }
}
