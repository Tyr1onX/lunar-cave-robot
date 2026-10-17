import SwiftUI

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

struct TopologyNodeEvidencePanel: View {
    let node: TopologyNodeDefinition
    let status: TopologyElementStatus
    let evaluation: TopologyEvaluation
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
                    }
                    .buttonStyle(.plain)
                    .help("关闭节点详情")
                    .accessibilityIdentifier("topology.node-evidence.close")
                }

                neighborhoodSection

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

                VStack(alignment: .leading, spacing: 9) {
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

    private var neighborhoodSection: some View {
        let incoming = RobotTopologyCatalog.edges.filter { $0.destinationNodeID == node.id }
        let outgoing = RobotTopologyCatalog.edges.filter { $0.sourceNodeID == node.id }
        return VStack(alignment: .leading, spacing: 8) {
            evidenceHeading("当前位置 · \(TopologyFriendlyNames.subsystem(node.groupID))")
            relationshipGroup(title: "上游输入", edges: incoming, sourceNode: true)
            relationshipGroup(title: "下游输出", edges: outgoing, sourceNode: false)
        }
    }

    private func relationshipGroup(title: String, edges: [TopologyEdgeDefinition], sourceNode: Bool) -> some View {
        VStack(alignment: .leading, spacing: 5) {
            Text(title).font(.system(size: 9, weight: .medium)).foregroundStyle(Palette.muted)
            if edges.isEmpty {
                Text("没有已验证的连接关系")
                    .font(.system(size: 9)).foregroundStyle(Palette.muted)
            } else {
                ForEach(edges) { edge in
                    let peerID = sourceNode ? edge.sourceNodeID : edge.destinationNodeID
                    let edgeStatus = evaluation.edgeStatus(edge.id)
                    VStack(alignment: .leading, spacing: 2) {
                        HStack(alignment: .firstTextBaseline, spacing: 6) {
                            Text("\(sourceNode ? "←" : "→") \(TopologyFriendlyNames.node(peerID)) · \(edge.label)")
                                .font(.system(size: 9, weight: .medium)).foregroundStyle(.white)
                                .fixedSize(horizontal: false, vertical: true)
                            Spacer(minLength: 0)
                            TopologyStatusBadge(signal: edgeStatus.signal)
                        }
                        Text("\(edge.evidence.rawValue)\(edge.topics.isEmpty ? "" : " · " + edge.topics.joined(separator: " / "))")
                            .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
                            .fixedSize(horizontal: false, vertical: true)
                        Text(edgeStatus.note)
                            .font(.system(size: 8)).foregroundStyle(Palette.muted)
                            .fixedSize(horizontal: false, vertical: true)
                    }
                }
            }
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
                    .fixedSize(horizontal: false, vertical: true)
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
                .fixedSize(horizontal: false, vertical: true)
            ForEach(paths, id: \.self) { path in
                Text(path).font(.system(size: 9, design: .monospaced)).foregroundStyle(Palette.blue).textSelection(.enabled)
                    .fixedSize(horizontal: false, vertical: true)
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
                .fixedSize(horizontal: false, vertical: true)
            Text("订阅周期 #\(stream.subscriptionGeneration.map { String($0) } ?? "未知") 开始 \(stream.subscriptionStartedAt ?? "未知") · 当前周期接收 \(stream.receivedAt ?? "尚无")")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
                .fixedSize(horizontal: false, vertical: true)
            Text("源时间 \(stream.sourceStamp ?? "未知") · header 对照 \(stream.headerToAgentMs.map { String(format: "%.1f ms", $0) } ?? "未知") · frame \(stream.frameId ?? "未知")")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
                .fixedSize(horizontal: false, vertical: true)
            if let previous = stream.previousReceivedAt {
                Text("历史接收（仅供参考，不参与当前状态判定）\(previous) · 上一周期 #\(stream.previousSubscriptionGeneration.map { String($0) } ?? "未知")")
                    .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
            }
            Text("发布者 \(stream.publisherCount.map { String($0) } ?? "未知") · 端点 \(publisherSummary) · 工作区归属：未确认 · 样本尺寸 \(sampleSummary) · 接收间断 \(stream.gapEvents) 次（最近 \(stream.lastGapMs.map { "\($0) ms" } ?? "无记录")）")
                .font(.system(size: 8)).foregroundStyle(Palette.muted).textSelection(.enabled)
                .fixedSize(horizontal: false, vertical: true)
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
