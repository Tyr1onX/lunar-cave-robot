import SwiftUI

struct WorkspaceMonitoringPanel: View {
    let inventory: WorkspaceInventoryReading?
    let freshness: WorkspaceSnapshotFreshness
    let agentOnline: Bool

    var body: some View {
        Panel(title: "统一工作区观测", subtitle: "同一只读 Agent 汇总新版与原始 ROS 工程；源码、进程与 ROS graph 分层显示") {
            if let inventory, !inventory.entries.isEmpty {
                VStack(alignment: .leading, spacing: 12) {
                    WorkspaceScanSummary(inventory: inventory, freshness: freshness, agentOnline: agentOnline)
                    ForEach(Array(inventory.entries.enumerated()), id: \.element.id) { index, workspace in
                        WorkspaceEvidenceCard(workspace: workspace)
                        if index < inventory.entries.count - 1 {
                            Divider().overlay(Palette.line)
                        }
                    }
                    Text("进程已发现不代表模块或硬件正常。实体模式仅表示进程环境声明；运行预期尚未核实的模块不产生故障结论。Domain 不一致或 Graph 过期时不判下游故障。历史快照不参与实时健康判定。")
                        .font(.system(size: 9)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                }
            } else {
                UnknownBlock(text: "Agent 未提供工作区来源证据；工程与模块运行状态未知。")
            }
        }
    }
}

struct WorkspaceCockpitSummary: View {
    let inventory: WorkspaceInventoryReading?
    let freshness: WorkspaceSnapshotFreshness
    let agentOnline: Bool

    var body: some View {
        Panel(title: "统一工程摘要", subtitle: "新版工程与两个原始 ROS 工作区使用同一只读观测来源") {
            if let inventory, !inventory.entries.isEmpty {
                VStack(alignment: .leading, spacing: 9) {
                    ForEach(inventory.entries) { workspace in
                        HStack(alignment: .top, spacing: 9) {
                            Image(systemName: workspace.sourceKind == "new" ? "shippingbox" : "archivebox")
                                .font(.system(size: 11)).foregroundStyle(Palette.blue).frame(width: 15)
                            VStack(alignment: .leading, spacing: 3) {
                                HStack {
                                    Text(workspace.label).font(.system(size: 10, weight: .semibold))
                                    Spacer(minLength: 4)
                                    Text(WorkspaceExecutionPresentation.label(
                                        executionState: workspace.executionState,
                                        runtimeStatus: workspace.runtimeStatus
                                    ))
                                        .font(.system(size: 9, weight: .medium))
                                        .foregroundStyle(executionTint(workspace.executionState))
                                }
                                Text("Domain \(domainsLabel(workspace.domainIds)) · 模式 \(modeLabel(workspace.runMode)) · \(freshness == .live ? "观测时间" : "上次观测") \(workspace.runtimeObservedAt ?? workspace.sourceObservedAt ?? "未知")")
                                    .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
                                    .lineLimit(2).textSelection(.enabled)
                            }
                        }
                    }
                    Text("项目或仿真源码不会被当作原始工程的实机状态；当前展示的是来源与观察证据。")
                        .font(.system(size: 9)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                    if freshness != .live {
                        Text("\(freshnessLabel(freshness, agentOnline: agentOnline))；历史数据不参与当前实时健康判定。")
                            .font(.system(size: 9, weight: .medium)).foregroundStyle(Palette.warning)
                    }
                }
            } else {
                UnknownBlock(text: "Agent 未返回工作区监测数据；兼容旧版 Agent，工作区健康保持未知。")
            }
        }
    }
}

private struct WorkspaceEvidenceCard: View {
    let workspace: WorkspaceReading

    var body: some View {
        VStack(alignment: .leading, spacing: 9) {
            HStack(spacing: 8) {
                Image(systemName: workspace.sourceKind == "new" ? "shippingbox" : "archivebox")
                    .font(.system(size: 12)).foregroundStyle(Palette.blue)
                Text(workspace.label).font(.system(size: 11, weight: .semibold))
                Text(workspace.sourceKind == "new" ? "新版" : "原始")
                    .font(.system(size: 8, weight: .semibold)).foregroundStyle(Palette.muted)
                    .padding(.horizontal, 7).padding(.vertical, 3)
                    .background(Palette.panelRaised, in: Capsule())
                Spacer(minLength: 5)
                Text(sourceLabel(workspace.sourceStatus))
                    .font(.system(size: 8, weight: .medium)).foregroundStyle(sourceTint(workspace.sourceStatus))
            }

            Text(workspace.root ?? "工作区路径未知")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
                .lineLimit(2)

            HStack(spacing: 7) {
                WorkspaceMetric(label: "包", value: countLabel(workspace.packageCount))
                WorkspaceMetric(label: "Launch", value: countLabel(workspace.launchCount))
                WorkspaceMetric(label: "配置", value: countLabel(workspace.configCount))
                WorkspaceMetric(label: "进程", value: runtimeLabel(workspace.runtimeStatus, count: workspace.processCount))
            }

            HStack(alignment: .top, spacing: 12) {
                WorkspaceEvidenceValue(label: "ROS Domain", value: domainsLabel(workspace.domainIds))
                WorkspaceEvidenceValue(label: "运行模式", value: modeLabel(workspace.runMode))
                WorkspaceEvidenceValue(label: "Graph 关系", value: graphLabel(workspace.graphRelationship, domain: workspace.graphDomainId))
            }

            WorkspaceEvidenceValue(
                label: "执行状态",
                value: WorkspaceExecutionPresentation.label(
                    executionState: workspace.executionState,
                    runtimeStatus: workspace.runtimeStatus
                )
            )

            DisclosureGroup("模块与源码接口候选") {
                VStack(alignment: .leading, spacing: 7) {
                    if let modules = workspace.modules, !modules.isEmpty {
                        ForEach(modules) { module in
                            VStack(alignment: .leading, spacing: 3) {
                                HStack {
                                    Text(module.label).font(.system(size: 9, weight: .semibold))
                                    Spacer()
                                    Text(WorkspaceExecutionPresentation.label(
                                        executionState: module.executionState,
                                        runtimeStatus: module.runtimeStatus
                                    ))
                                        .font(.system(size: 8, weight: .medium)).foregroundStyle(executionTint(module.executionState))
                                }
                                Text("包：\((module.packages ?? []).joined(separator: ", ").isEmpty ? "未知" : (module.packages ?? []).joined(separator: ", ")) · 运行预期：\(expectationLabel(module.runExpectation))")
                                    .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
                                if let expectationSource = module.runExpectationSource {
                                    Text("运行预期来源：\(expectationSource) · 观测时间：\(module.runExpectationObservedAt ?? "未知")")
                                        .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
                                }
                                if let expectationReason = module.runExpectationReason {
                                    Text(expectationReason).font(.system(size: 8)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                                }
                                if let reason = module.reason {
                                    Text(reason).font(.system(size: 8)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                                }
                                if let topics = module.interfaceTopics, !topics.isEmpty {
                                    Text("源码接口候选：\(topics.joined(separator: ", "))")
                                        .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
                                        .fixedSize(horizontal: false, vertical: true)
                                }
                                if workspace.graphRelationship == "shared_domain" {
                                    let matches = module.observedInterfaceTopics ?? []
                                    Text("Agent Graph 同名 Topic（仅名称匹配）：\(matches.isEmpty ? "当前未发现" : matches.joined(separator: ", "))")
                                        .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
                                        .fixedSize(horizontal: false, vertical: true)
                                } else {
                                    Text("Agent Graph Topic 匹配：未知（Domain / Graph 关系未确认）")
                                        .font(.system(size: 8)).foregroundStyle(Palette.muted)
                                }
                            }
                            .padding(.vertical, 3)
                        }
                    } else {
                        UnknownBlock(text: "没有可归属的模块清单。")
                    }
                    Text("接口候选来自源码，不代表运行时已发现；此面板不会订阅命令 Topic。")
                        .font(.system(size: 8)).foregroundStyle(Palette.muted)
                }
                .padding(.top, 7)
            }
            .font(.system(size: 9, weight: .medium))
            .tint(Palette.accent)

            DisclosureGroup("进程、Domain 与观测时间") {
                VStack(alignment: .leading, spacing: 6) {
                    WorkspaceEvidenceValue(label: "源码观测", value: workspace.sourceObservedAt ?? "未知")
                    WorkspaceEvidenceValue(label: "进程观测", value: workspace.runtimeObservedAt ?? "未知")
                    WorkspaceEvidenceValue(label: "Graph 数据", value: "Domain \(workspace.graphDomainId.map(String.init) ?? "未知") · \(workspace.graphObservedAt ?? "时间未知")")
                    if let processes = workspace.processes, !processes.isEmpty {
                        ForEach(processes) { process in
                            Text("PID \(process.pid) · \(process.processName ?? "未知") · 包 \(process.package ?? "未知") · Domain \(process.domainId.map { String($0) } ?? process.domainEvidence ?? "未知") · LOCALHOST_ONLY \(process.localhostOnly ?? "未知") · 模式 \(modeLabel(process.runMode)) · 模式来源 \(runModeEvidenceLabel(process.runModeEvidence)) · 归属依据 \((process.matchedBy ?? []).joined(separator: ", "))")
                                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
                                .textSelection(.enabled).fixedSize(horizontal: false, vertical: true)
                        }
                    } else {
                        Text(workspace.reason ?? "未发现关联的工作区 ROS 进程；这不是已确认故障。")
                            .font(.system(size: 8)).foregroundStyle(Palette.muted)
                    }
                    if let graphReason = workspace.graphReason {
                        Text(graphReason).font(.system(size: 8)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                    }
                    if let reason = workspace.reason {
                        Text(reason).font(.system(size: 8)).foregroundStyle(Palette.warning).fixedSize(horizontal: false, vertical: true)
                    }
                    inventoryPaths(title: "包清单", values: (workspace.packages ?? []).map { "\($0.name) · \($0.path)" })
                    inventoryPaths(title: "Launch 文件", values: workspace.launchFiles ?? [])
                    inventoryPaths(title: "配置文件", values: workspace.configFiles ?? [])
                }
                .padding(.top, 7)
            }
            .font(.system(size: 9, weight: .medium))
            .tint(Palette.accent)
        }
    }

    @ViewBuilder
    private func inventoryPaths(title: String, values: [String]) -> some View {
        if !values.isEmpty {
            VStack(alignment: .leading, spacing: 3) {
                Text("\(title) · \(values.count)").font(.system(size: 8, weight: .semibold)).foregroundStyle(Palette.muted)
                ForEach(Array(values.prefix(8).enumerated()), id: \.offset) { entry in
                    Text(entry.element).font(.system(size: 7, design: .monospaced)).foregroundStyle(Palette.muted).textSelection(.enabled)
                }
                if values.count > 8 {
                    Text("另有 \(values.count - 8) 项 · 完整清单来自 Agent 快照")
                        .font(.system(size: 7)).foregroundStyle(Palette.muted)
                }
            }
        }
    }
}

private struct WorkspaceMetric: View {
    let label: String
    let value: String

    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(label).font(.system(size: 7, weight: .medium)).foregroundStyle(Palette.muted)
            Text(value).font(.system(size: 8, weight: .semibold)).foregroundStyle(Palette.blue).lineLimit(1).minimumScaleFactor(0.7)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(.horizontal, 7).padding(.vertical, 6)
        .background(Palette.panelRaised, in: RoundedRectangle(cornerRadius: 7))
    }
}

private struct WorkspaceScanSummary: View {
    let inventory: WorkspaceInventoryReading
    let freshness: WorkspaceSnapshotFreshness
    let agentOnline: Bool

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack(alignment: .top, spacing: 12) {
                WorkspaceEvidenceValue(label: "快照状态", value: freshnessLabel(freshness, agentOnline: agentOnline))
                WorkspaceEvidenceValue(label: "进程扫描", value: processStatus)
                WorkspaceEvidenceValue(label: "进程范围", value: inventory.processScan?.scope ?? "未知")
            }
            WorkspaceEvidenceValue(label: "工作区快照时间", value: inventory.collectedAt ?? "未知")
            if let expectationStatus = inventory.expectationStatus {
                WorkspaceEvidenceValue(
                    label: "模块运行预期",
                    value: "\(expectationStatusLabel(expectationStatus)) · \(inventory.expectationObservedAt ?? "时间未知")"
                )
                if let expectationError = inventory.expectationError {
                    Text(expectationError).font(.system(size: 8)).foregroundStyle(Palette.warning).fixedSize(horizontal: false, vertical: true)
                }
            }
            HStack(alignment: .top, spacing: 12) {
                WorkspaceEvidenceValue(label: "Agent Graph", value: "Domain \(inventory.graphSource?.domainId.map(String.init) ?? "未知") · \(inventory.graphSource?.graphAgeMs.map { String($0) + " ms" } ?? "时龄未知")")
                WorkspaceEvidenceValue(label: "Graph 时间", value: inventory.graphSource?.observedAt ?? "未知")
            }
            if let reason = inventory.processScan?.reason {
                Text(reason).font(.system(size: 8)).foregroundStyle(Palette.warning).fixedSize(horizontal: false, vertical: true)
            }
        }
        .padding(9)
        .background(Palette.panelRaised, in: RoundedRectangle(cornerRadius: 8))
    }

    private var processStatus: String {
        switch inventory.processScan?.status {
        case "ok": return "完整 · ROS/工作区候选（Agent UID）"
        case "partial": return "部分可读 · 工作区 Graph 关系为未知"
        default: return "未知"
        }
    }
}

private func freshnessLabel(_ freshness: WorkspaceSnapshotFreshness, agentOnline: Bool) -> String {
    switch freshness {
    case .live: return "实时快照 · 当前连接"
    case .historical: return agentOnline ? "历史快照 · 等待本次读取" : "历史快照 · Agent 断连"
    case .expired: return "历史快照已过期 · 超过 15 秒"
    case .unavailable: return "工作区快照未知"
    }
}

private struct WorkspaceEvidenceValue: View {
    let label: String
    let value: String

    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(label).font(.system(size: 7, weight: .medium)).foregroundStyle(Palette.muted)
            Text(value).font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
                .fixedSize(horizontal: false, vertical: true).textSelection(.enabled)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

private func sourceLabel(_ status: String?) -> String {
    status == "present" ? "源码存在" : "源码未知"
}

private func sourceTint(_ status: String?) -> Color {
    status == "present" ? Palette.blue : Palette.warning
}

private func countLabel(_ count: Int?) -> String {
    count.map { String($0) } ?? "未知"
}

private func runtimeLabel(_ status: String?, count: Int?) -> String {
    switch status {
    case "running": return "发现 \(count ?? 0)"
    case "not_running": return "未发现"
    default: return "未知"
    }
}

private func expectationLabel(_ expectation: String?) -> String {
    switch expectation {
    case "confirmed": return "已确认"
    case "unconfigured": return "未配置"
    default: return "未知"
    }
}

private func expectationStatusLabel(_ value: String) -> String {
    switch value {
    case "configured": return "显式配置"
    case "unconfigured": return "未配置"
    case "invalid": return "配置无效"
    default: return "未知"
    }
}

private func runtimeTint(_ status: String?) -> Color {
    switch status {
    case "running": return Palette.blue
    case "not_running": return Palette.muted
    default: return Palette.warning
    }
}

private func executionTint(_ state: String?) -> Color {
    switch state {
    case "not_running": return Palette.muted
    case "simulation_running", "hardware_declared_running": return Palette.blue
    default: return Palette.warning
    }
}

private func domainsLabel(_ domains: [Int]?) -> String {
    guard let domains, !domains.isEmpty else { return "未知" }
    return domains.map { String($0) }.joined(separator: ", ")
}

private func modeLabel(_ mode: String?) -> String {
    switch mode {
    case "hardware": return "实体模式（进程环境声明，硬件状态未知）"
    case "simulation": return "仿真模式（进程环境声明）"
    case "offline": return "离线模式（进程环境声明）"
    case "read-only": return "只读模式（进程环境声明）"
    case "mixed": return "多个模式"
    default: return "未知"
    }
}

private func runModeEvidenceLabel(_ evidence: String?) -> String {
    evidence == "explicit_environment" ? "LUNAR_CONSOLE_RUN_MODE" : "未提供"
}

enum WorkspaceExecutionPresentation {
    static func label(executionState: String?, runtimeStatus: String?) -> String {
        switch executionState {
        case "not_running": return "未发现关联进程（Agent UID 可见范围）"
        case "simulation_running": return "仿真运行（进程环境声明）"
        case "hardware_declared_running": return "实体模式已声明 · 硬件状态未知"
        default:
            switch runtimeStatus {
            case "not_running": return "未发现关联进程（旧版 Agent 扫描结果）"
            case "running": return "进程已发现 · 执行模式未知"
            default: return "状态未知"
            }
        }
    }
}

private func graphLabel(_ relationship: String?, domain: Int?) -> String {
    switch relationship {
    case "shared_domain": return "与 Agent Domain \(domain.map { String($0) } ?? "未知") 相同"
    case "partially_shared": return "部分 Domain 可见"
    case "different_domain": return "Domain 不同 · 未覆盖"
    default: return "未知"
    }
}
