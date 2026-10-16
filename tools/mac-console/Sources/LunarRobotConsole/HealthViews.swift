import SwiftUI

struct HealthDiagnosticsPanel: View {
    let events: [HealthEvent]
    let workspaceInventory: WorkspaceInventoryReading?

    var body: some View {
        Panel(title: "系统健康", subtitle: "基于 NUC 只读 Agent 与 ROS 运行时证据；不代表实体硬件状态已验证") {
            VStack(alignment: .leading, spacing: 11) {
                HealthSummaryHeader(events: events, workspaceInventory: workspaceInventory)
                Divider().overlay(Palette.line)
                ForEach(Array(events.enumerated()), id: \.element.id) { index, event in
                    HealthEventRow(event: event)
                    if index < events.count - 1 {
                        Divider().overlay(Palette.line)
                    }
                }
                Text("仅报告可验证的数据链路与已配置的运行期望；未确认运行的模块不会按故障处理。导航、机械臂与底盘实体状态仍需独立证据。")
                    .font(.system(size: 9)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                    .padding(.top, 2)
            }
        }
    }
}

struct HealthCockpitSummary: View {
    let events: [HealthEvent]
    let workspaceInventory: WorkspaceInventoryReading?

    private var abnormalEvents: [HealthEvent] { events.filter { $0.state == .abnormal } }
    private var unknownCount: Int { events.filter { $0.state == .unknown }.count }
    private var notRunningCount: Int { events.filter { $0.state == .notRunning }.count }
    private var summaryState: HealthState {
        HealthSummaryState.resolve(events, workspaceInventory: workspaceInventory, requireWorkspaceTelemetry: true)
    }
    private var workspaceEvidenceText: String {
        guard let workspaceInventory else { return "工作区监测数据未提供" }
        let prefix = workspaceInventory.status == "ok" ? "" : "工作区证据不完整 · "
        return "\(prefix)\(workspaceInventory.unconfirmedWorkspaceCount) 个工作区运行状态未确认"
    }

    private var summaryTitle: String {
        switch summaryState {
        case .normal: return "当前无已确认异常"
        case .abnormal: return "发现 \(abnormalEvents.count) 项异常"
        case .notRunning: return "存在未运行模块"
        case .unknown: return "系统状态未知"
        }
    }

    private var summarySymbol: String {
        switch summaryState {
        case .normal: return "checkmark.circle.fill"
        case .abnormal: return "exclamationmark.triangle.fill"
        case .notRunning: return "minus.circle.fill"
        case .unknown: return "questionmark.circle.fill"
        }
    }

    private var summaryTint: Color {
        switch summaryState {
        case .normal: return Palette.accent
        case .abnormal: return Palette.danger
        case .notRunning: return Palette.warning
        case .unknown: return Palette.muted
        }
    }

    var body: some View {
        Panel(title: "异常摘要", subtitle: "只读数据状态 · 完整证据见「系统诊断」") {
            VStack(alignment: .leading, spacing: 10) {
                HStack(spacing: 9) {
                    Image(systemName: summarySymbol).foregroundStyle(summaryTint)
                    Text(summaryTitle).font(.system(size: 13, weight: .semibold))
                    Spacer()
                    HealthStateBadge(state: summaryState)
                }
                if !abnormalEvents.isEmpty {
                    ForEach(abnormalEvents.prefix(3)) { event in
                        HStack(alignment: .top, spacing: 8) {
                            Circle().fill(Palette.danger).frame(width: 6, height: 6).padding(.top, 4)
                            VStack(alignment: .leading, spacing: 3) {
                                Text(event.title).font(.system(size: 10, weight: .semibold))
                                Text(event.observation).font(.system(size: 9)).foregroundStyle(Palette.muted).lineLimit(2)
                            }
                            Spacer(minLength: 0)
                        }
                    }
                    if abnormalEvents.count > 3 {
                        Text("另有 \(abnormalEvents.count - 3) 项异常；系统诊断中列出完整观测结果。")
                            .font(.system(size: 9)).foregroundStyle(Palette.muted)
                    }
                }
                Text("\(unknownCount) 项未知 · \(notRunningCount) 项未运行 · \(workspaceEvidenceText)。未配置为当前模式预期运行的模块不会判为故障。")
                    .font(.system(size: 10)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                Text("ROS 通信或图像流正常仅说明数据链路可观测，不等同于导航、机械臂或底盘实体正常。")
                    .font(.system(size: 9)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
            }
        }
    }
}

private struct HealthSummaryHeader: View {
    let events: [HealthEvent]
    let workspaceInventory: WorkspaceInventoryReading?
    private var abnormalCount: Int { events.filter { $0.state == .abnormal }.count }
    private var unknownCount: Int { events.filter { $0.state == .unknown }.count }
    private var notRunningCount: Int { events.filter { $0.state == .notRunning }.count }
    private var summaryState: HealthState {
        HealthSummaryState.resolve(events, workspaceInventory: workspaceInventory, requireWorkspaceTelemetry: true)
    }

    private var summarySymbol: String {
        switch summaryState {
        case .normal: return "checkmark.circle.fill"
        case .abnormal: return "exclamationmark.triangle.fill"
        case .notRunning: return "minus.circle.fill"
        case .unknown: return "questionmark.circle.fill"
        }
    }

    private var summaryTint: Color {
        switch summaryState {
        case .normal: return Palette.accent
        case .abnormal: return Palette.danger
        case .notRunning: return Palette.warning
        case .unknown: return Palette.muted
        }
    }

    private var summaryTitle: String {
        switch summaryState {
        case .normal: return "未发现已确认异常"
        case .abnormal: return "发现 \(abnormalCount) 项异常"
        case .notRunning: return "存在未运行模块"
        case .unknown: return "系统健康状态未知"
        }
    }

    var body: some View {
        HStack(spacing: 10) {
            Image(systemName: summarySymbol)
                .font(.system(size: 16)).foregroundStyle(summaryTint)
            VStack(alignment: .leading, spacing: 2) {
                Text(summaryTitle).font(.system(size: 12, weight: .semibold))
                Text("未知 \(unknownCount) · 未运行 \(notRunningCount) · \(workspaceEvidenceText)")
                    .font(.system(size: 9)).foregroundStyle(Palette.muted)
            }
            Spacer()
            HealthStateBadge(state: summaryState)
        }
        .padding(.bottom, 2)
    }

    private var workspaceEvidenceText: String {
        guard let workspaceInventory else { return "工作区证据未提供" }
        let prefix = workspaceInventory.status == "ok" ? "" : "工作区证据不完整 · "
        return "\(prefix)待确认 \(workspaceInventory.unconfirmedWorkspaceCount)"
    }
}

private struct HealthEventRow: View {
    let event: HealthEvent

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 8) {
                Image(systemName: symbol).font(.system(size: 12, weight: .semibold)).foregroundStyle(tint)
                Text(event.title).font(.system(size: 11, weight: .semibold))
                Spacer(minLength: 6)
                HealthStateBadge(state: event.state)
            }
            Text(event.observation)
                .font(.system(size: 9)).foregroundStyle(Palette.muted).textSelection(.enabled)
                .fixedSize(horizontal: false, vertical: true)
            HStack(alignment: .top, spacing: 12) {
                HealthEvidenceField(label: "Topic / 节点", value: event.resource)
                HealthEvidenceField(label: "数据时间", value: event.dataTime)
                HealthEvidenceField(label: "ROS Domain", value: event.rosDomain)
            }
            HStack(alignment: .top, spacing: 12) {
                HealthEvidenceField(label: "观测时间", value: event.observationWindow)
                HealthEvidenceField(label: "运行模式", value: event.runMode.rawValue)
                HealthEvidenceField(label: "来源", value: event.source)
            }
            HStack(alignment: .top, spacing: 12) {
                HealthEvidenceField(label: "模块", value: event.module)
                HealthEvidenceField(label: "规则 ID", value: event.ruleID)
                HealthEvidenceField(label: "严重程度", value: event.severity.label)
            }
            HealthEvidenceField(label: "源码位置", value: event.sourceLocation)
            HealthEvidenceField(label: "异常持续性", value: event.persistenceLabel)
            Text("建议检查 · \(event.suggestedCheck)")
                .font(.system(size: 9)).foregroundStyle(Palette.muted).textSelection(.enabled)
                .fixedSize(horizontal: false, vertical: true)
        }
        .padding(.vertical, 2)
    }

    private var tint: Color {
        switch event.state {
        case .normal: return Palette.accent
        case .abnormal: return Palette.danger
        case .notRunning: return Palette.warning
        case .unknown: return Palette.muted
        }
    }

    private var symbol: String {
        switch event.state {
        case .normal: return "checkmark.circle.fill"
        case .abnormal: return "exclamationmark.triangle.fill"
        case .notRunning: return "minus.circle.fill"
        case .unknown: return "questionmark.circle.fill"
        }
    }
}

private struct HealthEvidenceField: View {
    let label: String
    let value: String

    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            Text(label).font(.system(size: 8, weight: .medium)).foregroundStyle(Palette.muted)
            Text(value).font(.system(size: 8, design: .monospaced)).textSelection(.enabled)
                .fixedSize(horizontal: false, vertical: true)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

private struct HealthStateBadge: View {
    let state: HealthState

    var body: some View {
        Text(state.label).font(.system(size: 8, weight: .semibold))
            .foregroundStyle(tint).padding(.horizontal, 8).padding(.vertical, 4)
            .background(tint.opacity(0.12), in: Capsule())
    }

    private var tint: Color {
        switch state {
        case .normal: return Palette.accent
        case .abnormal: return Palette.danger
        case .notRunning: return Palette.warning
        case .unknown: return Palette.muted
        }
    }
}
