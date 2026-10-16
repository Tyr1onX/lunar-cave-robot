import SwiftUI

struct HealthDiagnosticsPanel: View {
    let events: [HealthEvent]

    var body: some View {
        Panel(title: "系统健康", subtitle: "基于 NUC 只读 Agent 与 ROS 运行时证据；不代表实体硬件状态已验证") {
            VStack(alignment: .leading, spacing: 11) {
                HealthSummaryHeader(events: events)
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

    private var abnormalEvents: [HealthEvent] { events.filter { $0.state == .abnormal } }
    private var unknownCount: Int { events.filter { $0.state == .unknown }.count }
    private var notRunningCount: Int { events.filter { $0.state == .notRunning }.count }

    var body: some View {
        Panel(title: "异常摘要", subtitle: "只读数据状态 · 完整证据见「系统诊断」") {
            VStack(alignment: .leading, spacing: 10) {
                HStack(spacing: 9) {
                    Image(systemName: abnormalEvents.isEmpty ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                        .foregroundStyle(abnormalEvents.isEmpty ? Palette.accent : Palette.danger)
                    Text(abnormalEvents.isEmpty ? "当前无已确认异常" : "发现 \(abnormalEvents.count) 项异常")
                        .font(.system(size: 13, weight: .semibold))
                    Spacer()
                    HealthStateBadge(state: abnormalEvents.isEmpty ? .normal : .abnormal)
                }
                if abnormalEvents.isEmpty {
                    Text("\(unknownCount) 项未知 · \(notRunningCount) 项未运行。未配置为当前模式预期运行的模块不会判为故障。")
                        .font(.system(size: 10)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                } else {
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
                Text("ROS 通信或图像流正常仅说明数据链路可观测，不等同于导航、机械臂或底盘实体正常。")
                    .font(.system(size: 9)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
            }
        }
    }
}

private struct HealthSummaryHeader: View {
    let events: [HealthEvent]
    private var abnormalCount: Int { events.filter { $0.state == .abnormal }.count }
    private var unknownCount: Int { events.filter { $0.state == .unknown }.count }
    private var notRunningCount: Int { events.filter { $0.state == .notRunning }.count }

    var body: some View {
        HStack(spacing: 10) {
            Image(systemName: abnormalCount == 0 ? "checkmark.circle.fill" : "exclamationmark.triangle.fill")
                .font(.system(size: 16)).foregroundStyle(abnormalCount == 0 ? Palette.accent : Palette.danger)
            VStack(alignment: .leading, spacing: 2) {
                Text(abnormalCount == 0 ? "未发现已确认异常" : "发现 \(abnormalCount) 项异常")
                    .font(.system(size: 12, weight: .semibold))
                Text("未知 \(unknownCount) · 未运行 \(notRunningCount)")
                    .font(.system(size: 9)).foregroundStyle(Palette.muted)
            }
            Spacer()
            HealthStateBadge(state: abnormalCount == 0 ? (unknownCount > 0 ? .unknown : .normal) : .abnormal)
        }
        .padding(.bottom, 2)
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
                HealthEvidenceField(label: "运行模式", value: event.runMode.rawValue)
            }
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
