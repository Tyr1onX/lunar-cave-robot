import SwiftUI

struct DiagnosticsView: View {
    @ObservedObject var model: ConsoleModel
    private var snapshot: ConsoleSnapshot? { model.snapshot }

    var body: some View {
        VStack(alignment: .leading, spacing: 17) {
            OverviewView(model: model)
            HealthDiagnosticsPanel(events: model.healthEvents)
            HStack(spacing: 15) {
                DiagnosticCard(title: "Agent 状态", value: model.agentOnline ? "在线" : "不可用", note: model.agentMessage, tint: model.agentOnline ? Palette.accent : Palette.warning)
                DiagnosticCard(title: "系统", value: snapshot?.host.operatingSystem ?? "未知", note: snapshot?.host.platform ?? "等待设备", tint: Palette.blue)
                DiagnosticCard(title: "ROS Domain", value: snapshot?.ros.domainId.map(String.init) ?? "未知", note: snapshot?.ros.discovery ?? snapshot?.ros.reason ?? "等待运行时发现", tint: Palette.warning)
            }
            HStack(alignment: .top, spacing: 16) {
                Panel(title: "运行时节点", subtitle: "当前 Domain 中实际发现的节点") {
                    if let nodes = snapshot?.ros.nodes, !nodes.isEmpty {
                        VStack(alignment: .leading, spacing: 8) {
                            ForEach(nodes) { node in
                                HStack {
                                    Image(systemName: "circle.fill").font(.system(size: 5)).foregroundStyle(Palette.accent)
                                    Text("\(node.namespace)\(node.name)").font(.system(size: 10, design: .monospaced))
                                    Spacer()
                                    Text("已发现").font(.system(size: 9)).foregroundStyle(Palette.muted)
                                }
                            }
                        }
                    } else {
                        UnknownBlock(text: snapshot?.ros.status == "ok" ? "当前运行时图没有业务节点" : snapshot?.ros.reason ?? "ROS 节点状态未知")
                    }
                }
                Panel(title: "运行时 Topic", subtitle: "名称与类型来自 NUC ROS graph") {
                    if let topics = snapshot?.ros.topics, !topics.isEmpty {
                        VStack(alignment: .leading, spacing: 8) {
                            ForEach(topics) { topic in
                                VStack(alignment: .leading, spacing: 3) {
                                    Text(topic.name).font(.system(size: 10, weight: .medium, design: .monospaced))
                                    Text(topic.types.joined(separator: ", ")).font(.system(size: 9, design: .monospaced)).foregroundStyle(Palette.muted)
                                }
                                .padding(.vertical, 2)
                            }
                        }
                    } else {
                        UnknownBlock(text: snapshot?.ros.status == "ok" ? "当前运行时图没有 Topic" : snapshot?.ros.reason ?? "ROS Topic 状态未知")
                    }
                }
            }
            Panel(title: "主机指标来源", subtitle: "仅展示 Agent 实际读取到的数据") {
                VStack(alignment: .leading, spacing: 10) {
                    SourceRow(name: "CPU", status: snapshot?.system.cpu.status, reason: snapshot?.system.cpu.reason ?? "Linux /proc/stat")
                    SourceRow(name: "内存", status: snapshot?.system.memory.status, reason: snapshot?.system.memory.reason ?? "Linux /proc/meminfo")
                    SourceRow(name: "磁盘", status: snapshot?.system.disk.status, reason: snapshot?.system.disk.reason ?? "根文件系统用量")
                    SourceRow(name: "温度", status: snapshot?.system.temperature.status, reason: snapshot?.system.temperature.reason ?? "Linux sysfs thermal sensors")
                }
            }
            HStack(alignment: .top, spacing: 16) {
                Panel(title: "部署版本", subtitle: snapshot?.deployment.source ?? "真实设备部署记录") {
                    UnknownBlock(text: snapshot?.deployment.sha ?? snapshot?.deployment.reason ?? "未记录")
                }
                Panel(title: "SSH 隧道", subtitle: "固定转发 127.0.0.1:18765 → NUC 127.0.0.1:8765") {
                    Text(model.connectionMessage).font(.system(size: 10)).foregroundStyle(Palette.muted).textSelection(.enabled)
                }
            }
        }
    }
}

private struct DiagnosticCard: View {
    let title: String
    let value: String
    let note: String
    let tint: Color
    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text(title).font(.system(size: 10, weight: .medium)).foregroundStyle(Palette.muted)
            Text(value).font(.system(size: 17, weight: .semibold, design: .rounded)).foregroundStyle(tint).lineLimit(1).minimumScaleFactor(0.8)
            Text(note).font(.system(size: 9)).foregroundStyle(Palette.muted).lineLimit(2)
        }
        .frame(maxWidth: .infinity, minHeight: 90, alignment: .leading).padding(15)
        .background(Palette.panel, in: RoundedRectangle(cornerRadius: 12))
    }
}

private struct SourceRow: View {
    let name: String
    let status: String?
    let reason: String
    var body: some View {
        HStack(spacing: 9) {
            StatusDot(isOn: status == "ok")
            Text(name).font(.system(size: 10, weight: .medium)).frame(width: 54, alignment: .leading)
            Text(status == "ok" ? "已采集" : "未知").font(.system(size: 9, weight: .semibold)).foregroundStyle(status == "ok" ? Palette.accent : Palette.warning)
            Text(reason).font(.system(size: 9)).foregroundStyle(Palette.muted).lineLimit(1)
            Spacer()
        }
    }
}
