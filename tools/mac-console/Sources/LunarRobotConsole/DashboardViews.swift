import SwiftUI

struct OverviewView: View {
    @ObservedObject var model: ConsoleModel
    private var data: ConsoleSnapshot? { model.snapshot }

    var body: some View {
        VStack(alignment: .leading, spacing: 19) {
            HStack(spacing: 13) {
                MetricCard(title: "CPU", symbol: "cpu", value: data?.system.cpu.percent.map { String(format: "%.0f%%", $0) } ?? "未知", detail: data?.system.cpu.status == "ok" ? "来自 /proc/stat" : data?.system.cpu.reason ?? "等待真实数据", progress: data?.system.cpu.percent, tint: Palette.blue)
                MetricCard(title: "内存", symbol: "memorychip", value: data?.system.memory.usedPercent.map { String(format: "%.0f%%", $0) } ?? "未知", detail: memoryDetail(data?.system.memory), progress: data?.system.memory.usedPercent, tint: Palette.accent)
                MetricCard(title: "系统盘", symbol: "internaldrive", value: data?.system.disk.usedPercent.map { String(format: "%.0f%%", $0) } ?? "未知", detail: diskDetail(data?.system.disk), progress: data?.system.disk.usedPercent, tint: Palette.warning)
                MetricCard(title: "温度", symbol: "thermometer.medium", value: hottestTemperature(data?.system.temperature), detail: data?.system.temperature.status == "ok" ? "\(data?.system.temperature.readings?.count ?? 0) 个可读传感器" : data?.system.temperature.reason ?? "等待真实数据", progress: nil, tint: Palette.danger)
            }
            HStack(alignment: .top, spacing: 16) {
                VersionPanel(model: model).frame(maxWidth: .infinity)
                ROSPanel(reading: data?.ros).frame(maxWidth: .infinity)
            }
            HStack(alignment: .top, spacing: 16) {
                SafetyPanel(reading: data?.safety).frame(maxWidth: .infinity, alignment: .leading)
                LogsPanel(reading: model.logs).frame(maxWidth: .infinity, alignment: .leading)
            }
            HStack {
                Text("快照来源：NUC 本机 Agent · 只读采集").font(.system(size: 10)).foregroundStyle(Palette.muted)
                Spacer()
                Text(data?.collectedAt ?? "尚无快照").font(.system(size: 10, design: .monospaced)).foregroundStyle(Palette.muted)
            }
        }
    }

    private func memoryDetail(_ memory: MemoryReading?) -> String {
        guard let memory, let total = memory.totalBytes, let available = memory.availableBytes else { return memory?.reason ?? "等待真实数据" }
        return "\((total - available).formattedBytes) / \(total.formattedBytes)"
    }

    private func diskDetail(_ disk: DiskReading?) -> String {
        guard let disk, let used = disk.usedBytes, let total = disk.totalBytes, let free = disk.freeBytes else { return disk?.reason ?? "等待真实数据" }
        return "\(used.formattedBytes) / \(total.formattedBytes) · 剩余 \(free.formattedBytes)"
    }

    private func hottestTemperature(_ reading: TemperatureReading?) -> String {
        guard let sensors = reading?.readings, let hottest = sensors.max(by: { $0.celsius < $1.celsius }) else { return "未知" }
        return String(format: "%.1f°C", hottest.celsius)
    }
}

private struct MetricCard: View {
    let title: String
    let symbol: String
    let value: String
    let detail: String
    let progress: Double?
    let tint: Color

    var body: some View {
        VStack(alignment: .leading, spacing: 13) {
            HStack {
                Image(systemName: symbol).foregroundStyle(tint).font(.system(size: 13, weight: .semibold))
                Text(title).font(.system(size: 12, weight: .medium)).foregroundStyle(Palette.muted)
                Spacer()
                Circle().fill(value == "未知" ? Palette.warning : Palette.accent).frame(width: 6, height: 6)
            }
            Text(value).font(.system(size: 29, weight: .semibold, design: .rounded)).monospacedDigit()
            if let progress {
                ProgressView(value: min(100, max(0, progress)), total: 100).tint(tint)
            } else {
                Rectangle().fill(Palette.line).frame(height: 4).clipShape(Capsule())
            }
            Text(detail).font(.system(size: 10)).foregroundStyle(Palette.muted).lineLimit(1).truncationMode(.middle)
        }
        .padding(16)
        .frame(maxWidth: .infinity, minHeight: 145, alignment: .topLeading)
        .background(Palette.panel, in: RoundedRectangle(cornerRadius: 13))
        .overlay(RoundedRectangle(cornerRadius: 13).stroke(Palette.line, lineWidth: 1))
    }
}

struct Panel<Content: View>: View {
    let title: String
    let subtitle: String?
    @ViewBuilder let content: Content

    var body: some View {
        VStack(alignment: .leading, spacing: 15) {
            VStack(alignment: .leading, spacing: 3) {
                Text(title).font(.system(size: 13, weight: .semibold))
                if let subtitle { Text(subtitle).font(.system(size: 10)).foregroundStyle(Palette.muted) }
            }
            content
        }
        .padding(17)
        .frame(maxWidth: .infinity, alignment: .topLeading)
        .background(Palette.panel, in: RoundedRectangle(cornerRadius: 13))
        .overlay(RoundedRectangle(cornerRadius: 13).stroke(Palette.line, lineWidth: 1))
    }
}

private struct VersionPanel: View {
    @ObservedObject var model: ConsoleModel
    private var snapshot: ConsoleSnapshot? { model.snapshot }

    var body: some View {
        Panel(title: "版本来源", subtitle: "源码同步、工作树与实际部署分别显示") {
            VStack(spacing: 10) {
                VersionRow(title: "GitHub main", value: model.githubMainSHA?.shortSHA ?? "未知", note: githubNote, status: sourceComparisonStatus)
                VersionRow(title: "NUC 工作树 HEAD", value: snapshot?.git.headSha?.shortSHA ?? "未知", note: gitNote, status: nucVersionStatus)
                VersionRow(title: "实际部署 SHA", value: snapshot?.deployment.sha?.shortSHA ?? "未记录", note: snapshot?.deployment.reason ?? snapshot?.deployment.source ?? "等待真实数据", status: snapshot?.deployment.status ?? "unknown")
            }
        }
    }

    private var sourceComparisonStatus: String {
        guard let main = model.githubMainSHA, let head = snapshot?.git.headSha else { return "unknown" }
        return main == head ? "ok" : "warning"
    }

    private var nucVersionStatus: String {
        guard snapshot?.git.status == "ok" else { return "unknown" }
        return sourceComparisonStatus == "warning" ? "warning" : sourceComparisonStatus
    }

    private var githubNote: String {
        guard sourceComparisonStatus == "warning" else { return model.githubMessage }
        return "\(model.githubMessage) · NUC SHA 不同"
    }

    private var gitNote: String {
        guard let git = snapshot?.git else { return "等待 NUC 快照" }
        let dirty = git.dirty == true ? "工作树有 \(git.dirtyFiles ?? 0) 项改动" : "工作树干净"
        let comparison = sourceComparisonStatus == "ok" ? "与 GitHub main 相同" : sourceComparisonStatus == "warning" ? "与 GitHub main 不同" : "版本比较未知"
        return "\(git.branch ?? "分支未知") · \(dirty) · \(comparison)"
    }
}

private struct VersionRow: View {
    let title: String
    let value: String
    let note: String
    let status: String

    var body: some View {
        HStack(spacing: 12) {
            StatusDot(isOn: status == "ok")
            VStack(alignment: .leading, spacing: 4) {
                Text(title).font(.system(size: 11, weight: .medium))
                Text(note).font(.system(size: 9)).foregroundStyle(Palette.muted).lineLimit(1)
            }
            Spacer(minLength: 4)
            Text(value).font(.system(size: 10, design: .monospaced)).foregroundStyle(status == "ok" ? .white : Palette.warning)
        }
        .padding(.vertical, 3)
    }
}

private struct ROSPanel: View {
    let reading: ROSReading?

    var body: some View {
        Panel(title: "ROS 2 运行时图", subtitle: reading?.reason ?? reading?.discovery ?? "通过 NUC rclpy 发现，不从源码推测") {
            if let reading, reading.status == "ok" {
                HStack(spacing: 9) {
                    CountTile(title: "节点", value: reading.nodes.count, tint: Palette.blue)
                    CountTile(title: "Topic", value: reading.topics.count, tint: Palette.accent)
                    CountTile(title: "Domain ID", value: reading.domainId ?? -1, tint: Palette.warning)
                }
                if reading.nodes.isEmpty {
                    Text("运行时未发现业务节点").font(.system(size: 11)).foregroundStyle(Palette.muted).padding(.top, 1)
                } else {
                    Text(reading.nodes.prefix(4).map { "\($0.namespace)\($0.name)" }.joined(separator: "  ·  "))
                        .font(.system(size: 10, design: .monospaced)).foregroundStyle(Palette.muted).lineLimit(1)
                }
            } else {
                UnknownBlock(text: reading?.reason ?? "ROS 图状态未知")
            }
        }
    }
}

private struct CountTile: View {
    let title: String
    let value: Int
    let tint: Color
    var body: some View {
        HStack(spacing: 7) {
            Text("\(value)").font(.system(size: 17, weight: .semibold, design: .rounded)).foregroundStyle(tint)
            Text(title).font(.system(size: 10)).foregroundStyle(Palette.muted)
        }
        .frame(maxWidth: .infinity, minHeight: 43)
        .background(Palette.panelRaised, in: RoundedRectangle(cornerRadius: 8))
    }
}

private struct SafetyPanel: View {
    let reading: SafetyReading?

    var body: some View {
        Panel(title: "安全状态", subtitle: "只读监控 · 实体执行器不会由此应用驱动") {
            VStack(alignment: .leading, spacing: 11) {
                SafetyRow(title: "实体急停输入", value: reading?.physicalEstop.status == "ok" ? "有真实反馈" : "未知", note: reading?.physicalEstop.reason ?? "无经验证的实体信号接入")
                SafetyRow(title: "硬件健康", value: reading?.hardwareHealth.status == "ok" ? "有证据" : "未知", note: reading?.hardwareHealth.reason ?? "等待 NUC 快照")
                HStack(spacing: 7) {
                    Image(systemName: "lock.fill").foregroundStyle(Palette.accent)
                    Text("控制接口关闭").font(.system(size: 11, weight: .medium))
                    Spacer()
                    Text("只读").font(.system(size: 10, weight: .semibold)).foregroundStyle(Palette.accent)
                }
                .padding(.top, 2)
            }
        }
    }
}

private struct SafetyRow: View {
    let title: String
    let value: String
    let note: String
    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            HStack {
                Text(title).font(.system(size: 11, weight: .medium))
                Spacer()
                Text(value).font(.system(size: 10, weight: .semibold)).foregroundStyle(Palette.warning)
            }
            Text(note).font(.system(size: 9)).foregroundStyle(Palette.muted).lineLimit(2)
        }
    }
}

private struct LogsPanel: View {
    let reading: LogReading?

    var body: some View {
        Panel(title: "ROS 日志", subtitle: reading?.source ?? "NUC 本机 ROS 日志目录") {
            if let entries = reading?.entries, !entries.isEmpty {
                VStack(alignment: .leading, spacing: 9) {
                    ForEach(entries.prefix(3)) { entry in
                        HStack(alignment: .top, spacing: 8) {
                            Text(entry.severity.uppercased()).font(.system(size: 8, weight: .bold, design: .rounded))
                                .foregroundStyle(severityColor(entry.severity)).frame(width: 43, alignment: .leading)
                            VStack(alignment: .leading, spacing: 3) {
                                Text(entry.message).font(.system(size: 10, design: .monospaced)).lineLimit(2)
                                Text("\(entry.source) · 文件修改 \(entry.fileModifiedAt)").font(.system(size: 8)).foregroundStyle(Palette.muted)
                            }
                        }
                    }
                }
            } else {
                UnknownBlock(text: reading?.reason ?? "等待真实日志数据")
            }
        }
    }

    private func severityColor(_ severity: String) -> Color {
        switch severity {
        case "error": return Palette.danger
        case "warning": return Palette.warning
        default: return Palette.blue
        }
    }
}

struct UnknownBlock: View {
    let text: String
    var body: some View {
        HStack(spacing: 9) {
            Image(systemName: "questionmark.circle").foregroundStyle(Palette.warning)
            Text(text).font(.system(size: 10)).foregroundStyle(Palette.muted).lineLimit(2)
            Spacer(minLength: 0)
        }
        .padding(.vertical, 5)
    }
}
