import SwiftUI

struct HistoryView: View {
    @ObservedObject var model: ConsoleModel
    private var records: [TestRecord] { model.snapshot?.tests.records ?? [] }

    var body: some View {
        VStack(alignment: .leading, spacing: 17) {
            HStack {
                VStack(alignment: .leading, spacing: 4) {
                    Text("历史测试记录").font(.system(size: 17, weight: .semibold))
                    Text("读取 NUC 既有状态目录 · 不创建或修改记录").font(.system(size: 10)).foregroundStyle(Palette.muted)
                }
                Spacer()
                Text("\(records.count) 条").font(.system(size: 11, weight: .medium)).foregroundStyle(Palette.accent)
            }
            if records.isEmpty {
                Panel(title: "没有可展示的记录", subtitle: "只有设备上实际存在的 YAML 记录会出现在这里") {
                    UnknownBlock(text: model.snapshot?.tests.reason ?? "等待 NUC 快照；空列表不会被显示为通过")
                }
            } else {
                VStack(spacing: 10) {
                    ForEach(records) { record in
                        HistoryRecordCard(record: record)
                    }
                }
            }
        }
    }
}

private struct HistoryRecordCard: View {
    let record: TestRecord

    var body: some View {
        HStack(alignment: .top, spacing: 14) {
            ZStack {
                RoundedRectangle(cornerRadius: 9).fill(resultColor.opacity(0.13))
                Image(systemName: resultSymbol).font(.system(size: 14, weight: .semibold)).foregroundStyle(resultColor)
            }
            .frame(width: 36, height: 36)
            VStack(alignment: .leading, spacing: 7) {
                HStack {
                    Text(record.test.replacingOccurrences(of: "_", with: " ")).font(.system(size: 12, weight: .semibold))
                    Text(record.result).font(.system(size: 9, weight: .bold)).foregroundStyle(resultColor).padding(.horizontal, 7).padding(.vertical, 3).background(resultColor.opacity(0.12), in: Capsule())
                    Spacer()
                    Text(record.timestamp ?? "时间未记录").font(.system(size: 9, design: .monospaced)).foregroundStyle(Palette.muted)
                }
                Text(record.scope).font(.system(size: 10)).foregroundStyle(Palette.muted)
                HStack(spacing: 13) {
                    Text("SHA \(record.sha?.shortSHA ?? "未知")")
                    if let branch = record.branch { Text(branch) }
                    if let artifact = record.artifact { Text(artifact).lineLimit(1).truncationMode(.middle) }
                }
                .font(.system(size: 9, design: .monospaced)).foregroundStyle(Palette.muted.opacity(0.86))
            }
        }
        .padding(14)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Palette.panel, in: RoundedRectangle(cornerRadius: 12))
        .overlay(RoundedRectangle(cornerRadius: 12).stroke(Palette.line, lineWidth: 1))
    }

    private var resultColor: Color {
        switch record.result.uppercased() {
        case "PASS": return Palette.accent
        case "FAIL": return Palette.danger
        default: return Palette.warning
        }
    }

    private var resultSymbol: String {
        switch record.result.uppercased() {
        case "PASS": return "checkmark"
        case "FAIL": return "xmark"
        default: return "minus"
        }
    }
}
