import SwiftUI

enum Palette {
    static let background = Color(red: 0.035, green: 0.055, blue: 0.085)
    static let panel = Color(red: 0.075, green: 0.098, blue: 0.137)
    static let panelRaised = Color(red: 0.105, green: 0.13, blue: 0.18)
    static let line = Color.white.opacity(0.08)
    static let muted = Color(red: 0.53, green: 0.60, blue: 0.70)
    static let accent = Color(red: 0.22, green: 0.77, blue: 0.72)
    static let blue = Color(red: 0.30, green: 0.61, blue: 0.94)
    static let warning = Color(red: 0.96, green: 0.67, blue: 0.28)
    static let danger = Color(red: 0.92, green: 0.36, blue: 0.38)
}

struct ConsoleRootView: View {
    @ObservedObject var model: ConsoleModel
    @StateObject private var simulator = SimulationModel()
    @StateObject private var armPreview = ArmPreviewModel()
    @StateObject private var moduleTests = ModuleTestsModel()

    var body: some View {
        NavigationSplitView {
            VStack(alignment: .leading, spacing: 0) {
                HStack(spacing: 12) {
                    ZStack {
                        RoundedRectangle(cornerRadius: 12).fill(Palette.accent.opacity(0.15))
                        Image(systemName: "moon.stars.fill").font(.system(size: 19, weight: .semibold)).foregroundStyle(Palette.accent)
                    }
                    .frame(width: 42, height: 42)
                    VStack(alignment: .leading, spacing: 3) {
                        Text("LUNAR CONSOLE").font(.system(size: 12, weight: .bold, design: .rounded)).tracking(1.1)
                        Text("机器人测试与可视化").font(.system(size: 11)).foregroundStyle(Palette.muted)
                    }
                }
                .padding(.horizontal, 17)
                .padding(.top, 21)
                .padding(.bottom, 28)

                Text("工作区").font(.system(size: 10, weight: .bold)).tracking(1.2).foregroundStyle(Palette.muted)
                    .padding(.horizontal, 18).padding(.bottom, 8)
                ForEach(ConsoleModel.Section.allCases) { item in
                    Button {
                        model.section = item
                    } label: {
                        HStack(spacing: 12) {
                            Image(systemName: item.symbol).frame(width: 18)
                            Text(item.rawValue)
                            Spacer()
                            if model.section == item {
                                Capsule().fill(Palette.accent).frame(width: 3, height: 18)
                            }
                        }
                        .font(.system(size: 13, weight: model.section == item ? .semibold : .medium))
                        .foregroundStyle(model.section == item ? .white : Palette.muted)
                        .padding(.horizontal, 12).frame(height: 42)
                        .background(model.section == item ? Palette.panelRaised : .clear, in: RoundedRectangle(cornerRadius: 9))
                    }
                    .buttonStyle(.plain)
                    .padding(.horizontal, 9)
                }

                Spacer()
                VStack(alignment: .leading, spacing: 11) {
                    Divider().overlay(Palette.line)
                    HStack(spacing: 9) {
                        Circle().fill(model.tunnelRunning ? Palette.accent : Palette.warning).frame(width: 7, height: 7)
                        Text("转发进程").font(.system(size: 11, weight: .medium))
                        Spacer()
                        Text(model.tunnelRunning ? "运行中" : "连接中").font(.system(size: 10)).foregroundStyle(Palette.muted)
                    }
                    HStack(spacing: 9) {
                        Circle().fill(model.agentOnline ? Palette.accent : Palette.warning).frame(width: 7, height: 7)
                        Text("NUC Agent").font(.system(size: 11, weight: .medium))
                        Spacer()
                        Text(model.agentOnline ? "在线" : "等待").font(.system(size: 10)).foregroundStyle(Palette.muted)
                    }
                    Button(action: model.reconnect) {
                        Label("重新连接", systemImage: "arrow.clockwise")
                            .font(.system(size: 11, weight: .medium))
                            .foregroundStyle(Palette.muted)
                    }
                    .buttonStyle(.plain)
                    .padding(.top, 3)
                }
                .padding(.horizontal, 16).padding(.bottom, 18)
            }
            .background(Palette.background)
            .navigationSplitViewColumnWidth(min: 190, ideal: 215, max: 240)
        } detail: {
            VStack(spacing: 0) {
                if model.section != .simulation && model.section != .armPreview {
                    ConsoleHeader(model: model)
                }
                ScrollView {
                    Group {
                        switch model.section {
                        case .cockpit: TestCockpitView(model: model)
                        case .moduleTests: ModuleTestsView(model: moduleTests)
                        case .simulation: SimulationDriveView(model: simulator)
                        case .armPreview: ArmPreviewView(model: armPreview)
                        case .diagnostics: DiagnosticsView(model: model)
                        case .history: HistoryView(model: model)
                        }
                    }
                    .frame(maxWidth: 1320, alignment: .topLeading)
                    .padding(.horizontal, 28)
                    .padding(.vertical, 24)
                    .frame(maxWidth: .infinity, alignment: .top)
                }
            }
            .background(Palette.background)
        }
        .tint(Palette.accent)
        .onDisappear {
            simulator.stop()
            armPreview.stop()
        }
    }
}

private struct ConsoleHeader: View {
    @ObservedObject var model: ConsoleModel

    var body: some View {
        HStack(alignment: .center) {
            VStack(alignment: .leading, spacing: 4) {
                Text(model.section.rawValue).font(.system(size: 23, weight: .semibold, design: .rounded))
                Text(model.snapshot.map { "\($0.host.hostname) · \($0.host.operatingSystem) · ROS Domain \($0.ros.domainId.map(String.init) ?? "未知")" } ?? "NUC · 等待真实设备快照")
                    .font(.system(size: 11)).foregroundStyle(Palette.muted)
            }
            Spacer()
            VStack(alignment: .trailing, spacing: 6) {
                HStack(spacing: 7) {
                    StatusDot(isOn: model.tunnelRunning)
                    Text(model.tunnelRunning ? "转发进程" : "转发未启动").font(.system(size: 11, weight: .medium))
                    Rectangle().fill(Palette.line).frame(width: 1, height: 12).padding(.horizontal, 4)
                    StatusDot(isOn: model.agentOnline)
                    Text(model.agentOnline ? "Agent 在线" : "Agent 未连接").font(.system(size: 11, weight: .medium))
                }
                Text(model.agentMessage).font(.system(size: 10)).foregroundStyle(Palette.muted).lineLimit(1)
            }
            Button(action: model.reconnect) {
                Image(systemName: "arrow.clockwise").font(.system(size: 12, weight: .semibold)).frame(width: 34, height: 34)
                    .background(Palette.panelRaised, in: RoundedRectangle(cornerRadius: 9))
            }
            .buttonStyle(.plain).help("重新连接 NUC")
            .padding(.leading, 10)
        }
        .padding(.horizontal, 28).padding(.vertical, 18)
        .background(Palette.background.opacity(0.96))
        .overlay(alignment: .bottom) { Rectangle().fill(Palette.line).frame(height: 1) }
    }
}

struct StatusDot: View {
    let isOn: Bool
    var body: some View { Circle().fill(isOn ? Palette.accent : Palette.warning).frame(width: 7, height: 7) }
}
