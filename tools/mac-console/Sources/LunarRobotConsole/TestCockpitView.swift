import AppKit
import SwiftUI

struct TestCockpitView: View {
    @ObservedObject var model: ConsoleModel

    private var source: VisionStreamReading? { model.vision?.stream(model.selectedVisionStream) }
    private var isLive: Bool { model.agentOnline && source?.status == "streaming" && model.visionFrame != nil }
    private var isSynthetic: Bool { source?.frameId?.hasPrefix("lunar-v0.2a-synthetic/") == true }

    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            HStack {
                VStack(alignment: .leading, spacing: 5) {
                    Text("视觉模块测试").font(.system(size: 20, weight: .semibold, design: .rounded))
                    Text("先确认真实输入，再判断模块输出；本页不会启动设备或下发控制命令")
                        .font(.system(size: 11)).foregroundStyle(Palette.muted)
                }
                Spacer()
                HStack(spacing: 7) {
                    StatusDot(isOn: isLive)
                    Text(isLive ? (isSynthetic ? "合成测试流" : "ROS 图像流") : "只读观测")
                        .font(.system(size: 10, weight: .semibold))
                        .foregroundStyle(isLive ? Palette.accent : Palette.warning)
                }
                .padding(.horizontal, 11).padding(.vertical, 8)
                .background(Palette.panelRaised, in: Capsule())
            }

            HealthCockpitSummary(events: model.healthEvents, workspaceInventory: model.snapshot?.workspaces)
            WorkspaceCockpitSummary(inventory: model.snapshot?.workspaces, agentOnline: model.agentOnline)

            HStack(alignment: .top, spacing: 16) {
                VStack(alignment: .leading, spacing: 13) {
                    HStack(spacing: 8) {
                        ForEach(VisionStreamID.allCases) { stream in
                            let reading = model.vision?.stream(stream)
                            Button { model.selectVisionStream(stream) } label: {
                                HStack(spacing: 7) {
                                    StatusDot(isOn: reading?.status == "streaming")
                                    Text(stream.label)
                                    if reading?.status == "unavailable" { Text("不可用").font(.system(size: 9)).foregroundStyle(Palette.muted) }
                                }
                                .font(.system(size: 11, weight: model.selectedVisionStream == stream ? .semibold : .medium))
                                .foregroundStyle(model.selectedVisionStream == stream ? .white : Palette.muted)
                                .padding(.horizontal, 12).frame(height: 36)
                                .background(model.selectedVisionStream == stream ? Palette.panelRaised : Palette.panel, in: RoundedRectangle(cornerRadius: 9))
                                .overlay(RoundedRectangle(cornerRadius: 9).stroke(model.selectedVisionStream == stream ? Palette.accent.opacity(0.6) : Palette.line, lineWidth: 1))
                            }
                            .buttonStyle(.plain)
                        }
                        Spacer(minLength: 0)
                    }

                    Panel(title: "\(model.selectedVisionStream.label) 观测区", subtitle: source?.topic ?? "固定 ROS Image 接口 · 等待 Agent") {
                        ZStack(alignment: .topLeading) {
                            RoundedRectangle(cornerRadius: 9).fill(Color.black.opacity(0.48))
                            if source?.status == "streaming", let data = model.visionFrame, let image = NSImage(data: data) {
                                Image(nsImage: image)
                                    .resizable()
                                    .scaledToFit()
                                    .padding(8)
                                HStack(spacing: 6) {
                                    Circle().fill(Palette.danger).frame(width: 7, height: 7)
                                    Text(isSynthetic ? "SYNTHETIC" : "ROS LIVE").font(.system(size: 9, weight: .bold, design: .monospaced))
                                }
                                .padding(.horizontal, 9).padding(.vertical, 6)
                                .background(.black.opacity(0.72), in: Capsule())
                                .padding(12)
                            } else {
                                VStack(spacing: 10) {
                                    Image(systemName: placeholderSymbol)
                                        .font(.system(size: 31, weight: .light)).foregroundStyle(Palette.muted)
                                    Text(stateTitle).font(.system(size: 14, weight: .semibold))
                                    Text(stateDetail).font(.system(size: 10)).foregroundStyle(Palette.muted)
                                        .multilineTextAlignment(.center).textSelection(.enabled)
                                        .frame(maxWidth: 460)
                                }
                                .frame(maxWidth: .infinity, maxHeight: .infinity)
                            }
                        }
                        .frame(maxWidth: .infinity)
                        .aspectRatio(16 / 9, contentMode: .fit)

                        HStack(spacing: 0) {
                            VisionMetric(title: "NUC 接收", value: source?.frameRateHz.map { String(format: "%.1f FPS", $0) } ?? "未知")
                            VisionMetric(title: "Mac 更新", value: model.macFrameUpdateHz.map { String(format: "%.1f FPS", $0) } ?? "未知")
                            VisionMetric(title: "帧新鲜度", value: source?.frameAgeMs.map { "\($0) ms" } ?? "未知")
                            VisionMetric(title: "传输往返", value: model.visionFrameRoundTripMs.map { String(format: "%.0f ms", $0) } ?? "未知")
                            VisionMetric(title: "编码 / 分辨率", value: imageFormat)
                        }
                        .padding(.top, 3)

                        HStack {
                            Text(source?.sourceStamp.map { "ROS Header · \($0)" } ?? "ROS Header 时间未知")
                            Spacer()
                            Text(source?.headerToAgentMs.map { String(format: "Header→NUC %.0f ms · 时钟未校验", $0) } ?? "采集时延不可确认")
                        }
                        .font(.system(size: 9, design: .monospaced)).foregroundStyle(Palette.muted)

                        if let frameNote, !frameNote.isEmpty {
                            Label(frameNote, systemImage: "info.circle")
                                .font(.system(size: 9)).foregroundStyle(Palette.warning)
                        }
                    }
                }
                .frame(maxWidth: .infinity, alignment: .topLeading)

                VStack(alignment: .leading, spacing: 14) {
                    Panel(title: "视觉链路", subtitle: "Topic 发布与新鲜帧分别判断") {
                        VStack(alignment: .leading, spacing: 11) {
                            ForEach(VisionStreamID.allCases) { stream in
                                VisionSourceRow(stream: stream, reading: model.vision?.stream(stream))
                            }
                            Divider().overlay(Palette.line)
                            VStack(alignment: .leading, spacing: 4) {
                                Text("当前观测结果").font(.system(size: 10, weight: .medium))
                                Text(moduleHealth)
                                    .font(.system(size: 12, weight: .semibold))
                                    .foregroundStyle(isLive ? Palette.accent : Palette.warning)
                                Text(moduleHealthDetail).font(.system(size: 9)).foregroundStyle(Palette.muted).lineLimit(3)
                            }
                        }
                    }

                    Panel(title: "模块测试区", subtitle: "离线机械臂可查看；真实设备仍待验收") {
                        VStack(alignment: .leading, spacing: 11) {
                            PreviewRow(title: "机械臂离线姿态", detail: "六轴与夹爪 SIMULATION · 左侧选择「机械臂预览」")
                            ReservedRow(title: "MID-360 点云 / IMU", reason: "尚未接入可验证的只读可视化流")
                            ReservedRow(title: "实体运动控制", reason: "保持锁定；WASD 仅在单独的模拟驾驶区可用")
                        }
                    }

                    Panel(title: "连接与安全", subtitle: "沿用固定 SSH 转发 · ROS 不暴露给 Mac") {
                        VStack(alignment: .leading, spacing: 8) {
                            Text(model.connectionMessage).font(.system(size: 10)).foregroundStyle(Palette.muted)
                            Text(model.agentMessage).font(.system(size: 10)).foregroundStyle(model.agentOnline ? Palette.accent : Palette.warning)
                            Label("只读订阅 · 不启动相机 / 执行器", systemImage: "lock.fill")
                                .font(.system(size: 9, weight: .medium)).foregroundStyle(Palette.accent).padding(.top, 2)
                        }
                    }
                }
                .frame(width: 300)
            }
        }
    }

    private var stateTitle: String {
        guard model.agentOnline else { return "NUC Agent 未连接" }
        if let message = model.visionMessage { return message }
        switch source?.status {
        case "streaming": return model.visionFrameMessage ?? "等待图像帧"
        case "stale": return "数据流已断开"
        case "waiting": return "等待首帧"
        case "unavailable": return "数据源不可用"
        case "unsupported": return "图像类型不支持"
        case "unknown": return "数据源状态未知"
        default: return "等待视觉数据接口"
        }
    }

    private var stateDetail: String {
        guard model.agentOnline else { return "连接恢复后将读取 NUC 的实时状态；不会显示缓存帧。" }
        if let message = model.visionMessage { return message }
        return source?.reason ?? model.visionFrameMessage ?? "当前 NUC Agent 尚未报告此数据源。"
    }

    private var placeholderSymbol: String {
        source?.status == "stale" ? "wifi.slash" : model.agentOnline ? "camera" : "network.slash"
    }

    private var imageFormat: String {
        guard let source, let width = source.width, let height = source.height else { return "未知" }
        return "\(source.encoding ?? "?") · \(width)×\(height)"
    }

    private var frameNote: String? {
        model.selectedVisionStream == .depth && source?.status == "streaming" ? "深度灰度按当前帧范围拉伸显示，不作为距离刻度。" : nil
    }

    private var moduleHealth: String {
        guard model.agentOnline else { return "未连接" }
        guard model.visionMessage == nil else { return "视觉接口不可用" }
        switch source?.status {
        case "streaming": return isSynthetic ? "合成测试图像正在更新" : "ROS 图像正在更新（实机未验收）"
        case "stale": return "数据流断开"
        case "waiting": return "Topic 存在，尚未收到帧"
        case "unavailable": return "未发现数据发布者"
        case "unsupported": return "消息类型不兼容"
        default: return "状态未知"
        }
    }

    private var moduleHealthDetail: String {
        if isSynthetic { return "独立 ROS Domain 合成数据；不代表真实相机、深度测距或 YOLO 推理通过验收。" }
        if model.selectedVisionStream == .yolo && source?.status == "streaming" {
            return "检测标注图在更新；这证明输出流存活，不代表识别精度已验收。"
        }
        return source?.reason ?? "以 Topic 发布状态和实际到帧时间判断，不根据节点名称推断健康。"
    }
}

private struct VisionMetric: View {
    let title: String
    let value: String

    var body: some View {
        VStack(alignment: .leading, spacing: 5) {
            Text(title).font(.system(size: 9)).foregroundStyle(Palette.muted)
            Text(value).font(.system(size: 10, weight: .medium, design: .monospaced)).lineLimit(1).minimumScaleFactor(0.8)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

private struct VisionSourceRow: View {
    let stream: VisionStreamID
    let reading: VisionStreamReading?

    var body: some View {
        HStack(alignment: .top, spacing: 8) {
            StatusDot(isOn: reading?.status == "streaming")
            VStack(alignment: .leading, spacing: 3) {
                HStack {
                    Text(stream.label).font(.system(size: 10, weight: .medium))
                    Spacer()
                    Text(statusLabel).font(.system(size: 9, weight: .semibold)).foregroundStyle(reading?.status == "streaming" ? Palette.accent : Palette.warning)
                }
                Text(reading?.reason ?? reading?.topic ?? "等待 Agent 状态")
                    .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted).lineLimit(2)
            }
        }
    }

    private var statusLabel: String {
        switch reading?.status {
        case "streaming": return "实时"
        case "stale": return "断流"
        case "waiting": return "等待帧"
        case "unavailable": return "不可用"
        case "unsupported": return "不兼容"
        default: return "未知"
        }
    }
}

private struct ReservedRow: View {
    let title: String
    let reason: String

    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            HStack(spacing: 7) {
                Image(systemName: "minus.circle").foregroundStyle(Palette.muted)
                Text(title).font(.system(size: 10, weight: .medium))
                Spacer()
                Text("不可用").font(.system(size: 8, weight: .semibold)).foregroundStyle(Palette.warning)
            }
            Text(reason).font(.system(size: 8)).foregroundStyle(Palette.muted).padding(.leading, 17)
        }
    }
}

private struct PreviewRow: View {
    let title: String
    let detail: String

    var body: some View {
        VStack(alignment: .leading, spacing: 3) {
            HStack(spacing: 7) {
                Image(systemName: "cube.transparent").foregroundStyle(Palette.accent)
                Text(title).font(.system(size: 10, weight: .medium))
                Spacer()
                Text("SIMULATION").font(.system(size: 8, weight: .semibold)).foregroundStyle(Palette.accent)
            }
            Text(detail).font(.system(size: 8)).foregroundStyle(Palette.muted).padding(.leading, 17)
        }
    }
}
