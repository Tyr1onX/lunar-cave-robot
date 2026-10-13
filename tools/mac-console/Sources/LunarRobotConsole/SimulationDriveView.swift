import AppKit
import SwiftUI

// An actual AppKit responder receives simultaneous physical key-down/up events.
// It stops on resign-first-responder, window deactivation, or removal from window.
private final class DriveKeyView: NSView {
    var onKeys: ((Set<DriveKey>) -> Void)?
    var onFocus: ((Bool) -> Void)?
    private var keys = Set<DriveKey>()
    private var notification: NSObjectProtocol?

    override var acceptsFirstResponder: Bool { true }

    override func viewDidMoveToWindow() {
        super.viewDidMoveToWindow()
        if let notification { NotificationCenter.default.removeObserver(notification) }
        notification = nil
        if let window {
            notification = NotificationCenter.default.addObserver(
                forName: NSWindow.didResignKeyNotification, object: window, queue: .main
            ) { [weak self] _ in
                self?.clearKeys()
                self?.onFocus?(false)
            }
        } else {
            clearKeys()
            onFocus?(false)
        }
    }

    override func mouseDown(with event: NSEvent) {
        window?.makeFirstResponder(self)
    }

    override func becomeFirstResponder() -> Bool {
        onFocus?(true)
        return true
    }

    override func resignFirstResponder() -> Bool {
        clearKeys()
        onFocus?(false)
        return true
    }

    override func keyDown(with event: NSEvent) {
        guard let key = DriveKey(rawValue: Int(event.keyCode)) else {
            super.keyDown(with: event)
            return
        }
        if keys.insert(key).inserted { onKeys?(keys) }
    }

    override func keyUp(with event: NSEvent) {
        guard let key = DriveKey(rawValue: Int(event.keyCode)) else {
            super.keyUp(with: event)
            return
        }
        if keys.remove(key) != nil { onKeys?(keys) }
    }

    private func clearKeys() {
        if !keys.isEmpty {
            keys.removeAll()
            onKeys?([])
        }
    }

    deinit {
        if let notification { NotificationCenter.default.removeObserver(notification) }
    }
}

private struct DriveKeyboardCapture: NSViewRepresentable {
    let onKeys: (Set<DriveKey>) -> Void
    let onFocus: (Bool) -> Void

    func makeNSView(context: Context) -> DriveKeyView {
        let view = DriveKeyView()
        view.onKeys = onKeys
        view.onFocus = onFocus
        return view
    }

    func updateNSView(_ view: DriveKeyView, context: Context) {
        view.onKeys = onKeys
        view.onFocus = onFocus
    }
}

struct SimulationDriveView: View {
    @ObservedObject var model: SimulationModel

    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            HStack(alignment: .center) {
                VStack(alignment: .leading, spacing: 4) {
                    Text("SIMULATION").font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(2.0).foregroundStyle(Palette.accent)
                    Text("月面驾驶 · 离线模拟").font(.system(size: 23, weight: .semibold, design: .rounded))
                    Text("P6 Cave Simulator · 数学差速模型，不是六轮车真实运动反馈")
                        .font(.system(size: 11)).foregroundStyle(Palette.muted)
                }
                Spacer()
                HStack(spacing: 8) {
                    Circle().fill(model.ready ? Palette.accent : Palette.warning).frame(width: 8, height: 8)
                    Text(model.ready ? "模拟环境就绪" : "模拟环境不可用")
                        .font(.system(size: 11, weight: .semibold))
                }
                .padding(11)
                .background(Palette.panel, in: Capsule())
            }

            VStack(alignment: .leading, spacing: 12) {
                HStack(spacing: 10) {
                    Image(systemName: "gamecontroller.fill").foregroundStyle(Palette.accent)
                    Text(model.focused ? "键盘已接管模拟驾驶 · WASD" : "点击地图以启用 WASD")
                        .foregroundStyle(model.focused ? Palette.accent : .white)
                    Spacer()
                    Text("ROS 217 · /console_sim/cmd_vel").foregroundStyle(Palette.muted)
                }
                .font(.system(size: 11, weight: .medium, design: .monospaced))

                ZStack {
                    RoundedRectangle(cornerRadius: 12).fill(Color.black.opacity(0.44))
                    if let map = model.map, let state = model.state, state.ready, let pose = state.odom {
                        SimulationMapCanvas(map: map, pose: pose, trail: state.trail)
                            .padding(16)
                    } else {
                        VStack(spacing: 12) {
                            Image(systemName: "point.3.connected.trianglepath.dotted").font(.system(size: 33))
                                .foregroundStyle(Palette.muted)
                            Text("等待独立 P6 模拟器").font(.system(size: 14, weight: .medium))
                            Text("需在 NUC 运行 tools/console-sim/run.sh；不会自动启动任何设备。")
                                .font(.system(size: 11)).foregroundStyle(Palette.muted)
                        }
                    }
                    // Captures only this simulation view; never receives keys elsewhere.
                    if model.ready {
                        DriveKeyboardCapture(onKeys: model.setKeys, onFocus: model.setFocus)
                            .background(Color.clear)
                            .contentShape(Rectangle())
                    }
                }
                .frame(maxWidth: .infinity)
                .frame(height: 500)
                .clipShape(RoundedRectangle(cornerRadius: 12))
                .overlay(RoundedRectangle(cornerRadius: 12)
                    .stroke(model.focused ? Palette.accent.opacity(0.8) : Palette.line, lineWidth: 1))

                HStack(spacing: 16) {
                    readout("X 坐标", model.state?.odom?.x, unit: "m")
                    readout("Y 坐标", model.state?.odom?.y, unit: "m")
                    readout("航向", model.state?.odom.map { $0.yaw * 180 / .pi }, unit: "°")
                    readout("线速度", model.state?.odom?.linear, unit: "m/s")
                    readout("角速度", model.state?.odom?.angular, unit: "rad/s")
                }
                .frame(maxWidth: .infinity, alignment: .leading)
            }
            .padding(18)
            .background(Palette.panel, in: RoundedRectangle(cornerRadius: 15))

            HStack(spacing: 18) {
                VStack(alignment: .leading, spacing: 6) {
                    Text("W 前进 · S 后退 · A 左转 · D 右转").font(.system(size: 12, weight: .semibold))
                    Text("支持组合键 · 松键、失焦、断网或离开本页会停止模拟指令")
                        .font(.system(size: 10)).foregroundStyle(Palette.muted)
                }
                Spacer()
                HStack(spacing: 6) {
                    ForEach(DriveKey.allCases, id: \.rawValue) { key in
                        Text(String(describing: key).uppercased())
                            .font(.system(size: 12, weight: .semibold, design: .monospaced))
                            .frame(width: 32, height: 32)
                            .background(model.pressed.contains(key) ? Palette.accent.opacity(0.38) : Palette.panelRaised,
                                        in: RoundedRectangle(cornerRadius: 6))
                    }
                }
            }
            Text(model.message).font(.system(size: 10, design: .monospaced))
                .foregroundStyle(model.ready ? Palette.muted : Palette.warning)
        }
        .onAppear { model.start() }
        .onDisappear { model.stop() }
    }

    private func readout(_ label: String, _ value: Double?, unit: String) -> some View {
        VStack(alignment: .leading, spacing: 7) {
            Text(label).font(.system(size: 10)).foregroundStyle(Palette.muted)
            Text(value.map { String(format: "%.2f %@", $0, unit) } ?? "—")
                .font(.system(size: 16, weight: .semibold, design: .monospaced))
                .lineLimit(1)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
    }
}

private struct SimulationMapCanvas: View {
    let map: SimMap
    let pose: SimPose
    let trail: [[Double]]

    var body: some View {
        Canvas { context, size in
            let scale = min(size.width / CGFloat(map.width), size.height / CGFloat(map.height))
            let offset = CGPoint(x: (size.width - CGFloat(map.width) * scale) * 0.5,
                                 y: (size.height - CGFloat(map.height) * scale) * 0.5)

            func pt(_ x: Double, _ y: Double) -> CGPoint {
                CGPoint(x: offset.x + CGFloat((x - map.originX) / map.resolution) * scale,
                        y: offset.y + (CGFloat(map.height) - CGFloat((y - map.originY) / map.resolution)) * scale)
            }

            let background = CGRect(origin: offset,
                                    size: CGSize(width: CGFloat(map.width) * scale,
                                                 height: CGFloat(map.height) * scale))
            context.fill(Path(background), with: .color(Color(red: 0.08, green: 0.12, blue: 0.16)))

            var obstacles = Path()
            for row in 0..<map.height {
                var column = 0
                while column < map.width {
                    if map.cells[row * map.width + column] < 50 { column += 1; continue }
                    let first = column
                    while column < map.width && map.cells[row * map.width + column] >= 50 { column += 1 }
                    obstacles.addRect(CGRect(x: offset.x + CGFloat(first) * scale,
                                             y: offset.y + CGFloat(map.height - 1 - row) * scale,
                                             width: CGFloat(column - first) * scale, height: scale))
                }
            }
            context.fill(obstacles, with: .color(Color(red: 0.29, green: 0.38, blue: 0.47)))

            var grid = Path()
            let spacing = max(1, Int(1.0 / map.resolution))
            for x in stride(from: 0, through: map.width, by: spacing) {
                let xx = offset.x + CGFloat(x) * scale
                grid.move(to: CGPoint(x: xx, y: background.minY))
                grid.addLine(to: CGPoint(x: xx, y: background.maxY))
            }
            for y in stride(from: 0, through: map.height, by: spacing) {
                let yy = offset.y + CGFloat(y) * scale
                grid.move(to: CGPoint(x: background.minX, y: yy))
                grid.addLine(to: CGPoint(x: background.maxX, y: yy))
            }
            context.stroke(grid, with: .color(.white.opacity(0.08)), lineWidth: 1)

            var path = Path()
            for (index, point) in trail.enumerated() where point.count == 2 {
                let p = pt(point[0], point[1])
                if index == 0 { path.move(to: p) } else { path.addLine(to: p) }
            }
            context.stroke(path, with: .color(Palette.blue), style: StrokeStyle(lineWidth: 2.5, lineCap: .round))

            let center = pt(pose.x, pose.y)
            let heading = pose.yaw
            let tip = CGPoint(x: center.x + 19 * cos(heading), y: center.y - 19 * sin(heading))
            let left = CGPoint(x: center.x + 12 * cos(heading + 2.4),
                               y: center.y - 12 * sin(heading + 2.4))
            let right = CGPoint(x: center.x + 12 * cos(heading - 2.4),
                                y: center.y - 12 * sin(heading - 2.4))
            var arrow = Path()
            arrow.move(to: tip); arrow.addLine(to: left); arrow.addLine(to: right); arrow.closeSubpath()
            context.fill(arrow, with: .color(Palette.accent))
            context.stroke(arrow, with: .color(.white.opacity(0.9)), lineWidth: 1.5)
            context.stroke(Path(ellipseIn: CGRect(x: center.x - 23, y: center.y - 23, width: 46, height: 46)),
                           with: .color(Palette.accent.opacity(0.3)), lineWidth: 1)
        }
        .accessibilityLabel("SIMULATION 数学地图：位置、朝向及轨迹")
    }
}
