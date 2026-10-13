import SwiftUI

struct ArmPreviewView: View {
    @ObservedObject var model: ArmPreviewModel

    private var foxgloveURL: URL? {
        var components = URLComponents()
        components.scheme = "https"
        components.host = "app.foxglove.dev"
        components.path = "/~/view"
        components.queryItems = [
            URLQueryItem(name: "ds", value: "foxglove-websocket"),
            URLQueryItem(name: "ds.url", value: "ws://127.0.0.1:18768"),
        ]
        return components.url
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            HStack(alignment: .center) {
                VStack(alignment: .leading, spacing: 4) {
                    Text("SIMULATION").font(.system(size: 11, weight: .bold, design: .monospaced))
                        .tracking(2).foregroundStyle(Palette.accent)
                    Text("机械臂三维姿态预览").font(.system(size: 23, weight: .semibold, design: .rounded))
                    Text("复用 tzb_description URDF / STL / SRDF · Foxglove 3D")
                        .font(.system(size: 11)).foregroundStyle(Palette.muted)
                }
                Spacer()
                HStack(spacing: 8) {
                    StatusDot(isOn: model.ready)
                    Text(model.ready ? "隔离模拟已连接" : "等待模拟服务")
                        .font(.system(size: 11, weight: .semibold))
                }
                .padding(11).background(Palette.panel, in: Capsule())
            }

            Panel(title: "六轴关节", subtitle: "角度由现有 URDF 限位约束 · 数值为弧度") {
                if model.ready, !model.joints.isEmpty {
                    VStack(spacing: 13) {
                        ForEach(model.joints) { joint in
                            jointControl(joint)
                        }
                        Divider().overlay(Palette.line).padding(.vertical, 2)
                        HStack(spacing: 12) {
                            Text("夹爪模拟开合").font(.system(size: 11, weight: .medium))
                                .frame(width: 110, alignment: .leading)
                            Slider(value: Binding(
                                get: { model.gripperOpening },
                                set: model.setGripperOpening
                            ), in: 0...1)
                            Text(String(format: "%.0f%%", model.gripperOpening * 100))
                                .font(.system(size: 10, design: .monospaced)).frame(width: 54, alignment: .trailing)
                        }
                        .tint(Palette.accent)
                        HStack {
                            Text("夹爪两轴按现有 URDF 范围镜像；开合方向未实机确认")
                                .font(.system(size: 9)).foregroundStyle(Palette.muted)
                            Spacer()
                            Button("复位模拟姿态", systemImage: "arrow.counterclockwise", action: model.resetPose)
                                .buttonStyle(.bordered).tint(Palette.blue)
                        }
                    }
                } else {
                    VStack(alignment: .leading, spacing: 9) {
                        Text("等待 NUC 上手动启动离线模拟")
                            .font(.system(size: 13, weight: .semibold))
                        Text("运行 tools/console-arm-preview/run.sh 后，本页才会建立固定 SSH 转发并读取六轴关节范围。此页面不会远程启动 ROS、控制器或实体设备。")
                            .font(.system(size: 10)).foregroundStyle(Palette.muted)
                    }
                    .frame(maxWidth: .infinity, alignment: .leading).padding(.vertical, 6)
                }
            }

            HStack(alignment: .top, spacing: 14) {
                Panel(title: "三维查看器", subtitle: "由 Foxglove 渲染 URDF 与 ROS TF") {
                    VStack(alignment: .leading, spacing: 9) {
                        Text("Foxglove WebSocket：ws://127.0.0.1:18768")
                            .font(.system(size: 9, design: .monospaced)).textSelection(.enabled)
                        if let foxgloveURL {
                            Link(destination: foxgloveURL) {
                                Label("打开 Foxglove 3D", systemImage: "cube.transparent")
                                    .font(.system(size: 11, weight: .semibold))
                            }
                            .disabled(!model.ready)
                        }
                        Text("保持本页打开以保留安全 SSH 转发。若 Foxglove 布局为空，添加 3D Panel；URDF 从 robot_state_publisher 的 robot_description 读取，位姿来自 TF。")
                            .font(.system(size: 9)).foregroundStyle(Palette.muted)
                        Text("Foxglove 网页可能要求登录；Console 不会替你提交账户信息。ROS 数据连接经本机 SSH 转发。")
                            .font(.system(size: 9)).foregroundStyle(Palette.muted)
                    }
                }
                Panel(title: "模型基准", subtitle: "仓库几何值 · 未经实机标定") {
                    VStack(alignment: .leading, spacing: 7) {
                        frameRow("J3 相机", model.camera)
                        frameRow("TCP", model.tcp)
                        if let state = model.state {
                            Text("SRDF hand parent：\(state.srdfHandParentLink) · STL \(state.meshCount) 件 · SHA \(state.urdfSha256.prefix(12))")
                                .font(.system(size: 9, design: .monospaced)).foregroundStyle(Palette.muted)
                            Text("URDF base_link 只是简化底盘网格，不是完整六轮数字孪生。零位、相机安装、TCP 和夹爪方向均未实机确认。")
                                .font(.system(size: 9)).foregroundStyle(Palette.warning)
                        }
                    }
                }
            }

            Text(model.message).font(.system(size: 10, design: .monospaced))
                .foregroundStyle(model.ready ? Palette.muted : Palette.warning)
        }
        .onAppear { model.start() }
        .onDisappear { model.stop() }
    }

    private func jointControl(_ joint: ArmPreviewJoint) -> some View {
        let value = model.position(for: joint)
        return VStack(spacing: 5) {
            HStack(spacing: 12) {
                Text(joint.label).font(.system(size: 11, weight: .semibold, design: .monospaced))
                    .frame(width: 82, alignment: .leading)
                Slider(value: Binding(
                    get: { model.position(for: joint) },
                    set: { model.setJoint(joint.name, to: $0) }
                ), in: joint.lower...joint.upper)
                .tint(Palette.accent)
                Text(String(format: "%.3f rad · %+.1f°", value, value * 180 / .pi))
                    .font(.system(size: 9, design: .monospaced))
                    .frame(width: 128, alignment: .trailing)
            }
            HStack {
                Text(String(format: "%.4f rad", joint.lower))
                Spacer()
                Text("轴 (\(joint.axis.map { String(format: "%.0f", $0) }.joined(separator: ",")))")
                Spacer()
                Text(String(format: "%.4f rad", joint.upper))
            }
            .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
                .padding(.leading, 94)
        }
    }

    private func frameRow(_ title: String, _ frame: ArmPreviewFrame?) -> some View {
        let xyz = frame?.xyz ?? []
        let rpy = frame?.rpy ?? []
        return VStack(alignment: .leading, spacing: 3) {
            Text("\(title) · \(frame?.parent ?? "—") → \(frame?.child ?? "—")")
                .font(.system(size: 9, weight: .semibold))
            Text("xyz \(vector(xyz)) m · rpy \(vector(rpy)) rad")
                .font(.system(size: 8, design: .monospaced)).foregroundStyle(Palette.muted)
        }
    }

    private func vector(_ values: [Double]) -> String {
        guard values.count == 3 else { return "—" }
        return String(format: "(%.4f, %.4f, %.4f)", values[0], values[1], values[2])
    }
}
