import AppKit
import Foundation
import SwiftUI
import UniformTypeIdentifiers

struct ModuleTestCatalog: Decodable {
    let busy: Bool
    let tests: [ModuleTestDefinition]
}

struct ModuleTestDefinition: Decodable, Identifiable {
    let id: String
    let category: String
    let title: String
    let description: String
    let mode: String
    let state: String
    let reason: String?
    let inputSource: String
    let prerequisites: [String]
    let timeoutSeconds: Int
}

private struct ModuleTestStart: Decodable {
    let runId: String
    let testId: String
    let status: String
}

private struct ModuleTestLogChunk: Decodable {
    let lines: [String]
    let cursor: Int
}

struct ModuleTestRun: Decodable {
    let runId: String
    let testId: String
    let status: String
    let result: String
    let codeSha: String?
    let codeDirty: Bool?
    let startedAt: String
    let finishedAt: String?
    let durationMs: Int?
    let reason: String?
}

@MainActor
final class ModuleTestsModel: ObservableObject {
    @Published private(set) var catalog: ModuleTestCatalog?
    @Published private(set) var selectedTestID: String?
    @Published private(set) var run: ModuleTestRun?
    @Published private(set) var logs: [String] = []
    @Published private(set) var runnerOnline = false
    @Published private(set) var checkingEnvironment = false
    @Published private(set) var submitting = false
    @Published private(set) var savingReport = false
    @Published private(set) var checkedTestID: String?
    @Published private(set) var message = "等待 NUC 受限测试服务"

    private let apiRoot = URL(string: "http://127.0.0.1:18769")!
    private let session: URLSession = {
        let configuration = URLSessionConfiguration.ephemeral
        configuration.timeoutIntervalForRequest = 5
        configuration.timeoutIntervalForResource = 8
        configuration.waitsForConnectivity = false
        return URLSession(configuration: configuration)
    }()
    private var catalogTask: Task<Void, Never>?
    private var runTask: Task<Void, Never>?
    private var logCursor = 0

    var selectedTest: ModuleTestDefinition? {
        catalog?.tests.first(where: { $0.id == selectedTestID })
    }

    var isRunning: Bool {
        run?.status == "running" || run?.status == "cancelling"
    }

    var anotherRunIsActive: Bool {
        catalog?.busy == true && !isRunning
    }

    var canSaveReport: Bool {
        guard let status = run?.status else { return false }
        return ["passed", "failed", "cancelled", "timed_out"].contains(status)
    }

    func start() {
        guard catalogTask == nil else { return }
        catalogTask = Task { [weak self] in
            while !Task.isCancelled {
                await self?.refreshCatalog()
                try? await Task.sleep(for: .seconds(4))
            }
        }
    }

    func stop() {
        catalogTask?.cancel()
        catalogTask = nil
        runTask?.cancel()
        runTask = nil
    }

    func select(_ test: ModuleTestDefinition) {
        guard selectedTestID != test.id else { return }
        selectedTestID = test.id
        checkedTestID = nil
        run = nil
        logs = []
        logCursor = 0
        message = "已选择：\(test.title)"
    }

    func checkEnvironment() async {
        guard let selectedTestID else {
            message = "请先选择测试项目"
            return
        }
        checkingEnvironment = true
        defer { checkingEnvironment = false }
        await refreshCatalog()
        checkedTestID = selectedTestID
        guard let test = selectedTest else {
            message = "NUC 测试项目列表暂不可用"
            return
        }
        message = test.state == "ready" ? "NUC 前置条件核对完成 · 可执行 \(test.mode) 检查" : (test.reason ?? "前置条件未满足")
    }

    func runSelected() {
        guard !submitting, !isRunning else { return }
        submitting = true
        Task { [weak self] in
            guard let self else { return }
            defer { self.submitting = false }
            await self.checkEnvironment()
            guard let test = self.selectedTest else { return }
            guard test.state == "ready" else { return }
            do {
                var request = URLRequest(url: self.apiRoot.appendingPathComponent("v1/tests/run"))
                request.httpMethod = "POST"
                request.setValue("application/json", forHTTPHeaderField: "Content-Type")
                request.httpBody = try JSONSerialization.data(withJSONObject: ["test_id": test.id])
                let (data, response) = try await self.session.data(for: request)
                guard let http = response as? HTTPURLResponse else { throw URLError(.cannotParseResponse) }
                guard http.statusCode == 202 else {
                    let error = (try? JSONDecoder().decode(ModuleTestError.self, from: data).error) ?? "NUC 未接受该测试"
                    self.message = error
                    await self.refreshCatalog()
                    return
                }
                let decoder = JSONDecoder()
                decoder.keyDecodingStrategy = .convertFromSnakeCase
                let started = try decoder.decode(ModuleTestStart.self, from: data)
                self.run = ModuleTestRun(runId: started.runId, testId: started.testId, status: started.status,
                                         result: started.status, codeSha: nil, codeDirty: nil, startedAt: "", finishedAt: nil,
                                         durationMs: nil, reason: nil)
                self.logs = []
                self.logCursor = 0
                self.message = test.mode == "LIVE" ? "NUC 已开始只读巡检 · LIVE" : "NUC 已开始执行 · \(test.mode)"
                self.runTask?.cancel()
                self.runTask = Task { await self.pollRun(started.runId) }
            } catch {
                self.message = "无法启动测试 · \(error.localizedDescription)"
            }
        }
    }

    func cancelRun() {
        guard let runID = run?.runId, isRunning else { return }
        Task {
            do {
                var request = URLRequest(url: apiRoot.appendingPathComponent("v1/tests/\(runID)/cancel"))
                request.httpMethod = "POST"
                _ = try await session.data(for: request)
                message = "已请求停止；NUC 正在清理测试进程"
            } catch {
                message = "停止请求失败 · \(error.localizedDescription)"
            }
        }
    }

    func saveReport() {
        guard let runID = run?.runId, canSaveReport, !savingReport else { return }
        savingReport = true
        Task {
            defer { savingReport = false }
            do {
                let (data, response) = try await session.data(from: apiRoot.appendingPathComponent("v1/tests/\(runID)/report"))
                guard let http = response as? HTTPURLResponse, http.statusCode == 200 else {
                    throw URLError(.cannotParseResponse)
                }
                let panel = NSSavePanel()
                panel.allowedContentTypes = [.json]
                panel.nameFieldStringValue = "lunar-module-test-\(runID).json"
                panel.canCreateDirectories = true
                guard panel.runModal() == .OK, let url = panel.url else {
                    message = "NUC 报告已保存；已取消 Mac 导出"
                    return
                }
                try data.write(to: url, options: .atomic)
                message = "报告已保存到 NUC，并导出到 Mac"
            } catch {
                message = "报告导出失败 · \(error.localizedDescription)"
            }
        }
    }

    private func refreshCatalog() async {
        do {
            let (data, response) = try await session.data(from: apiRoot.appendingPathComponent("v1/tests/catalog"))
            guard let http = response as? HTTPURLResponse, http.statusCode == 200 else { throw URLError(.badServerResponse) }
            let decoder = JSONDecoder()
            decoder.keyDecodingStrategy = .convertFromSnakeCase
            let received = try decoder.decode(ModuleTestCatalog.self, from: data)
            catalog = received
            runnerOnline = true
            if let selectedTestID, !received.tests.contains(where: { $0.id == selectedTestID }) {
                self.selectedTestID = nil
            }
            if selectedTestID == nil { selectedTestID = received.tests.first?.id }
        } catch {
            runnerOnline = false
            if !isRunning { message = "NUC 受限测试服务未连接 · 首次使用需安装一次" }
        }
    }

    private func pollRun(_ runID: String) async {
        var finished = false
        while !Task.isCancelled && !finished {
            do {
                let statusURL = apiRoot.appendingPathComponent("v1/tests/\(runID)")
                let (statusData, statusResponse) = try await session.data(from: statusURL)
                guard let http = statusResponse as? HTTPURLResponse, http.statusCode == 200 else { throw URLError(.badServerResponse) }
                let decoder = JSONDecoder()
                decoder.keyDecodingStrategy = .convertFromSnakeCase
                let received = try decoder.decode(ModuleTestRun.self, from: statusData)
                run = received
                if received.status == "passed" || received.status == "failed" || received.status == "cancelled" || received.status == "timed_out" {
                    finished = true
                }

                var components = URLComponents(url: apiRoot.appendingPathComponent("v1/tests/\(runID)/logs"), resolvingAgainstBaseURL: false)!
                components.queryItems = [URLQueryItem(name: "after", value: String(logCursor))]
                let (logData, _) = try await session.data(from: components.url!)
                let chunk = try decoder.decode(ModuleTestLogChunk.self, from: logData)
                logs.append(contentsOf: chunk.lines)
                if logs.count > 800 { logs.removeFirst(logs.count - 800) }
                logCursor = chunk.cursor

                if finished {
                    switch received.status {
                    case "passed": message = "测试通过 · OFFLINE，不代表实机验收"
                    case "cancelled": message = "测试已停止；NUC 已清理子进程"
                    case "timed_out": message = received.reason ?? "测试超时；NUC 已清理子进程"
                    default: message = received.reason ?? "测试失败 · 请查看日志"
                    }
                }
            } catch {
                message = "测试状态暂不可读 · \(error.localizedDescription)"
            }
            if !finished { try? await Task.sleep(for: .milliseconds(450)) }
        }
        await refreshCatalog()
    }
}

private struct ModuleTestError: Decodable {
    let error: String
}

struct ModuleTestsView: View {
    @ObservedObject var model: ModuleTestsModel

    private struct TestGroup: Identifiable {
        let name: String
        let tests: [ModuleTestDefinition]
        var id: String { name }
    }

    private let lockedLiveTests: [(String, String, String)] = [
        ("相机实机", "camera", "相机驱动、序列号确认与图像验收流程尚未接入"),
        ("MID-360 实机", "sensor", "点云录制、时间同步与外参验收条件未确认"),
        ("底盘实机", "truck.box", "需独立的场地隔离、急停与实机审批证据"),
        ("机械臂实机", "move.3d", "需实机标定、工作区隔离与双人验收"),
    ]

    private var testGroups: [TestGroup] {
        guard let tests = model.catalog?.tests else { return [] }
        let grouped = Dictionary(grouping: tests, by: \.category)
        let preferredOrder = ["基础环境", "视觉离线测试", "底盘离线协议", "Nav2 / 任务离线测试", "机械臂离线模型", "只读设备巡检"]
        return grouped.keys.sorted {
            (preferredOrder.firstIndex(of: $0) ?? preferredOrder.count) <
                (preferredOrder.firstIndex(of: $1) ?? preferredOrder.count)
        }.map { name in
            TestGroup(name: name, tests: grouped[name, default: []].sorted { $0.title < $1.title })
        }
    }

    var body: some View {
        HStack(alignment: .top, spacing: 18) {
            testList
                .frame(minWidth: 280, idealWidth: 320, maxWidth: 350)
            detailPanel
                .frame(maxWidth: .infinity, alignment: .topLeading)
        }
        .task { model.start() }
        .onDisappear { model.stop() }
    }

    private var testList: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 14) {
            HStack {
                Text("可运行测试").font(.system(size: 15, weight: .semibold))
                Spacer()
                statusChip(model.runnerOnline ? "NUC 在线" : "NUC 离线", color: model.runnerOnline ? Palette.accent : Palette.warning)
            }
            HStack(spacing: 6) {
                modeBadge("SIMULATION", color: Palette.blue)
                modeBadge("OFFLINE", color: Palette.accent)
                modeBadge("LIVE", color: Palette.danger)
                Spacer()
            }
            if model.catalog != nil {
                ForEach(testGroups) { group in
                    Text(group.name.uppercased())
                        .font(.system(size: 9, weight: .bold, design: .monospaced))
                        .foregroundStyle(Palette.muted)
                        .padding(.top, 4)
                    ForEach(group.tests) { test in
                        Button { model.select(test) } label: {
                            testRow(test)
                        }
                        .buttonStyle(.plain)
                    }
                }
            } else {
                Text("正在连接受限测试服务…")
                    .font(.system(size: 12)).foregroundStyle(Palette.muted)
                    .frame(maxWidth: .infinity, alignment: .leading).padding(16)
                    .background(Palette.panel, in: RoundedRectangle(cornerRadius: 12))
            }

            Text("后续实机运动验收").font(.system(size: 12, weight: .semibold)).foregroundStyle(Palette.muted).padding(.top, 5)
            ForEach(Array(lockedLiveTests.enumerated()), id: \.offset) { _index, item in
                HStack(alignment: .top, spacing: 10) {
                    Image(systemName: item.1).foregroundStyle(Palette.muted).frame(width: 18)
                    VStack(alignment: .leading, spacing: 5) {
                        HStack {
                            Text(item.0).font(.system(size: 12, weight: .medium))
                            Spacer()
                            modeBadge("LIVE", color: Palette.danger)
                        }
                        Text("已锁定 · \(item.2)").font(.system(size: 10)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                    }
                }
                .padding(12).background(Palette.panel.opacity(0.6), in: RoundedRectangle(cornerRadius: 10))
            }
            }
            .padding(16)
            .background(Palette.panel.opacity(0.55), in: RoundedRectangle(cornerRadius: 15))
        }
        .frame(maxHeight: 780)
    }

    private func testRow(_ test: ModuleTestDefinition) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(alignment: .center) {
                Text(test.title).font(.system(size: 13, weight: .semibold))
                Spacer(minLength: 8)
                modeBadge(test.mode, color: modeColor(test.mode))
                statusChip(test.state == "ready" ? "可运行" : "锁定", color: test.state == "ready" ? Palette.accent : Palette.warning)
            }
            Text(test.description).font(.system(size: 10)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
        }
        .padding(13)
        .background(model.selectedTestID == test.id ? Palette.panelRaised : Palette.panel, in: RoundedRectangle(cornerRadius: 11))
        .overlay(RoundedRectangle(cornerRadius: 11).stroke(model.selectedTestID == test.id ? Palette.accent.opacity(0.45) : Palette.line, lineWidth: 1))
    }

    private var detailPanel: some View {
        VStack(alignment: .leading, spacing: 16) {
            HStack(alignment: .center) {
                VStack(alignment: .leading, spacing: 5) {
                    Text(model.selectedTest?.title ?? "选择一个测试项目").font(.system(size: 19, weight: .semibold, design: .rounded))
                    Text(model.selectedTest?.description ?? "运行前会在 NUC 检查前置条件；只有固定白名单测试 ID 可以执行。")
                        .font(.system(size: 11)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
                }
                Spacer()
                if let selected = model.selectedTest { modeBadge(selected.mode, color: modeColor(selected.mode)) }
            }

            if let test = model.selectedTest {
                infoCard(title: "前置条件", values: test.prerequisites, reason: test.reason)
                infoCard(title: "输入来源", values: [test.inputSource], reason: nil)
            }

            HStack(spacing: 9) {
                Button {
                    Task { await model.checkEnvironment() }
                } label: {
                    Label(model.checkingEnvironment ? "检查中…" : "检查环境", systemImage: "checkmark.shield")
                }
                .disabled(model.checkingEnvironment || !model.runnerOnline || model.selectedTestID == nil)
                .buttonStyle(.bordered)

                Button(action: model.runSelected) {
                    Label(model.submitting ? "启动中…" : "一键执行", systemImage: "play.fill")
                }
                .disabled(model.submitting || model.isRunning || model.anotherRunIsActive || !model.runnerOnline || model.selectedTest == nil || model.selectedTest?.state != "ready")
                .buttonStyle(.borderedProminent)

                if model.isRunning {
                    Button(role: .destructive, action: model.cancelRun) {
                        Label("停止测试", systemImage: "stop.fill")
                    }
                    .buttonStyle(.bordered)
                }

                Button(action: model.saveReport) {
                    Label(model.savingReport ? "保存中…" : "保存报告", systemImage: "square.and.arrow.down")
                }
                .disabled(!model.canSaveReport || model.savingReport)
                .buttonStyle(.bordered)
                Spacer()
            }

            HStack(spacing: 8) {
                Circle().fill(messageColor).frame(width: 7, height: 7)
                Text(model.message).font(.system(size: 11, weight: .medium)).foregroundStyle(Palette.muted).lineLimit(2)
                Spacer()
                if let run = model.run {
                    Text(run.status.uppercased()).font(.system(size: 10, weight: .bold, design: .monospaced)).foregroundStyle(statusColor(run.status))
                }
            }
            .padding(11).background(Palette.panel, in: RoundedRectangle(cornerRadius: 9))

            if let run = model.run {
                HStack(spacing: 16) {
                    reportMetric("结果", value: resultLabel(run.result, mode: model.selectedTest?.mode ?? "OFFLINE"))
                    reportMetric("耗时", value: run.durationMs.map { String(format: "%.2f 秒", Double($0) / 1000) } ?? "执行中")
                    reportMetric("代码 SHA", value: run.codeSha.map { String($0.prefix(12)) } ?? "记录中")
                    reportMetric("工作树", value: run.codeDirty == true ? "有未提交修改" : (run.codeDirty == false ? "干净" : "未知"))
                }
            }

            VStack(alignment: .leading, spacing: 8) {
                HStack {
                    Text("实时日志").font(.system(size: 13, weight: .semibold))
                    Spacer()
                    Text("日志与报告自动保存在 NUC").font(.system(size: 10)).foregroundStyle(Palette.muted)
                }
                ScrollViewReader { proxy in
                    ScrollView {
                        LazyVStack(alignment: .leading, spacing: 3) {
                            if model.logs.isEmpty {
                                Text("选择测试并点击「一键执行」后，日志会显示在这里。")
                                    .foregroundStyle(Palette.muted)
                            }
                            ForEach(Array(model.logs.enumerated()), id: \.offset) { index, line in
                                Text(line.isEmpty ? " " : line)
                                    .foregroundStyle(line.hasPrefix("FAIL") || line.contains("Traceback") ? Palette.danger : Palette.muted)
                                    .id(index)
                            }
                        }
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .padding(12)
                    }
                    .onChange(of: model.logs.count) { _, count in
                        if count > 0 { proxy.scrollTo(count - 1, anchor: .bottom) }
                    }
                }
                .frame(height: 235)
                .background(Color.black.opacity(0.22), in: RoundedRectangle(cornerRadius: 10))
            }

        Text("OFFLINE 只表示离线软件检查通过；SIMULATION 使用合成数据。LIVE 连接巡检只读现有 ROS 状态，不启动设备；实机运动测试仍锁定。")
                .font(.system(size: 10)).foregroundStyle(Palette.warning).fixedSize(horizontal: false, vertical: true)
        }
        .padding(18)
        .background(Palette.panel.opacity(0.55), in: RoundedRectangle(cornerRadius: 15))
    }

    private func infoCard(title: String, values: [String], reason: String?) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            Text(title).font(.system(size: 11, weight: .semibold)).foregroundStyle(Palette.muted)
            ForEach(values, id: \.self) { value in
                Label(value, systemImage: "checkmark.circle")
                    .font(.system(size: 11)).foregroundStyle(Palette.muted).fixedSize(horizontal: false, vertical: true)
            }
            if let reason {
                Label(reason, systemImage: "lock.fill")
                    .font(.system(size: 11)).foregroundStyle(Palette.warning).fixedSize(horizontal: false, vertical: true)
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(12).background(Palette.panel, in: RoundedRectangle(cornerRadius: 10))
    }

    private func reportMetric(_ title: String, value: String) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title).font(.system(size: 9)).foregroundStyle(Palette.muted)
            Text(value).font(.system(size: 10, weight: .semibold, design: .monospaced)).lineLimit(1)
        }
    }

    private func statusChip(_ title: String, color: Color) -> some View {
        Text(title).font(.system(size: 9, weight: .semibold)).foregroundStyle(color)
            .padding(.horizontal, 7).padding(.vertical, 4)
            .background(color.opacity(0.11), in: Capsule())
    }

    private func modeBadge(_ title: String, color: Color) -> some View {
        Text(title).font(.system(size: 9, weight: .bold, design: .monospaced)).foregroundStyle(color)
            .padding(.horizontal, 7).padding(.vertical, 4)
            .background(color.opacity(0.12), in: Capsule())
    }

    private func modeColor(_ mode: String) -> Color {
        switch mode {
        case "SIMULATION": return Palette.blue
        case "OFFLINE": return Palette.accent
        default: return Palette.danger
        }
    }

    private func statusColor(_ status: String) -> Color {
        switch status {
        case "passed": return Palette.accent
        case "running", "cancelling": return Palette.blue
        case "cancelled": return Palette.warning
        default: return Palette.danger
        }
    }

    private var messageColor: Color {
        if model.message.contains("通过") || model.message.contains("满足") { return Palette.accent }
        if model.message.contains("失败") || model.message.contains("不可用") { return Palette.warning }
        return Palette.blue
    }

    private func resultLabel(_ result: String, mode: String) -> String {
        switch result {
        case "passed": return mode == "LIVE" ? "只读检查完成" : "通过 · \(mode)"
        case "failed": return "失败"
        case "cancelled": return "已停止"
        case "timed_out": return "超时"
        default: return "执行中"
        }
    }
}
