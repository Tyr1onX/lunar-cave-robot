import Foundation
import SwiftUI

private final class SSHDiagnosticBuffer: @unchecked Sendable {
    private let lock = NSLock()
    private var contents = Data()

    func append(_ data: Data) {
        guard !data.isEmpty else { return }
        lock.lock()
        contents.append(data)
        if contents.count > 4096 {
            contents = Data(contents.suffix(4096))
        }
        lock.unlock()
    }

    func message() -> String? {
        lock.lock()
        let data = contents
        lock.unlock()
        guard let text = String(data: data, encoding: .utf8) else { return nil }
        let message = text.trimmingCharacters(in: .whitespacesAndNewlines)
        return message.isEmpty ? nil : String(message.suffix(180))
    }
}

@MainActor
final class ConsoleModel: ObservableObject {
    enum Section: String, CaseIterable, Identifiable {
        case cockpit = "测试驾驶舱"
        case moduleTests = "模块测试"
        case simulation = "模拟驾驶"
        case armPreview = "机械臂预览"
        case diagnostics = "系统诊断"
        case history = "历史记录"

        var id: String { rawValue }

        var symbol: String {
            switch self {
            case .cockpit: return "viewfinder"
            case .moduleTests: return "checklist"
            case .simulation: return "gamecontroller.fill"
            case .armPreview: return "cube.transparent"
            case .diagnostics: return "waveform.path.ecg"
            case .history: return "clock.arrow.circlepath"
            }
        }
    }

    @Published var section: Section = .cockpit
    @Published private(set) var snapshot: ConsoleSnapshot?
    @Published private(set) var logs: LogReading?
    @Published private(set) var vision: VisionReading?
    @Published private(set) var visionFrame: Data?
    @Published private(set) var visionMessage: String?
    @Published private(set) var visionFrameMessage: String?
    @Published private(set) var visionFrameRoundTripMs: Double?
    @Published private(set) var macFrameUpdateHz: Double?
    @Published private(set) var selectedVisionStream: VisionStreamID = .rgb
    @Published private(set) var tunnelRunning = false
    @Published private(set) var agentOnline = false
    @Published private(set) var connectionMessage = "等待 SSH 隧道"
    @Published private(set) var agentMessage = "等待 NUC Agent"
    @Published private(set) var githubMainSHA: String?
    @Published private(set) var githubMessage = "尚未读取 GitHub main"
    @Published private(set) var lastSuccessfulRead: Date?
    @Published private(set) var healthEvents: [HealthEvent] = []

    private(set) var hasEverConnectedToAgent = false
    private(set) var lastKnownDomainID: Int?

    private var tunnel: Process?
    private var tunnelErrorPipe: Pipe?
    private var refreshTask: Task<Void, Never>?
    private var visionRefreshTask: Task<Void, Never>?
    private var visionFrameTask: Task<Void, Never>?
    private(set) var agentSession = 0
    private var isRefreshing = false
    private var isReadingGitHub = false
    private var isReadingLogs = false
    private var isReadingVision = false
    private var isReadingVisionFrame = false
    private var lastLogRead = Date.distantPast
    private var lastFrameUpdateUptime: TimeInterval?
    private var healthEventTracker = HealthEventTracker()

    init() {
        refreshHealthEvents()
    }

    private let apiRoot = URL(string: "http://127.0.0.1:18765")!
    private let session: URLSession = {
        let configuration = URLSessionConfiguration.ephemeral
        configuration.timeoutIntervalForRequest = 3
        configuration.timeoutIntervalForResource = 4
        configuration.waitsForConnectivity = false
        return URLSession(configuration: configuration)
    }()

    func start() {
        guard refreshTask == nil else { return }
        ensureTunnel()
        refreshTask = Task { [weak self] in
            while !Task.isCancelled {
                await self?.refresh()
                try? await Task.sleep(for: .seconds(3))
            }
        }
        visionRefreshTask = Task { [weak self] in
            while !Task.isCancelled {
                await self?.refreshVision()
                try? await Task.sleep(for: .seconds(1))
            }
        }
        visionFrameTask = Task { [weak self] in
            while !Task.isCancelled {
                await self?.refreshVisionFrame()
                try? await Task.sleep(for: .milliseconds(500))
            }
        }
        refreshGitHubMain()
    }

    func stop() {
        refreshTask?.cancel()
        refreshTask = nil
        visionRefreshTask?.cancel()
        visionRefreshTask = nil
        visionFrameTask?.cancel()
        visionFrameTask = nil
        invalidateAgentData(message: "NUC Agent 已停止")
        tunnel?.terminationHandler = nil
        if tunnel?.isRunning == true { tunnel?.terminate() }
        tunnel = nil
        tunnelErrorPipe?.fileHandleForReading.readabilityHandler = nil
        tunnelErrorPipe = nil
        tunnelRunning = false
    }

    func reconnect() {
        invalidateAgentData(message: "正在重新连接 NUC Agent")
        tunnel?.terminationHandler = nil
        if tunnel?.isRunning == true { tunnel?.terminate() }
        tunnel = nil
        tunnelRunning = false
        connectionMessage = "正在重新建立固定 SSH 转发"
        ensureTunnel()
        refreshGitHubMain()
        Task { await refresh() }
    }

    private func ensureTunnel() {
        if let tunnel, tunnel.isRunning {
            tunnelRunning = true
            return
        }
        let process = Process()
        process.executableURL = URL(fileURLWithPath: "/usr/bin/ssh")
        process.arguments = [
            "-N", "-T",
            "-L", "127.0.0.1:18765:127.0.0.1:8765",
            "-L", "127.0.0.1:18769:127.0.0.1:8769",
            "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=5",
            "-o", "ExitOnForwardFailure=yes",
            "-o", "ServerAliveInterval=5",
            "-o", "ServerAliveCountMax=2",
            "-o", "StrictHostKeyChecking=yes",
            "nuc"
        ]
        let errors = Pipe()
        let diagnosticBuffer = SSHDiagnosticBuffer()
        process.standardOutput = FileHandle.nullDevice
        process.standardError = errors
        errors.fileHandleForReading.readabilityHandler = { handle in
            let data = handle.availableData
            diagnosticBuffer.append(data)
            if data.isEmpty { handle.readabilityHandler = nil }
        }
        process.terminationHandler = { [weak self] terminated in
            errors.fileHandleForReading.readabilityHandler = nil
            diagnosticBuffer.append(errors.fileHandleForReading.readDataToEndOfFile())
            let diagnostic = diagnosticBuffer.message()
            Task { @MainActor [weak self] in
                self?.handleTunnelTermination(terminated, diagnostic: diagnostic)
            }
        }
        do {
            try process.run()
            tunnel = process
            tunnelErrorPipe = errors
            tunnelRunning = true
            if connectionMessage == "等待 SSH 隧道" || connectionMessage.hasPrefix("无法启动 SSH：") {
                connectionMessage = "SSH 正在建立固定转发 · nuc"
            }
        } catch {
            tunnelRunning = false
            connectionMessage = "无法启动 SSH：\(error.localizedDescription)"
        }
    }

    private func handleTunnelTermination(_ terminated: Process, diagnostic: String?) {
        guard tunnel === terminated else { return }
        tunnelRunning = false
        connectionMessage = diagnostic ?? "SSH 隧道已退出（状态 \(terminated.terminationStatus)）"
        invalidateAgentData(message: "NUC Agent 连接中断 · \(connectionMessage)")
    }

    private func refreshHealthEvents(observedAt: Date = Date()) {
        let candidates = HealthEvaluator.evaluate(
            agentOnline: agentOnline,
            hasEverConnectedToAgent: hasEverConnectedToAgent,
            agentMessage: agentMessage,
            lastSuccessfulRead: lastSuccessfulRead,
            lastKnownDomainID: lastKnownDomainID,
            snapshot: snapshot,
            vision: vision,
            evaluationTime: observedAt
        )
        healthEvents = healthEventTracker.update(candidates)
    }

    private func refresh() async {
        guard !isRefreshing else { return }
        isRefreshing = true
        defer { isRefreshing = false }
        ensureTunnel()
        let requestSession = agentSession

        do {
            let (healthData, _) = try await session.data(from: apiRoot.appendingPathComponent("health"))
            guard requestSession == agentSession else { return }
            let healthDecoder = JSONDecoder()
            healthDecoder.keyDecodingStrategy = .convertFromSnakeCase
            let health = try healthDecoder.decode(HealthReading.self, from: healthData)
            guard health.ok, health.schemaVersion == 1 else {
                throw URLError(.cannotParseResponse)
            }
            markAgentAvailable(message: "Agent 在线 · \(health.collectedAt)")
            connectionMessage = "SSH 转发可用 · nuc"

            let (data, _) = try await session.data(from: apiRoot.appendingPathComponent("v1/snapshot"))
            guard requestSession == agentSession else { return }
            let decoder = JSONDecoder()
            decoder.keyDecodingStrategy = .convertFromSnakeCase
            let received = try decoder.decode(ConsoleSnapshot.self, from: data)
            guard received.schemaVersion == 1, received.source == "nuc-read-only-agent" else {
                throw URLError(.cannotParseResponse)
            }
            snapshot = received
            lastKnownDomainID = received.ros.domainId
            lastSuccessfulRead = Date()
            agentMessage = "真实快照已更新 · \(received.collectedAt)"
            refreshHealthEvents()
            if Date().timeIntervalSince(lastLogRead) > 6 { refreshLogs() }
        } catch {
            guard requestSession == agentSession else { return }
            invalidateAgentData(message: "Agent 暂不可用 · \(error.localizedDescription)")
        }
    }

    func markAgentAvailable(message: String) {
        agentOnline = true
        hasEverConnectedToAgent = true
        agentMessage = message
        refreshHealthEvents()
    }

    func selectVisionStream(_ stream: VisionStreamID) {
        guard selectedVisionStream != stream else { return }
        selectedVisionStream = stream
        visionFrame = nil
        visionFrameMessage = nil
        visionFrameRoundTripMs = nil
        macFrameUpdateHz = nil
        lastFrameUpdateUptime = nil
    }

    func invalidateAgentData(message: String) {
        agentSession += 1
        agentOnline = false
        agentMessage = message
        snapshot = nil
        logs = nil
        vision = nil
        visionFrame = nil
        visionMessage = nil
        visionFrameMessage = nil
        visionFrameRoundTripMs = nil
        macFrameUpdateHz = nil
        lastFrameUpdateUptime = nil
        refreshHealthEvents()
    }

    func applyLogReading(_ reading: LogReading, requestSession: Int) {
        guard agentOnline, requestSession == agentSession else { return }
        logs = reading
    }

    func applyVisionReading(_ reading: VisionReading, requestSession: Int) {
        guard agentOnline, requestSession == agentSession else { return }
        vision = reading
        visionMessage = nil
        refreshHealthEvents()
        guard reading.stream(selectedVisionStream)?.status == "streaming" else {
            visionFrame = nil
            visionFrameRoundTripMs = nil
            macFrameUpdateHz = nil
            lastFrameUpdateUptime = nil
            visionFrameMessage = reading.stream(selectedVisionStream)?.reason
            return
        }
    }

    func applyVisionFrame(_ data: Data, streamID: VisionStreamID, requestSession: Int, roundTripMs: Double) {
        guard agentOnline, requestSession == agentSession, selectedVisionStream == streamID,
              vision?.stream(streamID)?.status == "streaming" else { return }
        let now = ProcessInfo.processInfo.systemUptime
        if let lastFrameUpdateUptime, now > lastFrameUpdateUptime {
            macFrameUpdateHz = 1 / (now - lastFrameUpdateUptime)
        }
        lastFrameUpdateUptime = now
        visionFrame = data
        visionFrameMessage = nil
        visionFrameRoundTripMs = roundTripMs
    }

    func applyVisionError(_ message: String, requestSession: Int) {
        guard agentOnline, requestSession == agentSession else { return }
        vision = nil
        visionFrame = nil
        visionMessage = message
        visionFrameMessage = message
        visionFrameRoundTripMs = nil
        macFrameUpdateHz = nil
        lastFrameUpdateUptime = nil
        refreshHealthEvents()
    }

    func applyVisionFrameError(_ message: String, streamID: VisionStreamID, requestSession: Int) {
        guard agentOnline, requestSession == agentSession, selectedVisionStream == streamID else { return }
        visionFrame = nil
        visionFrameMessage = message
        visionFrameRoundTripMs = nil
        macFrameUpdateHz = nil
        lastFrameUpdateUptime = nil
    }

    private func refreshLogs() {
        guard !isReadingLogs else { return }
        isReadingLogs = true
        lastLogRead = Date()
        let requestSession = agentSession
        Task {
            defer { isReadingLogs = false }
            do {
                let (data, _) = try await session.data(from: apiRoot.appendingPathComponent("v1/logs"))
                let decoder = JSONDecoder()
                decoder.keyDecodingStrategy = .convertFromSnakeCase
                let reading = try decoder.decode(LogReading.self, from: data)
                applyLogReading(reading, requestSession: requestSession)
            } catch {
                let reading = LogReading(status: "unknown", reason: error.localizedDescription, entries: [], source: nil)
                applyLogReading(reading, requestSession: requestSession)
            }
        }
    }

    private func refreshVision() async {
        guard agentOnline, !isReadingVision else { return }
        isReadingVision = true
        defer { isReadingVision = false }
        let requestSession = agentSession
        do {
            let (data, response) = try await session.data(from: apiRoot.appendingPathComponent("v1/vision"))
            guard requestSession == agentSession, agentOnline else { return }
            guard let http = response as? HTTPURLResponse else { throw URLError(.cannotParseResponse) }
            guard http.statusCode == 200 else {
                if http.statusCode == 404 {
                    applyVisionError("当前 NUC Agent 版本没有视觉数据接口", requestSession: requestSession)
                    return
                }
                throw URLError(.badServerResponse)
            }
            let decoder = JSONDecoder()
            decoder.keyDecodingStrategy = .convertFromSnakeCase
            applyVisionReading(try decoder.decode(VisionReading.self, from: data), requestSession: requestSession)
        } catch {
            applyVisionError("视觉状态暂不可读 · \(error.localizedDescription)", requestSession: requestSession)
        }
    }

    private func refreshVisionFrame() async {
        guard agentOnline, !isReadingVisionFrame else { return }
        let stream = selectedVisionStream
        guard let source = vision?.stream(stream) else { return }
        guard source.status == "streaming" else {
            visionFrame = nil
            visionFrameRoundTripMs = nil
            macFrameUpdateHz = nil
            lastFrameUpdateUptime = nil
            visionFrameMessage = source.reason
            return
        }
        isReadingVisionFrame = true
        defer { isReadingVisionFrame = false }
        let requestSession = agentSession
        let startedAt = ProcessInfo.processInfo.systemUptime
        let url = URL(string: "/v1/vision/\(stream.rawValue)/frame.png", relativeTo: apiRoot)!.absoluteURL
        do {
            let (data, response) = try await session.data(from: url)
            guard requestSession == agentSession, agentOnline, selectedVisionStream == stream else { return }
            guard let http = response as? HTTPURLResponse else { throw URLError(.cannotParseResponse) }
            guard http.statusCode == 200, http.value(forHTTPHeaderField: "Content-Type")?.hasPrefix("image/png") == true else {
                let reason = (try? JSONDecoder().decode(VisionFrameError.self, from: data).reason) ?? "图像帧暂不可用"
                applyVisionFrameError(reason, streamID: stream, requestSession: requestSession)
                return
            }
            applyVisionFrame(data, streamID: stream, requestSession: requestSession,
                             roundTripMs: (ProcessInfo.processInfo.systemUptime - startedAt) * 1000)
        } catch {
            applyVisionFrameError("实时图像请求失败 · \(error.localizedDescription)", streamID: stream, requestSession: requestSession)
        }
    }

    private func refreshGitHubMain() {
        guard !isReadingGitHub else { return }
        isReadingGitHub = true
        githubMessage = "正在读取 GitHub main"
        let resultTask = Task.detached(priority: .utility) {
            Self.readGitHubMainSHA()
        }
        Task {
            let result = await resultTask.value
            isReadingGitHub = false
            githubMainSHA = result.sha
            githubMessage = result.message
        }
    }

    nonisolated private static func readGitHubMainSHA() -> (sha: String?, message: String) {
        let process = Process()
        process.executableURL = URL(fileURLWithPath: "/usr/bin/git")
        process.arguments = ["ls-remote", "git@github.com:Tyr1onX/lunar-cave-robot.git", "refs/heads/main"]
        var environment = ProcessInfo.processInfo.environment
        environment["GIT_TERMINAL_PROMPT"] = "0"
        environment["GIT_SSH_COMMAND"] = "/usr/bin/ssh -oBatchMode=yes -oConnectTimeout=5 -oServerAliveInterval=5 -oServerAliveCountMax=1"
        process.environment = environment
        let output = Pipe()
        process.standardOutput = output
        process.standardError = FileHandle.nullDevice
        do {
            try process.run()
            let data = output.fileHandleForReading.readDataToEndOfFile()
            process.waitUntilExit()
            guard process.terminationStatus == 0,
                  let line = String(data: data, encoding: .utf8)?.split(whereSeparator: \.isNewline).first,
                  let sha = line.split(separator: "\t").first,
                  sha.count == 40 else {
                return (nil, "GitHub main 当前无法读取")
            }
            return (String(sha), "GitHub main · 实时读取")
        } catch {
            return (nil, "GitHub main 当前无法读取")
        }
    }
}

private struct VisionFrameError: Decodable {
    let reason: String
}
