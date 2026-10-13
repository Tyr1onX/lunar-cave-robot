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
        case overview = "总览"
        case diagnostics = "系统诊断"
        case history = "历史记录"

        var id: String { rawValue }

        var symbol: String {
            switch self {
            case .overview: return "rectangle.grid.2x2.fill"
            case .diagnostics: return "waveform.path.ecg"
            case .history: return "clock.arrow.circlepath"
            }
        }
    }

    @Published var section: Section = .overview
    @Published private(set) var snapshot: ConsoleSnapshot?
    @Published private(set) var logs: LogReading?
    @Published private(set) var tunnelRunning = false
    @Published private(set) var agentOnline = false
    @Published private(set) var connectionMessage = "等待 SSH 隧道"
    @Published private(set) var agentMessage = "等待 NUC Agent"
    @Published private(set) var githubMainSHA: String?
    @Published private(set) var githubMessage = "尚未读取 GitHub main"
    @Published private(set) var lastSuccessfulRead: Date?

    private var tunnel: Process?
    private var tunnelErrorPipe: Pipe?
    private var refreshTask: Task<Void, Never>?
    private(set) var agentSession = 0
    private var isRefreshing = false
    private var isReadingGitHub = false
    private var isReadingLogs = false
    private var lastLogRead = Date.distantPast

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
        refreshGitHubMain()
    }

    func stop() {
        refreshTask?.cancel()
        refreshTask = nil
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
            Task { @MainActor in
                guard let self, self.tunnel === terminated else { return }
                self.tunnelRunning = false
                self.connectionMessage = diagnostic ?? "SSH 隧道已退出（状态 \(terminated.terminationStatus)）"
                self.invalidateAgentData(message: "等待 NUC Agent")
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
            lastSuccessfulRead = Date()
            agentMessage = "真实快照已更新 · \(received.collectedAt)"
            if Date().timeIntervalSince(lastLogRead) > 6 { refreshLogs() }
        } catch {
            guard requestSession == agentSession else { return }
            invalidateAgentData(message: "Agent 暂不可用 · \(error.localizedDescription)")
        }
    }

    func markAgentAvailable(message: String) {
        agentOnline = true
        agentMessage = message
    }

    func invalidateAgentData(message: String) {
        agentSession += 1
        agentOnline = false
        agentMessage = message
        snapshot = nil
        logs = nil
    }

    func applyLogReading(_ reading: LogReading, requestSession: Int) {
        guard agentOnline, requestSession == agentSession else { return }
        logs = reading
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
