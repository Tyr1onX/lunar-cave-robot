import Foundation
import SwiftUI

struct SimPose: Decodable {
    let x: Double
    let y: Double
    let yaw: Double
    let linear: Double
    let angular: Double
}

struct SimState: Decodable {
    let schemaVersion: Int
    let source: String
    let mode: String
    let domainId: Int
    let commandTopic: String
    let ready: Bool
    let odom: SimPose?
    let trail: [[Double]]
    let hasMap: Bool
}

struct SimMap: Decodable {
    let width: Int
    let height: Int
    let resolution: Double
    let originX: Double
    let originY: Double
    let cells: [Int]

    var valid: Bool {
        width == 240 && height == 160 && cells.count == width * height && abs(resolution - 0.05) < 0.00001
    }
}

enum DriveKey: Int, CaseIterable {
    case w = 13, a = 0, s = 1, d = 2

    static func axes(_ keys: Set<DriveKey>) -> (forward: Double, turn: Double) {
        let forward = (keys.contains(.w) ? 1.0 : 0.0) - (keys.contains(.s) ? 1.0 : 0.0)
        let turn = (keys.contains(.a) ? 1.0 : 0.0) - (keys.contains(.d) ? 1.0 : 0.0)
        return (forward, turn)
    }
}

private struct SimClaim: Decodable { let token: String }
private struct SimAck: Decodable { let applied: Bool }

@MainActor
final class SimulationModel: ObservableObject {
    @Published private(set) var ready = false
    @Published private(set) var focused = false
    @Published private(set) var state: SimState?
    @Published private(set) var map: SimMap?
    @Published private(set) var message = "等待独立模拟服务（不自动启动）"
    @Published private(set) var pressed: Set<DriveKey> = []

    private let root = URL(string: "http://127.0.0.1:18766")!
    private let session: URLSession = {
        let configuration = URLSessionConfiguration.ephemeral
        configuration.timeoutIntervalForRequest = 0.65
        configuration.timeoutIntervalForResource = 0.8
        configuration.waitsForConnectivity = false
        return URLSession(configuration: configuration)
    }()
    private var tunnel: Process?
    private var pollTask: Task<Void, Never>?
    private var driveTask: Task<Void, Never>?
    private var generation = 0
    private var stopping = false
    private var token: String?
    private var sequence = 0
    private var lastCommandWasMoving = false

    func start() {
        guard pollTask == nil else { return }
        stopping = false
        generation += 1
        ensureTunnel()
        pollTask = Task { [weak self] in
            while let self, !Task.isCancelled {
                await self.poll()
                try? await Task.sleep(for: .milliseconds(150))
            }
        }
        driveTask = Task { [weak self] in
            while let self, !Task.isCancelled {
                await self.driveTick()
                try? await Task.sleep(for: .milliseconds(90))
            }
        }
    }

    func stop() {
        guard !stopping else { return }
        stopping = true
        generation += 1
        focused = false
        pressed = []
        lastCommandWasMoving = false
        let previousToken = token
        token = nil
        sequence = 0
        pollTask?.cancel()
        driveTask?.cancel()
        pollTask = nil
        driveTask = nil
        ready = false
        state = nil
        map = nil
        let stoppedGeneration = generation
        if let previousToken {
            // Release before closing the tunnel. A rapid return to this view
            // reuses it; an old stop completion cannot close a new session.
            Task {
                _ = try? await post("release", body: ["token": previousToken])
                guard generation == stoppedGeneration, pollTask == nil else { return }
                if tunnel?.isRunning == true { tunnel?.terminate() }
                tunnel = nil
            }
        } else {
            if tunnel?.isRunning == true { tunnel?.terminate() }
            tunnel = nil
        }
    }

    func setFocus(_ isFocused: Bool) {
        guard focused != isFocused else { return }
        focused = isFocused
        if !isFocused { releaseFocus() }
    }

    func setKeys(_ keys: Set<DriveKey>) {
        guard focused && ready else {
            pressed = []
            return
        }
        let previouslyMoving = DriveKey.axes(pressed)
        pressed = keys
        let current = DriveKey.axes(keys)
        if (previouslyMoving.forward != 0 || previouslyMoving.turn != 0) &&
           current.forward == 0 && current.turn == 0 {
            // A zero command is queued immediately; stop sending nonzero commands.
            // If an earlier request is delayed, the server rejects old sequence IDs.
            Task { await sendZero() }
        }
    }

    private func releaseFocus() {
        focused = false
        pressed = []
        lastCommandWasMoving = false
        let previousToken = token
        token = nil
        sequence = 0
        if let previousToken {
            Task { _ = try? await post("release", body: ["token": previousToken]) }
        }
    }

    private func ensureTunnel() {
        if tunnel?.isRunning == true { return }
        let process = Process()
        process.executableURL = URL(fileURLWithPath: "/usr/bin/ssh")
        process.arguments = ["-N", "-T", "-L", "127.0.0.1:18766:127.0.0.1:8766",
                             "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
                             "-o", "ExitOnForwardFailure=yes",
                             "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2",
                             "-o", "StrictHostKeyChecking=yes", "nuc"]
        process.standardOutput = FileHandle.nullDevice
        process.standardError = FileHandle.nullDevice
        do { try process.run(); tunnel = process }
        catch { message = "无法建立模拟隧道：\(error.localizedDescription)" }
    }

    private func invalidate(_ explanation: String) {
        guard ready || state != nil || token != nil || map != nil else {
            message = explanation
            return
        }
        releaseFocus()
        ready = false
        state = nil
        map = nil
        message = explanation
    }

    private func request(_ name: String) -> URL {
        root.appendingPathComponent("v1/simulation/\(name)")
    }

    private func fetch<T: Decodable>(_ name: String, as: T.Type) async throws -> T {
        let (data, response) = try await session.data(from: request(name))
        guard (response as? HTTPURLResponse)?.statusCode == 200 else { throw URLError(.badServerResponse) }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(T.self, from: data)
    }

    private func post(_ name: String, body: [String: Any]) async throws -> Data {
        var req = URLRequest(url: request(name))
        req.httpMethod = "POST"
        req.setValue("application/json", forHTTPHeaderField: "Content-Type")
        req.httpBody = try JSONSerialization.data(withJSONObject: body)
        let (data, response) = try await session.data(for: req)
        guard (response as? HTTPURLResponse)?.statusCode == 200 else { throw URLError(.badServerResponse) }
        return data
    }

    private func poll() async {
        let currentGeneration = generation
        ensureTunnel()
        do {
            let reading = try await fetch("state", as: SimState.self)
            guard currentGeneration == generation else { return }
            guard reading.schemaVersion == 1,
                  reading.source == "isolated-p6-cave-simulator",
                  reading.mode == "SIMULATION",
                  reading.domainId == 217,
                  reading.commandTopic == "/console_sim/cmd_vel",
                  reading.ready, reading.odom != nil else {
                invalidate("隔离配置或模拟里程计尚未就绪，禁止驾驶")
                return
            }
            if map == nil {
                let loaded = try await fetch("map", as: SimMap.self)
                guard currentGeneration == generation else { return }
                guard loaded.valid else {
                    invalidate("收到非预期的模拟地图，禁止驾驶")
                    return
                }
                map = loaded
            }
            ready = true
            state = reading
            message = "独立 ROS Domain 217 · P6 数学模拟器"
        } catch {
            guard currentGeneration == generation else { return }
            invalidate("模拟服务失联或不可用 · \(error.localizedDescription)")
        }
    }

    private func driveTick() async {
        guard ready, focused else { return }
        let currentAxes = DriveKey.axes(pressed)
        guard currentAxes.forward != 0 || currentAxes.turn != 0 || lastCommandWasMoving else { return }
        guard let credential = await acquireToken() else { return }
        // Recheck after the awaited claim: a key may have been released meanwhile.
        guard ready, focused, token == credential else { return }
        let axes = DriveKey.axes(pressed)
        guard axes.forward != 0 || axes.turn != 0 || lastCommandWasMoving else { return }
        let currentGeneration = generation
        let seq = sequence + 1
        sequence = seq
        do {
            let response = try await post("command", body: [
                "token": credential, "seq": seq, "forward": axes.forward, "turn": axes.turn
            ])
            guard currentGeneration == generation, token == credential, sequence == seq else { return }
            guard try JSONDecoder().decode(SimAck.self, from: response).applied else { throw URLError(.badServerResponse) }
            lastCommandWasMoving = axes.forward != 0 || axes.turn != 0
        } catch {
            guard currentGeneration == generation, token == credential, sequence == seq else { return }
            invalidate("模拟控制请求失败，已停止发送速度指令")
        }
    }

    private func acquireToken() async -> String? {
        if let token { return token }
        let currentGeneration = generation
        do {
            let data = try await post("claim", body: [:])
            let claimed = try JSONDecoder().decode(SimClaim.self, from: data).token
            guard currentGeneration == generation && focused && ready else {
                _ = try? await post("release", body: ["token": claimed])
                return nil
            }
            token = claimed
            sequence = 0
            return claimed
        } catch {
            guard currentGeneration == generation else { return nil }
            invalidate("未能取得模拟控制会话，已停用按键")
            return nil
        }
    }

    private func sendZero() async {
        guard let credential = token else { return }
        let seq = sequence + 1
        sequence = seq
        // Zero is explicit on key release; no repeated idle command traffic.
        do {
            _ = try await post("command", body: [
                "token": credential, "seq": seq, "forward": 0.0, "turn": 0.0
            ])
            if token == credential && sequence == seq { lastCommandWasMoving = false }
        } catch {
            if token == credential && sequence == seq { invalidate("停止指令请求失败；模拟器将在 0.25 秒内超时停车") }
        }
    }
}
