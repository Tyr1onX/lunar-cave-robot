import Foundation
import SwiftUI

struct ArmPreviewJoint: Decodable, Identifiable {
    let name: String
    let lower: Double
    let upper: Double
    let axis: [Double]
    let position: Double

    var id: String { name }
    var label: String { name.replacingOccurrences(of: "_joint", with: "") }
}

struct ArmPreviewFrame: Decodable {
    let parent: String
    let child: String
    let xyz: [Double]
    let rpy: [Double]
}

struct ArmPreviewState: Decodable {
    let schemaVersion: Int
    let source: String
    let mode: String
    let domainId: Int
    let localhostOnly: Bool
    let ready: Bool
    let jointTopic: String
    let urdfSha256: String
    let baseLink: String
    let tipLink: String
    let joints: [ArmPreviewJoint]
    let gripperOpening: Double
    let srdfHandParentLink: String
    let fixedFrames: [String: ArmPreviewFrame]
    let meshCount: Int
    let note: String
}

private struct ArmPreviewClaim: Decodable { let token: String; let mode: String }
private struct ArmPreviewAck: Decodable { let applied: Bool; let seq: Int; let mode: String }

@MainActor
final class ArmPreviewModel: ObservableObject {
    @Published private(set) var ready = false
    @Published private(set) var state: ArmPreviewState?
    @Published private(set) var jointPositions: [String: Double] = [:]
    @Published private(set) var gripperOpening = 0.0
    @Published private(set) var message = "等待独立机械臂模拟服务（不会自动启动）"
    @Published private(set) var lastAcceptedSequence = 0

    private let root = URL(string: "http://127.0.0.1:18767")!
    private let session: URLSession = {
        let configuration = URLSessionConfiguration.ephemeral
        configuration.timeoutIntervalForRequest = 1.0
        configuration.timeoutIntervalForResource = 1.4
        configuration.waitsForConnectivity = false
        return URLSession(configuration: configuration)
    }()
    private var tunnel: Process?
    private var pollTask: Task<Void, Never>?
    private var commandTask: Task<Void, Never>?
    private var generation = 0
    private var token: String?
    private var sequence = 0
    private var stopping = false

    var joints: [ArmPreviewJoint] { state?.joints ?? [] }
    var camera: ArmPreviewFrame? { state?.fixedFrames["camera_joint"] }
    var tcp: ArmPreviewFrame? { state?.fixedFrames["tcp_joint"] }

    func start() {
        guard pollTask == nil else { return }
        stopping = false
        generation += 1
        ensureTunnel()
        pollTask = Task { [weak self] in
            while let self, !Task.isCancelled {
                await self.poll()
                try? await Task.sleep(for: .milliseconds(250))
            }
        }
    }

    func stop() {
        guard !stopping else { return }
        stopping = true
        generation += 1
        pollTask?.cancel()
        pollTask = nil
        commandTask?.cancel()
        commandTask = nil
        ready = false
        state = nil
        jointPositions = [:]
        gripperOpening = 0
        let previousToken = token
        token = nil
        sequence = 0
        let stoppedGeneration = generation
        if let previousToken {
            Task {
                _ = try? await post("release", body: ["token": previousToken])
                guard generation == stoppedGeneration, pollTask == nil else { return }
                closeTunnel()
            }
        } else {
            closeTunnel()
        }
    }

    func setJoint(_ name: String, to requested: Double) {
        guard ready, let joint = joints.first(where: { $0.name == name }), requested.isFinite else { return }
        let value = min(joint.upper, max(joint.lower, requested))
        jointPositions[name] = value
        scheduleCommand()
    }

    func setGripperOpening(_ requested: Double) {
        guard ready, requested.isFinite else { return }
        gripperOpening = min(1.0, max(0.0, requested))
        scheduleCommand()
    }

    func resetPose() {
        guard ready else { return }
        jointPositions = Dictionary(uniqueKeysWithValues: joints.map { ($0.name, 0.0) })
        gripperOpening = 0.0
        scheduleCommand()
    }

    func position(for joint: ArmPreviewJoint) -> Double {
        jointPositions[joint.name] ?? joint.position
    }

    private func ensureTunnel() {
        if tunnel?.isRunning == true { return }
        let process = Process()
        process.executableURL = URL(fileURLWithPath: "/usr/bin/ssh")
        process.arguments = [
            "-N", "-T",
            "-L", "127.0.0.1:18767:127.0.0.1:8767",
            "-L", "127.0.0.1:18768:127.0.0.1:8768",
            "-o", "BatchMode=yes",
            "-o", "ConnectTimeout=5",
            "-o", "ExitOnForwardFailure=yes",
            "-o", "ServerAliveInterval=5",
            "-o", "ServerAliveCountMax=2",
            "-o", "StrictHostKeyChecking=yes",
            "nuc",
        ]
        process.standardOutput = FileHandle.nullDevice
        process.standardError = FileHandle.nullDevice
        do {
            try process.run()
            tunnel = process
        } catch {
            message = "无法建立隔离预览 SSH 转发：\(error.localizedDescription)"
        }
    }

    private func closeTunnel() {
        if tunnel?.isRunning == true { tunnel?.terminate() }
        tunnel = nil
    }

    private func endpoint(_ name: String) -> URL {
        root.appendingPathComponent("v1/arm-preview/\(name)")
    }

    private func fetchState() async throws -> ArmPreviewState {
        let (data, response) = try await session.data(from: endpoint("state"))
        guard (response as? HTTPURLResponse)?.statusCode == 200 else { throw URLError(.badServerResponse) }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(ArmPreviewState.self, from: data)
    }

    private func post(_ name: String, body: [String: Any]) async throws -> Data {
        var request = URLRequest(url: endpoint(name))
        request.httpMethod = "POST"
        request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        request.httpBody = try JSONSerialization.data(withJSONObject: body)
        let (data, response) = try await session.data(for: request)
        guard (response as? HTTPURLResponse)?.statusCode == 200 else { throw URLError(.badServerResponse) }
        return data
    }

    private func poll() async {
        let currentGeneration = generation
        ensureTunnel()
        do {
            let reading = try await fetchState()
            guard currentGeneration == generation else { return }
            let names = reading.joints.map(\.name)
            guard reading.schemaVersion == 1,
                  reading.source == "isolated-arm-joint-state-simulation",
                  reading.mode == "SIMULATION",
                  reading.domainId == 218,
                  reading.localhostOnly,
                  reading.jointTopic == "/arm_preview/joint_states",
                  reading.ready,
                  reading.joints.count == 6,
                  Set(names).count == 6,
                  reading.meshCount == 10,
                  reading.joints.allSatisfy({ $0.lower <= $0.position && $0.position <= $0.upper }) else {
                invalidate("机械臂预览数据与隔离 URDF 合同不符")
                return
            }
            state = reading
            if jointPositions.isEmpty {
                jointPositions = Dictionary(uniqueKeysWithValues: reading.joints.map { ($0.name, $0.position) })
                gripperOpening = reading.gripperOpening
            }
            ready = true
            message = "SIMULATION · ROS Domain 218 · \(reading.jointTopic) · 模型 \(reading.urdfSha256.prefix(12))"
            if token == nil { await claim() }
        } catch {
            guard currentGeneration == generation else { return }
            invalidate("机械臂模拟服务不可用 · 请确认 NUC 已手动运行 run.sh")
        }
    }

    private func claim() async {
        let currentGeneration = generation
        do {
            let data = try await post("claim", body: [:])
            let claimed = try JSONDecoder().decode(ArmPreviewClaim.self, from: data)
            guard currentGeneration == generation, claimed.mode == "SIMULATION", ready else {
                _ = try? await post("release", body: ["token": claimed.token])
                return
            }
            token = claimed.token
            sequence = 0
        } catch {
            guard currentGeneration == generation else { return }
            invalidate("未能取得机械臂模拟会话")
        }
    }

    private func scheduleCommand() {
        commandTask?.cancel()
        commandTask = Task { [weak self] in
            try? await Task.sleep(for: .milliseconds(35))
            await self?.sendPose()
        }
    }

    private func sendPose() async {
        guard ready, let credential = token else { return }
        let currentGeneration = generation
        sequence += 1
        let currentSequence = sequence
        let positions = Dictionary(uniqueKeysWithValues: joints.map { ($0.name, position(for: $0)) })
        do {
            let data = try await post("command", body: [
                "token": credential,
                "seq": currentSequence,
                "joints": positions,
                "gripper_opening": gripperOpening,
            ])
            let ack = try JSONDecoder().decode(ArmPreviewAck.self, from: data)
            guard currentGeneration == generation, token == credential else { return }
            guard ack.applied, ack.seq == currentSequence, ack.mode == "SIMULATION" else {
                throw URLError(.badServerResponse)
            }
            lastAcceptedSequence = ack.seq
            message = "SIMULATION · JointState 已发布 · 序号 \(ack.seq)"
        } catch {
            if currentGeneration == generation, token == credential {
                invalidate("模拟 JointState 更新失败；请检查 SSH 转发和 NUC 日志")
            }
        }
    }

    private func invalidate(_ explanation: String) {
        let previousToken = token
        token = nil
        sequence = 0
        ready = false
        state = nil
        jointPositions = [:]
        gripperOpening = 0
        message = explanation
        if let previousToken { Task { _ = try? await post("release", body: ["token": previousToken]) } }
    }
}
