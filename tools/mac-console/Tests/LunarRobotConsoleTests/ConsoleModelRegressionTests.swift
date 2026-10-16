import XCTest
@testable import LunarRobotConsole

@MainActor
final class ConsoleModelRegressionTests: XCTestCase {
    func testDelayedLogResponseFromPriorAgentSessionIsIgnoredAfterReconnect() {
        let model = ConsoleModel()
        model.markAgentAvailable(message: "Agent 在线")
        let requestSession = model.agentSession

        model.invalidateAgentData(message: "Agent 暂不可用")
        XCTAssertFalse(model.agentOnline)
        model.markAgentAvailable(message: "Agent 已恢复")

        let staleReading = LogReading(
            status: "ok",
            reason: nil,
            entries: [LogEntry(fileModifiedAt: "2026-10-13T05:00:00Z", severity: "error", message: "stale", source: "old.log")],
            source: "/home/robot/.ros/log/latest"
        )
        model.applyLogReading(staleReading, requestSession: requestSession)

        XCTAssertTrue(model.agentOnline)
        XCTAssertNil(model.logs, "A response from before disconnect must not repopulate the recovered session")

        let currentReading = LogReading(status: "ok", reason: nil, entries: [], source: "/home/robot/.ros/log/latest")
        model.applyLogReading(currentReading, requestSession: model.agentSession)
        XCTAssertNotNil(model.logs, "The recovered session must still accept its own log response")
    }

    func testStopClearsConnectionStateAndInvalidatesPendingLogResponse() {
        let model = ConsoleModel()
        model.markAgentAvailable(message: "Agent 在线")
        let requestSession = model.agentSession

        model.stop()

        XCTAssertFalse(model.agentOnline)
        XCTAssertNil(model.snapshot)
        XCTAssertNil(model.logs)
        XCTAssertNil(model.vision)
        XCTAssertNil(model.visionFrame)
        let delayedReading = LogReading(status: "ok", reason: nil, entries: [], source: "old session")
        model.applyLogReading(delayedReading, requestSession: requestSession)
        XCTAssertNil(model.logs, "Stopping the console must invalidate pending log responses")
    }

    func testWorkspaceInventoryIsRetainedAsHistoricalAndExcludedFromLiveHealthAfterDisconnect() throws {
        let model = ConsoleModel()
        model.markAgentAvailable(message: "Agent 在线")
        let observedAt = Date()
        let snapshot = try makeSnapshot(collectedAt: "2026-10-16T10:00:00Z")
        model.applySnapshot(snapshot, requestSession: model.agentSession, readAt: observedAt)

        XCTAssertEqual(model.workspaceSnapshotFreshness, .live)
        XCTAssertEqual(model.healthWorkspaceInventory?.entries.first?.id, "original-navigation")

        model.invalidateAgentData(message: "Agent 断连")

        XCTAssertNil(model.snapshot, "The full system snapshot is no longer a live reading after disconnect")
        XCTAssertEqual(model.workspaceInventorySnapshot?.entries.first?.id, "original-navigation")
        XCTAssertEqual(model.workspaceSnapshotFreshness, .historical)
        XCTAssertNil(model.healthSnapshot, "A retained workspace snapshot must not feed live health evaluation")
        XCTAssertNil(model.healthWorkspaceInventory)
    }

    func testDisconnectedWorkspaceSnapshotExpiresButRemainsVisible() throws {
        let model = ConsoleModel()
        model.markAgentAvailable(message: "Agent 在线")
        let observedAt = Date()
        model.applySnapshot(try makeSnapshot(collectedAt: "2026-10-16T10:00:00Z"), requestSession: model.agentSession, readAt: observedAt)
        model.invalidateAgentData(message: "Agent 断连")

        model.updateWorkspaceSnapshotFreshness(at: observedAt.addingTimeInterval(ConsoleModel.workspaceSnapshotFreshnessLimit + 1))

        XCTAssertEqual(model.workspaceSnapshotFreshness, .expired)
        XCTAssertEqual(model.workspaceInventorySnapshot?.entries.first?.id, "original-navigation")
        XCTAssertNil(model.healthWorkspaceInventory)
    }

    func testRecoveredConnectionUsesNewWorkspaceSnapshotAsLiveData() throws {
        let model = ConsoleModel()
        model.markAgentAvailable(message: "Agent 在线")
        let firstRead = Date()
        model.applySnapshot(try makeSnapshot(collectedAt: "2026-10-16T10:00:00Z"), requestSession: model.agentSession, readAt: firstRead)
        model.invalidateAgentData(message: "Agent 断连")
        model.markAgentAvailable(message: "Agent 已恢复")

        XCTAssertEqual(model.workspaceSnapshotFreshness, .historical, "Health endpoint recovery alone does not make old workspace evidence live")
        XCTAssertNil(model.healthWorkspaceInventory)

        let recoveredSnapshot = try makeSnapshot(collectedAt: "2026-10-16T10:00:10Z")
        model.applySnapshot(recoveredSnapshot, requestSession: model.agentSession, readAt: firstRead.addingTimeInterval(2))

        XCTAssertEqual(model.workspaceSnapshotFreshness, .live)
        XCTAssertEqual(model.healthWorkspaceInventory?.entries.first?.id, "original-navigation")
        XCTAssertEqual(model.snapshot?.collectedAt, "2026-10-16T10:00:10Z")
    }

    func testOlderAgentSnapshotDoesNotEraseOrPromoteHistoricalWorkspaceEvidence() throws {
        let model = ConsoleModel()
        model.markAgentAvailable(message: "Agent 在线")
        let firstRead = Date()
        model.applySnapshot(try makeSnapshot(collectedAt: "2026-10-16T10:00:00Z"), requestSession: model.agentSession, readAt: firstRead)
        model.invalidateAgentData(message: "Agent 断连")
        model.markAgentAvailable(message: "兼容旧版 Agent")

        let legacySnapshot = try makeSnapshot(collectedAt: "2026-10-16T10:00:10Z", includeWorkspaces: false)
        model.applySnapshot(legacySnapshot, requestSession: model.agentSession, readAt: firstRead.addingTimeInterval(2))

        XCTAssertNil(model.snapshot?.workspaces, "The legacy payload remains valid without workspace telemetry")
        XCTAssertEqual(model.workspaceInventorySnapshot?.entries.first?.id, "original-navigation")
        XCTAssertEqual(model.workspaceSnapshotFreshness, .historical)
        XCTAssertNil(model.healthWorkspaceInventory, "Legacy Agent data cannot promote an older workspace snapshot to live")
    }

    func testDelayedVisionFrameFromPriorAgentSessionCannotRepopulateAfterReconnect() {
        let model = ConsoleModel()
        model.markAgentAvailable(message: "Agent 在线")
        let oldSession = model.agentSession
        model.applyVisionReading(streamingVisionReading(), requestSession: oldSession)

        model.invalidateAgentData(message: "Agent 断开")
        model.markAgentAvailable(message: "Agent 已恢复")
        let currentSession = model.agentSession
        model.applyVisionReading(streamingVisionReading(), requestSession: currentSession)
        model.applyVisionFrame(Data([1]), streamID: .rgb, requestSession: oldSession, roundTripMs: 4)

        XCTAssertNil(model.visionFrame, "A delayed image from a disconnected Agent session must not be shown as live")
        model.applyVisionFrame(Data([2]), streamID: .rgb, requestSession: currentSession, roundTripMs: 5)
        XCTAssertEqual(model.visionFrame, Data([2]), "A fresh image from the recovered session must be accepted")
        XCTAssertNil(model.macFrameUpdateHz, "One accepted frame cannot establish Mac update FPS")
    }

    func testFrameUpdateRateIsLocalAndClearedOnSwitchAndDisconnect() {
        let model = ConsoleModel()
        model.markAgentAvailable(message: "Agent 在线")
        model.applyVisionReading(streamingVisionReading(), requestSession: model.agentSession)
        model.applyVisionFrame(Data([1]), streamID: .rgb, requestSession: model.agentSession, roundTripMs: 3)
        model.applyVisionFrame(Data([2]), streamID: .rgb, requestSession: model.agentSession, roundTripMs: 3)
        XCTAssertNotNil(model.macFrameUpdateHz)
        model.selectVisionStream(.depth)
        XCTAssertNil(model.visionFrame)
        XCTAssertNil(model.macFrameUpdateHz)
        model.invalidateAgentData(message: "Agent 断开")
        XCTAssertNil(model.macFrameUpdateHz)
    }

    private func streamingVisionReading() -> VisionReading {
        let stream = VisionStreamReading(
            id: "rgb", label: "RGB", topic: "/camera/color/image_raw", topicType: "sensor_msgs/msg/Image",
            status: "streaming", reason: nil, publisherCount: 1, width: 640, height: 480, encoding: "rgb8",
            frameRateHz: 30, frameAgeMs: 12, sourceStamp: "2026-10-13T05:00:00Z", frameId: "lunar-v0.2a-synthetic/rgb", headerToAgentMs: nil,
            receivedAt: "2026-10-13T05:00:00Z", gapEvents: 0
        )
        return VisionReading(status: "ok", source: "nuc-ros2-read-only-subscriber", collectedAt: "2026-10-13T05:00:00Z", reason: nil, streams: [stream])
    }

    private func makeSnapshot(collectedAt: String, includeWorkspaces: Bool = true) throws -> ConsoleSnapshot {
        var value: [String: Any] = [
            "schema_version": 1,
            "collected_at": collectedAt,
            "source": "nuc-read-only-agent",
            "host": ["hostname": "robot", "platform": "x86_64", "os_release": []],
            "system": [
                "cpu": ["status": "ok", "percent": 5],
                "memory": ["status": "unknown", "reason": "unavailable"],
                "disk": ["status": "unknown", "reason": "unavailable"],
                "temperature": ["status": "unknown", "reason": "unavailable"]
            ],
            "git": ["status": "unknown"],
            "deployment": ["status": "unknown"],
            "ros": ["status": "ok", "domain_id": 187, "nodes": [], "topics": []],
            "tests": ["status": "unknown", "records": []],
            "safety": [
                "physical_estop": ["status": "unknown", "reason": "not connected"],
                "hardware_health": ["status": "unknown", "reason": "no hardware evidence"],
                "control": "disabled"
            ]
        ]
        if includeWorkspaces {
            value["workspaces"] = [
                "status": "ok",
                "source": "nuc-read-only-workspace-and-process-observer",
                "collected_at": collectedAt,
                "workspaces": [[
                    "id": "original-navigation",
                    "label": "原始导航与雷达工程",
                    "source_kind": "original",
                    "source_status": "present",
                    "runtime_status": "unknown",
                    "run_expectation": "unconfigured"
                ]]
            ]
        }
        let data = try JSONSerialization.data(withJSONObject: value)
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(ConsoleSnapshot.self, from: data)
    }
}
