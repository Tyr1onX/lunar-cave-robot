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
}
