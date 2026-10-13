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
        let delayedReading = LogReading(status: "ok", reason: nil, entries: [], source: "old session")
        model.applyLogReading(delayedReading, requestSession: requestSession)
        XCTAssertNil(model.logs, "Stopping the console must invalidate pending log responses")
    }
}
