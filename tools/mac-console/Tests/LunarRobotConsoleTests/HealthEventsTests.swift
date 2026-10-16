import Foundation
import XCTest
@testable import LunarRobotConsole

final class HealthEventsTests: XCTestCase {
    func testHealthyAgentGraphAndExpectedNodesAreNormal() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(),
            vision: makeVision(rgb: stream(.rgb, status: "streaming", age: 80), depth: stream(.depth, status: "streaming", age: 90), yolo: stream(.yolo, status: "streaming", age: 120))
        )

        XCTAssertEqual(event("agent-connection", in: events).state, .normal)
        XCTAssertEqual(event("ros-graph-freshness", in: events).state, .normal)
        XCTAssertEqual(event("ros-domain", in: events).state, .normal)
        XCTAssertEqual(event("expected-ros-nodes", in: events).state, .normal)
        XCTAssertEqual(event("vision-rgb", in: events).state, .normal)
        XCTAssertEqual(event("vision-depth", in: events).state, .normal)
        XCTAssertEqual(event("vision-yolo", in: events).state, .normal)
    }

    func testUnconfiguredModulesAreNotReportedAsFailures() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(expectedDomainID: nil, expectedNodes: [], nodes: []),
            vision: makeVision(rgb: stream(.rgb, status: "unavailable", publisherCount: 0), depth: stream(.depth, status: "unavailable", publisherCount: 0), yolo: stream(.yolo, status: "unavailable", publisherCount: 0))
        )

        XCTAssertEqual(event("expected-ros-nodes", in: events).state, .notRunning)
        XCTAssertEqual(event("vision-rgb", in: events).state, .notRunning)
        XCTAssertEqual(event("vision-depth", in: events).state, .notRunning)
        XCTAssertEqual(event("ros-domain", in: events).state, .unknown)
        XCTAssertFalse(events.contains { $0.state == .abnormal })
    }

    func testGraphDataAndRGBDepthFrameTimeoutsAreDetected() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(graphAgeMs: 6_000),
            vision: makeVision(rgb: stream(.rgb, status: "stale", age: 2_200), depth: stream(.depth, status: "stale", age: 2_800), yolo: stream(.yolo, status: "waiting"))
        )

        XCTAssertEqual(event("ros-graph-freshness", in: events).state, .abnormal)
        XCTAssertEqual(event("vision-rgb", in: events).state, .abnormal)
        XCTAssertEqual(event("vision-depth", in: events).state, .abnormal)
        XCTAssertTrue(event("vision-rgb", in: events).observation.contains("2200 ms"))
        XCTAssertEqual(event("vision-rgb", in: events).dataTime, "2026-10-14T05:00:00.008Z")
    }

    func testPreviouslyReachableAgentDisconnectIsAnAbnormalEvent() {
        let events = HealthEvaluator.evaluate(
            agentOnline: false,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 暂不可用 · timeout",
            lastSuccessfulRead: Date(timeIntervalSince1970: 1_792_000_000),
            lastKnownDomainID: 42,
            snapshot: nil,
            vision: nil
        )

        let agent = event("agent-connection", in: events)
        XCTAssertEqual(agent.state, .abnormal)
        XCTAssertEqual(agent.rosDomain, "实际 42 · 预期 未配置")
        XCTAssertNotEqual(agent.dataTime, "无成功读取")
        XCTAssertTrue(events.filter { $0.id != "agent-connection" }.allSatisfy { $0.state == .unknown })
    }

    func testDomainMismatchRequiresAnExplicitFreshExpectation() throws {
        let mismatch = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(domainID: 17, expectedDomainID: 42),
            vision: nil
        )
        XCTAssertEqual(event("ros-domain", in: mismatch).state, .abnormal)
        XCTAssertTrue(event("ros-domain", in: mismatch).observation.contains("17"))
        XCTAssertTrue(event("ros-domain", in: mismatch).observation.contains("42"))

        let noExpectation = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(expectedDomainID: nil),
            vision: nil
        )
        XCTAssertEqual(event("ros-domain", in: noExpectation).state, .unknown)
    }

    func testSyntheticFramesRemainLabeledAsSyntheticAndDoNotClaimHardwareHealth() {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: 42,
            snapshot: nil,
            vision: makeVision(rgb: stream(.rgb, status: "streaming", age: 40, frameID: "lunar-v0.2a-synthetic/rgb"), depth: stream(.depth, status: "waiting"), yolo: stream(.yolo, status: "waiting"))
        )

        let rgb = event("vision-rgb", in: events)
        XCTAssertEqual(rgb.state, .normal)
        XCTAssertEqual(rgb.runMode, .synthetic)
        XCTAssertFalse(rgb.observation.localizedCaseInsensitiveContains("hardware healthy"))
        XCTAssertEqual(event("vision-yolo", in: events).state, .unknown)
    }

    func testYOLOMissingOrStaleOutputIsUnknownBecauseOfConfidenceGate() {
        for status in ["stale", "waiting", "unavailable"] {
            let events = HealthEvaluator.evaluate(
                agentOnline: true,
                hasEverConnectedToAgent: true,
                agentMessage: "Agent 在线",
                lastSuccessfulRead: nil,
                lastKnownDomainID: 42,
                snapshot: nil,
                vision: makeVision(rgb: stream(.rgb, status: "streaming", age: 30), depth: stream(.depth, status: "streaming", age: 30), yolo: stream(.yolo, status: status, age: 4_000, publisherCount: status == "unavailable" ? 0 : 1))
            )

            let yolo = event("vision-yolo", in: events)
            XCTAssertEqual(yolo.state, .unknown, "YOLO \(status) output is not enough to infer a fault")
            XCTAssertTrue(yolo.observation.contains("置信度"))
        }
    }

    func testMultipleIndependentAnomaliesCanBeReportedTogether() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(domainID: 17, expectedDomainID: 42, expectedNodes: ["/camera_node", "/lidar_node"], nodes: ["/camera_node"]),
            vision: makeVision(rgb: stream(.rgb, status: "stale", age: 2_100), depth: stream(.depth, status: "stale", age: 2_400), yolo: stream(.yolo, status: "waiting"))
        )

        let abnormalIDs = Set(events.filter { $0.state == .abnormal }.map(\.id))
        XCTAssertTrue(abnormalIDs.isSuperset(of: ["ros-domain", "expected-ros-nodes", "vision-rgb", "vision-depth"]))
        XCTAssertEqual(event("vision-yolo", in: events).state, .unknown)
    }

    func testNoPriorAgentConnectionRemainsUnknown() {
        let events = HealthEvaluator.evaluate(
            agentOnline: false,
            hasEverConnectedToAgent: false,
            agentMessage: "等待 NUC Agent",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: nil,
            vision: nil
        )
        XCTAssertEqual(event("agent-connection", in: events).state, .unknown)
    }

    func testRequestedReconnectIsNotReportedAsAgentLoss() {
        let events = HealthEvaluator.evaluate(
            agentOnline: false,
            hasEverConnectedToAgent: true,
            agentMessage: "正在重新连接 NUC Agent",
            lastSuccessfulRead: Date(timeIntervalSince1970: 1_792_000_000),
            lastKnownDomainID: 42,
            snapshot: nil,
            vision: nil
        )
        XCTAssertEqual(event("agent-connection", in: events).state, .unknown)
    }

    private func event(_ id: String, in events: [HealthEvent], file: StaticString = #filePath, line: UInt = #line) -> HealthEvent {
        guard let result = events.first(where: { $0.id == id }) else {
            XCTFail("Missing health event \(id)", file: file, line: line)
            return HealthEvent(id: id, title: id, state: .unknown, observation: "", resource: "", dataTime: "", rosDomain: "", runMode: .unknown, suggestedCheck: "")
        }
        return result
    }

    private func makeSnapshot(
        domainID: Int? = 42,
        expectedDomainID: Int? = 42,
        expectedNodes: [String] = ["/camera_node"],
        nodes: [String] = ["/camera_node"],
        graphAgeMs: Int? = 1_000
    ) throws -> ConsoleSnapshot {
        var ros: [String: Any] = [
            "status": "ok",
            "nodes": nodes.map(nodeValue),
            "topics": [],
            "observed_at": "2026-10-14T05:00:00.000Z",
            "expected_nodes": expectedNodes
        ]
        ros["domain_id"] = domainID ?? NSNull()
        ros["expected_domain_id"] = expectedDomainID ?? NSNull()
        ros["graph_age_ms"] = graphAgeMs ?? NSNull()
        let value: [String: Any] = [
            "schema_version": 1,
            "collected_at": "2026-10-14T05:00:00.000Z",
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
            "ros": ros,
            "tests": ["status": "unknown", "records": []],
            "safety": [
                "physical_estop": ["status": "unknown", "reason": "not connected"],
                "hardware_health": ["status": "unknown", "reason": "no hardware evidence"],
                "control": "disabled"
            ]
        ]
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(ConsoleSnapshot.self, from: JSONSerialization.data(withJSONObject: value))
    }

    private func nodeValue(_ value: String) -> [String: String] {
        let components = value.split(separator: "/").map(String.init)
        return [
            "name": components.last ?? "node",
            "namespace": components.count > 1 ? "/" + components.dropLast().joined(separator: "/") : "/"
        ]
    }

    private func stream(_ id: VisionStreamID, status: String, age: Int? = nil, publisherCount: Int = 1, frameID: String? = nil) -> VisionStreamReading {
        let topic: String
        switch id {
        case .rgb: topic = "/camera/color/image_raw"
        case .depth: topic = "/camera/depth/image_rect_raw"
        case .yolo: topic = "/yolo/annotated_image"
        }
        return VisionStreamReading(
            id: id.rawValue,
            label: id.label,
            topic: topic,
            topicType: publisherCount == 0 ? nil : "sensor_msgs/msg/Image",
            status: status,
            reason: status == "unavailable" ? "ROS Topic 当前没有发布者" : nil,
            publisherCount: publisherCount,
            width: status == "streaming" ? 640 : nil,
            height: status == "streaming" ? 480 : nil,
            encoding: status == "streaming" ? "rgb8" : nil,
            frameRateHz: status == "streaming" ? 30 : nil,
            frameAgeMs: age,
            sourceStamp: status == "streaming" || status == "stale" ? "2026-10-14T05:00:00.000Z" : nil,
            frameId: frameID,
            headerToAgentMs: nil,
            receivedAt: status == "streaming" || status == "stale" ? "2026-10-14T05:00:00.008Z" : nil,
            gapEvents: 0
        )
    }

    private func makeVision(rgb: VisionStreamReading, depth: VisionStreamReading, yolo: VisionStreamReading) -> VisionReading {
        VisionReading(status: "ok", source: "nuc-ros2-read-only-subscriber", collectedAt: "2026-10-14T05:00:00.010Z", reason: nil, streams: [rgb, depth, yolo])
    }
}
