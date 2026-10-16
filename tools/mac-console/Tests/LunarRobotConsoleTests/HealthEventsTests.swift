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
        XCTAssertEqual(event("ros-graph-freshness", in: events).observedAt, "2026-10-14T05:00:00.000Z")
        XCTAssertEqual(event("ros-graph-freshness", in: events).source, "NUC Agent · ROS graph observer")
        XCTAssertEqual(event("vision-rgb", in: events).source, "nuc-ros2-read-only-subscriber")
        XCTAssertNotEqual(event("vision-rgb", in: events).observedAt, "未知")
        for item in events {
            XCTAssertFalse(item.module.isEmpty)
            XCTAssertFalse(item.ruleID.isEmpty)
            XCTAssertFalse(item.sourceLocation.isEmpty)
            XCTAssertEqual(item.severity, .informational)
        }
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

    func testFreshGraphAllowsRGBDepthFrameTimeoutsToBeDetected() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(graphAgeMs: 1_000),
            vision: makeVision(rgb: stream(.rgb, status: "stale", age: 2_200), depth: stream(.depth, status: "stale", age: 2_800), yolo: stream(.yolo, status: "waiting"))
        )

        XCTAssertEqual(event("ros-graph-freshness", in: events).state, .normal)
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
        XCTAssertEqual(agent.severity, .error)
        XCTAssertEqual(agent.rosDomain, "实际 42 · 预期 未配置")
        XCTAssertNotEqual(agent.dataTime, "无成功读取")
        XCTAssertTrue(events.filter { $0.id != "agent-connection" }.allSatisfy { $0.state == .unknown })
        XCTAssertTrue(agent.source.contains("Mac Console"))
        XCTAssertNotEqual(agent.observedAt, "未知")
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
        XCTAssertEqual(event("ros-domain", in: mismatch).severity, .error)
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
        XCTAssertEqual(event("ros-domain", in: noExpectation).severity, .warning)
    }

    func testDomainMismatchSuppressesNodeAndVisionFindings() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(
                domainID: 17,
                expectedDomainID: 42,
                expectedNodes: ["/camera_node", "/lidar_node"],
                nodes: ["/camera_node"]
            ),
            vision: makeVision(
                rgb: stream(.rgb, status: "stale", age: 2_100),
                depth: stream(.depth, status: "stale", age: 2_200),
                yolo: stream(.yolo, status: "waiting")
            )
        )

        XCTAssertEqual(event("ros-domain", in: events).state, .abnormal)
        XCTAssertEqual(event("expected-ros-nodes", in: events).state, .unknown)
        XCTAssertTrue(event("expected-ros-nodes", in: events).observation.contains("Domain"))
        for id in ["vision-rgb", "vision-depth", "vision-yolo"] {
            XCTAssertEqual(event(id, in: events).state, .unknown, "\(id) must not cascade into a false fault")
            XCTAssertTrue(event(id, in: events).observation.contains("Domain"))
        }
        XCTAssertEqual(Set(events.filter { $0.state == .abnormal }.map(\.id)), ["ros-domain"])
    }

    func testInvalidExpectedDomainSuppressesNodeAndVisionFindings() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(
                expectedDomainID: nil,
                expectedDomainError: "必须是 0 到 232 之间的整数",
                expectedNodes: ["/camera_node", "/lidar_node"],
                nodes: ["/camera_node"]
            ),
            vision: makeVision(
                rgb: stream(.rgb, status: "stale", age: 2_100),
                depth: stream(.depth, status: "stale", age: 2_200),
                yolo: stream(.yolo, status: "waiting")
            )
        )

        XCTAssertEqual(event("ros-graph-freshness", in: events).state, .normal)
        XCTAssertEqual(event("ros-domain", in: events).state, .unknown)
        XCTAssertTrue(event("ros-domain", in: events).observation.contains("配置无效"))
        XCTAssertEqual(event("expected-ros-nodes", in: events).state, .unknown)
        for id in ["vision-rgb", "vision-depth", "vision-yolo"] {
            XCTAssertEqual(event(id, in: events).state, .unknown, "\(id) must not cascade from invalid Domain configuration")
            XCTAssertTrue(event(id, in: events).observation.contains("Domain"))
        }
        XCTAssertEqual(Set(events.filter { $0.state == .abnormal }.map(\.id)), [])
    }

    func testExpiredGraphSuppressesNodeAndVisionFindings() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(
                expectedNodes: ["/camera_node", "/lidar_node"],
                nodes: ["/camera_node"],
                graphAgeMs: 6_000
            ),
            vision: makeVision(
                rgb: stream(.rgb, status: "stale", age: 3_000),
                depth: stream(.depth, status: "stale", age: 3_000),
                yolo: stream(.yolo, status: "waiting")
            )
        )

        XCTAssertEqual(event("ros-graph-freshness", in: events).state, .abnormal)
        XCTAssertEqual(event("ros-domain", in: events).state, .unknown)
        XCTAssertEqual(event("expected-ros-nodes", in: events).state, .unknown)
        for id in ["vision-rgb", "vision-depth", "vision-yolo"] {
            XCTAssertEqual(event(id, in: events).state, .unknown, "\(id) must be suppressed when the graph is stale")
            XCTAssertTrue(event(id, in: events).observation.contains("graph"))
        }
        XCTAssertEqual(Set(events.filter { $0.state == .abnormal }.map(\.id)), ["ros-graph-freshness"])
    }

    func testUnknownHealthStatusCannotResolveToGreen() {
        let normal = HealthEvent(
            id: "normal",
            title: "正常项目",
            state: .normal,
            observation: "",
            resource: "",
            dataTime: "",
            rosDomain: "",
            runMode: .live,
            suggestedCheck: ""
        )
        let unknown = HealthEvent(
            id: "unknown",
            title: "未知项目",
            state: .unknown,
            observation: "",
            resource: "",
            dataTime: "",
            rosDomain: "",
            runMode: .unknown,
            suggestedCheck: ""
        )

        XCTAssertEqual(HealthSummaryState.resolve([normal, unknown]), .unknown)
        XCTAssertEqual(HealthSummaryState.resolve([]), .unknown)
        XCTAssertEqual(HealthSummaryState.resolve([normal]), .normal)
    }

    func testCockpitSummaryRequiresWorkspaceTelemetryAndConfirmedExpectations() throws {
        let normal = HealthEvent(
            id: "normal",
            title: "正常项目",
            state: .normal,
            observation: "",
            resource: "",
            dataTime: "",
            rosDomain: "",
            runMode: .live,
            suggestedCheck: ""
        )
        XCTAssertEqual(
            HealthSummaryState.resolve([normal], requireWorkspaceTelemetry: true),
            .unknown,
            "Older Agent snapshots without workspace inventory cannot produce a green all-workspace summary."
        )

        let unconfigured = try decodeWorkspaceInventory(runExpectation: "unconfigured")
        XCTAssertTrue(unconfigured.hasUnconfirmedHealth)
        XCTAssertEqual(
            HealthSummaryState.resolve([normal], workspaceInventory: unconfigured, requireWorkspaceTelemetry: true),
            .unknown,
            "Observed processes are insufficient while runtime expectations remain unconfigured."
        )

        let confirmed = try decodeWorkspaceInventory(runExpectation: "confirmed")
        XCTAssertFalse(confirmed.hasUnconfirmedHealth)
        XCTAssertEqual(
            HealthSummaryState.resolve([normal], workspaceInventory: confirmed, requireWorkspaceTelemetry: true),
            .normal
        )
    }

    private func decodeWorkspaceInventory(runExpectation: String) throws -> WorkspaceInventoryReading {
        let json = #"{"status":"ok","workspaces":[{"id":"original-navigation","label":"原始导航与雷达工程","source_kind":"original","source_status":"present","runtime_status":"running","run_expectation":"\#(runExpectation)","graph_relationship":"shared_domain","modules":[{"id":"navigation","label":"导航","source_status":"present","runtime_status":"running","run_expectation":"\#(runExpectation)"}]}]}"#
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(WorkspaceInventoryReading.self, from: Data(json.utf8))
    }

    func testSustainedAnomalyRecoversAndStartsANewEpisode() throws {
        var tracker = HealthEventTracker()
        let t0 = "2026-10-14T05:00:00.000Z"
        let t1 = "2026-10-14T05:00:01.000Z"
        let t2 = "2026-10-14T05:00:02.000Z"
        let t3 = "2026-10-14T05:00:03.000Z"
        let snapshot = try makeSnapshot()

        let firstRaw = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: snapshot,
            vision: makeVision(
                rgb: stream(.rgb, status: "stale", age: 2_000),
                depth: stream(.depth, status: "streaming", age: 40),
                yolo: stream(.yolo, status: "waiting"),
                collectedAt: t0
            )
        )
        let first = event("vision-rgb", in: tracker.update(firstRaw))
        XCTAssertEqual(first.persistence, .singleSnapshot)
        XCTAssertEqual(first.observationCount, 1)

        let secondRaw = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: snapshot,
            vision: makeVision(
                rgb: stream(.rgb, status: "stale", age: 2_500),
                depth: stream(.depth, status: "streaming", age: 40),
                yolo: stream(.yolo, status: "waiting"),
                collectedAt: t1
            )
        )
        let second = event("vision-rgb", in: tracker.update(secondRaw))
        XCTAssertEqual(second.persistence, .sustained)
        XCTAssertEqual(second.observationCount, 2)
        XCTAssertEqual(second.firstObservedAt, t0)
        XCTAssertEqual(second.observedAt, t1)
        XCTAssertEqual(second.observationWindow, "\(t0) → \(t1)")

        let recoveredRaw = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: snapshot,
            vision: makeVision(
                rgb: stream(.rgb, status: "streaming", age: 80),
                depth: stream(.depth, status: "streaming", age: 40),
                yolo: stream(.yolo, status: "waiting"),
                collectedAt: t2
            )
        )
        let recovered = event("vision-rgb", in: tracker.update(recoveredRaw))
        XCTAssertEqual(recovered.state, .normal)
        XCTAssertEqual(recovered.persistence, .singleSnapshot)
        XCTAssertEqual(recovered.observationCount, 1)

        let recurredRaw = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: snapshot,
            vision: makeVision(
                rgb: stream(.rgb, status: "stale", age: 2_000),
                depth: stream(.depth, status: "streaming", age: 40),
                yolo: stream(.yolo, status: "waiting"),
                collectedAt: t3
            )
        )
        let recurred = event("vision-rgb", in: tracker.update(recurredRaw))
        XCTAssertEqual(recurred.state, .abnormal)
        XCTAssertEqual(recurred.persistence, .singleSnapshot)
        XCTAssertEqual(recurred.observationCount, 1)
    }

    func testSyntheticFramesRemainLabeledAsSyntheticAndDoNotClaimHardwareHealth() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: 42,
            snapshot: try makeSnapshot(),
            vision: makeVision(rgb: stream(.rgb, status: "streaming", age: 40, frameID: "lunar-v0.2a-synthetic/rgb"), depth: stream(.depth, status: "waiting"), yolo: stream(.yolo, status: "waiting"))
        )

        let rgb = event("vision-rgb", in: events)
        XCTAssertEqual(rgb.state, .normal)
        XCTAssertEqual(rgb.runMode, .synthetic)
        XCTAssertFalse(rgb.observation.localizedCaseInsensitiveContains("hardware healthy"))
        XCTAssertEqual(event("vision-yolo", in: events).state, .unknown)
    }

    func testYOLOMissingOrStaleOutputIsUnknownBecauseOfConfidenceGate() throws {
        for status in ["stale", "waiting", "unavailable"] {
            let events = HealthEvaluator.evaluate(
                agentOnline: true,
                hasEverConnectedToAgent: true,
                agentMessage: "Agent 在线",
                lastSuccessfulRead: nil,
                lastKnownDomainID: 42,
                snapshot: try makeSnapshot(),
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
            snapshot: try makeSnapshot(domainID: 42, expectedDomainID: 42, expectedNodes: ["/camera_node", "/lidar_node"], nodes: ["/camera_node"]),
            vision: makeVision(rgb: stream(.rgb, status: "stale", age: 2_100), depth: stream(.depth, status: "stale", age: 2_400), yolo: stream(.yolo, status: "waiting"))
        )

        let abnormalIDs = Set(events.filter { $0.state == .abnormal }.map(\.id))
        XCTAssertTrue(abnormalIDs.isSuperset(of: ["expected-ros-nodes", "vision-rgb", "vision-depth"]))
        XCTAssertFalse(abnormalIDs.contains("ros-domain"))
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
        expectedDomainError: String? = nil,
        expectedNodes: [String] = ["/camera_node"],
        nodes: [String] = ["/camera_node"],
        graphAgeMs: Int? = 1_000,
        collectedAt: String = "2026-10-14T05:00:00.000Z"
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
        if let expectedDomainError {
            ros["expected_domain_error"] = expectedDomainError
        }
        let value: [String: Any] = [
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

    private func makeVision(
        rgb: VisionStreamReading,
        depth: VisionStreamReading,
        yolo: VisionStreamReading,
        collectedAt: String = "2026-10-14T05:00:00.010Z"
    ) -> VisionReading {
        VisionReading(status: "ok", source: "nuc-ros2-read-only-subscriber", collectedAt: collectedAt, reason: nil, streams: [rgb, depth, yolo])
    }
}
