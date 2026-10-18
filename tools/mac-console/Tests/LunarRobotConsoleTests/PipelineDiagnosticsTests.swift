import Foundation
import XCTest
@testable import LunarRobotConsole

final class PipelineDiagnosticsTests: XCTestCase {
    private let collectedAt = "2026-10-18T12:00:00.000Z"

    func testReportFilenameIsStableAndPathSafe() {
        let filename = PipelineReportFilename.make(for: Date(timeIntervalSince1970: 1_792_000_000))

        XCTAssertEqual(filename, "lunar-pipeline-diagnosis-2026-10-14T17-46-40Z.md")
        XCTAssertFalse(filename.contains("/"))
        XCTAssertFalse(filename.contains(":"))
        XCTAssertTrue(filename.hasSuffix(".md"))
    }

    func testUnconfiguredScenarioNeverReportsMissingStreamsAsFaults() throws {
        let result = evaluate(expectation: .unconfigured, statuses: allUnavailable())

        XCTAssertNil(result.firstFinding)
        XCTAssertTrue(result.stages.allSatisfy { $0.state == .notConfigured })
        XCTAssertTrue(result.summary.contains("不判定"))
    }

    func testNormalSyntheticSnapshotShowsOnlyAgentReceivesData() throws {
        let result = evaluate(expectation: configuredExpectation(), statuses: [:])

        XCTAssertNil(result.firstFinding)
        XCTAssertTrue(result.stages.allSatisfy { $0.state == .observed })
        XCTAssertTrue(result.stages.allSatisfy { $0.confidence == .observedOnly })
        XCTAssertTrue(result.stages.allSatisfy { $0.summary.contains("工作区归属") })
    }

    func testRadarDropoutIsFirstAndDownstreamIsOnlyPossiblyAffected() throws {
        let result = evaluate(expectation: configuredExpectation(), statuses: ["/livox/lidar": "stale"])

        XCTAssertEqual(result.firstFinding?.stageID, .radar)
        XCTAssertEqual(result.firstFinding?.confidence, .suspected)
        XCTAssertEqual(result.stage(.radar)?.state, .suspectedGap)
        XCTAssertEqual(result.stage(.pointLIO)?.state, .possiblyAffected)
        XCTAssertEqual(result.stage(.slam)?.state, .possiblyAffected)
        XCTAssertEqual(result.stage(.nav2)?.state, .possiblyAffected)
        XCTAssertEqual(result.workflowState(for: "sensor-inputs"), .suspectedGap)
        XCTAssertEqual(result.workflowState(for: "route-planning"), .possiblyAffected)
    }

    func testPointLIOOutputDropoutLocalizesToPointLIOStage() throws {
        let result = evaluate(expectation: configuredExpectation(), statuses: ["/odometry": "stale"])

        XCTAssertEqual(result.firstFinding?.stageID, .pointLIO)
        XCTAssertEqual(result.firstFinding?.evidenceTopics, ["/odometry"])
        XCTAssertEqual(result.workflowState(for: "localization-estimate"), .suspectedGap)
        XCTAssertEqual(result.stage(.slam)?.state, .possiblyAffected)
    }

    func testTerrainAndScanGapsMapToTheirFirstObservableBoundaries() throws {
        let terrain = evaluate(expectation: configuredExpectation(), statuses: ["/terrain_map": "unavailable"])
        XCTAssertEqual(terrain.firstFinding?.stageID, .pointLIO)
        XCTAssertEqual(terrain.firstFinding?.workflowStepID, "terrain-filter")

        let scan = evaluate(expectation: configuredExpectation(), statuses: ["/scan": "stale"])
        XCTAssertEqual(scan.firstFinding?.stageID, .slam)
        XCTAssertEqual(scan.firstFinding?.workflowStepID, "scan-generation")
        XCTAssertEqual(scan.stage(.nav2)?.state, .possiblyAffected)
    }

    func testSLAMMapTimeoutIsSuspectedNotConfirmed() throws {
        let result = evaluate(expectation: configuredExpectation(), statuses: ["/map": "stale"])

        XCTAssertEqual(result.firstFinding?.stageID, .slam)
        XCTAssertEqual(result.firstFinding?.confidence, .suspected)
        XCTAssertEqual(result.stage(.nav2)?.state, .possiblyAffected)
    }

    func testMissingExplicitNodeIsConfirmedExpectationMismatch() throws {
        var expectation = configuredExpectation()
        expectation.expectedNodesByStage = [.slam: ["/slam_toolbox"]]
        let result = evaluate(expectation: expectation, statuses: [:], nodeNames: ["/pointcloud_to_laserscan"])

        XCTAssertEqual(result.firstFinding?.stageID, .slam)
        XCTAssertEqual(result.firstFinding?.kind, .nodeMissing)
        XCTAssertEqual(result.firstFinding?.confidence, .confirmedExpectationMismatch)
        XCTAssertTrue(result.firstFinding?.observation.contains("不是实体硬件故障") == true)
        XCTAssertEqual(result.stage(.nav2)?.state, .possiblyAffected)
    }

    func testSeveralGapsReportEarliestAndDoNotCreateDownstreamRootCauses() throws {
        let result = evaluate(
            expectation: configuredExpectation(),
            statuses: ["/livox/imu": "unavailable", "/scan": "stale", "/map": "unavailable"]
        )

        XCTAssertEqual(result.firstFinding?.stageID, .radar)
        XCTAssertEqual(result.stage(.pointLIO)?.state, .possiblyAffected)
        XCTAssertEqual(result.stage(.slam)?.state, .possiblyAffected)
        XCTAssertEqual(result.stage(.nav2)?.state, .possiblyAffected)
        XCTAssertEqual(result.stages.filter { $0.state == .suspectedGap }.count, 1)
    }

    func testNoVelocityCommandIsNotAnIssueWithoutAnActiveNavigationGoal() throws {
        let idle = evaluate(expectation: configuredExpectation(), statuses: ["/cmd_vel": "unavailable"])
        XCTAssertNil(idle.firstFinding)
        XCTAssertEqual(idle.stage(.nav2)?.state, .observed)

        var active = configuredExpectation()
        active.navigationGoalActive = true
        let navigating = evaluate(expectation: active, statuses: ["/cmd_vel": "unavailable"])
        XCTAssertEqual(navigating.firstFinding?.stageID, .nav2)
        XCTAssertEqual(navigating.firstFinding?.evidenceTopics, ["/cmd_vel"])
    }

    func testAgentDisconnectAndOldAgentTelemetryAbsenceKeepAllStagesUnknown() throws {
        let disconnected = PipelineDiagnosisEvaluator.evaluate(
            expectation: configuredExpectation(), agentOnline: false, snapshot: nil,
            telemetry: nil, logs: nil, evaluationTime: Date(timeIntervalSince1970: 1_792_000_000)
        )
        XCTAssertNil(disconnected.firstFinding)
        XCTAssertTrue(disconnected.stages.allSatisfy { $0.state == .unknown })
        XCTAssertTrue(disconnected.blockedReason?.contains("断连") == true)

        let oldAgent = PipelineDiagnosisEvaluator.evaluate(
            expectation: configuredExpectation(), agentOnline: true, snapshot: try snapshot(),
            telemetry: nil, logs: nil, evaluationTime: Date(timeIntervalSince1970: 1_792_000_000)
        )
        XCTAssertNil(oldAgent.firstFinding)
        XCTAssertTrue(oldAgent.stages.allSatisfy { $0.state == .unknown })
        XCTAssertTrue(oldAgent.blockedReason?.contains("旧版 Agent") == true)
    }

    func testDomainMismatchAndExpiredGraphBlockAllDownstreamChecks() throws {
        let mismatch = evaluate(expectation: configuredExpectation(domain: 47), statuses: [:], actualDomain: 42)
        XCTAssertNil(mismatch.firstFinding)
        XCTAssertTrue(mismatch.stages.allSatisfy { $0.state == .unknown })
        XCTAssertTrue(mismatch.blockedReason?.contains("Domain") == true)

        let expired = evaluate(expectation: configuredExpectation(), statuses: [:], snapshotGraphAge: HealthEvaluator.rosGraphTimeoutMs + 1)
        XCTAssertNil(expired.firstFinding)
        XCTAssertTrue(expired.stages.allSatisfy { $0.state == .unknown })
        XCTAssertTrue(expired.blockedReason?.contains("过期") == true)

        let telemetryExpired = evaluate(expectation: configuredExpectation(), statuses: [:], telemetryGraphAge: HealthEvaluator.rosGraphTimeoutMs + 1)
        XCTAssertNil(telemetryExpired.firstFinding)
        XCTAssertTrue(telemetryExpired.stages.allSatisfy { $0.state == .unknown })
    }

    func testAgentConfiguredDomainConflictAndInvalidConfigurationBlockPipeline() throws {
        let invalid = evaluate(expectation: configuredExpectation(), statuses: [:], agentDomainError: "bad domain")
        XCTAssertNil(invalid.firstFinding)
        XCTAssertTrue(invalid.stages.allSatisfy { $0.state == .unknown })
        XCTAssertTrue(invalid.blockedReason?.contains("无效") == true)

        let conflict = evaluate(expectation: configuredExpectation(domain: 47), statuses: [:], agentExpectedDomain: 48)
        XCTAssertNil(conflict.firstFinding)
        XCTAssertTrue(conflict.stages.allSatisfy { $0.state == .unknown })
    }

    func testUnsupportedOrMissingTelemetryRecordIsEvidenceInsufficient() throws {
        let unsupported = evaluate(expectation: configuredExpectation(), statuses: ["/livox/lidar": "unsupported"])
        XCTAssertNil(unsupported.firstFinding)
        XCTAssertEqual(unsupported.stage(.radar)?.state, .unknown)
        XCTAssertEqual(unsupported.stage(.pointLIO)?.state, .possiblyAffected)

        let missingRecord = evaluate(expectation: configuredExpectation(), statuses: [:], missingTopics: ["/livox/lidar"])
        XCTAssertNil(missingRecord.firstFinding)
        XCTAssertEqual(missingRecord.stage(.radar)?.state, .unknown)
    }

    func testRelevantLogsAndRawStreamEvidenceAreAttachedToStage() throws {
        let logs = LogReading(
            status: "ok", reason: nil,
            entries: [
                LogEntry(fileModifiedAt: collectedAt, severity: "error", message: "point_lio: input timeout", source: "rosout"),
                LogEntry(fileModifiedAt: collectedAt, severity: "info", message: "unrelated service ready", source: "systemd")
            ], source: "NUC Agent"
        )
        let result = evaluate(expectation: configuredExpectation(), statuses: ["/odometry": "stale"], logs: logs)
        let stage = try XCTUnwrap(result.stage(.pointLIO))

        XCTAssertEqual(stage.logs.count, 1)
        XCTAssertTrue(stage.logs[0].message.contains("input timeout"))
        XCTAssertEqual(stage.outputEvidence.first(where: { $0.topic == "/odometry" })?.receiveRateHz, 10)
        XCTAssertEqual(stage.outputEvidence.first(where: { $0.topic == "/odometry" })?.publisherEndpoints, ["point_lio"])
        XCTAssertEqual(stage.outputEvidence.first(where: { $0.topic == "/odometry" })?.workspaceAttribution, "unknown")
    }

    func testRecoveryClearsFindingAndPersistenceTrackerResets() throws {
        var tracker = PipelineDiagnosisTracker()
        let first = tracker.update(evaluate(expectation: configuredExpectation(), statuses: ["/scan": "stale"], collectedAt: collectedAt))
        let second = tracker.update(evaluate(expectation: configuredExpectation(), statuses: ["/scan": "stale"], collectedAt: "2026-10-18T12:00:02.000Z"))

        XCTAssertEqual(first.firstFinding?.observationCount, 1)
        XCTAssertEqual(second.firstFinding?.observationCount, 2)
        XCTAssertEqual(second.firstFinding?.firstObservedAt, collectedAt)

        let recovered = tracker.update(evaluate(expectation: configuredExpectation(), statuses: [:], collectedAt: "2026-10-18T12:00:04.000Z"))
        XCTAssertNil(recovered.firstFinding)
        XCTAssertTrue(recovered.stages.allSatisfy { $0.state == .observed })

        let recurred = tracker.update(evaluate(expectation: configuredExpectation(), statuses: ["/scan": "stale"], collectedAt: "2026-10-18T12:00:06.000Z"))
        XCTAssertEqual(recurred.firstFinding?.observationCount, 1)
        XCTAssertEqual(recurred.firstFinding?.firstObservedAt, "2026-10-18T12:00:06.000Z")
    }

    private func evaluate(
        expectation: PipelineRunExpectation,
        statuses: [String: String],
        nodeNames: [String] = ["/livox_lidar_publisher", "/point_lio", "/pointcloud_to_laserscan", "/slam_toolbox", "/controller_server"],
        actualDomain: Int = 47,
        snapshotGraphAge: Int = 100,
        telemetryGraphAge: Int = 100,
        missingTopics: Set<String> = [],
        agentDomainError: String? = nil,
        agentExpectedDomain: Int = 47,
        collectedAt: String? = nil,
        logs: LogReading? = nil
    ) -> PipelineDiagnosis {
        let timestamp = collectedAt ?? self.collectedAt
        return PipelineDiagnosisEvaluator.evaluate(
            expectation: expectation,
            agentOnline: true,
            snapshot: try! snapshot(domain: actualDomain, graphAge: snapshotGraphAge, nodeNames: nodeNames,
                                    expectedDomain: agentExpectedDomain, expectedDomainError: agentDomainError,
                                    collectedAt: timestamp),
            telemetry: try! telemetry(domain: actualDomain, graphAge: telemetryGraphAge,
                                      expectedDomain: agentExpectedDomain, statuses: statuses,
                                      missingTopics: missingTopics, collectedAt: timestamp,
                                      domainError: agentDomainError),
            logs: logs,
            evaluationTime: Date(timeIntervalSince1970: 1_792_000_000)
        )
    }

    private func configuredExpectation(domain: Int? = 47) -> PipelineRunExpectation {
        PipelineRunExpectation(
            scenario: .originalWorkspaceSingleLidar,
            expectedDomainID: domain,
            expectedNodesByStage: [:],
            navigationGoalActive: false
        )
    }

    private func allUnavailable() -> [String: String] {
        ["/livox/lidar": "unavailable", "/livox/imu": "unavailable", "/imu/data": "unavailable",
         "/odometry": "unavailable", "/cloud_registered": "unavailable", "/terrain_map": "unavailable",
         "/scan": "unavailable", "/map": "unavailable"]
    }

    private func snapshot(
        domain: Int = 47,
        graphAge: Int = 100,
        nodeNames: [String] = ["/livox_lidar_publisher", "/point_lio", "/pointcloud_to_laserscan", "/slam_toolbox", "/controller_server"],
        expectedDomain: Int = 47,
        expectedDomainError: String? = nil,
        collectedAt: String = "2026-10-18T12:00:00.000Z"
    ) throws -> ConsoleSnapshot {
        var value: [String: Any] = [
            "schema_version": 1,
            "collected_at": collectedAt,
            "source": "nuc-read-only-agent",
            "host": ["hostname": "synthetic-ros", "platform": "linux", "os_release": []],
            "system": [
                "cpu": ["status": "ok", "percent": 1],
                "memory": ["status": "unknown", "reason": "synthetic"],
                "disk": ["status": "unknown", "reason": "synthetic"],
                "temperature": ["status": "unknown", "reason": "synthetic"]
            ],
            "git": ["status": "unknown"],
            "deployment": ["status": "unknown"],
            "ros": [
                "status": "ok", "domain_id": domain,
                "nodes": nodeNames.map { ["name": URL(fileURLWithPath: $0).lastPathComponent, "namespace": "/"] },
                "topics": [], "observed_at": collectedAt, "graph_age_ms": graphAge,
                "expected_domain_id": expectedDomain, "expected_domain_error": expectedDomainError as Any? ?? NSNull(),
                "expected_nodes": [], "expected_nodes_error": NSNull(), "discovery": "synthetic", "reason": NSNull()
            ],
            "workspaces": NSNull(),
            "tests": ["status": "unknown", "records": []],
            "safety": ["physical_estop": ["status": "unknown"], "hardware_health": ["status": "unknown"], "control": "read-only"]
        ]
        if expectedDomainError == nil {
            var ros = value["ros"] as! [String: Any]
            ros["expected_domain_error"] = NSNull()
            value["ros"] = ros
        }
        return try decode(value)
    }

    private func telemetry(
        domain: Int,
        graphAge: Int,
        expectedDomain: Int,
        statuses: [String: String],
        missingTopics: Set<String>,
        collectedAt: String,
        domainError: String?
    ) throws -> LinkTelemetryReading {
        let definitions: [(String, Int)] = [
            ("/livox/lidar", 1_500), ("/livox/imu", 1_500), ("/imu/data", 1_500),
            ("/odometry", 2_500), ("/cloud_registered", 2_500), ("/terrain_map", 2_500),
            ("/scan", 2_500), ("/map", 5_000), ("/cmd_vel", 2_500)
        ]
        let streams: [[String: Any]] = definitions.compactMap { topic, staleAfter in
            guard !missingTopics.contains(topic) else { return nil }
            let status = statuses[topic] ?? "streaming"
            let hasRecentSample = status == "streaming" || status == "stale"
            let isStreaming = status == "streaming"
            let age = status == "stale" ? staleAfter + 100 : isStreaming ? 100 : nil
            let receivedAt = age.flatMap { timestamp(collectedAt, minusMilliseconds: $0) }
            let sourceStamp = age.flatMap { timestamp(collectedAt, minusMilliseconds: $0 + 4) }
            let publisherCount = status == "unavailable" ? 0 : 1
            let topicType = topic == "/scan" ? "sensor_msgs/msg/LaserScan"
                : topic == "/map" ? "nav_msgs/msg/OccupancyGrid"
                : topic == "/livox/imu" || topic == "/imu/data" ? "sensor_msgs/msg/Imu"
                : topic == "/odometry" ? "nav_msgs/msg/Odometry"
                : "sensor_msgs/msg/PointCloud2"
            return [
                "id": topic.replacingOccurrences(of: "/", with: "").replacingOccurrences(of: "_", with: "-"),
                "label": topic, "topic": topic,
                "topic_type": topicType, "topic_types": [topicType],
                "status": status, "reason": status == "streaming" ? NSNull() : "synthetic status",
                "publisher_count": publisherCount,
                "publisher_endpoints": publisherCount == 0 ? [] : [["name": "point_lio", "namespace": "/", "topic_type": topicType]],
                "workspace_attribution": "unknown", "subscription_generation": 2,
                "subscription_started_at": timestamp(collectedAt, minusMilliseconds: 10_000) ?? collectedAt,
                "previous_received_at": NSNull(),
                "previous_subscription_generation": NSNull(), "received_at": receivedAt as Any? ?? NSNull(),
                "source_stamp": sourceStamp as Any? ?? NSNull(), "age_ms": age as Any? ?? NSNull(),
                "receive_rate_hz": hasRecentSample && (age ?? 0) <= 5_000 ? 10.0 as Any : NSNull(),
                "header_to_agent_ms": hasRecentSample ? 4.0 as Any : NSNull(),
                "frame_id": hasRecentSample ? "synthetic" as Any : NSNull(),
                "sample_count": hasRecentSample ? 10 as Any : NSNull(), "sample_unit": "points",
                "gap_events": isStreaming ? 0 : 1, "last_gap_ms": isStreaming ? NSNull() : staleAfter + 100,
                "stale_after_ms": staleAfter, "receive_count": hasRecentSample ? 20 : 0
            ]
        }
        var value: [String: Any] = [
            "schema_version": 1, "status": "ok", "source": "nuc-ros2-fixed-topic-read-only-subscriber",
            "collected_at": collectedAt, "domain_id": domain, "expected_domain_id": expectedDomain,
            "expected_domain_error": domainError as Any? ?? NSNull(), "domain_matches_expectation": domain == expectedDomain,
            "graph_status": "ok", "graph_observed_at": collectedAt, "graph_age_ms": graphAge,
            "graph_reason": NSNull(), "workspace_attribution": "unknown",
            "clock_note": "synthetic clock", "streams": streams
        ]
        if domainError == nil { value["expected_domain_error"] = NSNull() }
        return try decode(value)
    }

    private func decode<T: Decodable>(_ value: [String: Any]) throws -> T {
        let data = try JSONSerialization.data(withJSONObject: value)
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(T.self, from: data)
    }

    private func timestamp(_ value: String, minusMilliseconds milliseconds: Int) -> String? {
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        guard let date = formatter.date(from: value) else { return nil }
        return formatter.string(from: date.addingTimeInterval(-Double(milliseconds) / 1_000))
    }
}
