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
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "running", graphRelationship: "shared_domain", expectedNodes: ["/camera_node"]),
            vision: makeVision(rgb: stream(.rgb, status: "streaming", age: 80), depth: stream(.depth, status: "streaming", age: 90), yolo: stream(.yolo, status: "streaming", age: 120))
        )

        XCTAssertEqual(event("agent-connection", in: events).state, .normal)
        XCTAssertEqual(event("ros-graph-freshness", in: events).state, .normal)
        XCTAssertEqual(event("ros-domain", in: events).state, .normal)
        XCTAssertEqual(event("expected-ros-nodes", in: events).state, .normal)
        XCTAssertEqual(event("vision-rgb", in: events).state, .normal)
        XCTAssertEqual(event("vision-depth", in: events).state, .normal)
        XCTAssertEqual(event("vision-yolo", in: events).state, .normal)
        XCTAssertEqual(event("module-camera", in: events).state, .unknown)
        XCTAssertEqual(event("module-camera", in: events).module, "Orbbec Gemini 336L")
        XCTAssertEqual(event("module-camera", in: events).ruleID, "camera.rgb_depth.runtime_and_freshness")
        XCTAssertTrue(event("module-camera", in: events).resource.contains("/camera/color/image_raw"))
        XCTAssertTrue(event("module-camera", in: events).observation.contains("30.00 Hz"))
        XCTAssertTrue(event("module-camera", in: events).observation.contains("相机模块来源未确认"))
        XCTAssertTrue(event("vision-rgb", in: events).observation.contains("只确认 /camera/color/image_raw 数据链路"))
        XCTAssertTrue(event("vision-depth", in: events).observation.contains("只确认 /camera/depth/image_rect_raw 数据链路"))
        XCTAssertEqual(event("module-camera", in: events).observedAt, "2026-10-14T05:00:00.010Z")
        XCTAssertEqual(event("ros-graph-freshness", in: events).observedAt, "2026-10-14T05:00:00.000Z")
        XCTAssertEqual(event("ros-graph-freshness", in: events).source, "NUC Agent · ROS graph observer")
        XCTAssertEqual(event("vision-rgb", in: events).source, "nuc-ros2-read-only-subscriber")
        XCTAssertNotEqual(event("vision-rgb", in: events).observedAt, "未知")
        for item in events {
            XCTAssertFalse(item.module.isEmpty)
            XCTAssertFalse(item.ruleID.isEmpty)
            XCTAssertFalse(item.sourceLocation.isEmpty)
            let expectedSeverity: HealthSeverity
            switch item.state {
            case .normal, .notRunning: expectedSeverity = .informational
            case .unknown: expectedSeverity = .warning
            case .abnormal: expectedSeverity = .error
            }
            XCTAssertEqual(item.severity, expectedSeverity)
        }
    }

    func testUnconfiguredModulesAreNotReportedAsFailures() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "unconfigured", runtimeStatus: "not_running", graphRelationship: "unknown", expectedDomainID: nil, expectedNodes: [], nodes: []),
            vision: makeVision(rgb: stream(.rgb, status: "unavailable", publisherCount: 0), depth: stream(.depth, status: "unavailable", publisherCount: 0), yolo: stream(.yolo, status: "unavailable", publisherCount: 0))
        )

        XCTAssertEqual(event("expected-ros-nodes", in: events).state, .notRunning)
        XCTAssertEqual(event("vision-rgb", in: events).state, .notRunning)
        XCTAssertEqual(event("vision-depth", in: events).state, .notRunning)
        XCTAssertEqual(event("module-camera", in: events).state, .notRunning)
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
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "running", graphRelationship: "shared_domain", graphAgeMs: 1_000),
            vision: makeVision(rgb: stream(.rgb, status: "stale", age: 2_200), depth: stream(.depth, status: "stale", age: 2_800), yolo: stream(.yolo, status: "waiting"))
        )

        XCTAssertEqual(event("ros-graph-freshness", in: events).state, .normal)
        XCTAssertEqual(event("module-camera", in: events).state, .unknown)
        XCTAssertEqual(event("vision-rgb", in: events).state, .unknown)
        XCTAssertEqual(event("vision-depth", in: events).state, .unknown)
        XCTAssertTrue(event("module-camera", in: events).observation.contains("帧龄 2200 ms"))
        XCTAssertTrue(event("module-camera", in: events).observation.contains("相机模块来源未确认"))
        XCTAssertFalse(events.contains { $0.id == "module-camera" && $0.state == .abnormal })
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

    func testCameraStaleFramesRequireConfirmedRunExpectation() throws {
        let staleVision = makeVision(
            rgb: stream(.rgb, status: "stale", age: 2_100),
            depth: stream(.depth, status: "stale", age: 2_300),
            yolo: stream(.yolo, status: "waiting")
        )
        let unconfigured = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "unconfigured", runtimeStatus: "not_running", graphRelationship: "unknown"),
            vision: staleVision
        )
        XCTAssertEqual(event("module-camera", in: unconfigured).state, .unknown)
        XCTAssertEqual(event("vision-rgb", in: unconfigured).state, .unknown)
        XCTAssertFalse(unconfigured.contains { $0.state == .abnormal })

        let expected = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "running", graphRelationship: "shared_domain"),
            vision: staleVision
        )
        XCTAssertEqual(event("module-camera", in: expected).state, .unknown)
        XCTAssertEqual(event("vision-rgb", in: expected).state, .unknown)
        XCTAssertFalse(expected.contains { $0.id == "module-camera" && $0.state == .abnormal })
    }

    func testConfirmedMissingCameraProcessRequiresCompleteProcessScan() throws {
        let noImage = makeVision(
            rgb: stream(.rgb, status: "unavailable", publisherCount: 0),
            depth: stream(.depth, status: "unavailable", publisherCount: 0),
            yolo: stream(.yolo, status: "waiting")
        )
        let complete = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "not_running", graphRelationship: "unknown"),
            vision: noImage
        )
        XCTAssertEqual(event("module-camera", in: complete).state, .abnormal)
        XCTAssertTrue(event("module-camera", in: complete).observation.contains("Agent Graph Domain 42"))
        XCTAssertTrue(event("module-camera", in: complete).observation.contains("显式预期 Domain 42"))

        let noConfiguredDomain = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "not_running", graphRelationship: "unknown", expectedDomainID: nil),
            vision: noImage
        )
        XCTAssertEqual(event("module-camera", in: noConfiguredDomain).state, .unknown)
        XCTAssertFalse(noConfiguredDomain.contains { $0.id == "module-camera" && $0.state == .abnormal })

        let invalidDomainConfiguration = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(
                expectation: "confirmed",
                runtimeStatus: "not_running",
                graphRelationship: "unknown",
                expectedDomainID: nil,
                expectedDomainError: "ROS_DOMAIN_ID must be in 0...232"
            ),
            vision: noImage
        )
        XCTAssertEqual(event("module-camera", in: invalidDomainConfiguration).state, .unknown)
        XCTAssertFalse(invalidDomainConfiguration.contains { $0.id == "module-camera" && $0.state == .abnormal })

        let anotherModuleRunning = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(
                expectation: "confirmed",
                runtimeStatus: "not_running",
                graphRelationship: "shared_domain",
                workspaceProcessPackages: ["yolo_node"]
            ),
            vision: noImage
        )
        XCTAssertEqual(event("module-camera", in: anotherModuleRunning).state, .abnormal)

        let anotherWorkspaceModuleRunning = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(
                expectation: "confirmed",
                runtimeStatus: "not_running",
                graphRelationship: "unknown",
                newWorkspaceProcessPackages: ["yolo_node"]
            ),
            vision: noImage
        )
        XCTAssertEqual(event("module-camera", in: anotherWorkspaceModuleRunning).state, .abnormal)

        let partial = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "not_running", graphRelationship: "unknown", processScanStatus: "partial"),
            vision: noImage
        )
        XCTAssertEqual(event("module-camera", in: partial).state, .unknown)
        XCTAssertFalse(partial.contains { $0.id == "module-camera" && $0.state == .abnormal })
    }

    func testVisionEndpointUnavailableOrIncompleteCannotConfirmMissingPublishers() throws {
        let snapshot = try makeCameraSnapshot(
            expectation: "confirmed",
            runtimeStatus: "not_running",
            graphRelationship: "unknown"
        )
        let unavailable = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: snapshot,
            vision: nil
        )
        XCTAssertEqual(event("module-camera", in: unavailable).state, .unknown)
        XCTAssertTrue(event("module-camera", in: unavailable).observation.contains("/v1/vision 不可用"))
        XCTAssertFalse(event("module-camera", in: unavailable).observation.contains("Topic 均无发布者"))
        XCTAssertFalse(unavailable.contains { $0.id == "module-camera" && $0.state == .abnormal })

        let completeButEndpointUnavailable = makeVision(
            rgb: stream(.rgb, status: "unavailable", publisherCount: 0),
            depth: stream(.depth, status: "unavailable", publisherCount: 0),
            yolo: stream(.yolo, status: "waiting")
        )
        let endpointUnavailable = VisionReading(
            status: "unavailable",
            source: completeButEndpointUnavailable.source,
            collectedAt: completeButEndpointUnavailable.collectedAt,
            reason: "visual observer disconnected",
            streams: completeButEndpointUnavailable.streams
        )
        let endpointOffline = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: snapshot,
            vision: endpointUnavailable
        )
        XCTAssertEqual(event("module-camera", in: endpointOffline).state, .unknown)
        XCTAssertFalse(event("module-camera", in: endpointOffline).observation.contains("Topic 均无发布者"))
        XCTAssertFalse(endpointOffline.contains { $0.id == "module-camera" && $0.state == .abnormal })

        let complete = makeVision(
            rgb: stream(.rgb, status: "unavailable", publisherCount: 0),
            depth: stream(.depth, status: "unavailable", publisherCount: 0),
            yolo: stream(.yolo, status: "waiting")
        )
        let incomplete = VisionReading(
            status: "ok",
            source: complete.source,
            collectedAt: complete.collectedAt,
            reason: nil,
            streams: [complete.stream(.rgb)!]
        )
        let missingDepth = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: snapshot,
            vision: incomplete
        )
        XCTAssertEqual(event("module-camera", in: missingDepth).state, .unknown)
        XCTAssertFalse(event("module-camera", in: missingDepth).observation.contains("Topic 均无发布者"))
        XCTAssertFalse(missingDepth.contains { $0.id == "module-camera" && $0.state == .abnormal })
    }

    func testOldAgentWithoutVisionOrWorkspaceTelemetryKeepsCameraUnknown() throws {
        let oldAgentSnapshot = try makeSnapshot()
        XCTAssertNil(oldAgentSnapshot.workspaces)

        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "旧版 Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: 42,
            snapshot: oldAgentSnapshot,
            vision: nil
        )

        XCTAssertEqual(event("module-camera", in: events).state, .unknown)
        XCTAssertFalse(events.contains { $0.id == "module-camera" && $0.state == .abnormal })
    }

    func testCameraDomainMismatchAndExpiredGraphSuppressModuleFinding() throws {
        let vision = makeVision(
            rgb: stream(.rgb, status: "stale", age: 2_100),
            depth: stream(.depth, status: "stale", age: 2_200),
            yolo: stream(.yolo, status: "waiting")
        )
        let mismatch = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "running", graphRelationship: "shared_domain", domainID: 17, expectedDomainID: 42),
            vision: vision
        )
        XCTAssertEqual(event("module-camera", in: mismatch).state, .unknown)
        XCTAssertFalse(mismatch.contains { $0.id == "module-camera" && $0.state == .abnormal })

        let expired = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "running", graphRelationship: "shared_domain", graphAgeMs: 6_000),
            vision: vision
        )
        XCTAssertEqual(event("module-camera", in: expired).state, .unknown)
        XCTAssertFalse(expired.contains { $0.id == "module-camera" && $0.state == .abnormal })

        let workspaceDomainMismatch = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(
                expectation: "confirmed",
                runtimeStatus: "not_running",
                graphRelationship: "different_domain",
                expectedDomainID: nil,
                workspaceProcessPackages: ["yolo_node"],
                workspaceProcessDomainIDs: [17]
            ),
            vision: makeVision(
                rgb: stream(.rgb, status: "unavailable", publisherCount: 0),
                depth: stream(.depth, status: "unavailable", publisherCount: 0),
                yolo: stream(.yolo, status: "waiting")
            )
        )
        XCTAssertEqual(event("module-camera", in: workspaceDomainMismatch).state, .unknown)
        XCTAssertFalse(workspaceDomainMismatch.contains { $0.id == "module-camera" && $0.state == .abnormal })

        let workspaceDomainUnknown = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "not_running", graphRelationship: "unknown", expectedDomainID: nil),
            vision: makeVision(
                rgb: stream(.rgb, status: "unavailable", publisherCount: 0),
                depth: stream(.depth, status: "unavailable", publisherCount: 0),
                yolo: stream(.yolo, status: "waiting")
            )
        )
        XCTAssertEqual(event("module-camera", in: workspaceDomainUnknown).state, .unknown)
        XCTAssertFalse(workspaceDomainUnknown.contains { $0.id == "module-camera" && $0.state == .abnormal })
    }

    func testSharedCameraTopicAcrossWorkspacesDoesNotImplyTargetSource() throws {
        let otherWorkspaceCameraOnly = try makeCameraSnapshot(
            expectation: "confirmed",
            runtimeStatus: "not_running",
            graphRelationship: "unknown",
            newWorkspaceProcessPackages: ["orbbec_camera"]
        )
        XCTAssertEqual(otherWorkspaceCameraOnly.workspaces?.workspaces?.count, 3)
        XCTAssertEqual(otherWorkspaceCameraOnly.workspaces?.workspaces?.first(where: { $0.id == "new-engineering" })?.modules?.first(where: { $0.id == "camera" })?.interfaceTopics,
                       ["/camera/color/image_raw", "/camera/depth/image_rect_raw", "/camera/depth/color/points", "/camera/color/camera_info"])

        let duplicateWorkspaceEvidence = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: otherWorkspaceCameraOnly,
            vision: makeVision(rgb: stream(.rgb, status: "streaming", age: 30), depth: stream(.depth, status: "streaming", age: 35), yolo: stream(.yolo, status: "waiting"))
        )
        XCTAssertEqual(event("module-camera", in: duplicateWorkspaceEvidence).state, .unknown)
        XCTAssertTrue(event("module-camera", in: duplicateWorkspaceEvidence).observation.contains("另一工作区"))
        XCTAssertEqual(event("vision-rgb", in: duplicateWorkspaceEvidence).state, .normal)
        XCTAssertTrue(event("vision-rgb", in: duplicateWorkspaceEvidence).observation.contains("只确认 /camera/color/image_raw 数据链路"))

        let duplicatePublishers = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "running", graphRelationship: "shared_domain"),
            vision: makeVision(
                rgb: stream(.rgb, status: "streaming", age: 30, publisherCount: 2),
                depth: stream(.depth, status: "streaming", age: 35, publisherCount: 2),
                yolo: stream(.yolo, status: "waiting")
            )
        )
        XCTAssertEqual(event("module-camera", in: duplicatePublishers).state, .unknown)
        XCTAssertTrue(event("module-camera", in: duplicatePublishers).observation.contains("多个发布者"))
    }

    func testInvalidModuleExpectationKeepsCameraStateUnknown() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(
                expectation: "unknown",
                runtimeStatus: "not_running",
                graphRelationship: "unknown",
                expectationStatus: "invalid"
            ),
            vision: makeVision(
                rgb: stream(.rgb, status: "stale", age: 2_200),
                depth: stream(.depth, status: "unavailable", publisherCount: 0),
                yolo: stream(.yolo, status: "waiting")
            )
        )

        XCTAssertEqual(event("module-camera", in: events).state, .unknown)
        XCTAssertFalse(events.contains { $0.state == .abnormal })
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

        let notStarted = try decodeWorkspaceInventory(runExpectation: "unconfigured", runtimeStatus: "not_running")
        XCTAssertTrue(notStarted.hasUnconfirmedHealth)
        XCTAssertEqual(
            HealthSummaryState.resolve([normal], workspaceInventory: notStarted, requireWorkspaceTelemetry: true),
            .unknown,
            "A not-started workspace with no confirmed run expectation must not be shown as healthy or faulty."
        )

        let confirmed = try decodeWorkspaceInventory(runExpectation: "confirmed")
        XCTAssertFalse(confirmed.hasUnconfirmedHealth)
        XCTAssertEqual(
            HealthSummaryState.resolve([normal], workspaceInventory: confirmed, requireWorkspaceTelemetry: true),
            .normal
        )
    }

    private func decodeWorkspaceInventory(runExpectation: String, runtimeStatus: String = "running") throws -> WorkspaceInventoryReading {
        let json = #"{"status":"ok","workspaces":[{"id":"original-navigation","label":"原始导航与雷达工程","source_kind":"original","source_status":"present","runtime_status":"\#(runtimeStatus)","run_expectation":"\#(runExpectation)","graph_relationship":"shared_domain","modules":[{"id":"navigation","label":"导航","source_status":"present","runtime_status":"\#(runtimeStatus)","run_expectation":"\#(runExpectation)"}]}]}"#
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
        let missingProcessSnapshot = try makeCameraSnapshot(
            expectation: "confirmed",
            runtimeStatus: "not_running",
            graphRelationship: "unknown"
        )
        let runningProcessSnapshot = try makeCameraSnapshot(
            expectation: "confirmed",
            runtimeStatus: "running",
            graphRelationship: "shared_domain"
        )

        let firstRaw = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: missingProcessSnapshot,
            vision: makeVision(
                rgb: stream(.rgb, status: "unavailable", publisherCount: 0),
                depth: stream(.depth, status: "unavailable", publisherCount: 0),
                yolo: stream(.yolo, status: "waiting"),
                collectedAt: t0
            )
        )
        let first = event("module-camera", in: tracker.update(firstRaw))
        XCTAssertEqual(first.persistence, .singleSnapshot)
        XCTAssertEqual(first.observationCount, 1)

        let secondRaw = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: missingProcessSnapshot,
            vision: makeVision(
                rgb: stream(.rgb, status: "unavailable", publisherCount: 0),
                depth: stream(.depth, status: "unavailable", publisherCount: 0),
                yolo: stream(.yolo, status: "waiting"),
                collectedAt: t1
            )
        )
        let second = event("module-camera", in: tracker.update(secondRaw))
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
            snapshot: runningProcessSnapshot,
            vision: makeVision(
                rgb: stream(.rgb, status: "streaming", age: 80),
                depth: stream(.depth, status: "streaming", age: 40),
                yolo: stream(.yolo, status: "waiting"),
                collectedAt: t2
            )
        )
        let recovered = event("module-camera", in: tracker.update(recoveredRaw))
        XCTAssertEqual(recovered.state, .unknown)
        XCTAssertTrue(recovered.observation.contains("相机模块来源未确认"))
        XCTAssertEqual(recovered.persistence, .singleSnapshot)
        XCTAssertEqual(recovered.observationCount, 1)

        let recurredRaw = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: missingProcessSnapshot,
            vision: makeVision(
                rgb: stream(.rgb, status: "unavailable", publisherCount: 0),
                depth: stream(.depth, status: "unavailable", publisherCount: 0),
                yolo: stream(.yolo, status: "waiting"),
                collectedAt: t3
            )
        )
        let recurred = event("module-camera", in: tracker.update(recurredRaw))
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

    func testYOLOUnsupportedOutputWithoutConfirmedExpectationIsUnknown() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeSnapshot(),
            vision: makeVision(
                rgb: stream(.rgb, status: "streaming", age: 30),
                depth: stream(.depth, status: "streaming", age: 30),
                yolo: stream(.yolo, status: "unsupported", topicType: "std_msgs/msg/String")
            )
        )

        XCTAssertEqual(event("vision-yolo", in: events).state, .unknown)
        XCTAssertFalse(events.contains { $0.id == "vision-yolo" && $0.state == .abnormal })
    }

    func testMultipleIndependentAnomaliesCanBeReportedTogether() throws {
        let events = HealthEvaluator.evaluate(
            agentOnline: true,
            hasEverConnectedToAgent: true,
            agentMessage: "Agent 在线",
            lastSuccessfulRead: nil,
            lastKnownDomainID: nil,
            snapshot: try makeCameraSnapshot(expectation: "confirmed", runtimeStatus: "not_running", graphRelationship: "unknown", domainID: 42, expectedDomainID: 42, expectedNodes: ["/camera_node", "/lidar_node"], nodes: ["/camera_node"]),
            vision: makeVision(
                rgb: stream(.rgb, status: "unavailable", publisherCount: 0),
                depth: stream(.depth, status: "unavailable", publisherCount: 0),
                yolo: stream(.yolo, status: "waiting")
            )
        )

        let abnormalIDs = Set(events.filter { $0.state == .abnormal }.map(\.id))
        XCTAssertTrue(abnormalIDs.isSuperset(of: ["expected-ros-nodes", "module-camera"]))
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
        expectedNodes: [String] = [],
        nodes: [String]? = nil,
        graphAgeMs: Int? = 1_000,
        collectedAt: String = "2026-10-14T05:00:00.000Z",
        workspaceData: [String: Any]? = nil,
        graphTopics: [String] = []
    ) throws -> ConsoleSnapshot {
        var ros: [String: Any] = [
            "status": "ok",
            "nodes": nodes?.map(nodeValue) ?? [],
            "topics": graphTopics.map { name in
                ["name": name, "types": [topicType(name)]]
            },
            "observed_at": "2026-10-14T05:00:00.000Z",
            "expected_nodes": expectedNodes
        ]
        ros["domain_id"] = domainID ?? NSNull()
        ros["expected_domain_id"] = expectedDomainID ?? NSNull()
        ros["graph_age_ms"] = graphAgeMs ?? NSNull()
        if let expectedDomainError {
            ros["expected_domain_error"] = expectedDomainError
        }
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
            "ros": ros,
            "tests": ["status": "unknown", "records": []],
            "safety": [
                "physical_estop": ["status": "unknown", "reason": "not connected"],
                "hardware_health": ["status": "unknown", "reason": "no hardware evidence"],
                "control": "disabled"
            ]
        ]
        if let workspaceData {
            value["workspaces"] = workspaceData
        }
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(ConsoleSnapshot.self, from: JSONSerialization.data(withJSONObject: value))
    }

    private func makeCameraSnapshot(
        expectation: String,
        runtimeStatus: String,
        graphRelationship: String,
        domainID: Int? = 42,
        expectedDomainID: Int? = 42,
        expectedNodes: [String] = [],
        nodes: [String]? = nil,
        graphAgeMs: Int? = 1_000,
        processScanStatus: String = "ok",
        expectationStatus: String? = nil,
        expectedDomainError: String? = nil,
        workspaceProcessPackages: [String]? = nil,
        workspaceProcessDomainIDs: [Int?]? = nil,
        newWorkspaceProcessPackages: [String] = []
    ) throws -> ConsoleSnapshot {
        let observedAt = "2026-10-14T05:00:00.000Z"
        let effectiveExpectationStatus = expectationStatus ?? (expectation == "confirmed" ? "configured" : (expectation == "unknown" ? "invalid" : "unconfigured"))
        let effectiveProcessPackages = workspaceProcessPackages ?? (runtimeStatus == "running" ? ["orbbec_camera"] : [])
        let effectiveProcessDomains = workspaceProcessDomainIDs ?? effectiveProcessPackages.map { _ in domainID }
        let cameraRuntime = moduleRuntimeStatus(package: "orbbec_camera", processes: effectiveProcessPackages, scanStatus: processScanStatus)
        let workspaceRuntime = workspaceRuntimeStatus(processes: effectiveProcessPackages, scanStatus: processScanStatus)
        var graphTopics: [String] = []
        if effectiveProcessPackages.contains("orbbec_camera") || newWorkspaceProcessPackages.contains("orbbec_camera") {
            graphTopics += ["/camera/color/image_raw", "/camera/depth/image_rect_raw"]
        }
        if effectiveProcessPackages.contains("yolo_node") || newWorkspaceProcessPackages.contains("yolo_node") {
            graphTopics += ["/yolo/detections", "/yolo/annotated_image"]
        }
        let inventoryExpectationStatus = expectationStatus ?? effectiveExpectationStatus
        let targetWorkspace = agentWorkspace(
            id: "original-arm-vision",
            label: "原始机械臂与视觉工程",
            sourceKind: "original",
            root: "/home/tzb/tzb_ws",
            packages: ["orbbec_camera", "yolo_node", "pose_estimator", "lite_motion_planner", "rm_serial_driver"],
            processPackages: effectiveProcessPackages,
            processDomainIDs: effectiveProcessDomains,
            processScanStatus: processScanStatus,
            runtimeStatus: workspaceRuntime,
            graphRelationship: graphRelationship,
            graphDomainID: domainID,
            graphAgeMs: graphAgeMs,
            graphTopics: graphTopics,
            observedAt: observedAt,
            expectationStatus: inventoryExpectationStatus,
            cameraExpectation: expectation,
            cameraRuntimeStatus: cameraRuntime
        )
        let newWorkspaceRuntime = workspaceRuntimeStatus(processes: newWorkspaceProcessPackages, scanStatus: processScanStatus)
        let newWorkspaceCameraRuntime = moduleRuntimeStatus(package: "orbbec_camera", processes: newWorkspaceProcessPackages, scanStatus: processScanStatus)
        let newWorkspaceDomainIDs = newWorkspaceProcessPackages.map { _ in domainID }
        let newWorkspaceRelationship = newWorkspaceProcessPackages.isEmpty
            ? "unknown"
            : (processScanStatus == "ok" && newWorkspaceDomainIDs.allSatisfy { $0 == domainID } ? "shared_domain" : "unknown")
        let newWorkspace = agentWorkspace(
            id: "new-engineering",
            label: "新版工程",
            sourceKind: "new",
            root: "/home/tzb/lunar-robot/code",
            packages: ["orbbec_camera", "yolo_node", "pose_estimator", "lite_motion_planner", "rm_serial_driver", "livox_ros_driver2", "point_lio", "fast_lio", "dlio", "bringup", "rm_navigation", "chassis_serial_bridge"],
            processPackages: newWorkspaceProcessPackages,
            processDomainIDs: newWorkspaceDomainIDs,
            processScanStatus: processScanStatus,
            runtimeStatus: newWorkspaceRuntime,
            graphRelationship: newWorkspaceRelationship,
            graphDomainID: domainID,
            graphAgeMs: graphAgeMs,
            graphTopics: graphTopics,
            observedAt: observedAt,
            expectationStatus: inventoryExpectationStatus,
            cameraExpectation: "unconfigured",
            cameraRuntimeStatus: newWorkspaceCameraRuntime
        )
        let navWorkspace = agentWorkspace(
            id: "original-navigation",
            label: "原始导航与雷达工程",
            sourceKind: "original",
            root: "/home/tzb/Nav2_rea_ws",
            packages: ["livox_ros_driver2", "point_lio", "fast_lio", "dlio", "bringup", "rm_navigation", "behavior_ext_plugins", "rm_serial_driver"],
            processPackages: [],
            processDomainIDs: [],
            processScanStatus: processScanStatus,
            runtimeStatus: workspaceRuntimeStatus(processes: [], scanStatus: processScanStatus),
            graphRelationship: "unknown",
            graphDomainID: domainID,
            graphAgeMs: graphAgeMs,
            graphTopics: graphTopics,
            observedAt: observedAt,
            expectationStatus: inventoryExpectationStatus,
            cameraExpectation: "unconfigured",
            cameraRuntimeStatus: "unknown",
            includeCamera: false
        )
        let workspaceData: [String: Any] = [
            "status": processScanStatus == "unknown" ? "unknown" : "ok",
            "source": "nuc-read-only-workspace-and-process-observer",
            "collected_at": observedAt,
            "expectation_status": effectiveExpectationStatus,
            "expectation_source": effectiveExpectationStatus == "configured" ? "LUNAR_CONSOLE_EXPECTED_MODULES" : NSNull() as Any,
            "expectation_observed_at": observedAt,
            "expectation_error": effectiveExpectationStatus == "invalid" ? "LUNAR_CONSOLE_EXPECTED_MODULES contains an invalid entry" : NSNull() as Any,
            "process_scan": [
                "status": processScanStatus,
                "scope": "Agent UID; process identity and workspace references",
                "source": "/proc",
                "observed_at": observedAt,
                "reason": processScanStatus == "ok" ? NSNull() as Any : "部分 Agent UID 进程信息不可读"
            ],
            "graph_source": [
                "domain_id": domainID ?? NSNull() as Any,
                "observed_at": observedAt,
                "graph_age_ms": graphAgeMs ?? NSNull() as Any,
                "source": "existing Agent ROS graph observer; no additional ROS participant is started"
            ],
            "workspaces": [newWorkspace, targetWorkspace, navWorkspace]
        ]
        let effectiveNodes = nodes ?? ((effectiveProcessPackages.contains("orbbec_camera") || newWorkspaceProcessPackages.contains("orbbec_camera")) ? ["/camera_node"] : [])
        return try makeSnapshot(
            domainID: domainID,
            expectedDomainID: expectedDomainID,
            expectedDomainError: expectedDomainError,
            expectedNodes: expectedNodes,
            nodes: effectiveNodes,
            graphAgeMs: graphAgeMs,
            collectedAt: observedAt,
            workspaceData: workspaceData,
            graphTopics: graphTopics
        )
    }

    private func moduleRuntimeStatus(package: String, processes: [String], scanStatus: String) -> String {
        if processes.contains(package) { return "running" }
        return scanStatus == "ok" ? "not_running" : "unknown"
    }

    private func workspaceRuntimeStatus(processes: [String], scanStatus: String) -> String {
        if !processes.isEmpty { return "running" }
        return scanStatus == "ok" ? "not_running" : "unknown"
    }

    private func agentProcess(pid: Int, package: String, domainID: Int?, observedAt: String) -> [String: Any] {
        [
            "pid": pid,
            "process_name": package,
            "package": package,
            "domain_id": domainID ?? NSNull() as Any,
            "domain_evidence": domainID == nil ? "unset" : "observed",
            "localhost_only": "1",
            "run_mode": "unknown",
            "run_mode_evidence": "unknown",
            "matched_by": ["overlay_environment"],
            "observed_at": observedAt
        ]
    }

    private func agentModule(
        id: String,
        label: String,
        packages: [String],
        runtimeStatus: String,
        runExpectation: String,
        interfaceTopics: [String],
        observedTopics: [String],
        observedAt: String
    ) -> [String: Any] {
        [
            "id": id,
            "label": label,
            "packages": packages,
            "source_status": "present",
            "runtime_status": runtimeStatus,
            "execution_state": runtimeStatus == "not_running" ? "not_running" : "unknown",
            "run_expectation": runExpectation,
            "run_expectation_source": runExpectation == "confirmed" ? "LUNAR_CONSOLE_EXPECTED_MODULES" : NSNull() as Any,
            "run_expectation_observed_at": observedAt,
            "run_expectation_reason": runExpectation == "confirmed" ? "由显式 Agent 配置声明为当前运行模式的预期模块。" : "未配置此模块的运行预期。",
            "interface_topics": interfaceTopics,
            "observed_interface_topics": observedTopics,
            "reason": "源码接口候选不代表运行时存在；运行预期仅来自显式配置。"
        ]
    }

    private func agentWorkspace(
        id: String,
        label: String,
        sourceKind: String,
        root: String,
        packages: [String],
        processPackages: [String],
        processDomainIDs: [Int?],
        processScanStatus: String,
        runtimeStatus: String,
        graphRelationship: String,
        graphDomainID: Int?,
        graphAgeMs: Int?,
        graphTopics: [String],
        observedAt: String,
        expectationStatus: String,
        cameraExpectation: String,
        cameraRuntimeStatus: String,
        includeCamera: Bool = true
    ) -> [String: Any] {
        let packagePaths = packages.map { ["name": $0, "path": "src/\($0)/package.xml"] }
        let pidBase = id == "new-engineering" ? 5_000 : (id == "original-arm-vision" ? 6_000 : 7_000)
        let processRecords = processPackages.enumerated().map { index, package in
            agentProcess(
                pid: pidBase + index,
                package: package,
                domainID: index < processDomainIDs.count ? processDomainIDs[index] : nil,
                observedAt: observedAt
            )
        }
        let discoveredDomains = Array(Set(processDomainIDs.compactMap { $0 })).sorted()
        var modules: [[String: Any]] = []
        for spec in agentModuleSpecs(for: id) where packages.contains(where: { spec.packages.contains($0) }) && (includeCamera || spec.id != "camera") {
            let status: String
            if spec.id == "camera" {
                status = cameraRuntimeStatus
            } else if processPackages.contains(where: { spec.packages.contains($0) }) {
                status = "running"
            } else {
                status = processScanStatus == "ok" ? "not_running" : "unknown"
            }
            let expectation: String
            if expectationStatus == "invalid" {
                expectation = "unknown"
            } else if id == "original-arm-vision" && spec.id == "camera" {
                expectation = cameraExpectation
            } else {
                expectation = "unconfigured"
            }
            modules.append(agentModule(
                id: spec.id, label: spec.label, packages: spec.packages, runtimeStatus: status,
                runExpectation: expectation, interfaceTopics: spec.topics,
                observedTopics: graphRelationship == "shared_domain" ? spec.topics.filter { graphTopics.contains($0) } : [], observedAt: observedAt
            ))
        }
        let runtimeValue: Any = runtimeStatus
        return [
            "id": id,
            "label": label,
            "source_kind": sourceKind,
            "root": root,
            "source_status": "present",
            "source_observed_at": observedAt,
            "package_count": packagePaths.count,
            "packages": packagePaths,
            "launch_count": 0,
            "launch_files": [],
            "config_count": 0,
            "config_files": [],
            "runtime_status": runtimeValue,
            "runtime_scope": "Agent UID 可见进程；/proc 与工作区路径只读检查",
            "runtime_observed_at": observedAt,
            "process_count": processRecords.count,
            "processes": processRecords,
            "domain_ids": discoveredDomains,
            "run_mode": "unknown",
            "execution_state": runtimeStatus == "not_running" ? "not_running" : "unknown",
            "run_expectation": "unconfigured",
            "graph_domain_id": graphDomainID ?? NSNull() as Any,
            "graph_observed_at": observedAt,
            "graph_relationship": graphRelationship,
            "graph_reason": graphRelationship == "shared_domain" ? "进程 Domain 与 Agent 当前观测 Domain \(graphDomainID ?? -1) 相同。" : "Agent Graph 与此工作区的 Domain 关系尚未确认。",
            "modules": modules,
            "reason": NSNull() as Any
        ]
    }

    private func agentModuleSpecs(for workspaceID: String) -> [(id: String, label: String, packages: [String], topics: [String])] {
        let camera = ("camera", "相机", ["orbbec_camera"], ["/camera/color/image_raw", "/camera/depth/image_rect_raw", "/camera/depth/color/points", "/camera/color/camera_info"])
        let yolo = ("yolo", "YOLO", ["yolo_node"], ["/camera/color/image_raw", "/yolo/detections", "/yolo/annotated_image"])
        let pose = ("pose", "位姿估计", ["pose_estimator"], ["/camera/depth/color/points", "/camera/color/camera_info", "/yolo/detections", "/pose_estimator/object_pose"])
        let arm = ("arm", "机械臂", ["lite_motion_planner", "rm_serial_driver"], ["/serial/arm_joint_state", "/tzb_catch/arm_joint_command", "/tzb_catch/gripper_command"])
        let lidar = ("lidar", "雷达与定位", ["livox_ros_driver2", "point_lio", "fast_lio", "dlio"], ["/livox/lidar", "/imu/data", "/terrain_map"])
        let navigation = ("navigation", "导航", ["bringup", "rm_navigation"], ["/scan", "/map", "/cmd_vel"])
        let chassis = ("chassis", "底盘", ["chassis_serial_bridge"], ["/cmd_vel", "/chassis/serial_link_status"])
        let originalNavigation = ("navigation", "Nav2 导航", ["bringup", "rm_navigation", "behavior_ext_plugins"], ["/scan", "/map", "/cmd_vel", "/cmd_vel_chassis"])
        let originalChassis = ("chassis", "底盘", ["rm_serial_driver"], ["/cmd_vel_chassis"])
        switch workspaceID {
        case "new-engineering": return [camera, yolo, pose, arm, lidar, navigation, chassis]
        case "original-arm-vision": return [camera, yolo, pose, arm]
        case "original-navigation": return [lidar, originalNavigation, originalChassis]
        default: return []
        }
    }

    private func nodeValue(_ value: String) -> [String: String] {
        let components = value.split(separator: "/").map(String.init)
        return [
            "name": components.last ?? "node",
            "namespace": components.count > 1 ? "/" + components.dropLast().joined(separator: "/") : "/"
        ]
    }

    private func topicType(_ name: String) -> String {
        if name.hasSuffix("/detections") { return "vision_msgs/msg/Detection2DArray" }
        if name.hasSuffix("/image_raw") || name.hasSuffix("/annotated_image") { return "sensor_msgs/msg/Image" }
        return "sensor_msgs/msg/PointCloud2"
    }

    private func stream(_ id: VisionStreamID, status: String, age: Int? = nil, publisherCount: Int = 1, frameID: String? = nil, topicType: String? = nil) -> VisionStreamReading {
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
            topicType: publisherCount == 0 ? nil : (topicType ?? "sensor_msgs/msg/Image"),
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
