import Foundation
import XCTest
@testable import LunarRobotConsole

final class RobotTopologyTests: XCTestCase {
    func testCatalogHasUniqueResolvableGroupsEdgesAndFlows() {
        let nodeIDs = RobotTopologyCatalog.nodes.map(\.id)
        let edgeIDs = RobotTopologyCatalog.edges.map(\.id)
        XCTAssertEqual(Set(nodeIDs).count, nodeIDs.count)
        XCTAssertEqual(Set(edgeIDs).count, edgeIDs.count)

        for group in RobotTopologyCatalog.groups {
            for id in group.nodeIDs {
                XCTAssertNotNil(RobotTopologyCatalog.node(id), "Unknown node \(id) in group \(group.id)")
            }
        }
        for edge in RobotTopologyCatalog.edges {
            XCTAssertNotNil(RobotTopologyCatalog.node(edge.sourceNodeID))
            XCTAssertNotNil(RobotTopologyCatalog.node(edge.destinationNodeID))
        }
        for flow in RobotTopologyCatalog.flows {
            for id in flow.nodeIDs { XCTAssertNotNil(RobotTopologyCatalog.node(id)) }
            for id in flow.edgeIDs { XCTAssertNotNil(RobotTopologyCatalog.edge(id)) }
        }

        let chassis = try! XCTUnwrap(RobotTopologyCatalog.node("chassis-bridge"))
        XCTAssertTrue(chassis.sourceNote.contains("没有 STM32 acknowledgement"))
        XCTAssertTrue(RobotTopologyCatalog.edge("arm-driver-mcu")?.sourceNote.contains("未由 Agent 验证") == true)
        XCTAssertEqual(RobotTopologyCatalog.edge("mission-nav2")?.evidence, .openInterface)
    }

    func testGraphTopicNameIsObservationNotProofOfEndpointConnection() throws {
        let context = try makeContext(topics: ["/camera/color/image_raw"])
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.edgeStatus("camera-yolo").signal, .topicObserved)
        XCTAssertTrue(evaluation.edgeStatus("camera-yolo").note.contains("不提供该 Topic 的发布 / 订阅两端归属"))
        XCTAssertEqual(evaluation.edgeStatus("mission-nav2").signal, .unknown)
        XCTAssertTrue(evaluation.edgeStatus("mission-nav2").note.contains("主线没有纳入任务协调器"))
    }

    func testDomainMismatchBlocksNodeAndLinkRuntimeClaims() throws {
        let context = try makeContext(
            domainID: 7,
            expectedDomainID: 42,
            topics: ["/camera/color/image_raw"],
            events: [healthEvent("vision-rgb", state: .normal, resource: "/camera/color/image_raw")]
        )
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.nodeStatus("camera").signal, .unknown)
        XCTAssertEqual(evaluation.nodeStatus("yolo").signal, .unknown)
        XCTAssertEqual(evaluation.edgeStatus("camera-yolo").signal, .unknown)
        XCTAssertTrue(evaluation.edgeStatus("camera-yolo").note.contains("Domain 不匹配"))
    }

    func testStaleGraphBlocksDownstreamStateEvenWhenTopicIsListed() throws {
        let context = try makeContext(
            graphAgeMs: 6_500,
            topics: ["/camera/color/image_raw"],
            events: [healthEvent("vision-rgb", state: .normal, resource: "/camera/color/image_raw")]
        )
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.nodeStatus("yolo").signal, .unknown)
        XCTAssertEqual(evaluation.edgeStatus("camera-yolo").signal, .unknown)
        XCTAssertTrue(evaluation.edgeStatus("camera-yolo").note.contains("过期"))
    }

    func testDisconnectedAgentKeepsHistoricalSnapshotOutOfLiveEvaluation() throws {
        let context = try makeContext(
            agentOnline: false,
            workspaceFreshness: .historical,
            includeUnconfiguredStoppedLidar: true,
            events: [
                healthEvent("agent-connection", state: .abnormal, resource: "SSH / Agent"),
                healthEvent("module-camera", state: .normal, resource: "/camera/color/image_raw")
            ]
        )
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.nodeStatus("agent").signal, .abnormal)
        XCTAssertEqual(evaluation.nodeStatus("camera").signal, .unknown)
        XCTAssertEqual(evaluation.nodeStatus("lidar-stack").signal, .unknown)
        XCTAssertTrue(evaluation.nodeStatus("lidar-stack").note.contains("ROS 运行状态未知"))
    }

    func testDirectCameraAnomalyMarksPossibleDownstreamWithoutPropagatingFault() throws {
        let context = try makeContext(
            events: [healthEvent("module-camera", state: .abnormal, resource: "/camera/color/image_raw")]
        )
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.nodeStatus("camera").signal, .abnormal)
        XCTAssertEqual(evaluation.nodeStatus("yolo").signal, .unknown)
        XCTAssertTrue(evaluation.nodeStatus("yolo").possiblyAffectedDownstream)
        XCTAssertTrue(evaluation.edgeStatus("camera-yolo").possiblyAffectedDownstream)
        XCTAssertFalse(evaluation.possiblyAffectedNodeIDs.contains("mission-coordinator"), "Unconnected open interfaces must not propagate impact.")
        XCTAssertFalse(evaluation.possiblyAffectedNodeIDs.contains("arm-planner"))
    }

    func testRgbStreamDoesNotConfirmCameraModuleSource() throws {
        let context = try makeContext(
            topics: ["/camera/color/image_raw"],
            events: [healthEvent("vision-rgb", state: .normal, resource: "/camera/color/image_raw")]
        )
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.nodeStatus("camera").signal, .unknown)
        XCTAssertTrue(evaluation.nodeStatus("camera").note.contains("来源尚未确认"))
        XCTAssertEqual(evaluation.edgeStatus("camera-yolo").signal, .topicObserved)
    }

    func testUnknownOrNotObservedElementsKeepOverallSummaryOutOfGreen() {
        let unknown = TopologyEvaluation(
            nodeStatuses: ["camera": TopologyElementStatus(signal: .unknown, note: "source unknown", directEvents: [], possiblyAffectedDownstream: false)],
            edgeStatuses: ["camera-yolo": TopologyElementStatus(signal: .topicObserved, note: "name only", directEvents: [], possiblyAffectedDownstream: false)],
            possiblyAffectedNodeIDs: []
        )
        XCTAssertEqual(TopologyOverallStatus.signal(for: unknown), .unknown)

        let abnormal = TopologyEvaluation(
            nodeStatuses: ["camera": TopologyElementStatus(signal: .abnormal, note: "direct event", directEvents: [], possiblyAffectedDownstream: false)],
            edgeStatuses: [:],
            possiblyAffectedNodeIDs: []
        )
        XCTAssertEqual(TopologyOverallStatus.signal(for: abnormal), .abnormal)
    }

    func testLegacyAgentWithoutWorkspaceOrGraphAgeRemainsUnknown() throws {
        let context = try makeContext(graphAgeMs: nil, includeUnconfiguredStoppedLidar: false)
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertNil(context.snapshot?.workspaces)
        XCTAssertEqual(evaluation.nodeStatus("camera").signal, .unknown)
        XCTAssertEqual(evaluation.nodeStatus("nav2").signal, .unknown)
        XCTAssertEqual(evaluation.edgeStatus("scan-nav2").signal, .unknown)
    }

    func testUnconfiguredMissingAggregateProcessIsNotAnAbnormalEvent() throws {
        let context = try makeContext(includeUnconfiguredStoppedLidar: true)
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.nodeStatus("lidar-stack").signal, .processNotFound)
        XCTAssertTrue(evaluation.nodeStatus("lidar-stack").note.contains("运行预期未配置时此状态不表示故障"))
        XCTAssertNotEqual(evaluation.nodeStatus("lidar-stack").signal, .abnormal)
        XCTAssertEqual(evaluation.nodeStatus("point-lio").signal, .unknown, "A workspace-level process scan cannot identify a specific Point-LIO node.")
    }

    func testTopologySelectionAndExpansionStateSupportsClickFlow() {
        var interaction = TopologyInteractionState()

        interaction.toggleFlow("visual-flow")
        XCTAssertEqual(interaction.expandedFlowID, "visual-flow")
        interaction.selectNode("camera", in: "visual-flow")
        XCTAssertEqual(interaction.selectedNodeID, "camera")
        XCTAssertEqual(interaction.expandedFlowID, "visual-flow")
        interaction.toggleFlow("navigation-flow")
        XCTAssertEqual(interaction.expandedFlowID, "navigation-flow")
        XCTAssertNil(interaction.selectedNodeID)
        interaction.selectNode("nav2", in: "navigation-flow")
        interaction.clearSelection()
        XCTAssertNil(interaction.selectedNodeID)
    }

    private func makeContext(
        agentOnline: Bool = true,
        domainID: Int? = 42,
        expectedDomainID: Int? = 42,
        graphAgeMs: Int? = 100,
        topics: [String] = [],
        workspaceFreshness: WorkspaceSnapshotFreshness = .live,
        includeUnconfiguredStoppedLidar: Bool = false,
        events: [HealthEvent] = []
    ) throws -> TopologyRuntimeContext {
        let snapshot = try makeSnapshot(
            domainID: domainID,
            expectedDomainID: expectedDomainID,
            graphAgeMs: graphAgeMs,
            topics: topics,
            includeUnconfiguredStoppedLidar: includeUnconfiguredStoppedLidar
        )
        return TopologyRuntimeContext(
            agentOnline: agentOnline,
            snapshot: snapshot,
            displayedWorkspaceInventory: snapshot.workspaces,
            workspaceFreshness: workspaceFreshness,
            vision: nil,
            healthEvents: events
        )
    }

    private func makeSnapshot(
        domainID: Int?,
        expectedDomainID: Int?,
        graphAgeMs: Int?,
        topics: [String],
        includeUnconfiguredStoppedLidar: Bool
    ) throws -> ConsoleSnapshot {
        let domain = domainID.map(String.init) ?? "null"
        let expectedDomain = expectedDomainID.map(String.init) ?? "null"
        let graphAge = graphAgeMs.map(String.init) ?? "null"
        let topicRows = topics.map { #"{"name":"\#($0)","types":["sensor_msgs/msg/Image"]}"# }.joined(separator: ",")
        let inventoryField: String
        if includeUnconfiguredStoppedLidar {
            inventoryField = #", "workspaces":{"status":"ok","process_scan":{"status":"ok","scope":"Agent UID"},"workspaces":[{"id":"new-engineering","label":"新版工程","source_status":"present","modules":[{"id":"lidar","label":"雷达与定位","source_status":"present","runtime_status":"not_running","run_expectation":"unconfigured"}]},{"id":"original-navigation","label":"原始导航与雷达工程","source_status":"present","modules":[{"id":"lidar","label":"雷达与定位","source_status":"present","runtime_status":"not_running","run_expectation":"unconfigured"}]}]}"#
        } else {
            inventoryField = ""
        }

        let json = #"{"schema_version":1,"collected_at":"2026-10-17T01:00:00.000Z","source":"nuc-read-only-agent","host":{"hostname":"robot","platform":"x86_64","os_release":["PRETTY_NAME=Ubuntu 22.04"]},"system":{"cpu":{"status":"ok","percent":1},"memory":{"status":"ok","total_bytes":100,"available_bytes":50,"used_percent":50},"disk":{"status":"ok","path":"/","total_bytes":100,"used_bytes":20,"free_bytes":80,"used_percent":20},"temperature":{"status":"unknown","reason":"not exposed"}},"git":{"status":"ok","head_sha":null,"branch":"main","dirty":false,"dirty_files":0},"deployment":{"status":"unknown","reason":"not recorded"},"ros":{"status":"ok","domain_id":\#(domain),"nodes":[],"topics":[\#(topicRows)],"observed_at":"2026-10-17T01:00:00.000Z","graph_age_ms":\#(graphAge),"expected_domain_id":\#(expectedDomain),"expected_domain_error":null,"expected_nodes":[],"expected_nodes_error":null,"discovery":"runtime","reason":null}\#(inventoryField),"tests":{"status":"ok","records":[],"reason":null},"safety":{"physical_estop":{"status":"unknown","reason":"not connected"},"hardware_health":{"status":"unknown","reason":"no hardware evidence"},"control":"disabled"}}"#
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(ConsoleSnapshot.self, from: Data(json.utf8))
    }

    private func healthEvent(_ id: String, state: HealthState, resource: String) -> HealthEvent {
        HealthEvent(
            id: id,
            title: id,
            state: state,
            observation: "fixture \(state.rawValue)",
            resource: resource,
            dataTime: "2026-10-17T01:00:00.000Z",
            rosDomain: "实际 42 · 预期 42",
            runMode: .live,
            suggestedCheck: "检查测试证据",
            observedAt: "2026-10-17T01:00:00.000Z",
            source: "RobotTopologyTests"
        )
    }
}
