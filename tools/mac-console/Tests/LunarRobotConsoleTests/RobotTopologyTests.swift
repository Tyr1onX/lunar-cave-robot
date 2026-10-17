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
            XCTAssertEqual(Set(flow.nodeIDs).count, flow.nodeIDs.count, "A node must appear once in detail graph \(flow.id)")
        }
        for subsystem in RobotTopologyCatalog.subsystems {
            XCTAssertNotNil(RobotTopologyCatalog.flow(subsystem.flowID))
            for id in subsystem.statusNodeIDs { XCTAssertNotNil(RobotTopologyCatalog.node(id)) }
        }
        for edge in RobotTopologyCatalog.overviewEdges {
            XCTAssertNotNil(RobotTopologyCatalog.subsystem(edge.sourceSubsystemID))
            XCTAssertNotNil(RobotTopologyCatalog.subsystem(edge.destinationSubsystemID))
            XCTAssertTrue(edge.supportingEdgeIDs.allSatisfy { RobotTopologyCatalog.edge($0) != nil })
        }

        let chassis = try! XCTUnwrap(RobotTopologyCatalog.node("chassis-bridge"))
        XCTAssertTrue(chassis.sourceNote.contains("没有 STM32 acknowledgement"))
        XCTAssertTrue(RobotTopologyCatalog.edge("arm-driver-mcu")?.sourceNote.contains("未由 Agent 验证") == true)
        XCTAssertEqual(RobotTopologyCatalog.edge("mission-nav2")?.evidence, .offlineExercise)
        XCTAssertTrue(RobotTopologyCatalog.node("mission-coordinator")?.sourcePaths.contains("Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py") == true)
        XCTAssertTrue(RobotTopologyCatalog.node("mission-coordinator")?.summary.contains("实体主启动流程集成状态待确认") == true)
    }

    func testGraphTopicNameIsObservationNotProofOfEndpointConnection() throws {
        let context = try makeContext(topics: ["/camera/color/image_raw"])
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.edgeStatus("camera-yolo").signal, .topicsAllVisible)
        XCTAssertTrue(evaluation.edgeStatus("camera-yolo").note.contains("不证明发布 / 订阅两端归属"))
        XCTAssertEqual(evaluation.edgeStatus("mission-nav2").signal, .unknown)
        XCTAssertTrue(evaluation.edgeStatus("mission-nav2").note.contains("实体主启动流程集成仍待确认"))
    }

    func testMultiTopicConnectionDistinguishesAllPartialAndNoneVisible() throws {
        let all = TopologyStatusResolver.evaluate(try makeContext(topics: [
            "/camera/depth/color/points", "/camera/color/camera_info"
        ]))
        XCTAssertEqual(all.edgeStatus("camera-pose").signal, .topicsAllVisible)
        XCTAssertTrue(all.edgeStatus("camera-pose").note.contains("可见 2/2"))
        XCTAssertTrue(all.edgeStatus("camera-pose").note.contains("不证明"))

        let partial = TopologyStatusResolver.evaluate(try makeContext(topics: ["/camera/depth/color/points"]))
        XCTAssertEqual(partial.edgeStatus("camera-pose").signal, .topicsPartiallyVisible)
        XCTAssertTrue(partial.edgeStatus("camera-pose").note.contains("可见 1/2"))
        XCTAssertTrue(partial.edgeStatus("camera-pose").note.contains("未见：/camera/color/camera_info"))

        let none = TopologyStatusResolver.evaluate(try makeContext())
        XCTAssertEqual(none.edgeStatus("camera-pose").signal, .topicsNoneVisible)
        XCTAssertTrue(none.edgeStatus("camera-pose").note.contains("可见 0/2"))
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
        XCTAssertEqual(evaluation.edgeStatus("camera-yolo").signal, .topicsAllVisible)
    }

    func testUnknownOrNotObservedElementsKeepOverallSummaryOutOfGreen() {
        let unknown = TopologyEvaluation(
            nodeStatuses: ["camera": TopologyElementStatus(signal: .unknown, note: "source unknown", directEvents: [], possiblyAffectedDownstream: false)],
            edgeStatuses: ["camera-yolo": TopologyElementStatus(signal: .topicsAllVisible, note: "name only", directEvents: [], possiblyAffectedDownstream: false)],
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
        XCTAssertEqual(evaluation.edgeStatus("scan-slam-toolbox").signal, .unknown)
    }

    func testUnconfiguredMissingAggregateProcessIsNotAnAbnormalEvent() throws {
        let context = try makeContext(includeUnconfiguredStoppedLidar: true)
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.nodeStatus("lidar-stack").signal, .processNotFound)
        XCTAssertTrue(evaluation.nodeStatus("lidar-stack").note.contains("运行预期未配置时此状态不表示故障"))
        XCTAssertNotEqual(evaluation.nodeStatus("lidar-stack").signal, .abnormal)
        XCTAssertEqual(evaluation.nodeStatus("point-lio").signal, .unknown, "A workspace-level process scan cannot identify a specific Point-LIO node.")
    }

    func testOverviewAndSubsystemDetailLayoutsContainEachNodeOnceAndConnectReferencedNodes() {
        let overview = RobotTopologyLayout.overview(availableWidth: 1_400)
        let displayedSubsystemIDs = RobotTopologyCatalog.subsystems.filter { $0.id != "system" }.map(\.id)
        XCTAssertEqual(Set(displayedSubsystemIDs).count, displayedSubsystemIDs.count)
        XCTAssertEqual(Set(overview.nodeCenters.keys), Set(displayedSubsystemIDs))

        for flow in RobotTopologyCatalog.flows {
            let layout = RobotTopologyLayout.detail(flow)
            XCTAssertEqual(Set(layout.nodeCenters.keys), Set(flow.nodeIDs), flow.id)
            for edgeID in flow.edgeIDs {
                guard let edge = RobotTopologyCatalog.edge(edgeID),
                      flow.nodeIDs.contains(edge.sourceNodeID),
                      flow.nodeIDs.contains(edge.destinationNodeID) else { continue }
                XCTAssertNotNil(layout.nodeCenters[edge.sourceNodeID])
                XCTAssertNotNil(layout.nodeCenters[edge.destinationNodeID])
            }
        }
    }

    func testSingleLidarFlowMatchesLaunchLaserScanSlamMapAndNav2Branches() throws {
        let localization = try XCTUnwrap(RobotTopologyCatalog.flow("localization-flow"))
        let navigation = try XCTUnwrap(RobotTopologyCatalog.flow("navigation-flow"))
        XCTAssertTrue(localization.nodeIDs.contains("scan-conversion"))
        XCTAssertTrue(localization.nodeIDs.contains("slam-toolbox"))
        XCTAssertTrue(localization.nodeIDs.contains("localization-map"))
        XCTAssertTrue(localization.edgeIDs.contains("scan-slam-toolbox"))
        XCTAssertTrue(localization.edgeIDs.contains("slam-map"))

        let scanToSlam = try XCTUnwrap(RobotTopologyCatalog.edge("scan-slam-toolbox"))
        XCTAssertEqual(scanToSlam.sourceNodeID, "scan-conversion")
        XCTAssertEqual(scanToSlam.destinationNodeID, "slam-toolbox")
        XCTAssertEqual(scanToSlam.topics, ["/scan"])
        XCTAssertEqual(scanToSlam.evidence, .sourceWiring)

        let slamToMap = try XCTUnwrap(RobotTopologyCatalog.edge("slam-map"))
        XCTAssertEqual(slamToMap.sourceNodeID, "slam-toolbox")
        XCTAssertEqual(slamToMap.destinationNodeID, "localization-map")
        XCTAssertEqual(slamToMap.topics, ["/map"])
        XCTAssertEqual(slamToMap.evidence, .sourceWiring)

        let mapToNav = try XCTUnwrap(RobotTopologyCatalog.edge("map-nav2"))
        XCTAssertEqual(mapToNav.sourceNodeID, "localization-map")
        XCTAssertEqual(mapToNav.destinationNodeID, "nav2")
        XCTAssertTrue(navigation.edgeIDs.contains("map-nav2"))
        XCTAssertTrue(navigation.edgeIDs.contains("terrain-nav2"))
        XCTAssertFalse(navigation.edgeIDs.contains("scan-nav2"), "The selected single-lidar launch config gives Nav2 /terrain_map; /scan feeds SLAM Toolbox.")
        XCTAssertNil(RobotTopologyCatalog.edge("scan-nav2"))

        let root = try XCTUnwrap(repositoryRoot())
        let launch = try sourceText("Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", root: root)
        let slamConfig = try sourceText("Nav2_rea_ws/src/bringup/config/slam_toolbox_single_lidar.yaml", root: root)
        let navConfig = try sourceText("Nav2_rea_ws/src/bringup/config/nav2_params.yaml", root: root)

        XCTAssertTrue(launch.contains("remappings=[(\"cloud_in\", \"/terrain_map\"), (\"scan\", \"/scan\")]"))
        XCTAssertTrue(launch.contains("package=\"slam_toolbox\""))
        XCTAssertTrue(launch.contains("executable=\"sync_slam_toolbox_node\""))
        XCTAssertTrue(launch.contains("slam_toolbox_single_lidar.yaml"))
        XCTAssertTrue(launch.contains("source_file=os.path.join(bringup_share, \"config\", \"nav2_params.yaml\")"))
        XCTAssertTrue(slamConfig.contains("scan_topic: /scan"))
        XCTAssertTrue(slamConfig.contains("map_frame: map"))
        XCTAssertTrue(slamConfig.contains("odom_frame: odom_init"))
        XCTAssertTrue(slamConfig.contains("transform_publish_period: 0.0"))
        XCTAssertTrue(navConfig.contains("topic: /terrain_map"))
        XCTAssertTrue(navConfig.contains("static_layer:"))
        XCTAssertFalse(navConfig.contains("topic: /scan"), "The selected bringup params do not configure Nav2 to consume the generated LaserScan.")
    }

    func testSingleLidarImuPreprocessingChainMatchesLaunchAndPointLioConfig() throws {
        let flow = try XCTUnwrap(RobotTopologyCatalog.flow("localization-flow"))
        for nodeID in ["mid360", "imu-transform", "imu-filter", "point-lio"] {
            XCTAssertTrue(flow.nodeIDs.contains(nodeID), "Missing IMU path node \(nodeID)")
        }
        let expectedEdges: [(String, String, String, String)] = [
            ("mid360-imu-transform", "mid360", "imu-transform", "/livox/imu"),
            ("imu-transform-filter", "imu-transform", "imu-filter", "/livox/imu_aft_tf"),
            ("imu-filter-pointlio", "imu-filter", "point-lio", "/imu/data"),
        ]
        for (edgeID, source, destination, topic) in expectedEdges {
            let edge = try XCTUnwrap(RobotTopologyCatalog.edge(edgeID))
            XCTAssertEqual(edge.sourceNodeID, source, edgeID)
            XCTAssertEqual(edge.destinationNodeID, destination, edgeID)
            XCTAssertEqual(edge.topics, [topic], edgeID)
            XCTAssertTrue(flow.edgeIDs.contains(edgeID), edgeID)
        }

        let root = try XCTUnwrap(repositoryRoot())
        let launch = try sourceText("Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py", root: root)
        let imuTransformConfig = try sourceText("Nav2_rea_ws/src/preprocess/imu_tools/config/imu_transform.yaml", root: root)
        let imuFilterLaunch = try sourceText("Nav2_rea_ws/src/preprocess/imu_tools/launch/imu_complementary_filter.launch.py", root: root)
        let imuFilterSource = try sourceText("Nav2_rea_ws/src/preprocess/imu_tools/imu_complementary_filter/complementary_filter_ros.cpp", root: root)
        let pointLioConfig = try sourceText("Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml", root: root)

        XCTAssertTrue(launch.contains("imu_transform.launch.py"))
        XCTAssertTrue(launch.contains("imu_complementary_filter.launch.py"))
        XCTAssertTrue(imuTransformConfig.contains("sub_topic: \"/livox/imu\""))
        XCTAssertTrue(imuTransformConfig.contains("pub_topic: \"/livox/imu_aft_tf\""))
        XCTAssertTrue(imuFilterLaunch.contains("('/imu/data_raw', '/livox/imu_aft_tf')"))
        XCTAssertTrue(imuFilterSource.contains("create_publisher<ImuMsg>(\"imu/data\""))
        XCTAssertTrue(pointLioConfig.contains("lid_topic: \"livox/lidar\""))
        XCTAssertTrue(pointLioConfig.contains("imu_topic: \"imu/data\""))
    }

    func testNonSystemFlowsHaveNoUnexplainedOrphanNodes() throws {
        let intentionalMonitorOnlyNodes = [
            "localization-flow": Set(["lidar-stack"]),
            "arm-flow": Set(["arm-stack"]),
        ]
        for flow in RobotTopologyCatalog.flows where flow.id != "system-flow" {
            let incidentNodeIDs = Set(flow.edgeIDs.compactMap(RobotTopologyCatalog.edge).flatMap { [$0.sourceNodeID, $0.destinationNodeID] })
            let orphanNodeIDs = Set(flow.nodeIDs).subtracting(incidentNodeIDs)
            XCTAssertEqual(orphanNodeIDs, intentionalMonitorOnlyNodes[flow.id] ?? [], "Unexpected isolated topology node in \(flow.id)")
        }

        let safetyFlow = try XCTUnwrap(RobotTopologyCatalog.flow("safety-flow"))
        XCTAssertTrue(safetyFlow.nodeIDs.contains("agent"))
        XCTAssertTrue(safetyFlow.nodeIDs.contains("physical-estop"))
        XCTAssertTrue(safetyFlow.edgeIDs.contains("estop-unconnected"))
        XCTAssertEqual(RobotTopologyCatalog.edge("estop-unconnected")?.evidence, .openInterface)
    }

    func testTopologySelectionSupportsThreeInteractionLevels() {
        var interaction = TopologyInteractionState()

        XCTAssertNil(interaction.selectedSubsystemID)
        XCTAssertNil(interaction.selectedNodeID)
        interaction.selectSubsystem("perception")
        XCTAssertEqual(interaction.selectedSubsystemID, "perception")
        XCTAssertNil(interaction.selectedNodeID)
        interaction.selectNode("camera", in: "perception")
        XCTAssertEqual(interaction.selectedNodeID, "camera")
        XCTAssertEqual(interaction.selectedSubsystemID, "perception")
        interaction.clearSelection()
        XCTAssertNil(interaction.selectedNodeID)
        interaction.selectSubsystem("navigation")
        interaction.selectNode("nav2", in: "navigation")
        XCTAssertEqual(interaction.selectedSubsystemID, "navigation")
        XCTAssertEqual(interaction.selectedNodeID, "nav2")
        interaction.clearSelection()
        XCTAssertEqual(interaction.selectedSubsystemID, "navigation")
        XCTAssertNil(interaction.selectedNodeID)
        interaction.showOverview()
        XCTAssertNil(interaction.selectedSubsystemID)
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

    private func repositoryRoot() -> URL? {
        let marker = "Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"
        let starts = [URL(fileURLWithPath: #filePath).deletingLastPathComponent(), URL(fileURLWithPath: FileManager.default.currentDirectoryPath)]
        for start in starts {
            var current = start
            while true {
                if FileManager.default.fileExists(atPath: current.appendingPathComponent(marker).path) { return current }
                let parent = current.deletingLastPathComponent()
                if parent.path == current.path { break }
                current = parent
            }
        }
        return nil
    }

    private func sourceText(_ relativePath: String, root: URL) throws -> String {
        try String(contentsOf: root.appendingPathComponent(relativePath), encoding: .utf8)
    }
}
