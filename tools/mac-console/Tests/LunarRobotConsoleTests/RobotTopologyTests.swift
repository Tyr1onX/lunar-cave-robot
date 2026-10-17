import Foundation
import XCTest
@testable import LunarRobotConsole

final class RobotTopologyTests: XCTestCase {
    func testCatalogHasUniqueResolvableSubsystemsEdgesAndStatusNodes() {
        let nodeIDs = RobotTopologyCatalog.nodes.map(\.id)
        let edgeIDs = RobotTopologyCatalog.edges.map(\.id)
        let subsystemIDs = RobotTopologyCatalog.subsystems.map(\.id)
        XCTAssertEqual(Set(nodeIDs).count, nodeIDs.count)
        XCTAssertEqual(Set(edgeIDs).count, edgeIDs.count)
        XCTAssertEqual(Set(subsystemIDs).count, subsystemIDs.count)

        for edge in RobotTopologyCatalog.edges {
            XCTAssertNotNil(RobotTopologyCatalog.node(edge.sourceNodeID))
            XCTAssertNotNil(RobotTopologyCatalog.node(edge.destinationNodeID))
            XCTAssertFalse(edge.sourcePaths.isEmpty, "Missing source evidence for \(edge.id)")
            XCTAssertFalse(edge.sourceNote.isEmpty, "Missing relationship qualification for \(edge.id)")
        }
        for subsystem in RobotTopologyCatalog.subsystems {
            for id in subsystem.statusNodeIDs {
                XCTAssertNotNil(RobotTopologyCatalog.node(id), "Unknown status node \(id) in subsystem \(subsystem.id)")
            }
            let members = RobotTopologyCatalog.nodes.filter { $0.groupID == subsystem.id }.map(\.id)
            XCTAssertEqual(Set(members), Set(subsystem.statusNodeIDs), "Subsystem membership and status coverage should agree for \(subsystem.id)")
        }

        let chassis = try! XCTUnwrap(RobotTopologyCatalog.node("chassis-bridge"))
        XCTAssertTrue(chassis.sourceNote.contains("没有 STM32 acknowledgement"))
        XCTAssertTrue(RobotTopologyCatalog.edge("arm-serial-mcu-command")?.sourceNote.contains("串口设备连接与控制器接收未知") == true)
        XCTAssertEqual(RobotTopologyCatalog.edge("mission-nav2")?.evidence, .offlineExercise)
        XCTAssertTrue(RobotTopologyCatalog.node("mission-coordinator")?.sourcePaths.contains("Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py") == true)
        XCTAssertTrue(RobotTopologyCatalog.node("mission-coordinator")?.summary.contains("实体主启动流程集成状态待确认") == true)
        XCTAssertNil(RobotTopologyCatalog.edge("estop-unconnected"), "An unverified safety interface must not be drawn as a connection")
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

    func testRecentAgentReceivedDataIsNotModuleOrHardwareHealth() throws {
        let telemetry = try makeTelemetry([
            ("livox_lidar", "/livox/lidar", "streaming"),
            ("livox_imu", "/livox/imu", "streaming"),
            ("imu_data", "/imu/data", "streaming"),
            ("pointlio_odometry", "/odometry", "streaming"),
            ("cloud_registered", "/cloud_registered", "streaming"),
        ])
        XCTAssertEqual(telemetry.streams.first?.subscriptionGeneration, Optional(1))
        XCTAssertEqual(telemetry.streams.first?.subscriptionStartedAt, Optional("2026-10-17T00:59:58.000Z"))
        let context = try makeContext(linkTelemetry: telemetry)
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.edgeStatus("mid360-pointlio").signal, .dataObserved)
        XCTAssertEqual(evaluation.nodeStatus("mid360").signal, .unknown, "Receiving the same-named Topic does not establish physical sensor health or workspace ownership.")
        XCTAssertEqual(evaluation.nodeStatus("point-lio").signal, .unknown, "Agent receive evidence is not Point-LIO processing health.")
        XCTAssertFalse(evaluation.nodeStatuses.values.contains { $0.signal == .abnormal })
        XCTAssertTrue(evaluation.edgeStatus("mid360-pointlio").note.contains("工作区归属未确认"))
    }

    func testTelemetryMapsAcrossLivoxIMUPointLIOAndSLAMEdges() throws {
        let telemetry = try makeTelemetry([
            ("livox_lidar", "/livox/lidar", "streaming"),
            ("livox_imu", "/livox/imu", "streaming"),
            ("imu_transformed", "/livox/imu_aft_tf", "streaming"),
            ("imu_data", "/imu/data", "streaming"),
            ("pointlio_odometry", "/odometry", "streaming"),
            ("cloud_registered", "/cloud_registered", "streaming"),
            ("terrain_map", "/terrain_map", "streaming"),
            ("scan", "/scan", "streaming"),
            ("map", "/map", "streaming"),
        ])
        let evaluation = TopologyStatusResolver.evaluate(try makeContext(linkTelemetry: telemetry))

        for edgeID in [
            "mid360-pointlio", "mid360-imu-transform", "imu-transform-filter", "imu-filter-pointlio",
            "pointlio-terrain", "terrain-scan", "scan-slam-toolbox", "slam-map",
        ] {
            XCTAssertEqual(evaluation.edgeStatus(edgeID).signal, .dataObserved, edgeID)
        }
        XCTAssertEqual(evaluation.nodeStatus("mid360").signal, .unknown)
        XCTAssertEqual(evaluation.nodeStatus("point-lio").signal, .unknown)
        XCTAssertEqual(evaluation.nodeStatus("slam-toolbox").signal, .unknown)
        XCTAssertFalse(evaluation.nodeStatuses.values.contains { $0.signal == .abnormal })
    }

    func testStaleStreamMarksOnlyDataSymptomAndPossibleDownstreamImpact() throws {
        let telemetry = try makeTelemetry([
            ("livox_lidar", "/livox/lidar", "stale"),
            ("livox_imu", "/livox/imu", "streaming"),
        ])
        let context = try makeContext(linkTelemetry: telemetry)
        let evaluation = TopologyStatusResolver.evaluate(context)

        XCTAssertEqual(evaluation.edgeStatus("mid360-pointlio").signal, .dataStale)
        XCTAssertTrue(evaluation.edgeStatus("mid360-pointlio").possiblyAffectedDownstream)
        XCTAssertTrue(evaluation.nodeStatus("point-lio").possiblyAffectedDownstream)
        XCTAssertEqual(evaluation.nodeStatus("point-lio").signal, .unknown)
        XCTAssertFalse(evaluation.nodeStatuses.values.contains { $0.signal == .abnormal }, "An unconfigured stream must not create a module fault event.")
    }

    func testPointLIOAndSLAMPipelineCluesDoNotClaimRootCause() throws {
        let pointLIO = try makeTelemetry([
            ("livox_lidar", "/livox/lidar", "streaming"),
            ("imu_data", "/imu/data", "streaming"),
            ("pointlio_odometry", "/odometry", "unavailable"),
            ("cloud_registered", "/cloud_registered", "unavailable"),
            ("scan", "/scan", "unavailable"),
            ("map", "/map", "unavailable"),
        ])
        let pointSummary = try XCTUnwrap(LinkTelemetryDiagnostics.pipelineSummary(nodeID: "point-lio", telemetry: pointLIO, blockedReason: nil))
        XCTAssertTrue(pointSummary.contains("可能存在处理链路间断"))
        XCTAssertTrue(pointSummary.contains("不判定模块故障"))

        let slam = try makeTelemetry([
            ("scan", "/scan", "streaming"),
            ("map", "/map", "unavailable"),
        ])
        let slamSummary = try XCTUnwrap(LinkTelemetryDiagnostics.pipelineSummary(nodeID: "slam-toolbox", telemetry: slam, blockedReason: nil))
        XCTAssertTrue(slamSummary.contains("可能存在处理链路间断"))
        XCTAssertTrue(slamSummary.contains("工作区来源与运行预期未确认"))
    }

    func testDomainMismatchAndStaleTelemetryGraphKeepEdgesUnknown() throws {
        let mismatch = try makeTelemetry(
            [("livox_lidar", "/livox/lidar", "streaming")],
            domainID: 7,
            expectedDomainID: 42
        )
        let mismatchEvaluation = TopologyStatusResolver.evaluate(try makeContext(linkTelemetry: mismatch))
        XCTAssertEqual(mismatchEvaluation.edgeStatus("mid360-pointlio").signal, .unknown)
        XCTAssertTrue(mismatchEvaluation.edgeStatus("mid360-pointlio").note.contains("Domain"))

        let stale = try makeTelemetry(
            [("livox_lidar", "/livox/lidar", "streaming")],
            graphAgeMs: HealthEvaluator.rosGraphTimeoutMs + 1
        )
        let staleEvaluation = TopologyStatusResolver.evaluate(try makeContext(linkTelemetry: stale))
        XCTAssertEqual(staleEvaluation.edgeStatus("mid360-pointlio").signal, .unknown)
        XCTAssertTrue(staleEvaluation.edgeStatus("mid360-pointlio").note.contains("过期"))
    }

    func testMultiTopicRuntimeStateShowsPartialReceiptWithoutFault() throws {
        let telemetry = try makeTelemetry([
            ("cloud_registered", "/cloud_registered", "streaming"),
            ("pointlio_odometry", "/odometry", "unavailable"),
        ])
        let evaluation = TopologyStatusResolver.evaluate(try makeContext(linkTelemetry: telemetry))
        XCTAssertEqual(evaluation.edgeStatus("pointlio-terrain").signal, .dataPartiallyObserved)
        XCTAssertTrue(evaluation.edgeStatus("pointlio-terrain").note.contains("/odometry"))
        XCTAssertFalse(evaluation.nodeStatuses.values.contains { $0.signal == .abnormal })
    }

    func testUnconfiguredStoppedWorkspaceDoesNotBecomeAbnormalWhenTelemetryIsAbsent() throws {
        let telemetry = try makeTelemetry([
            ("livox_lidar", "/livox/lidar", "unavailable"),
            ("livox_imu", "/livox/imu", "unavailable"),
        ])
        let evaluation = TopologyStatusResolver.evaluate(try makeContext(
            includeUnconfiguredStoppedLidar: true,
            linkTelemetry: telemetry
        ))
        XCTAssertEqual(evaluation.nodeStatus("lidar-stack").signal, .processNotFound)
        XCTAssertNotEqual(evaluation.nodeStatus("lidar-stack").signal, .abnormal)
        XCTAssertEqual(evaluation.edgeStatus("mid360-pointlio").signal, .dataNotObserved)
    }

    func testUnifiedProjectionPreservesSubsystemAndModuleLevelsWithoutDuplicateNodes() {
        let collapsed = RobotTopologyCatalog.project(expandedSubsystemIDs: [])
        XCTAssertEqual(Set(collapsed.nodes.map(\.id)), Set(RobotTopologyCatalog.subsystemIDs))
        XCTAssertEqual(collapsed.nodes.count, RobotTopologyCatalog.subsystems.count)
        XCTAssertTrue(collapsed.nodes.allSatisfy(\.isSubsystem))

        let expectedEdges = Set(RobotTopologyCatalog.edges.compactMap { edge -> String? in
            guard let source = RobotTopologyCatalog.node(edge.sourceNodeID),
                  let destination = RobotTopologyCatalog.node(edge.destinationNodeID),
                  RobotTopologyCatalog.subsystem(source.groupID) != nil,
                  RobotTopologyCatalog.subsystem(destination.groupID) != nil,
                  source.groupID != destination.groupID else { return nil }
            return edge.id
        })
        XCTAssertEqual(Set(collapsed.edges.map(\.id)), expectedEdges)
        XCTAssertFalse(collapsed.nodes.contains { $0.id == "agent" || $0.id == "ros-graph" })

        let expandedPerception = RobotTopologyCatalog.project(expandedSubsystemIDs: ["perception"])
        let perceptionMembers = Set(RobotTopologyCatalog.nodes.filter { $0.groupID == "perception" }.map(\.id))
        XCTAssertEqual(Set(expandedPerception.nodes.filter { $0.subsystemID == "perception" }.map(\.id)), perceptionMembers)
        XCTAssertEqual(expandedPerception.nodes.filter { $0.subsystemID == "perception" }.count, perceptionMembers.count)
        XCTAssertTrue(expandedPerception.nodes.contains { $0.id == "localization" && $0.isSubsystem })
        XCTAssertTrue(expandedPerception.nodes.contains { $0.id == "arm" && $0.isSubsystem })
        XCTAssertTrue(expandedPerception.edges.contains { $0.id == "pose-arm-planner" && $0.sourceID == "pose-estimator" && $0.destinationID == "arm" })
        XCTAssertTrue(expandedPerception.edges.contains { $0.id == "camera-yolo" && $0.sourceID == "camera" && $0.destinationID == "yolo" })
        XCTAssertTrue(expandedPerception.edges.allSatisfy { edge in
            expandedPerception.nodes.contains { $0.id == edge.sourceID } && expandedPerception.nodes.contains { $0.id == edge.destinationID }
        })

        let expandedAll = RobotTopologyCatalog.project(expandedSubsystemIDs: Set(RobotTopologyCatalog.subsystemIDs))
        let expectedModuleIDs = Set(RobotTopologyCatalog.nodes.filter { RobotTopologyCatalog.subsystem($0.groupID) != nil }.map(\.id))
        XCTAssertEqual(Set(expandedAll.nodes.map(\.id)), expectedModuleIDs)
        XCTAssertEqual(expandedAll.nodes.count, expectedModuleIDs.count)
        XCTAssertEqual(Set(expandedAll.edges.map(\.id)), Set(RobotTopologyCatalog.edges.map(\.id)))
        XCTAssertEqual(Set(expandedAll.edges.map(\.id)).count, expandedAll.edges.count)
        XCTAssertFalse(expandedAll.nodes.contains { $0.id == "agent" || $0.id == "ros-graph" })
    }

    func testRecursiveVerticalLayoutsFitWidthsAndPlaceDependenciesTopDown() {
        let states: [Set<String>] = [[], ["perception"], ["localization", "navigation"], Set(RobotTopologyCatalog.subsystemIDs)]
        for expanded in states {
            for width in [CGFloat(320), 640, 1_180, 1_520] {
                let projection = RobotTopologyCatalog.project(expandedSubsystemIDs: expanded)
                let layout = RobotTopologyPresentationLayout.graph(width: width, expandedSubsystemIDs: expanded)
                XCTAssertEqual(Set(layout.nodeFrames.keys), Set(projection.nodes.map(\.id)))
                XCTAssertLessThanOrEqual(layout.size.width, width)
                for frame in layout.nodeFrames.values {
                    XCTAssertGreaterThanOrEqual(frame.minX, 0)
                    XCTAssertLessThanOrEqual(frame.maxX, width)
                    XCTAssertGreaterThanOrEqual(frame.minY, 0)
                    XCTAssertLessThanOrEqual(frame.maxY, layout.size.height)
                }
                for header in layout.expandedHeaderFrames.values {
                    XCTAssertGreaterThanOrEqual(header.minX, 0)
                    XCTAssertLessThanOrEqual(header.maxX, width)
                    XCTAssertLessThanOrEqual(header.maxY, layout.size.height)
                }
                XCTAssertEqual(layout.size.height, RobotTopologyPresentationLayout.graphHeight(expandedSubsystemIDs: expanded))
            }
        }

        let localization = RobotTopologyPresentationLayout.graph(width: 620, expandedSubsystemIDs: ["localization"])
        XCTAssertLessThan(localization.nodeFrames["mid360"]!.midY, localization.nodeFrames["point-lio"]!.midY)
        XCTAssertLessThan(localization.nodeFrames["point-lio"]!.midY, localization.nodeFrames["terrain"]!.midY)
        XCTAssertLessThan(localization.nodeFrames["terrain"]!.midY, localization.nodeFrames["slam-toolbox"]!.midY)
        XCTAssertLessThan(localization.nodeFrames["slam-toolbox"]!.midY, localization.nodeFrames["localization-map"]!.midY)

        let expandedAll = RobotTopologyCatalog.project(expandedSubsystemIDs: Set(RobotTopologyCatalog.subsystemIDs))
        let allLayout = RobotTopologyPresentationLayout.graph(width: 900, expandedSubsystemIDs: Set(RobotTopologyCatalog.subsystemIDs))
        for edge in expandedAll.edges where !edge.isFeedback {
            guard let source = allLayout.nodeFrames[edge.sourceID],
                  let destination = allLayout.nodeFrames[edge.destinationID] else {
                XCTFail("Projection edge \(edge.id) is missing a layout endpoint")
                continue
            }
            XCTAssertLessThan(source.midY, destination.midY, "Expected top-to-bottom flow for \(edge.id)")
        }
        for edge in expandedAll.edges where edge.isFeedback {
            guard let source = allLayout.nodeFrames[edge.sourceID],
                  let destination = allLayout.nodeFrames[edge.destinationID] else {
                XCTFail("Feedback edge \(edge.id) is missing a layout endpoint")
                continue
            }
            XCTAssertGreaterThan(source.midY, destination.midY, "Feedback should return upward for \(edge.id)")
        }
    }

    func testTopologyDisplayUsesFunctionalChineseNamesAndKeepsTechnicalNameInCatalog() {
        XCTAssertEqual(TopologyFriendlyNames.node("yolo"), "识别目标")
        XCTAssertEqual(TopologyFriendlyNames.node("point-lio"), "判断机器人位置")
        XCTAssertEqual(TopologyFriendlyNames.node("slam-toolbox"), "生成地图")
        XCTAssertEqual(TopologyFriendlyNames.node("nav2"), "自动规划导航路线")
        XCTAssertEqual(TopologyFriendlyNames.node("chassis-controller"), "底盘控制板")
        XCTAssertEqual(RobotTopologyCatalog.node("yolo")?.title, "YOLO 检测")
        XCTAssertEqual(RobotTopologyCatalog.node("slam-toolbox")?.title, "SLAM Toolbox")
    }

    func testNodeEvidenceMovesBelowGraphWhenWindowNarrows() {
        XCTAssertFalse(TopologyInspectorPlacement.usesTrailingPane(width: 820))
        XCTAssertFalse(TopologyInspectorPlacement.usesTrailingPane(width: 1_049))
        XCTAssertTrue(TopologyInspectorPlacement.usesTrailingPane(width: 1_050))
        XCTAssertTrue(TopologyInspectorPlacement.usesTrailingPane(width: 1_360))
    }

    func testSingleLidarFlowMatchesLaunchLaserScanSlamMapAndNav2Branches() throws {
        let localizationNodeIDs = Set(RobotTopologyCatalog.nodes.filter { $0.groupID == "localization" }.map(\.id))
        XCTAssertTrue(localizationNodeIDs.isSuperset(of: ["scan-conversion", "slam-toolbox", "localization-map", "map-odom-tf"]))

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
        XCTAssertEqual(mapToNav.topics, ["/map"])
        XCTAssertEqual(RobotTopologyCatalog.edge("terrain-nav2")?.topics, ["/terrain_map"])
        XCTAssertFalse(RobotTopologyCatalog.edges.contains { $0.id == "scan-nav2" }, "The selected single-lidar launch config gives Nav2 /terrain_map; /scan feeds SLAM Toolbox.")
        XCTAssertNil(RobotTopologyCatalog.edge("scan-nav2"))
        XCTAssertEqual(RobotTopologyCatalog.edge("map-tf-navigation")?.sourceNodeID, "map-odom-tf")
        XCTAssertEqual(RobotTopologyCatalog.edge("map-tf-navigation")?.topics, ["/tf_static"])

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
        let localizationNodeIDs = Set(RobotTopologyCatalog.nodes.filter { $0.groupID == "localization" }.map(\.id))
        for nodeID in ["mid360", "imu-transform", "imu-filter", "point-lio"] {
            XCTAssertTrue(localizationNodeIDs.contains(nodeID), "Missing IMU path node \(nodeID)")
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

    func testUnifiedRobotGraphHasOnlyDocumentedIsolatedObservationNodes() throws {
        let robotNodes = RobotTopologyCatalog.nodes.filter { RobotTopologyCatalog.subsystem($0.groupID) != nil }
        let incident = Set(RobotTopologyCatalog.edges.flatMap { [$0.sourceNodeID, $0.destinationNodeID] })
        let isolated = Set(robotNodes.map(\.id).filter { !incident.contains($0) })
        XCTAssertEqual(isolated, ["lidar-stack", "arm-stack", "physical-estop"])
        XCTAssertTrue(RobotTopologyCatalog.node("lidar-stack")?.sourceNote.contains("模块聚合") == true)
        XCTAssertTrue(RobotTopologyCatalog.node("arm-stack")?.sourceNote.contains("合并模块") == true)
        XCTAssertTrue(RobotTopologyCatalog.node("physical-estop")?.sourceNote.contains("不绘制连接线") == true)
        XCTAssertFalse(RobotTopologyCatalog.edges.contains { $0.sourceNodeID == "physical-estop" || $0.destinationNodeID == "physical-estop" })

        XCTAssertNil(RobotTopologyCatalog.edge("pose-coordinator"), "The coordinator default target differs from the perception topic")
        XCTAssertNil(RobotTopologyCatalog.edge("arm2nav-coordinator"), "No coordinator integration has been confirmed for this optional path")
        XCTAssertEqual(RobotTopologyCatalog.edge("arm2nav-arm-planner")?.evidence, .conditionalSourceWiring)
        XCTAssertEqual(RobotTopologyCatalog.edge("nav2-mission-feedback")?.isFeedback, true)
        XCTAssertEqual(RobotTopologyCatalog.edge("arm-mission-feedback")?.isFeedback, true)

        let coordinator = try XCTUnwrap(RobotTopologyCatalog.node("mission-coordinator"))
        XCTAssertTrue(coordinator.sourcePaths.contains("Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py"))
        XCTAssertTrue(coordinator.sourceNote.contains("不证明实体主启动流程"))
    }

    func testTopologyInteractionExpandsInPlaceAndKeepsOtherSubsystemsVisible() {
        var interaction = TopologyInteractionState()

        XCTAssertNil(interaction.selectedNodeID)
        XCTAssertTrue(interaction.expandedSubsystemIDs.isEmpty)
        interaction.toggleSubsystem("perception")
        XCTAssertEqual(interaction.expandedSubsystemIDs, ["perception"])
        interaction.toggleSubsystem("localization")
        XCTAssertEqual(interaction.expandedSubsystemIDs, ["perception", "localization"])

        let projection = RobotTopologyCatalog.project(expandedSubsystemIDs: interaction.expandedSubsystemIDs)
        XCTAssertTrue(projection.nodes.contains { $0.id == "camera" })
        XCTAssertTrue(projection.nodes.contains { $0.id == "mid360" })
        XCTAssertTrue(projection.nodes.contains { $0.id == "navigation" && $0.isSubsystem })
        XCTAssertTrue(projection.nodes.contains { $0.id == "chassis" && $0.isSubsystem })

        interaction.selectNode("camera")
        XCTAssertEqual(interaction.selectedNodeID, "camera")
        XCTAssertEqual(interaction.expandedSubsystemIDs, ["perception", "localization"])
        interaction.clearSelection()
        XCTAssertNil(interaction.selectedNodeID)

        interaction.selectNode("nav2")
        XCTAssertTrue(interaction.expandedSubsystemIDs.contains("navigation"), "Selecting a module must reveal its parent subsystem inline")
        XCTAssertEqual(interaction.selectedNodeID, "nav2")

        interaction.toggleSubsystem("navigation")
        XCTAssertNil(interaction.selectedNodeID, "Collapsing a selected module's parent clears the stale inspector selection")
        XCTAssertFalse(interaction.expandedSubsystemIDs.contains("navigation"))
        XCTAssertTrue(interaction.expandedSubsystemIDs.contains("perception"))
        interaction.collapseAll()
        XCTAssertTrue(interaction.expandedSubsystemIDs.isEmpty)
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
        linkTelemetry: LinkTelemetryReading? = nil,
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
            linkTelemetry: linkTelemetry,
            healthEvents: events
        )
    }

    private func makeTelemetry(
        _ rows: [(String, String, String)],
        domainID: Int = 42,
        expectedDomainID: Int = 42,
        graphAgeMs: Int = 100
    ) throws -> LinkTelemetryReading {
        let streams = rows.map { id, topic, status in
            let type: String
            switch topic {
            case "/livox/lidar": type = "livox_ros_driver2/msg/CustomMsg"
            case "/livox/imu", "/livox/imu_aft_tf", "/imu/data": type = "sensor_msgs/msg/Imu"
            case "/odometry": type = "nav_msgs/msg/Odometry"
            case "/cloud_registered", "/terrain_map": type = "sensor_msgs/msg/PointCloud2"
            case "/scan": type = "sensor_msgs/msg/LaserScan"
            default: type = "nav_msgs/msg/OccupancyGrid"
            }
            let age = status == "streaming" ? "20" : (status == "stale" ? "3000" : "null")
            let reason = status == "streaming" ? "null" : #""no recent sample""#
            let publisherCount = status == "unavailable" ? 0 : 1
            let endpoints = publisherCount == 0 ? "[]" : #"[{"name":"test_publisher","namespace":"/synthetic","topic_type":"\#(type)"}]"#
            let timestamp = publisherCount == 0 ? "null" : #""2026-10-17T01:00:00.000Z""#
            let rate = status == "streaming" ? "10.0" : "null"
            let receiveCount = status == "unavailable" ? 0 : 20
            return #"{"id":"\#(id)","label":"\#(id)","topic":"\#(topic)","topic_type":"\#(type)","topic_types":["\#(type)"],"status":"\#(status)","reason":\#(reason),"publisher_count":\#(publisherCount),"publisher_endpoints":\#(endpoints),"workspace_attribution":"unknown","subscription_generation":1,"subscription_started_at":"2026-10-17T00:59:58.000Z","previous_received_at":null,"previous_subscription_generation":null,"received_at":\#(timestamp),"source_stamp":\#(timestamp),"age_ms":\#(age),"receive_rate_hz":\#(rate),"header_to_agent_ms":null,"frame_id":"synthetic","sample_count":100,"sample_unit":"points","gap_events":0,"last_gap_ms":null,"stale_after_ms":1500,"receive_count":\#(receiveCount)}"#
        }.joined(separator: ",")
        let domainMatch = domainID == expectedDomainID ? "true" : "false"
        let json = #"{"schema_version":1,"status":"ok","source":"nuc-ros2-fixed-topic-read-only-subscriber","collected_at":"2026-10-17T01:00:00.000Z","domain_id":\#(domainID),"expected_domain_id":\#(expectedDomainID),"expected_domain_error":null,"domain_matches_expectation":\#(domainMatch),"graph_status":"ok","graph_observed_at":"2026-10-17T01:00:00.000Z","graph_age_ms":\#(graphAgeMs),"graph_reason":null,"workspace_attribution":"unknown","clock_note":"clock synchronization not verified","streams":[\#(streams)]}"#
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        return try decoder.decode(LinkTelemetryReading.self, from: Data(json.utf8))
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
