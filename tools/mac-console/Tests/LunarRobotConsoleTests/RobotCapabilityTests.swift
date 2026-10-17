import XCTest
@testable import LunarRobotConsole

final class RobotCapabilityTests: XCTestCase {
    func testHomeOffersThreeCapabilityOrientedEntrypoints() {
        XCTAssertEqual(RobotCapabilityCatalog.capabilities.map(\.title), [
            "观察并抓取目标", "定位并自主行驶", "任务与安全管理",
        ])
        XCTAssertEqual(Set(RobotCapabilityCatalog.capabilities.map(\.id)).count, 3)
    }

    func testEveryStepExplainsInputProcessOutputAndUsesResolvableEvidence() {
        for capability in RobotCapabilityCatalog.capabilities {
            XCTAssertFalse(capability.steps.isEmpty, capability.title)
            for step in capability.steps {
                XCTAssertFalse(step.input.isEmpty, "Missing input for \(step.id)")
                XCTAssertFalse(step.process.isEmpty, "Missing process for \(step.id)")
                XCTAssertFalse(step.output.isEmpty, "Missing output for \(step.id)")
                XCTAssertFalse(step.nodeIDs.isEmpty, "Missing node evidence for \(step.id)")
                for visibleDescription in [step.input, step.process, step.output] {
                    XCTAssertFalse(visibleDescription.contains("/"), "Workflow copy should keep ROS resource names in details: \(step.id)")
                    for implementationName in ["YOLO", "Point-LIO", "SLAM Toolbox", "Nav2", "rm_serial_driver", "chassis_serial_bridge"] {
                        XCTAssertFalse(visibleDescription.localizedCaseInsensitiveContains(implementationName), "Workflow copy should use a functional name: \(step.id)")
                    }
                }
                for nodeID in step.nodeIDs {
                    XCTAssertNotNil(RobotTopologyCatalog.node(nodeID), "Unknown node \(nodeID) in \(step.id)")
                }
                for edgeID in step.edgeIDs + step.secondaryEdgeIDs {
                    XCTAssertNotNil(RobotTopologyCatalog.edge(edgeID), "Unknown edge \(edgeID) in \(step.id)")
                }
            }
            for edgeID in capability.feedbackEdgeIDs {
                XCTAssertNotNil(RobotTopologyCatalog.edge(edgeID), "Unknown feedback edge \(edgeID)")
                XCTAssertTrue(RobotTopologyCatalog.edge(edgeID)?.isFeedback == true, "Feedback details must not contain a forward edge")
            }
        }
    }

    func testDefaultObserveAndGraspFlowUsesRealDefaultEdgesAndPreservesYOLOGate() throws {
        let capability = try XCTUnwrap(RobotCapabilityCatalog.capability("observe-and-grasp"))
        let required = [
            "camera-yolo", "camera-pose", "yolo-pose", "pose-arm-planner",
            "arm-planner-driver", "arm-serial-mcu-command",
        ]
        XCTAssertTrue(Set(capability.defaultEdgeIDs).isSuperset(of: required))
        XCTAssertFalse(capability.defaultEdgeIDs.contains("arm2nav-arm-planner"), "Optional Arm2Nav input is hidden from the default flow")
        XCTAssertTrue(capability.defaultEdgeIDs.allSatisfy { RobotTopologyCatalog.edge($0)?.isFeedback == false })

        let yoloStep = try XCTUnwrap(capability.steps.first { $0.id == "recognize-target" })
        XCTAssertTrue(yoloStep.note.contains("置信度门控"))
        XCTAssertTrue(yoloStep.note.contains("不能单独判定为故障"))
        XCTAssertTrue(capability.steps.contains { $0.id == "estimate-target-pose" && $0.edgeIDs.contains("camera-pose") })
    }

    func testDefaultNavigationFlowKeepsLaserScanSLAMAndTerrainBranchDistinct() throws {
        let capability = try XCTUnwrap(RobotCapabilityCatalog.capability("localize-and-drive"))
        let edges = Set(capability.defaultEdgeIDs)
        for edgeID in [
            "mid360-pointlio", "mid360-imu-transform", "imu-transform-filter", "imu-filter-pointlio",
            "pointlio-terrain", "terrain-scan", "scan-slam-toolbox", "slam-map", "terrain-nav2",
            "map-nav2", "pointlio-tf-navigation", "map-tf-navigation", "nav2-chassis", "chassis-bridge-mcu",
        ] {
            XCTAssertTrue(edges.contains(edgeID), "Missing real single-lidar workflow edge \(edgeID)")
        }
        XCTAssertFalse(edges.contains("mission-chassis-stop"), "Task-stop feedback is not a default navigation flow connection")

        let terrainStep = try XCTUnwrap(capability.steps.first { $0.id == "prepare-map-input" })
        XCTAssertTrue(terrainStep.note.contains("不会经过 /scan"))
        let mapStep = try XCTUnwrap(capability.steps.first { $0.id == "build-map" })
        XCTAssertTrue(mapStep.edgeIDs.contains("scan-slam-toolbox"))
    }

    func testTaskAndSafetyFlowDistinguishesOfflineCoordinatorFromUnconnectedPhysicalEStop() throws {
        let capability = try XCTUnwrap(RobotCapabilityCatalog.capability("task-and-safety"))
        XCTAssertEqual(RobotTopologyCatalog.edge("mission-nav2")?.evidence, .offlineExercise)
        XCTAssertTrue(try XCTUnwrap(RobotTopologyCatalog.node("mission-coordinator")).summary.contains("实体主启动流程集成状态待确认"))
        XCTAssertEqual(capability.boundaryNodeIDs, ["physical-estop"])
        XCTAssertTrue(capability.steps.first { $0.id == "safety-observation-boundary" }?.note.contains("没有已验证") == true)
        XCTAssertNil(RobotTopologyCatalog.edges.first { $0.sourceNodeID == "physical-estop" || $0.destinationNodeID == "physical-estop" })
        XCTAssertFalse(RobotCapabilityFlowRelations.hasDefaultConnection(
            from: capability.steps[0], to: capability.steps[1]
        ), "The task coordinator must not be visually connected to the unintegrated physical E-stop input")
    }

    func testDefaultStepConnectorsRequireAnActualForwardCatalogEdge() throws {
        let capability = try XCTUnwrap(RobotCapabilityCatalog.capability("observe-and-grasp"))
        XCTAssertTrue(RobotCapabilityFlowRelations.hasDefaultConnection(from: capability.steps[0], to: capability.steps[1]))
        XCTAssertTrue(RobotCapabilityFlowRelations.hasDefaultConnection(from: capability.steps[1], to: capability.steps[2]))
        XCTAssertFalse(RobotCapabilityFlowRelations.hasDefaultConnection(from: capability.steps[0], to: capability.steps[3]))

        let navigation = try XCTUnwrap(RobotCapabilityCatalog.capability("localize-and-drive"))
        XCTAssertTrue(RobotCapabilityFlowRelations.hasDefaultConnection(from: navigation.steps[0], to: navigation.steps[1]))
        XCTAssertTrue(RobotCapabilityFlowRelations.hasDefaultConnection(from: navigation.steps[3], to: navigation.steps[4]))
    }

    func testUnknownCapabilityEvidenceNeverBecomesGreenOrHealthy() {
        let unknown = TopologyElementStatus(
            signal: .unknown,
            note: "Domain or source not confirmed",
            directEvents: [],
            possiblyAffectedDownstream: false
        )
        let evaluation = TopologyEvaluation(
            nodeStatuses: ["camera": unknown],
            edgeStatuses: ["camera-yolo": TopologyElementStatus(signal: .unknown, note: "Graph unavailable", directEvents: [], possiblyAffectedDownstream: false)],
            possiblyAffectedNodeIDs: []
        )

        XCTAssertEqual(RobotCapabilityStatusSummary.landingHint(for: evaluation), .evidencePending)
        let captureStep = try! XCTUnwrap(RobotCapabilityCatalog.step(capabilityID: "observe-and-grasp", stepID: "capture-target"))
        XCTAssertEqual(RobotCapabilityStatusSummary.hint(for: captureStep, evaluation: evaluation), .evidencePending)
        XCTAssertFalse(RobotCapabilityStatusSummary.landingHint(for: evaluation).title.contains("正常"))
    }

    func testDirectAnomalyAndStaleStreamRemainDifferentObservations() throws {
        let abnormal = TopologyElementStatus(signal: .abnormal, note: "direct event", directEvents: [], possiblyAffectedDownstream: false)
        let stale = TopologyElementStatus(signal: .dataStale, note: "no recent messages", directEvents: [], possiblyAffectedDownstream: true)
        let evaluation = TopologyEvaluation(
            nodeStatuses: ["camera": abnormal],
            edgeStatuses: ["camera-yolo": stale],
            possiblyAffectedNodeIDs: ["yolo"]
        )

        let captureStep = try XCTUnwrap(RobotCapabilityCatalog.step(capabilityID: "observe-and-grasp", stepID: "capture-target"))
        let detectStep = try XCTUnwrap(RobotCapabilityCatalog.step(capabilityID: "observe-and-grasp", stepID: "recognize-target"))
        XCTAssertEqual(RobotCapabilityStatusSummary.hint(for: captureStep, evaluation: evaluation), .directAnomaly)
        XCTAssertEqual(RobotCapabilityStatusSummary.hint(for: detectStep, evaluation: evaluation), .staleData)
        XCTAssertEqual(RobotCapabilityStatusSummary.landingHint(for: evaluation), .directAnomaly)
    }
}
