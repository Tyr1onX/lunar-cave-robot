import XCTest
@testable import LunarRobotConsole

final class RobotSystemWorkflowTests: XCTestCase {
    func testFourStagesAndBothPrimaryProcessingLanesArePresent() throws {
        XCTAssertEqual(RobotSystemWorkflowCatalog.phases.map(\.id), ["perception", "planning", "control", "execution"])
        XCTAssertEqual(RobotSystemWorkflowCatalog.phases.map(\.number), ["01", "02", "03", "04"])
        XCTAssertEqual(RobotSystemWorkflowCatalog.phases.map(\.title), ["感知与目标识别", "规划与决策", "运动控制", "执行与反馈"])

        let perception = try XCTUnwrap(RobotSystemWorkflowCatalog.phase("perception"))
        XCTAssertEqual(perception.lanes.map(\.id), ["vision", "localization"])
        XCTAssertTrue(perception.nodeIDs.contains("camera"))
        XCTAssertTrue(perception.nodeIDs.contains("mid360"))
        XCTAssertTrue(perception.nodeIDs.contains("slam-toolbox"))

        let planning = try XCTUnwrap(RobotSystemWorkflowCatalog.phase("planning"))
        XCTAssertEqual(planning.lanes.map(\.id), ["navigation-planning", "arm-planning", "task-coordination"])
        XCTAssertTrue(planning.nodeIDs.contains("nav2"))
        XCTAssertTrue(planning.nodeIDs.contains("arm-planner"))
        XCTAssertTrue(planning.nodeIDs.contains("mission-coordinator"))
    }

    func testWorkflowReferencesEveryExistingTopologyEdgeAndNoUnknownEdge() {
        let catalogIDs = Set(RobotTopologyCatalog.edges.map(\.id))
        let workflowIDs = Set(RobotSystemWorkflowCatalog.referencedEdgeIDs)
        XCTAssertEqual(workflowIDs, catalogIDs)
        XCTAssertTrue(RobotSystemWorkflowCatalog.referencedEdgeIDs.allSatisfy { RobotTopologyCatalog.edge($0) != nil })
    }

    func testWorkflowDoesNotDuplicateNodesOrInventAnEmergencyStopConnection() {
        let nodeIDs = RobotSystemWorkflowCatalog.phases.flatMap(\.nodeIDs)
        XCTAssertEqual(nodeIDs.count, Set(nodeIDs).count)
        XCTAssertEqual(nodeIDs.count, RobotSystemWorkflowCatalog.phases.reduce(0) { $0 + $1.nodeIDs.count })
        XCTAssertTrue(nodeIDs.contains("physical-estop"))
        XCTAssertFalse(RobotTopologyCatalog.edges.contains { $0.sourceNodeID == "physical-estop" || $0.destinationNodeID == "physical-estop" })
        XCTAssertFalse(RobotSystemWorkflowCatalog.interfaces.flatMap(\.edgeIDs).contains { id in
            guard let edge = RobotTopologyCatalog.edge(id) else { return false }
            return edge.sourceNodeID == "physical-estop" || edge.destinationNodeID == "physical-estop"
        })
    }

    func testEveryStepHasPlainLanguageInputProcessAndOutput() {
        let steps = RobotSystemWorkflowCatalog.phases.flatMap { $0.lanes.flatMap(\.steps) }
        XCTAssertFalse(steps.isEmpty)
        XCTAssertEqual(steps.count, Set(steps.map(\.id)).count)
        for step in steps {
            XCTAssertFalse(step.input.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty, step.id)
            XCTAssertFalse(step.process.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty, step.id)
            XCTAssertFalse(step.output.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty, step.id)
            XCTAssertFalse(step.nodeIDs.isEmpty, step.id)
            XCTAssertTrue(step.nodeIDs.allSatisfy { RobotTopologyCatalog.node($0) != nil }, step.id)
        }
    }

    func testPointCloudAndIMUArePresentedAsParallelInputsToPointLIO() throws {
        let perception = try XCTUnwrap(RobotSystemWorkflowCatalog.phase("perception"))
        let localization = try XCTUnwrap(perception.lanes.first { $0.id == "localization" })
        let sensors = try XCTUnwrap(localization.steps.first { $0.id == "sensor-inputs" })
        let pointLIO = try XCTUnwrap(localization.steps.first { $0.id == "localization-estimate" })

        XCTAssertTrue(sensors.nodeIDs.contains("mid360"))
        XCTAssertTrue(sensors.nodeIDs.contains("imu-transform"))
        XCTAssertTrue(sensors.nodeIDs.contains("imu-filter"))
        XCTAssertTrue(sensors.note.contains("并行支路"))
        XCTAssertTrue(pointLIO.incomingEdgeIDs.contains("mid360-pointlio"))
        XCTAssertTrue(pointLIO.incomingEdgeIDs.contains("imu-filter-pointlio"))
    }

    func testDefaultOptionalAndOfflineLinksKeepTheirEvidenceGrades() throws {
        let optional = try XCTUnwrap(RobotTopologyCatalog.edge("arm2nav-arm-planner"))
        XCTAssertEqual(optional.evidence, .conditionalSourceWiring)

        let navigation = try XCTUnwrap(RobotTopologyCatalog.edge("mission-nav2"))
        let armTask = try XCTUnwrap(RobotTopologyCatalog.edge("mission-arm-request"))
        let chassis = try XCTUnwrap(RobotTopologyCatalog.edge("nav2-chassis"))
        XCTAssertEqual(navigation.evidence, .offlineExercise)
        XCTAssertEqual(armTask.evidence, .offlineExercise)
        XCTAssertEqual(chassis.evidence, .offlineExercise)
        XCTAssertTrue(try XCTUnwrap(RobotTopologyCatalog.node("mission-coordinator")).summary.contains("实体主启动流程集成状态待确认"))
    }

    func testExecutionStageDoesNotClaimChassisAcknowledgementOrConnectedEstop() throws {
        let chassis = try XCTUnwrap(RobotTopologyCatalog.edge("chassis-bridge-mcu"))
        XCTAssertTrue(chassis.sourceNote.contains("没有 STM32 acknowledgement"))
        let execution = try XCTUnwrap(RobotSystemWorkflowCatalog.phase("execution"))
        let estop = try XCTUnwrap(execution.lanes.flatMap(\.steps).first { $0.id == "estop-boundary" })
        XCTAssertTrue(estop.output.contains("无可验证连接"))
        XCTAssertTrue(estop.incomingEdgeIDs.isEmpty)
    }

    func testCrossPhaseInterfacesPreserveDirectionAndFeedbackClassification() throws {
        let perceptionToPlanning = try XCTUnwrap(RobotSystemWorkflowCatalog.interface("perception-to-planning"))
        XCTAssertEqual(perceptionToPlanning.sourcePhaseID, "perception")
        XCTAssertEqual(perceptionToPlanning.destinationPhaseID, "planning")
        XCTAssertFalse(perceptionToPlanning.isFeedback)

        let controllerFeedback = try XCTUnwrap(RobotSystemWorkflowCatalog.interface("execution-to-control"))
        XCTAssertEqual(controllerFeedback.sourcePhaseID, "execution")
        XCTAssertEqual(controllerFeedback.destinationPhaseID, "control")
        XCTAssertTrue(controllerFeedback.isFeedback)

        let softwareFeedback = try XCTUnwrap(RobotSystemWorkflowCatalog.interface("control-to-planning-feedback"))
        XCTAssertTrue(softwareFeedback.isFeedback)
        XCTAssertTrue(softwareFeedback.edgeIDs.contains("arm-safety-mission"))
        XCTAssertTrue(softwareFeedback.edgeIDs.contains("chassis-status-mission"))
    }
}
