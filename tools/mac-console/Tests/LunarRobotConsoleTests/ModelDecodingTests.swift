import Foundation
import XCTest
@testable import LunarRobotConsole

final class ModelDecodingTests: XCTestCase {
    func testWorkspaceExecutionPresentationKeepsModeAndHardwareClaimsConservative() {
        XCTAssertEqual(
            WorkspaceExecutionPresentation.label(executionState: "not_running", runtimeStatus: "not_running"),
            "未启动（扫描范围内未发现关联进程）"
        )
        XCTAssertEqual(
            WorkspaceExecutionPresentation.label(executionState: "simulation_running", runtimeStatus: "running"),
            "仿真运行（进程环境声明）"
        )
        XCTAssertEqual(
            WorkspaceExecutionPresentation.label(executionState: "hardware_declared_running", runtimeStatus: "running"),
            "实体模式已声明 · 硬件状态未知"
        )
        XCTAssertEqual(
            WorkspaceExecutionPresentation.label(executionState: "unknown", runtimeStatus: "running"),
            "进程已发现 · 执行模式未知"
        )
        XCTAssertEqual(
            WorkspaceExecutionPresentation.label(executionState: nil, runtimeStatus: "not_running"),
            "未启动（旧版 Agent 扫描结果）"
        )
    }

    func testOlderAgentWorkspaceTelemetryWithoutExecutionFieldsStillDecodes() throws {
        let data = Data(#"{"status":"ok","workspaces":[{"id":"original-navigation","label":"原始导航与雷达工程","source_kind":"original","source_status":"present","runtime_status":"not_running","run_mode":"unknown","run_expectation":"unconfigured","graph_relationship":"unknown","processes":[{"pid":321,"process_name":"ros2","package":"bringup","domain_id":187,"domain_evidence":"observed","localhost_only":"1","run_mode":"unknown","matched_by":["command_path"],"observed_at":"2026-10-16T10:00:00Z"}],"modules":[{"id":"navigation","label":"导航","source_status":"present","runtime_status":"unknown","run_expectation":"unconfigured"}]}]}"#.utf8)
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let inventory = try decoder.decode(WorkspaceInventoryReading.self, from: data)

        XCTAssertNil(inventory.entries.first?.executionState)
        XCTAssertNil(inventory.entries.first?.processes?.first?.runModeEvidence)
        XCTAssertNil(inventory.entries.first?.modules?.first?.executionState)
        XCTAssertEqual(
            WorkspaceExecutionPresentation.label(
                executionState: inventory.entries.first?.executionState,
                runtimeStatus: inventory.entries.first?.runtimeStatus
            ),
            "未启动（旧版 Agent 扫描结果）"
        )
    }

    func testSnapshotDecodesSnakeCaseAndUnknownStatuses() throws {
        let data = Data(#"{"schema_version":1,"collected_at":"2026-10-13T05:00:00Z","source":"nuc-read-only-agent","host":{"hostname":"robot","platform":"x86_64","os_release":["PRETTY_NAME=\"Ubuntu 22.04.5 LTS\""]},"system":{"cpu":{"status":"ok","percent":12.5},"memory":{"status":"unknown","reason":"missing"},"disk":{"status":"ok","total_bytes":1000,"used_bytes":400,"free_bytes":600,"used_percent":40},"temperature":{"status":"unknown","reason":"not exposed"}},"git":{"status":"ok","head_sha":"0123456789012345678901234567890123456789","branch":"main","dirty":false,"dirty_files":0},"deployment":{"status":"unknown","reason":"not recorded"},"ros":{"status":"ok","domain_id":0,"nodes":[],"topics":[],"discovery":"runtime"},"tests":{"status":"ok","records":[{"id":"ros-nav-build","timestamp":"2026-10-12T23:46:03Z","result":"PASS","test":"colcon_build","scope":"Nav2_rea_ws","sha":"abcdef0123456789","branch":"main","record_path":"/state/test.yaml"}]},"safety":{"physical_estop":{"status":"unknown","reason":"not connected"},"hardware_health":{"status":"unknown","reason":"no hardware evidence"},"control":"disabled"}}"#.utf8)
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let snapshot = try decoder.decode(ConsoleSnapshot.self, from: data)
        XCTAssertEqual(snapshot.host.hostname, "robot")
        XCTAssertEqual(snapshot.git.headSha?.count, 40)
        XCTAssertEqual(snapshot.deployment.status, "unknown")
        XCTAssertEqual(snapshot.tests.records.first?.scope, "Nav2_rea_ws")
        XCTAssertEqual(snapshot.safety.control, "disabled")
        XCTAssertNil(snapshot.ros.observedAt, "The console must decode snapshots from Agents predating health telemetry.")
        XCTAssertNil(snapshot.ros.graphAgeMs)
        XCTAssertNil(snapshot.ros.expectedDomainId)
        XCTAssertNil(snapshot.ros.expectedNodes)
        XCTAssertNil(snapshot.workspaces, "The console must remain compatible with Agents that predate workspace telemetry.")
    }

    func testWorkspaceInventoryDecodesNewAndOriginalSourcesAndEvidence() throws {
        let data = Data(#"{"status":"ok","source":"nuc-read-only-workspace-and-process-observer","collected_at":"2026-10-16T10:00:00Z","process_scan":{"status":"ok","scope":"Agent UID; ROS/workspace candidates only","source":"/proc","observed_at":"2026-10-16T10:00:00Z","reason":null},"graph_source":{"domain_id":187,"observed_at":"2026-10-16T10:00:00Z","graph_age_ms":120,"source":"existing Agent ROS graph observer"},"workspaces":[{"id":"new-engineering","label":"新版工程","source_kind":"new","root":"/home/tzb/lunar-robot/code","source_status":"present","source_observed_at":"2026-10-16T10:00:00Z","package_count":33,"packages":[{"name":"bringup","path":"Nav2_rea_ws/src/bringup/package.xml"}],"launch_count":4,"launch_files":["Nav2_rea_ws/src/bringup/launch/main.launch.py"],"config_count":2,"config_files":["Nav2_rea_ws/src/bringup/config/nav2.yaml"],"runtime_status":"running","execution_state":"simulation_running","runtime_scope":"Agent UID only","runtime_observed_at":"2026-10-16T10:00:00Z","process_count":1,"processes":[{"pid":321,"process_name":"component_container_mt","package":"bringup","domain_id":187,"domain_evidence":"observed","localhost_only":"1","run_mode":"simulation","run_mode_evidence":"explicit_environment","matched_by":["overlay_environment"],"observed_at":"2026-10-16T10:00:00Z"}],"domain_ids":[187],"run_mode":"simulation","run_expectation":"unconfigured","graph_domain_id":187,"graph_observed_at":"2026-10-16T10:00:00Z","graph_relationship":"shared_domain","graph_reason":"same explicit Domain","modules":[{"id":"navigation","label":"导航","packages":["bringup"],"source_status":"present","runtime_status":"running","execution_state":"simulation_running","run_expectation":"unconfigured","interface_topics":["/scan","/map","/cmd_vel"],"observed_interface_topics":["/scan"],"reason":"Source interface candidates"}],"reason":null},{"id":"original-navigation","label":"原始导航与雷达工程","source_kind":"original","root":"/home/tzb/Nav2_rea_ws","source_status":"present","package_count":18,"launch_count":3,"config_count":5,"runtime_status":"not_running","execution_state":"not_running","runtime_scope":"Agent UID only","process_count":0,"domain_ids":[],"run_mode":"unknown","run_expectation":"unconfigured","graph_relationship":"unknown","modules":[],"reason":null}]}"#.utf8)

        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let inventory = try decoder.decode(WorkspaceInventoryReading.self, from: data)

        XCTAssertEqual(inventory.source, "nuc-read-only-workspace-and-process-observer")
        XCTAssertEqual(inventory.graphSource?.domainId, 187)
        XCTAssertEqual(inventory.entries.compactMap(\.sourceKind), ["new", "original"])
        XCTAssertEqual(inventory.entries[0].runMode, "simulation")
        XCTAssertEqual(inventory.entries[0].executionState, "simulation_running")
        XCTAssertEqual(inventory.entries[0].processes?.first?.domainId, 187)
        XCTAssertEqual(inventory.entries[0].processes?.first?.runModeEvidence, "explicit_environment")
        XCTAssertEqual(inventory.entries[0].modules?.first?.interfaceTopics, ["/scan", "/map", "/cmd_vel"])
        XCTAssertEqual(inventory.entries[0].modules?.first?.executionState, "simulation_running")
        XCTAssertEqual(inventory.entries[0].modules?.first?.observedInterfaceTopics, ["/scan"])
        XCTAssertEqual(inventory.entries[1].runtimeStatus, "not_running")
        XCTAssertEqual(inventory.entries[1].executionState, "not_running")
        XCTAssertEqual(inventory.entries[1].runExpectation, "unconfigured")
        XCTAssertTrue(inventory.hasUnconfirmedHealth)
    }

    func testLogEntryDecodesFileModificationTimeAsMetadata() throws {
        let data = Data(#"{"file_modified_at":"2026-10-12T23:45:00Z","severity":"info","message":"started","source":"node.log"}"#.utf8)
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let entry = try decoder.decode(LogEntry.self, from: data)
        XCTAssertEqual(entry.fileModifiedAt, "2026-10-12T23:45:00Z")
        XCTAssertEqual(entry.source, "node.log")
    }

    func testVisionSnapshotDecodesLiveAndUnavailableSourcesWithoutInventingMetrics() throws {
        let data = Data(#"{"status":"ok","source":"nuc-ros2-read-only-subscriber","collected_at":"2026-10-13T05:00:00.000Z","reason":null,"streams":[{"id":"rgb","label":"RGB","topic":"/camera/color/image_raw","topic_type":"sensor_msgs/msg/Image","status":"streaming","reason":null,"publisher_count":1,"width":640,"height":480,"encoding":"rgb8","frame_rate_hz":29.8,"frame_age_ms":12,"source_stamp":"2026-10-13T05:00:00.000Z","header_to_agent_ms":8.2,"received_at":"2026-10-13T05:00:00.008Z","gap_events":0},{"id":"depth","label":"深度","topic":"/camera/depth/image_rect_raw","topic_type":null,"status":"unavailable","reason":"ROS Topic 当前没有发布者","publisher_count":0,"width":null,"height":null,"encoding":null,"frame_rate_hz":null,"frame_age_ms":null,"source_stamp":null,"header_to_agent_ms":null,"received_at":null,"gap_events":0}]}"#.utf8)
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let reading = try decoder.decode(VisionReading.self, from: data)
        XCTAssertEqual(reading.stream(.rgb)?.status, "streaming")
        XCTAssertEqual(reading.stream(.rgb)?.frameRateHz, 29.8)
        XCTAssertEqual(reading.stream(.depth)?.status, "unavailable")
        XCTAssertNil(reading.stream(.depth)?.frameRateHz)
        XCTAssertNil(reading.stream(.depth)?.sourceStamp)
    }

    func testModuleTestCatalogAndSavedOfflineReportDecodeSnakeCase() throws {
        let catalogData = Data(#"{"service":"lunar-console-module-test-runner","source":"nuc-restricted-module-test-runner","busy":false,"tests":[{"id":"arm_fk","category":"机械臂离线模型","title":"机械臂 FK / 运动学","description":"现有 URDF FK 检查","mode":"OFFLINE","state":"ready","reason":null,"input_source":"仓库 URDF 与固定六轴姿态","prerequisites":["现有 URDF"],"timeout_seconds":20}],"collected_at":"2026-10-13T05:00:00Z"}"#.utf8)
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        let catalog = try decoder.decode(ModuleTestCatalog.self, from: catalogData)
        XCTAssertFalse(catalog.busy)
        XCTAssertEqual(catalog.tests.first?.id, "arm_fk")
        XCTAssertEqual(catalog.tests.first?.category, "机械臂离线模型")
        XCTAssertEqual(catalog.tests.first?.mode, "OFFLINE")
        XCTAssertEqual(catalog.tests.first?.state, "ready")

        let reportData = Data(#"{"run_id":"0123456789abcdef0123456789abcdef","test_id":"arm_fk","mode":"OFFLINE","status":"passed","result":"passed","code_sha":"0123456789012345678901234567890123456789","code_dirty":false,"started_at":"2026-10-13T05:00:00Z","finished_at":"2026-10-13T05:00:01Z","duration_ms":1000,"reason":null}"#.utf8)
        let report = try decoder.decode(ModuleTestRun.self, from: reportData)
        XCTAssertEqual(report.status, "passed")
        XCTAssertEqual(report.durationMs, 1000)
        XCTAssertEqual(report.codeSha?.count, 40)

        let historyData = Data(#"{"reports":[{"run_id":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","test_id":"p7_scenarios","title":"P7 PTY 仿真","mode":"SIMULATION","status":"interrupted","result":"interrupted","code_sha":"0123456789012345678901234567890123456789","started_at":"2026-10-14T05:00:00Z","finished_at":"2026-10-14T05:02:00Z","duration_ms":120000,"reason":"service restart"}]}"#.utf8)
        let history = try decoder.decode(ModuleTestHistoryResponse.self, from: historyData)
        XCTAssertEqual(history.reports.first?.testId, "p7_scenarios")
        XCTAssertEqual(history.reports.first?.status, "interrupted")
    }
}
