import Foundation
import XCTest
@testable import LunarRobotConsole

final class ModelDecodingTests: XCTestCase {
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
}
