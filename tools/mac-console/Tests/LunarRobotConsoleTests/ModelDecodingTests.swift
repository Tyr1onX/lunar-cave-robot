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
}
