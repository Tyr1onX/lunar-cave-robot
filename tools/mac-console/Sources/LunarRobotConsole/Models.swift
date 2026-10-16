import Foundation

struct Reading: Decodable {
    let status: String
    let reason: String?
}

struct CPUReading: Decodable {
    let status: String
    let percent: Double?
    let reason: String?
}

struct MemoryReading: Decodable {
    let status: String
    let totalBytes: Int64?
    let availableBytes: Int64?
    let usedPercent: Double?
    let reason: String?
}

struct DiskReading: Decodable {
    let status: String
    let path: String?
    let totalBytes: Int64?
    let usedBytes: Int64?
    let freeBytes: Int64?
    let usedPercent: Double?
    let reason: String?
}

struct TemperatureSensor: Decodable, Identifiable {
    var id: String { "\(label)-\(source)" }
    let label: String
    let celsius: Double
    let source: String
}

struct TemperatureReading: Decodable {
    let status: String
    let readings: [TemperatureSensor]?
    let reason: String?
}

struct SystemReadings: Decodable {
    let cpu: CPUReading
    let memory: MemoryReading
    let disk: DiskReading
    let temperature: TemperatureReading
}

struct GitReading: Decodable {
    let status: String
    let headSha: String?
    let branch: String?
    let dirty: Bool?
    let dirtyFiles: Int?
    let repositoryPath: String?
    let reason: String?
}

struct DeploymentReading: Decodable {
    let status: String
    let sha: String?
    let source: String?
    let reason: String?
}

struct ROSNode: Decodable, Identifiable {
    var id: String { "\(namespace):\(name)" }
    let name: String
    let namespace: String
}

struct ROSTopic: Decodable, Identifiable {
    var id: String { name }
    let name: String
    let types: [String]
}

struct ROSReading: Decodable {
    let status: String
    let domainId: Int?
    let nodes: [ROSNode]
    let topics: [ROSTopic]
    let observedAt: String?
    let graphAgeMs: Int?
    let expectedDomainId: Int?
    let expectedDomainError: String?
    let expectedNodes: [String]?
    let expectedNodesError: String?
    let discovery: String?
    let reason: String?
}

struct TestRecord: Decodable, Identifiable {
    var id: String { recordID }
    private let recordID: String
    let timestamp: String?
    let result: String
    let test: String
    let scope: String
    let sha: String?
    let branch: String?
    let artifact: String?
    let recordPath: String?

    enum CodingKeys: String, CodingKey {
        case recordID = "id"
        case timestamp, result, test, scope, sha, branch, artifact, recordPath
    }
}

struct TestReading: Decodable {
    let status: String
    let records: [TestRecord]
    let reason: String?
}

struct SafetyReading: Decodable {
    let physicalEstop: Reading
    let hardwareHealth: Reading
    let control: String
}

struct ConsoleSnapshot: Decodable {
    let schemaVersion: Int
    let collectedAt: String
    let source: String
    let host: HostReading
    let system: SystemReadings
    let git: GitReading
    let deployment: DeploymentReading
    let ros: ROSReading
    let tests: TestReading
    let safety: SafetyReading
}

struct HostReading: Decodable {
    let hostname: String
    let platform: String
    let osRelease: [String]

    var operatingSystem: String {
        guard let pretty = osRelease.first(where: { $0.hasPrefix("PRETTY_NAME=") }),
              let value = pretty.split(separator: "=", maxSplits: 1).last else { return "Linux" }
        return String(value).trimmingCharacters(in: CharacterSet(charactersIn: "\""))
    }
}

struct LogEntry: Decodable, Identifiable {
    var id: String { "\(fileModifiedAt)-\(source)-\(message)" }
    let fileModifiedAt: String
    let severity: String
    let message: String
    let source: String
}

struct LogReading: Decodable {
    let status: String
    let reason: String?
    let entries: [LogEntry]
    let source: String?
}

struct HealthReading: Decodable {
    let ok: Bool
    let schemaVersion: Int
    let collectedAt: String
}

enum VisionStreamID: String, CaseIterable, Identifiable {
    case rgb
    case depth
    case yolo

    var id: String { rawValue }
    var label: String {
        switch self {
        case .rgb: return "RGB"
        case .depth: return "深度"
        case .yolo: return "YOLO"
        }
    }
}

struct VisionStreamReading: Decodable, Identifiable {
    let id: String
    let label: String
    let topic: String
    let topicType: String?
    let status: String
    let reason: String?
    let publisherCount: Int
    let width: Int?
    let height: Int?
    let encoding: String?
    let frameRateHz: Double?
    let frameAgeMs: Int?
    let sourceStamp: String?
    let frameId: String?
    let headerToAgentMs: Double?
    let receivedAt: String?
    let gapEvents: Int
}

struct VisionReading: Decodable {
    let status: String
    let source: String
    let collectedAt: String
    let reason: String?
    let streams: [VisionStreamReading]

    func stream(_ id: VisionStreamID) -> VisionStreamReading? {
        streams.first { $0.id == id.rawValue }
    }
}

extension Int64 {
    var formattedBytes: String {
        ByteCountFormatter.string(fromByteCount: self, countStyle: .file)
    }
}

extension String {
    var shortSHA: String {
        String(prefix(12))
    }
}
