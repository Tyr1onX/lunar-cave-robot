import Foundation

enum PipelineReportFilename {
    static func make(for date: Date) -> String {
        let formatter = DateFormatter()
        formatter.locale = Locale(identifier: "en_US_POSIX")
        formatter.timeZone = TimeZone(secondsFromGMT: 0)
        formatter.dateFormat = "yyyy-MM-dd'T'HH-mm-ss'Z'"
        return "lunar-pipeline-diagnosis-\(formatter.string(from: date)).md"
    }
}

enum PipelineStageID: String, CaseIterable, Codable, Identifiable {
    case radar
    case pointLIO = "point-lio"
    case slam
    case nav2

    var id: String { rawValue }

    var title: String {
        switch self {
        case .radar: return "雷达采集"
        case .pointLIO: return "定位与障碍处理"
        case .slam: return "扫描与地图生成"
        case .nav2: return "导航规划"
        }
    }

    var workflowStepID: String {
        switch self {
        case .radar: return "sensor-inputs"
        case .pointLIO: return "localization-estimate"
        case .slam: return "map-generation"
        case .nav2: return "route-planning"
        }
    }

    var sourcePaths: [String] {
        switch self {
        case .radar:
            return [
                "Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py",
                "Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2/launch/msg_MID360_launch.py",
                "Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml"
            ]
        case .pointLIO:
            return [
                "Nav2_rea_ws/src/localization/point_lio/config/mid360.yaml",
                "Nav2_rea_ws/src/localization/point_lio/src/laserMapping.cpp",
                "Nav2_rea_ws/src/navigation/terrain_analysis/src/terrain_analysis_plus.cpp",
                "Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"
            ]
        case .slam:
            return [
                "Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py",
                "Nav2_rea_ws/src/bringup/config/slam_toolbox_single_lidar.yaml"
            ]
        case .nav2:
            return [
                "Nav2_rea_ws/src/bringup/config/nav2_params.yaml",
                "Nav2_rea_ws/src/navigation/rm_navigation/launch/navigation_launch.py",
                "Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py"
            ]
        }
    }

    var possibleCauses: [String] {
        switch self {
        case .radar:
            return ["驱动或设备未运行", "当前 ROS Domain 中没有该数据源", "网络、设备配置或时钟异常"]
        case .pointLIO:
            return ["IMU 预处理或 Point-LIO 输入未更新", "定位或地形分析节点未运行 / 处理受阻", "消息类型、时间戳、TF 或参数不匹配"]
        case .slam:
            return ["地形点云到 /scan 的转换路径中断", "SLAM Toolbox 未运行或未接受扫描", "扫描坐标、TF 或时间戳不满足配置"]
        case .nav2:
            return ["地图、障碍点云或 TF 输入不可用", "Nav2 生命周期节点或配置未就绪", "尚无导航目标时没有 /cmd_vel 属正常可能性"]
        }
    }

    var logTerms: [String] {
        switch self {
        case .radar: return ["livox", "mid360", "/livox/lidar", "/livox/imu"]
        case .pointLIO: return ["point_lio", "point-lio", "terrain_analysis", "/odometry", "/cloud_registered", "/terrain_map"]
        case .slam: return ["slam_toolbox", "pointcloud_to_laserscan", "/scan", "/map"]
        case .nav2: return ["nav2", "controller_server", "planner_server", "bt_navigator", "/cmd_vel"]
        }
    }
}

enum PipelineScenario: String, CaseIterable, Codable, Identifiable {
    case unconfigured
    case newWorkspaceSingleLidar
    case originalWorkspaceSingleLidar

    var id: String { rawValue }

    var label: String {
        switch self {
        case .unconfigured: return "未配置运行预期"
        case .newWorkspaceSingleLidar: return "新版工程 · 单雷达建图与导航"
        case .originalWorkspaceSingleLidar: return "原始 Nav2_rea_ws · 单雷达建图与导航"
        }
    }

    var workspaceRoot: String? {
        switch self {
        case .unconfigured: return nil
        case .newWorkspaceSingleLidar: return "~/lunar-robot/code/Nav2_rea_ws"
        case .originalWorkspaceSingleLidar: return "/home/tzb/Nav2_rea_ws"
        }
    }

    var isConfigured: Bool { self != .unconfigured }
}

struct PipelineRunExpectation: Codable, Equatable {
    var scenario: PipelineScenario
    var expectedDomainID: Int?
    var expectedNodesByStage: [PipelineStageID: [String]]
    var navigationGoalActive: Bool

    static let unconfigured = PipelineRunExpectation(
        scenario: .unconfigured,
        expectedDomainID: nil,
        expectedNodesByStage: [:],
        navigationGoalActive: false
    )

    var configurationError: String? {
        if let expectedDomainID, !(0...232).contains(expectedDomainID) {
            return "预期 ROS Domain 必须是 0 到 232 的整数。"
        }
        guard scenario.isConfigured else { return nil }
        guard expectedDomainID != nil else { return "运行场景已选择，但预期 ROS Domain 尚未填写。" }
        for (stage, names) in expectedNodesByStage {
            for name in names where !Self.isValidNodeName(name) {
                return "\(stage.title)的预期节点名无效：\(name)；节点名应为绝对 ROS 名称，例如 /point_lio。"
            }
        }
        return nil
    }

    private static func isValidNodeName(_ value: String) -> Bool {
        value.range(of: #"^/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*$"#, options: .regularExpression) != nil
    }
}

enum PipelineStageState: String, Codable {
    case observed
    case confirmedExpectationMismatch
    case suspectedGap
    case possiblyAffected
    case notConfigured
    case unknown

    var label: String {
        switch self {
        case .observed: return "收到数据 · 模块处理未确认"
        case .confirmedExpectationMismatch: return "确认的运行预期偏差"
        case .suspectedGap: return "疑似链路中断"
        case .possiblyAffected: return "上游问题 · 可能受影响"
        case .notConfigured: return "运行预期未配置"
        case .unknown: return "证据不足 · 未知"
        }
    }
}

enum PipelineEvidenceConfidence: String, Codable {
    case confirmedExpectationMismatch
    case suspected
    case insufficient
    case observedOnly

    var label: String {
        switch self {
        case .confirmedExpectationMismatch: return "确认：不满足显式节点预期"
        case .suspected: return "疑似：数据链路证据缺口"
        case .insufficient: return "证据不足"
        case .observedOnly: return "仅确认 Agent 收到数据"
        }
    }
}

enum PipelineGapKind: String, Codable {
    case nodeMissing
    case streamStale
    case publisherMissing
    case publisherHasNoData
    case telemetryUnknown
}

struct PipelineStreamEvidence: Codable, Equatable, Identifiable {
    var id: String { topic }
    let topic: String
    let role: String
    let status: String
    let reason: String?
    let publisherCount: Int?
    let publisherEndpoints: [String]
    let workspaceAttribution: String
    let receivedAt: String?
    let sourceStamp: String?
    let ageMs: Int?
    let staleAfterMs: Int?
    let receiveRateHz: Double?
    let headerToAgentMs: Double?
    let sampleCount: Int?
    let subscriptionGeneration: Int?
    let receiveCount: Int?

    var isFresh: Bool {
        status == "streaming" && ageMs != nil && receivedAt != nil &&
            (staleAfterMs == nil || ageMs! <= staleAfterMs!)
    }

    var hasGapEvidence: Bool { ["stale", "unavailable", "waiting"].contains(status) }

    var oneLineEvidence: String {
        let rate = receiveRateHz.map { String(format: "%.2f Hz", $0) } ?? "频率未知"
        let age = ageMs.map { "\($0) ms" } ?? "接收时间未知"
        let publishers = publisherCount.map(String.init) ?? "未知"
        return "\(status) · \(rate) · 帧龄 \(age) · 发布者 \(publishers)"
    }
}

struct PipelineNodeEvidence: Codable, Equatable, Identifiable {
    var id: String { name }
    let name: String
    let expected: Bool
    let visible: Bool?
    let observedAt: String?
}

struct PipelineLogEvidence: Codable, Equatable, Identifiable {
    var id: String { "\(time)|\(source)|\(message)" }
    let time: String
    let severity: String
    let source: String
    let message: String
}

struct PipelineStageDiagnosis: Codable, Equatable, Identifiable {
    let id: PipelineStageID
    var state: PipelineStageState
    var confidence: PipelineEvidenceConfidence
    var summary: String
    var firstGapKind: PipelineGapKind?
    var firstGapTopics: [String]
    var observedAt: String
    var dataTime: String
    var inputEvidence: [PipelineStreamEvidence]
    var outputEvidence: [PipelineStreamEvidence]
    var nodeEvidence: [PipelineNodeEvidence]
    var graphOnlyTopics: [String]
    var logs: [PipelineLogEvidence]
    var sourcePaths: [String]
    var possibleCauses: [String]

    var workflowStepID: String { id.workflowStepID }
}

struct PipelineFinding: Codable, Equatable {
    let stageID: PipelineStageID
    let kind: PipelineGapKind
    let confidence: PipelineEvidenceConfidence
    let title: String
    let observation: String
    let observedAt: String
    var firstObservedAt: String
    var observationCount: Int
    let evidenceTopics: [String]

    var isSustained: Bool { observationCount > 1 }
    var persistenceLabel: String { isSustained ? "持续异常 · \(observationCount) 次观测" : "单次快照" }
    var workflowStepID: String {
        if evidenceTopics.contains(where: { $0 == "/livox/lidar" || $0 == "/livox/imu" }) { return "sensor-inputs" }
        if evidenceTopics.contains(where: { ["/imu/data", "/odometry", "/cloud_registered"].contains($0) }) { return "localization-estimate" }
        if evidenceTopics.contains("/terrain_map") { return "terrain-filter" }
        if evidenceTopics.contains("/scan") { return "scan-generation" }
        if evidenceTopics.contains(where: { ["/map", "/tf", "/tf_static"].contains($0) }) { return stageID == .nav2 ? "route-planning" : "map-generation" }
        return stageID.workflowStepID
    }
}

struct PipelineDiagnosis: Codable, Equatable {
    let scenario: PipelineScenario
    let expectedDomainID: Int?
    let actualDomainID: Int?
    let observedAt: String
    let blockedReason: String?
    let summary: String
    let stages: [PipelineStageDiagnosis]
    var firstFinding: PipelineFinding?

    var findingStageID: PipelineStageID? { firstFinding?.stageID }
    var hasConfiguredExpectation: Bool { scenario.isConfigured }

    func stage(_ id: PipelineStageID) -> PipelineStageDiagnosis? { stages.first { $0.id == id } }

    var affectedWorkflowStepIDs: Set<String> {
        guard let firstFinding else { return [] }
        switch firstFinding.workflowStepID {
        case "sensor-inputs":
            return ["localization-estimate", "terrain-filter", "scan-generation", "map-generation", "route-planning"]
        case "localization-estimate":
            return ["terrain-filter", "scan-generation", "map-generation", "route-planning"]
        case "terrain-filter":
            return ["scan-generation", "map-generation", "route-planning"]
        case "scan-generation":
            return ["map-generation", "route-planning"]
        case "map-generation":
            return ["route-planning"]
        default:
            return []
        }
    }

    func workflowState(for stepID: String) -> PipelineStageState? {
        guard let finding = firstFinding else { return nil }
        if finding.workflowStepID == stepID {
            return finding.confidence == .confirmedExpectationMismatch ? .confirmedExpectationMismatch : .suspectedGap
        }
        return affectedWorkflowStepIDs.contains(stepID) ? .possiblyAffected : nil
    }
}

struct PipelineDiagnosisTracker {
    private struct Record {
        var firstObservedAt: String
        var lastObservedAt: String
        var count: Int
        var key: String
    }

    private var record: Record?

    mutating func update(_ diagnosis: PipelineDiagnosis) -> PipelineDiagnosis {
        guard var finding = diagnosis.firstFinding else {
            record = nil
            return diagnosis
        }
        let key = "\(finding.stageID.rawValue)|\(finding.kind.rawValue)|\(finding.evidenceTopics.sorted().joined(separator: ","))"
        if var current = record, current.key == key {
            if current.lastObservedAt != finding.observedAt {
                current.lastObservedAt = finding.observedAt
                current.count += 1
            }
            record = current
            finding.firstObservedAt = current.firstObservedAt
            finding.observationCount = current.count
        } else {
            record = Record(firstObservedAt: finding.observedAt, lastObservedAt: finding.observedAt, count: 1, key: key)
            finding.firstObservedAt = finding.observedAt
            finding.observationCount = 1
        }
        var result = diagnosis
        result.firstFinding = finding
        return result
    }
}

enum PipelineDiagnosisEvaluator {
    private struct StageSpec {
        let id: PipelineStageID
        let inputs: [String]
        let outputs: [String]
        let optionalOutputs: [String]
    }

    private static let stageSpecs: [StageSpec] = [
        StageSpec(id: .radar, inputs: [], outputs: ["/livox/lidar", "/livox/imu"], optionalOutputs: []),
        StageSpec(id: .pointLIO, inputs: ["/livox/lidar", "/imu/data"], outputs: ["/odometry", "/cloud_registered", "/terrain_map"], optionalOutputs: []),
        StageSpec(id: .slam, inputs: ["/scan"], outputs: ["/map"], optionalOutputs: []),
        StageSpec(id: .nav2, inputs: ["/map", "/terrain_map"], outputs: ["/cmd_vel"], optionalOutputs: ["/cmd_vel"])
    ]

    static func evaluate(
        expectation: PipelineRunExpectation,
        agentOnline: Bool,
        snapshot: ConsoleSnapshot?,
        telemetry: LinkTelemetryReading?,
        logs: LogReading?,
        evaluationTime: Date = Date()
    ) -> PipelineDiagnosis {
        let evaluatedAt = iso8601(evaluationTime)
        let actualDomain = telemetry?.domainId ?? snapshot?.ros.domainId
        let blocked = blockReason(expectation: expectation, agentOnline: agentOnline, snapshot: snapshot, telemetry: telemetry)
        let graphTopics = Set(snapshot?.ros.topics.map(\.name) ?? [])
        let effectiveNodes = Set((snapshot?.ros.nodes ?? []).map(fullNodeName))
        var stages: [PipelineStageDiagnosis] = []
        var firstFinding: PipelineFinding?
        var cascadeBlocked = false

        for spec in stageSpecs {
            let stage = spec.id
            let expectedNodeNames = (expectation.expectedNodesByStage[stage] ?? []).map(normalizeNodeName)
            let nodeEvidence = expectedNodeNames.map { name in
                PipelineNodeEvidence(
                    name: name,
                    expected: true,
                    visible: snapshot?.ros.status == "ok" ? effectiveNodes.contains(name) : nil,
                    observedAt: snapshot?.ros.observedAt
                )
            }
            let allTopics = orderedUnique(spec.inputs + spec.outputs.filter { $0 != "/cmd_vel" || expectation.navigationGoalActive })
            let inputEvidence = spec.inputs.compactMap { stream(topic: $0, role: "輸入", telemetry: telemetry) }
            let outputTopics = spec.outputs.filter { $0 != "/cmd_vel" || expectation.navigationGoalActive }
            let outputEvidence = outputTopics.compactMap { topic in
                stream(topic: topic, role: "輸出", telemetry: telemetry)
            }.filter { !spec.inputs.contains($0.topic) }
            let selected = inputEvidence + outputEvidence
            let logsForStage = matchingLogs(logs, stage: stage)
            let graphOnly = graphTopics.intersection(stage == .nav2 ? ["/tf", "/tf_static"] : []).sorted()
            let lastStreamTime = selected.compactMap(\.receivedAt).max() ?? "未知"
            let observedAt = telemetry?.collectedAt ?? snapshot?.ros.observedAt ?? evaluatedAt

            if !expectation.scenario.isConfigured {
                stages.append(stageResult(
                    id: stage, state: .notConfigured, confidence: .insufficient,
                    summary: "未设置该运行场景的预期；只列出可用观测，不生成缺失告警。",
                    kind: nil, gapTopics: [], observedAt: observedAt, dataTime: lastStreamTime,
                    inputs: inputEvidence, outputs: outputEvidence, nodes: nodeEvidence,
                    graphOnly: graphOnly, logs: logsForStage
                ))
                continue
            }

            if let blocked {
                stages.append(stageResult(
                    id: stage, state: .unknown, confidence: .insufficient, summary: blocked,
                    kind: nil, gapTopics: [], observedAt: observedAt, dataTime: lastStreamTime,
                    inputs: inputEvidence, outputs: outputEvidence, nodes: nodeEvidence,
                    graphOnly: graphOnly, logs: logsForStage
                ))
                continue
            }

            if cascadeBlocked {
                stages.append(stageResult(
                    id: stage, state: .possiblyAffected, confidence: .insufficient,
                    summary: "上游已出现首个证据缺口；此处只标记可能受影响，不推断独立故障。",
                    kind: nil, gapTopics: [], observedAt: observedAt, dataTime: lastStreamTime,
                    inputs: inputEvidence, outputs: outputEvidence, nodes: nodeEvidence,
                    graphOnly: graphOnly, logs: logsForStage
                ))
                continue
            }

            let missingNodes = nodeEvidence.filter { $0.visible == false }.map(\.name)
            if !missingNodes.isEmpty {
                let message = "新鲜 ROS Graph 在 Domain \(actualDomain.map(String.init) ?? "未知") 中未发现显式配置的预期节点：\(missingNodes.joined(separator: "、"))。这确认的是配置预期不满足，不是实体硬件故障。"
                firstFinding = PipelineFinding(
                    stageID: stage, kind: .nodeMissing, confidence: .confirmedExpectationMismatch,
                    title: "\(stage.title) · 预期节点未发现", observation: message,
                    observedAt: observedAt, firstObservedAt: observedAt, observationCount: 1,
                    evidenceTopics: missingNodes
                )
                stages.append(stageResult(
                    id: stage, state: .confirmedExpectationMismatch, confidence: .confirmedExpectationMismatch,
                    summary: message, kind: .nodeMissing, gapTopics: missingNodes,
                    observedAt: observedAt, dataTime: snapshot?.ros.observedAt ?? "未知",
                    inputs: inputEvidence, outputs: outputEvidence, nodes: nodeEvidence,
                    graphOnly: graphOnly, logs: logsForStage
                ))
                cascadeBlocked = true
                continue
            }

            let unknownStreams = selected.filter { !$0.isFresh && !$0.hasGapEvidence }
            if !unknownStreams.isEmpty || selected.count != allTopics.count {
                let missing = unknownStreams.map(\.topic)
                let summary = missing.isEmpty
                    ? "Agent 未提供该场景所需的完整 Topic 记录；无法判断链路是否满足预期。"
                    : "Topic 状态为未知或不可读取：\(missing.joined(separator: "、"))；不根据缺失遥测定位故障。"
                stages.append(stageResult(
                    id: stage, state: .unknown, confidence: .insufficient, summary: summary,
                    kind: .telemetryUnknown, gapTopics: missing, observedAt: observedAt, dataTime: lastStreamTime,
                    inputs: inputEvidence, outputs: outputEvidence, nodes: nodeEvidence,
                    graphOnly: graphOnly, logs: logsForStage
                ))
                cascadeBlocked = true
                continue
            }

            let gaps = selected.filter { $0.hasGapEvidence }
            if !gaps.isEmpty {
                let topics = gaps.map(\.topic)
                let kinds = gaps.map(gapKind(for:))
                let kind = kinds.first ?? .telemetryUnknown
                let details = gaps.map { "\($0.topic)=\($0.status)" }.joined(separator: "；")
                let message = "显式启用的运行预期下，首次未满足的数据证据位于此环节：\(details)。发布端与工作区归属、消费者接收及处理结果未由 Agent 证明，因此标记为疑似链路中断。"
                firstFinding = PipelineFinding(
                    stageID: stage, kind: kind, confidence: .suspected,
                    title: "\(stage.title) · 疑似链路中断", observation: message,
                    observedAt: observedAt, firstObservedAt: observedAt, observationCount: 1,
                    evidenceTopics: topics
                )
                stages.append(stageResult(
                    id: stage, state: .suspectedGap, confidence: .suspected,
                    summary: message, kind: kind, gapTopics: topics,
                    observedAt: observedAt, dataTime: lastStreamTime,
                    inputs: inputEvidence, outputs: outputEvidence, nodes: nodeEvidence,
                    graphOnly: graphOnly, logs: logsForStage
                ))
                cascadeBlocked = true
                continue
            }

            stages.append(stageResult(
                id: stage, state: .observed, confidence: .observedOnly,
                summary: "本环节配置要求的 Agent 接收数据均在时限内；Topic 发布者工作区归属、订阅者接收和算法处理结果仍未确认。",
                kind: nil, gapTopics: [], observedAt: observedAt, dataTime: lastStreamTime,
                inputs: inputEvidence, outputs: outputEvidence, nodes: nodeEvidence,
                graphOnly: graphOnly, logs: logsForStage
            ))
        }

        let summary: String
        if !expectation.scenario.isConfigured {
            summary = "运行预期未配置；当前仅展示遥测原始证据，不判定模块未运行或故障。"
        } else if let blocked {
            summary = "诊断暂停：\(blocked)"
        } else if let firstFinding {
            summary = firstFinding.title + "。下游状态仅标为可能受影响。"
        } else if stages.contains(where: { $0.state == .unknown }) {
            summary = "链路观测不完整；没有足够证据确认首个异常环节。"
        } else {
            summary = "配置的 Topic 数据均可由 Agent 近期观测；模块来源、订阅处理和实体硬件状态未确认。"
        }

        return PipelineDiagnosis(
            scenario: expectation.scenario,
            expectedDomainID: expectation.expectedDomainID,
            actualDomainID: actualDomain,
            observedAt: telemetry?.collectedAt ?? snapshot?.collectedAt ?? evaluatedAt,
            blockedReason: blocked,
            summary: summary,
            stages: stages,
            firstFinding: firstFinding
        )
    }

    private static func blockReason(
        expectation: PipelineRunExpectation,
        agentOnline: Bool,
        snapshot: ConsoleSnapshot?,
        telemetry: LinkTelemetryReading?
    ) -> String? {
        if let configurationError = expectation.configurationError { return configurationError }
        guard expectation.scenario.isConfigured else { return nil }
        guard agentOnline else { return "NUC Agent 断连；链路状态保持未知，不沿用历史数据生成实时故障。" }
        guard let snapshot, snapshot.source == "nuc-read-only-agent" else { return "没有当前 Agent 快照；工作区与 ROS Graph 状态未知。" }
        guard let telemetry else { return "当前 Agent 未提供 /v1/telemetry；旧版 Agent 保持兼容，链路状态未知。" }
        guard telemetry.status == "ok", telemetry.graphStatus == "ok" else {
            return telemetry.graphReason ?? "链路遥测或 ROS Graph 当前不可用。"
        }
        guard snapshot.ros.status == "ok",
              let graphAge = snapshot.ros.graphAgeMs,
              graphAge >= 0, graphAge <= HealthEvaluator.rosGraphTimeoutMs else {
            return "ROS Graph 已过期、缺少时间证据或不可用；下游状态保持未知。"
        }
        guard let expectedDomain = expectation.expectedDomainID else { return "预期 ROS Domain 未配置；下游状态保持未知。" }
        guard let snapshotDomain = snapshot.ros.domainId,
              let telemetryDomain = telemetry.domainId,
              snapshotDomain == telemetryDomain else {
            return "快照与链路遥测的实际 ROS Domain 不一致或缺失；下游状态保持未知。"
        }
        guard snapshotDomain == expectedDomain else {
            return "实际 ROS Domain \(snapshotDomain) 与本机设置的预期 Domain \(expectedDomain) 不一致；下游状态保持未知。"
        }
        if let error = snapshot.ros.expectedDomainError ?? telemetry.expectedDomainError {
            return "Agent 预期 ROS Domain 配置无效：\(error)；下游状态保持未知。"
        }
        if let agentExpected = snapshot.ros.expectedDomainId, agentExpected != expectedDomain {
            return "Mac 本机运行预期 Domain \(expectedDomain) 与 Agent 配置 Domain \(agentExpected) 冲突；下游状态保持未知。"
        }
        if let agentExpected = telemetry.expectedDomainId, agentExpected != expectedDomain {
            return "Mac 本机运行预期 Domain \(expectedDomain) 与链路 Agent 配置 Domain \(agentExpected) 冲突；下游状态保持未知。"
        }
        if telemetry.domainMatchesExpectation == false {
            return "Agent 报告 ROS Domain 与其预期不匹配；下游状态保持未知。"
        }
        guard let telemetryGraphAge = telemetry.graphAgeMs,
              telemetryGraphAge >= 0, telemetryGraphAge <= HealthEvaluator.rosGraphTimeoutMs else {
            return "链路遥测使用的 ROS Graph 已过期或缺少时间证据；下游状态保持未知。"
        }
        return nil
    }

    private static func stageResult(
        id: PipelineStageID,
        state: PipelineStageState,
        confidence: PipelineEvidenceConfidence,
        summary: String,
        kind: PipelineGapKind?,
        gapTopics: [String],
        observedAt: String,
        dataTime: String,
        inputs: [PipelineStreamEvidence],
        outputs: [PipelineStreamEvidence],
        nodes: [PipelineNodeEvidence],
        graphOnly: [String],
        logs: [PipelineLogEvidence]
    ) -> PipelineStageDiagnosis {
        PipelineStageDiagnosis(
            id: id,
            state: state,
            confidence: confidence,
            summary: summary,
            firstGapKind: kind,
            firstGapTopics: gapTopics,
            observedAt: observedAt,
            dataTime: dataTime,
            inputEvidence: inputs,
            outputEvidence: outputs,
            nodeEvidence: nodes,
            graphOnlyTopics: graphOnly,
            logs: logs,
            sourcePaths: id.sourcePaths,
            possibleCauses: id.possibleCauses
        )
    }

    private static func stream(topic: String, role: String, telemetry: LinkTelemetryReading?) -> PipelineStreamEvidence? {
        guard let stream = telemetry?.stream(topic: topic) else { return nil }
        let endpoints = stream.publisherEndpoints.map { endpoint in
            endpoint.namespace == "/" ? endpoint.name : "\(endpoint.namespace)/\(endpoint.name)"
        }
        return PipelineStreamEvidence(
            topic: topic,
            role: role,
            status: stream.status,
            reason: stream.reason,
            publisherCount: stream.publisherCount,
            publisherEndpoints: endpoints,
            workspaceAttribution: stream.workspaceAttribution,
            receivedAt: stream.receivedAt,
            sourceStamp: stream.sourceStamp,
            ageMs: stream.ageMs,
            staleAfterMs: stream.staleAfterMs,
            receiveRateHz: stream.receiveRateHz,
            headerToAgentMs: stream.headerToAgentMs,
            sampleCount: stream.sampleCount,
            subscriptionGeneration: stream.subscriptionGeneration,
            receiveCount: stream.receiveCount
        )
    }

    private static func gapKind(for stream: PipelineStreamEvidence) -> PipelineGapKind {
        switch stream.status {
        case "stale": return .streamStale
        case "unavailable": return .publisherMissing
        case "waiting": return .publisherHasNoData
        default: return .telemetryUnknown
        }
    }

    private static func matchingLogs(_ logs: LogReading?, stage: PipelineStageID) -> [PipelineLogEvidence] {
        guard let logs, logs.status == "ok" else { return [] }
        let terms = stage.logTerms.map { $0.lowercased() }
        return logs.entries.filter { entry in
            let text = "\(entry.source) \(entry.message)".lowercased()
            return terms.contains(where: text.contains)
        }.suffix(20).map {
            PipelineLogEvidence(time: $0.fileModifiedAt, severity: $0.severity, source: $0.source, message: $0.message)
        }
    }

    private static func fullNodeName(_ node: ROSNode) -> String {
        let namespace = node.namespace.split(separator: "/").joined(separator: "/")
        let name = node.name.trimmingCharacters(in: CharacterSet(charactersIn: "/"))
        return normalizeNodeName("/" + ([namespace, name].filter { !$0.isEmpty }.joined(separator: "/")))
    }

    private static func normalizeNodeName(_ value: String) -> String {
        "/" + value.split(separator: "/").joined(separator: "/")
    }

    private static func orderedUnique(_ values: [String]) -> [String] {
        var seen = Set<String>()
        return values.filter { seen.insert($0).inserted }
    }

    private static func iso8601(_ date: Date) -> String {
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        return formatter.string(from: date)
    }
}
