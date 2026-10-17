import Foundation

enum HealthState: String, CaseIterable, Equatable {
    case normal
    case abnormal
    case notRunning
    case unknown

    var label: String {
        switch self {
        case .normal: return "正常"
        case .abnormal: return "异常"
        case .notRunning: return "未运行"
        case .unknown: return "未知"
        }
    }
}

enum HealthRunMode: String, Equatable {
    case live = "NUC 实时只读"
    case synthetic = "ROS 合成数据"
    case unknown = "未知"
}

enum HealthPersistence: Equatable {
    case singleSnapshot
    case sustained
}

enum HealthSeverity: String, Equatable {
    case informational
    case warning
    case error

    var label: String {
        switch self {
        case .informational: return "提示"
        case .warning: return "待确认"
        case .error: return "异常"
        }
    }
}

private struct HealthRuleMetadata {
    let module: String
    let ruleID: String
    let sourceLocation: String

    static func forEvent(_ id: String) -> HealthRuleMetadata {
        switch id {
        case "agent-connection":
            return HealthRuleMetadata(module: "Agent 连接", ruleID: "agent.connection", sourceLocation: "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift#HealthEvaluator.evaluate")
        case "ros-graph-freshness":
            return HealthRuleMetadata(module: "ROS graph", ruleID: "ros.graph.freshness", sourceLocation: "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift#HealthEvaluator.rosGraphState")
        case "ros-domain":
            return HealthRuleMetadata(module: "ROS Domain", ruleID: "ros.domain.expectation", sourceLocation: "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift#HealthEvaluator.evaluate")
        case "expected-ros-nodes":
            return HealthRuleMetadata(module: "ROS 节点", ruleID: "ros.node.expectation", sourceLocation: "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift#HealthEvaluator.evaluate")
        case "vision-rgb":
            return HealthRuleMetadata(module: "Orbbec Gemini 336L", ruleID: "camera.rgb.stream_evidence", sourceLocation: "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift#HealthEvaluator.visionEvent")
        case "vision-depth":
            return HealthRuleMetadata(module: "Orbbec Gemini 336L", ruleID: "camera.depth.stream_evidence", sourceLocation: "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift#HealthEvaluator.visionEvent")
        case "vision-yolo":
            return HealthRuleMetadata(module: "YOLO 输出", ruleID: "vision.yolo.confidence_gate", sourceLocation: "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift#HealthEvaluator.visionEvent")
        case "module-camera":
            return HealthRuleMetadata(module: "Orbbec Gemini 336L", ruleID: "camera.rgb_depth.runtime_and_freshness", sourceLocation: "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift#HealthEvaluator.cameraModuleEvent")
        default:
            return HealthRuleMetadata(module: "未分类", ruleID: "health.unknown", sourceLocation: "tools/mac-console/Sources/LunarRobotConsole/HealthEvents.swift#HealthEvaluator")
        }
    }
}

private struct ModuleHealthContext {
    enum Resolution: Equatable {
        case confirmed
        case unconfiguredNotRunning
        case unconfigured
        case invalid
        case ambiguous
        case unavailable
    }

    let resolution: Resolution
    let workspace: WorkspaceReading?
    let module: WorkspaceModuleReading?
    let candidates: [(workspace: WorkspaceReading, module: WorkspaceModuleReading)]
    let reason: String
    let inventory: WorkspaceInventoryReading?

    var expectationConfirmed: Bool { resolution == .confirmed }

    var processAndGraphVisible: Bool {
        guard resolution == .confirmed,
              inventory?.status == "ok",
              inventory?.processScan?.status == "ok",
              module?.sourceStatus == "present",
              module?.runtimeStatus == "running",
              workspace?.sourceStatus == "present",
              workspace?.graphRelationship == "shared_domain" else { return false }
        return true
    }
}

struct HealthEvent: Identifiable, Equatable {
    let id: String
    let title: String
    let state: HealthState
    let module: String
    let ruleID: String
    let severity: HealthSeverity
    let sourceLocation: String
    let observation: String
    let resource: String
    let dataTime: String
    let rosDomain: String
    let runMode: HealthRunMode
    let suggestedCheck: String
    let observedAt: String
    let source: String
    let firstObservedAt: String
    let observationCount: Int

    var persistence: HealthPersistence {
        state == .abnormal && observationCount > 1 ? .sustained : .singleSnapshot
    }

    var persistenceLabel: String {
        persistence == .sustained ? "持续异常 · \(observationCount) 次观测" : "单次快照"
    }

    var observationWindow: String {
        firstObservedAt == observedAt ? observedAt : "\(firstObservedAt) → \(observedAt)"
    }

    init(
        id: String,
        title: String,
        state: HealthState,
        observation: String,
        resource: String,
        dataTime: String,
        rosDomain: String,
        runMode: HealthRunMode,
        suggestedCheck: String,
        observedAt: String = "未知",
        source: String = "未知来源",
        firstObservedAt: String? = nil,
        observationCount: Int = 1
    ) {
        self.id = id
        self.title = title
        self.state = state
        let metadata = HealthRuleMetadata.forEvent(id)
        self.module = metadata.module
        self.ruleID = metadata.ruleID
        self.severity = Self.severity(for: state)
        self.sourceLocation = metadata.sourceLocation
        self.observation = observation
        self.resource = resource
        self.dataTime = dataTime
        self.rosDomain = rosDomain
        self.runMode = runMode
        self.suggestedCheck = suggestedCheck
        self.observedAt = observedAt
        self.source = source
        self.firstObservedAt = firstObservedAt ?? observedAt
        self.observationCount = observationCount
    }

    private static func severity(for state: HealthState) -> HealthSeverity {
        switch state {
        case .normal, .notRunning: return .informational
        case .unknown: return .warning
        case .abnormal: return .error
        }
    }

    func withObservationHistory(firstObservedAt: String, observationCount: Int) -> HealthEvent {
        HealthEvent(
            id: id,
            title: title,
            state: state,
            observation: observation,
            resource: resource,
            dataTime: dataTime,
            rosDomain: rosDomain,
            runMode: runMode,
            suggestedCheck: suggestedCheck,
            observedAt: observedAt,
            source: source,
            firstObservedAt: firstObservedAt,
            observationCount: observationCount
        )
    }
}

struct HealthEventTracker {
    private struct Record {
        let firstObservedAt: String
        var lastObservationKey: String
        var observationCount: Int
    }

    private var records: [String: Record] = [:]

    mutating func update(_ events: [HealthEvent]) -> [HealthEvent] {
        let currentIDs = Set(events.map(\.id))
        for id in Array(records.keys) where !currentIDs.contains(id) {
            records.removeValue(forKey: id)
        }

        return events.map { event in
            guard event.state == .abnormal else {
                records.removeValue(forKey: event.id)
                return event.withObservationHistory(firstObservedAt: event.observedAt, observationCount: 1)
            }

            let observationKey = "\(event.source)|\(event.observedAt)"
            if var record = records[event.id] {
                if record.lastObservationKey != observationKey {
                    record.lastObservationKey = observationKey
                    record.observationCount += 1
                }
                records[event.id] = record
                return event.withObservationHistory(
                    firstObservedAt: record.firstObservedAt,
                    observationCount: record.observationCount
                )
            }

            records[event.id] = Record(
                firstObservedAt: event.observedAt,
                lastObservationKey: observationKey,
                observationCount: 1
            )
            return event.withObservationHistory(firstObservedAt: event.observedAt, observationCount: 1)
        }
    }
}

enum HealthSummaryState {
    static func resolve(
        _ events: [HealthEvent],
        workspaceInventory: WorkspaceInventoryReading? = nil,
        requireWorkspaceTelemetry: Bool = false
    ) -> HealthState {
        if events.contains(where: { $0.state == .abnormal }) { return .abnormal }
        if events.isEmpty || events.contains(where: { $0.state == .unknown }) { return .unknown }
        if requireWorkspaceTelemetry && (workspaceInventory == nil || workspaceInventory?.hasUnconfirmedHealth == true) { return .unknown }
        if events.contains(where: { $0.state == .notRunning }) { return .notRunning }
        return .normal
    }
}

enum HealthEvaluator {
    static let rosGraphTimeoutMs = 5_000
    static let imageTimeoutMs = 1_500

    static func evaluate(
        agentOnline: Bool,
        hasEverConnectedToAgent: Bool,
        agentMessage: String,
        lastSuccessfulRead: Date?,
        lastKnownDomainID: Int?,
        snapshot: ConsoleSnapshot?,
        vision: VisionReading?,
        evaluationTime: Date = Date()
    ) -> [HealthEvent] {
        let evaluationTimeText = formatted(evaluationTime) ?? "未知"
        let snapshotObservedAt = snapshot?.collectedAt ?? evaluationTimeText
        let ros = snapshot?.ros
        let domainID = ros?.domainId ?? lastKnownDomainID
        let mode = runMode(agentOnline: agentOnline, snapshot: snapshot, vision: vision)
        var events: [HealthEvent] = []

        let agentState: HealthState
        let agentObservation: String
        if agentOnline, snapshot != nil {
            agentState = .normal
            agentObservation = "GET /health 与 GET /v1/snapshot 已成功读取；\(agentMessage)"
        } else if agentOnline {
            agentState = .unknown
            agentObservation = "Agent 健康接口可达，但尚未取得有效快照。\(agentMessage)"
        } else if hasEverConnectedToAgent {
            if agentMessage.contains("正在重新连接") {
                agentState = .unknown
                agentObservation = "正在执行用户请求的重新连接，暂不报告为失联。\(agentMessage)"
            } else if agentMessage.contains("已停止") {
                agentState = .notRunning
                agentObservation = "控制台停止时 Agent 连接已关闭。\(agentMessage)"
            } else {
                agentState = .abnormal
                agentObservation = "此前已读取到 Agent，当前连接不可用。\(agentMessage)"
            }
        } else {
            agentState = .unknown
            agentObservation = "尚无成功连接记录；无法区分 Agent 未启动与网络/隧道不可达。\(agentMessage)"
        }
        events.append(HealthEvent(
            id: "agent-connection",
            title: "NUC Agent 连接",
            state: agentState,
            observation: agentObservation,
            resource: "Endpoint · GET /health、GET /v1/snapshot",
            dataTime: snapshot?.collectedAt ?? formatted(lastSuccessfulRead) ?? "无成功读取",
            rosDomain: domainLabel(actual: domainID, expected: ros?.expectedDomainId),
            runMode: agentOnline ? .live : (hasEverConnectedToAgent ? .live : .unknown),
            suggestedCheck: "系统诊断 → SSH 隧道；NUC Agent 的 loopback HTTP 8765 端口与 Mac 固定转发 18765",
            observedAt: agentOnline && snapshot != nil ? snapshotObservedAt : evaluationTimeText,
            source: agentOnline ? "NUC Agent · /health 与 /v1/snapshot" : "Mac Console · SSH 隧道与 Agent 探测"
        ))

        let graphState = rosGraphState(agentOnline: agentOnline, snapshot: snapshot)
        let graphObservation: String
        if !agentOnline || snapshot == nil {
            graphObservation = "当前没有可用的 NUC ROS 快照；ROS 运行状态未知。"
        } else if ros?.status != "ok" {
            graphObservation = ros?.reason ?? "NUC Agent 未能确认 ROS graph 状态。"
        } else if let age = ros?.graphAgeMs {
            graphObservation = String(format: "最后成功 graph 观测距今 %.2f 秒；包含 %d 个节点、%d 个 Topic。阈值 %.1f 秒。", Double(age) / 1000, ros?.nodes.count ?? 0, ros?.topics.count ?? 0, Double(rosGraphTimeoutMs) / 1000)
        } else {
            graphObservation = "当前 Agent 未提供 graph 单调时钟年龄；无法确认数据是否过期。"
        }
        events.append(HealthEvent(
            id: "ros-graph-freshness",
            title: "ROS graph 数据新鲜度",
            state: graphState,
            observation: graphObservation,
            resource: "ROS graph · 节点与 Topic 发现",
            dataTime: ros?.observedAt ?? snapshot?.collectedAt ?? "无成功观测时间",
            rosDomain: domainLabel(actual: domainID, expected: ros?.expectedDomainId),
            runMode: mode,
            suggestedCheck: "系统诊断 → 运行时节点与 Topic；核对 NUC Agent 的 ROS graph 观测时间",
            observedAt: snapshotObservedAt,
            source: "NUC Agent · ROS graph observer"
        ))

        let domainState: HealthState
        let domainObservation: String
        if !agentOnline || ros?.status != "ok" || graphState != .normal {
            domainState = .unknown
            domainObservation = "ROS graph 未处于可验证的新鲜状态，暂不判断 Domain 是否符合预期。"
        } else if let error = ros?.expectedDomainError {
            domainState = .unknown
            domainObservation = "预期 Domain 配置无效：\(error)"
        } else if let expected = ros?.expectedDomainId, let actual = ros?.domainId {
            domainState = actual == expected ? .normal : .abnormal
            domainObservation = "Agent 观测 Domain \(actual)；配置的预期 Domain 为 \(expected)。"
        } else if let expected = ros?.expectedDomainId {
            domainState = .unknown
            domainObservation = "预期 Domain 为 \(expected)，但当前 ROS graph 未提供可比较的实际 Domain。"
        } else {
            domainState = .unknown
            domainObservation = "观测 Domain \(ros?.domainId.map(String.init) ?? "未知")；未配置预期 Domain，不推断不一致。"
        }
        events.append(HealthEvent(
            id: "ros-domain",
            title: "ROS Domain",
            state: domainState,
            observation: domainObservation,
            resource: "ROS graph · NUC graph observer",
            dataTime: ros?.observedAt ?? snapshot?.collectedAt ?? "无成功观测时间",
            rosDomain: domainLabel(actual: domainID, expected: ros?.expectedDomainId),
            runMode: mode,
            suggestedCheck: "核对当前运行模式的预期 Domain 配置与 NUC Agent 观测值",
            observedAt: snapshotObservedAt,
            source: "NUC Agent · ROS graph observer"
        ))

        let downstreamUnknownReason = downstreamBlockReason(
            agentOnline: agentOnline,
            graphState: graphState,
            domainState: domainState,
            domainExpectationInvalid: ros?.expectedDomainError != nil,
            domainID: ros?.domainId,
            expectedDomainID: ros?.expectedDomainId
        )
        let cameraContext = moduleHealthContext(moduleID: "camera", inventory: snapshot?.workspaces)
        let yoloContext = moduleHealthContext(moduleID: "yolo", inventory: snapshot?.workspaces)
        let nodeState: HealthState
        let nodeObservation: String
        let configuredNodes = ros?.expectedNodes ?? []
        if let downstreamUnknownReason {
            nodeState = .unknown
            nodeObservation = downstreamUnknownReason
        } else if let error = ros?.expectedNodesError {
            nodeState = .unknown
            nodeObservation = "预期节点配置无效：\(error)"
        } else if configuredNodes.isEmpty {
            if ros?.nodes.isEmpty == true {
                nodeState = .notRunning
                nodeObservation = "当前未发现业务节点，且未配置预期节点集合；此状态不作为故障判定。"
            } else {
                nodeState = .unknown
                nodeObservation = "当前发现 \(ros?.nodes.count ?? 0) 个节点；未配置预期节点集合，缺失模块不作为故障判定。"
            }
        } else {
            let present = Set((ros?.nodes ?? []).map(fullNodeName))
            let missing = configuredNodes.filter { !present.contains(normalizedNodeName($0)) }
            if missing.isEmpty {
                nodeState = .normal
                nodeObservation = "预期节点均已发现：\(configuredNodes.joined(separator: "、"))。"
            } else {
                nodeState = .abnormal
                nodeObservation = "未发现配置为当前模式应运行的节点：\(missing.joined(separator: "、"))；当前共发现 \(present.count) 个节点。"
            }
        }
        events.append(HealthEvent(
            id: "expected-ros-nodes",
            title: "预期 ROS 节点",
            state: nodeState,
            observation: nodeObservation,
            resource: "节点 · \(configuredNodes.isEmpty ? "预期集合未配置" : configuredNodes.joined(separator: "、"))",
            dataTime: ros?.observedAt ?? snapshot?.collectedAt ?? "无成功观测时间",
            rosDomain: domainLabel(actual: domainID, expected: ros?.expectedDomainId),
            runMode: mode,
            suggestedCheck: "核对 Agent 的预期节点集合是否对应当前运行模式；查看系统诊断中的实时节点列表与 ROS 日志",
            observedAt: snapshotObservedAt,
            source: "NUC Agent · ROS graph observer"
        ))

        events.append(cameraModuleEvent(
            context: cameraContext,
            agentOnline: agentOnline,
            snapshot: snapshot,
            vision: vision,
            domainID: domainID,
            expectedDomainID: ros?.expectedDomainId,
            runMode: mode,
            downstreamUnknownReason: downstreamUnknownReason,
            evaluationTime: evaluationTimeText
        ))

        for streamID in VisionStreamID.allCases {
            events.append(visionEvent(
                streamID: streamID,
                reading: vision?.stream(streamID),
                vision: vision,
                agentOnline: agentOnline,
                domainID: domainID,
                expectedDomainID: ros?.expectedDomainId,
                fallbackDataTime: snapshotObservedAt,
                observedAt: vision?.collectedAt ?? snapshotObservedAt,
                source: vision?.source ?? "NUC Agent · ROS vision observer",
                moduleContext: streamID == .yolo ? yoloContext : cameraContext,
                downstreamUnknownReason: downstreamUnknownReason
            ))
        }
        return events
    }

    private static func moduleHealthContext(moduleID: String, inventory: WorkspaceInventoryReading?) -> ModuleHealthContext {
        guard let inventory else {
            return ModuleHealthContext(
                resolution: .unavailable, workspace: nil, module: nil, candidates: [],
                reason: "Agent 快照未提供工作区模块证据；模块运行预期与归属未知。", inventory: nil
            )
        }
        if inventory.expectationStatus == "invalid" || inventory.expectationError != nil {
            return ModuleHealthContext(
                resolution: .invalid, workspace: nil, module: nil, candidates: [],
                reason: inventory.expectationError ?? "模块运行预期配置无效；下游模块状态保持未知。", inventory: inventory
            )
        }
        guard inventory.status == "ok", inventory.processScan?.status == "ok" else {
            return ModuleHealthContext(
                resolution: .unavailable, workspace: nil, module: nil, candidates: [],
                reason: "工作区清单或 Agent UID 进程扫描不完整；无法依据模块缺失产生故障告警。", inventory: inventory
            )
        }

        let candidates = inventory.entries.flatMap { workspace in
            (workspace.modules ?? []).compactMap { module -> (workspace: WorkspaceReading, module: WorkspaceModuleReading)? in
                module.id == moduleID ? (workspace, module) : nil
            }
        }
        guard !candidates.isEmpty else {
            return ModuleHealthContext(
                resolution: .unavailable, workspace: nil, module: nil, candidates: [],
                reason: "Agent 工作区清单未包含可归属的 \(moduleID) 模块。", inventory: inventory
            )
        }

        let confirmed = candidates.filter { $0.module.runExpectation == "confirmed" }
        if confirmed.count > 1 {
            return ModuleHealthContext(
                resolution: .ambiguous, workspace: nil, module: nil, candidates: candidates,
                reason: "多个工作区将 \(moduleID) 标记为预期运行，来源归属有歧义。", inventory: inventory
            )
        }
        if let selected = confirmed.first {
            guard inventory.expectationStatus != "unconfigured" else {
                return ModuleHealthContext(
                    resolution: .invalid, workspace: selected.workspace, module: selected.module, candidates: candidates,
                    reason: "模块字段与运行预期清单状态不一致；下游状态保持未知。", inventory: inventory
                )
            }
            return ModuleHealthContext(
                resolution: .confirmed, workspace: selected.workspace, module: selected.module, candidates: candidates,
                reason: "运行预期来自 Agent 显式模块配置。", inventory: inventory
            )
        }
        if candidates.contains(where: { $0.module.runExpectation == "unknown" }) {
            return ModuleHealthContext(
                resolution: .invalid, workspace: nil, module: nil, candidates: candidates,
                reason: "模块运行预期证据为未知；不据此判断运行缺失。", inventory: inventory
            )
        }

        let completeSources = candidates.allSatisfy { $0.module.sourceStatus == "present" && $0.workspace.sourceStatus == "present" }
        if completeSources && candidates.allSatisfy({ $0.module.runtimeStatus == "not_running" }) {
            return ModuleHealthContext(
                resolution: .unconfiguredNotRunning, workspace: candidates.first?.workspace, module: candidates.first?.module,
                candidates: candidates, reason: "未配置 \(moduleID) 的运行预期；完整进程扫描未发现关联进程。", inventory: inventory
            )
        }
        if candidates.contains(where: { $0.module.runtimeStatus == "running" }) {
            return ModuleHealthContext(
                resolution: .unconfigured, workspace: nil, module: nil, candidates: candidates,
                reason: "发现 \(moduleID) 关联进程，但运行预期未配置；模块健康状态未知。", inventory: inventory
            )
        }
        return ModuleHealthContext(
            resolution: .unavailable, workspace: nil, module: nil, candidates: candidates,
            reason: "模块来源或运行状态证据不足。", inventory: inventory
        )
    }

    private static func cameraModuleEvent(
        context: ModuleHealthContext,
        agentOnline: Bool,
        snapshot: ConsoleSnapshot?,
        vision: VisionReading?,
        domainID: Int?,
        expectedDomainID: Int?,
        runMode: HealthRunMode,
        downstreamUnknownReason: String?,
        evaluationTime: String
    ) -> HealthEvent {
        let rgb = vision?.stream(.rgb)
        let depth = vision?.stream(.depth)
        let hasImagePublisher = (rgb?.publisherCount ?? 0) > 0 || (depth?.publisherCount ?? 0) > 0
        let evidenceTime = [rgb?.sourceStamp, depth?.sourceStamp, rgb?.receivedAt, depth?.receivedAt]
            .compactMap { $0 }.joined(separator: " · ")
        let eventTime = vision?.collectedAt ?? context.workspace?.runtimeObservedAt ?? snapshot?.collectedAt ?? evaluationTime
        let resource = "进程包 · orbbec_camera · Topic · /camera/color/image_raw、/camera/depth/image_rect_raw"
        let domain = domainLabel(actual: context.workspace?.graphDomainId ?? domainID, expected: expectedDomainID)
        let observation: String
        let state: HealthState
        let check: String

        if !agentOnline || snapshot == nil {
            state = .unknown
            observation = "Agent 当前不可用；相机模块运行状态与图像帧证据未知。"
            check = "先确认固定 SSH 转发与 NUC Agent 快照，再核对相机运行预期"
        } else if let downstreamUnknownReason {
            state = .unknown
            observation = downstreamUnknownReason
            check = "先核实 ROS graph 新鲜度和 Domain，再检查相机工作区与图像流"
        } else {
            switch context.resolution {
            case .unconfiguredNotRunning:
                if vision?.status != "ok" || rgb == nil || depth == nil {
                    state = .unknown
                    observation = "进程扫描未发现相机进程，但 Agent 视觉接口没有提供完整 RGB/深度发布者观测；状态未知。"
                    check = "确认旧版 Agent 的 /v1/vision 兼容性和相机模块证据"
                } else if hasImagePublisher {
                    state = .unknown
                    observation = "进程扫描未发现已归属的相机进程，但 Agent Graph 中存在图像发布者；来源证据不一致，状态未知。"
                    check = "核对进程归属与同名 Topic 的发布者来源"
                } else {
                    state = .notRunning
                    observation = "未配置相机模块运行预期；完整 Agent UID 进程扫描未发现关联进程，RGB 与深度 Topic 均无发布者。此状态不表示故障。"
                    check = "如当前运行模式要求 Gemini 336L，先显式配置对应工作区模块预期，再查看 camera_switch.launch.py 与设备节点"
                }
            case .unconfigured:
                state = .unknown
                observation = context.reason
                check = "核实当前工作区与运行模式预期；进程或 Topic 存在本身不足以确认模块健康"
            case .invalid, .ambiguous, .unavailable:
                state = .unknown
                observation = context.reason
                check = "检查 Agent 工作区清单、进程扫描完整度及模块运行预期配置"
            case .confirmed:
                guard let inventory = context.inventory,
                      let workspace = context.workspace,
                      let module = context.module,
                      module.sourceStatus == "present",
                      workspace.sourceStatus == "present",
                      inventory.status == "ok",
                      inventory.processScan?.status == "ok" else {
                    state = .unknown
                    observation = "已配置相机运行预期，但工作区源码或进程扫描证据不完整；不判断模块故障。"
                    check = "检查只读 Agent 工作区来源和进程扫描状态"
                    break
                }
                if module.runtimeStatus == "not_running", workspace.graphRelationship != "shared_domain" {
                    state = .unknown
                    observation = workspace.graphRelationship == "different_domain"
                        ? "相机进程未发现，但工作区其他进程的 ROS Domain 与 Agent Graph 不一致；无法确认缺失发生在目标运行域。"
                        : "相机进程未发现，但工作区与 Agent Graph 的 Domain 关系未知；无法确认缺失发生在目标运行域。"
                    check = "核对工作区进程的 ROS_DOMAIN_ID 与 Agent Graph Domain，再评估模块缺失"
                    break
                }
                let otherRunning = context.candidates.contains {
                    $0.workspace.id != workspace.id && $0.module.runtimeStatus == "running"
                }
                if otherRunning {
                    state = .unknown
                    observation = "另一个工作区也发现相机关联进程；当前预期工作区归属有歧义。"
                    check = "核对工作区进程归属及 Domain；不要依据同名 Topic 推断发布节点"
                } else if module.runtimeStatus == "not_running" {
                    if hasImagePublisher {
                        state = .unknown
                        observation = "预期工作区进程扫描未发现相机进程，但 ROS Graph 中存在图像发布者；发布者归属未确认。"
                        check = "核对相机进程归属、工作区路径及 ROS Domain"
                    } else {
                        state = .abnormal
                        observation = "运行预期由 \(module.runExpectationSource ?? "Agent 显式配置") 声明；完整进程扫描未发现相机关联进程，RGB 与深度 Topic 也没有发布者。"
                        check = "系统诊断 → 工作区进程证据；核对当前模式是否应启动原始或新版相机工作区"
                    }
                } else if module.runtimeStatus != "running" {
                    state = .unknown
                    observation = "相机运行预期已确认，但关联进程状态为未知。"
                    check = "检查 Agent UID 进程扫描权限与工作区归属证据"
                } else if workspace.graphRelationship != "shared_domain" {
                    state = .unknown
                    observation = workspace.graphReason ?? "相机工作区与 Agent ROS graph 的 Domain 关系未确认。"
                    check = "核对工作区进程 ROS_DOMAIN_ID 与 Agent Graph Domain"
                } else if vision?.status != "ok" || rgb == nil || depth == nil {
                    state = .unknown
                    observation = "相机进程与 Domain 关系已观测，但 RGB/深度帧接口未提供完整数据。"
                    check = "检查 Agent 视觉只读观测接口与 RGB/深度 Topic"
                } else {
                    let streams = [("RGB", rgb!), ("深度", depth!)]
                    let abnormal = streams.filter { label, stream in
                        stream.status == "unavailable" || stream.status == "stale" || stream.status == "waiting" ||
                        stream.status == "unsupported" || (stream.status == "streaming" && stream.frameAgeMs.map { $0 > imageTimeoutMs } == true)
                    }
                    let allFresh = streams.allSatisfy { _, stream in
                        stream.status == "streaming" && stream.frameAgeMs.map { $0 <= imageTimeoutMs } == true
                    }
                    if !abnormal.isEmpty {
                        state = .abnormal
                        let labels = abnormal.map { label, stream in
                            "\(label) \(stream.topic)=\(stream.status), 帧龄 \(stream.frameAgeMs.map { "\($0) ms" } ?? "未知")"
                        }.joined(separator: "；")
                        observation = "已确认相机进程运行且 Agent Graph Domain 可见；\(labels)。"
                        check = "检查 Gemini 336L 驱动日志、RGB/深度发布者与 USB 设备状态"
                    } else if allFresh {
                        state = .normal
                        observation = "运行预期、相机关联进程和共享 Domain 已确认；RGB \(streamEvidence(rgb!))；深度 \(streamEvidence(depth!))。数据流正常不等同于传感器精度或实体硬件已验收。"
                        check = "持续观察 RGB/深度帧龄；30 FPS 是源码 launch 请求值，不是本规则的性能阈值"
                    } else {
                        state = .unknown
                        observation = "相机进程和 Domain 已确认，但至少一条图像流缺少可比较帧龄；\(streamEvidence(rgb!))；\(streamEvidence(depth!))。"
                        check = "检查 Agent 视觉观测时间、ROS Header 时间与接收时钟差异"
                    }
                }
            }
        }

        return HealthEvent(
            id: "module-camera",
            title: "Gemini 336L 相机模块",
            state: state,
            observation: observation,
            resource: resource,
            dataTime: evidenceTime.isEmpty ? (context.workspace?.runtimeObservedAt ?? "无图像帧时间") : evidenceTime,
            rosDomain: domain,
            runMode: vision?.streams.contains(where: isSynthetic) == true ? .synthetic : runMode,
            suggestedCheck: check,
            observedAt: eventTime,
            source: "NUC Agent · /proc、ROS graph、/v1/vision"
        )
    }

    private static func streamEvidence(_ stream: VisionStreamReading) -> String {
        let fps = stream.frameRateHz.map { String(format: "%.2f Hz", $0) } ?? "接收 FPS 未知"
        let age = stream.frameAgeMs.map { "\($0) ms" } ?? "帧龄未知"
        return "\(stream.topic) 帧龄 \(age)、\(fps)、Header \(stream.sourceStamp ?? "未知")"
    }

    private static func rosGraphState(agentOnline: Bool, snapshot: ConsoleSnapshot?) -> HealthState {
        guard agentOnline, let ros = snapshot?.ros, ros.status == "ok", let age = ros.graphAgeMs, age >= 0 else { return .unknown }
        return age > rosGraphTimeoutMs ? .abnormal : .normal
    }

    private static func downstreamBlockReason(
        agentOnline: Bool,
        graphState: HealthState,
        domainState: HealthState,
        domainExpectationInvalid: Bool,
        domainID: Int?,
        expectedDomainID: Int?
    ) -> String? {
        guard agentOnline else {
            return "NUC Agent 已断连；节点和视觉检查状态为未知，避免将缺少数据连带判为故障。"
        }
        if graphState == .abnormal {
            return "ROS graph 数据已过期；下游节点和视觉检查状态为未知，避免使用过期发现结果。"
        }
        if graphState != .normal {
            return "ROS graph 新鲜度尚未确认；下游节点和视觉检查状态为未知。"
        }
        if domainExpectationInvalid {
            return "预期 ROS Domain 配置无效；无法确认当前 graph 是否属于目标运行域，下游节点和视觉检查状态为未知。"
        }
        if domainState == .abnormal {
            return "ROS Domain 实际值 \(domainID.map(String.init) ?? "未知") 与预期值 \(expectedDomainID.map(String.init) ?? "未知") 不匹配；下游检查状态为未知。"
        }
        return nil
    }

    private static func visionEvent(
        streamID: VisionStreamID,
        reading: VisionStreamReading?,
        vision: VisionReading?,
        agentOnline: Bool,
        domainID: Int?,
        expectedDomainID: Int?,
        fallbackDataTime: String,
        observedAt: String,
        source: String,
        moduleContext: ModuleHealthContext,
        downstreamUnknownReason: String?
    ) -> HealthEvent {
        let title = "\(streamID.label) 图像数据"
        let topic = reading?.topic ?? defaultTopic(streamID)
        let mode = runMode(agentOnline: agentOnline, snapshot: nil, vision: vision, stream: streamID)
        let domain = domainLabel(actual: domainID, expected: expectedDomainID)
        let receivedTime = reading?.receivedAt ?? reading?.sourceStamp ?? vision?.collectedAt ?? fallbackDataTime
        let ageText = reading?.frameAgeMs.map { "\($0) ms" } ?? "未知"
        let publisherText = reading.map { "\($0.publisherCount) 个发布者" } ?? "发布者数量未知"
        let status = reading?.status ?? "unknown"
        let isYOLO = streamID == .yolo

        let eventState: HealthState
        var observation: String
        let check: String
        if let downstreamUnknownReason {
            eventState = .unknown
            observation = downstreamUnknownReason
            check = "先核实 NUC Agent 可达性、ROS graph 新鲜度及 Domain，再检查下游图像 Topic"
        } else if !agentOnline {
            eventState = .unknown
            observation = "Agent 当前不可用；图像 Topic 状态未知。"
            check = "先确认 NUC Agent 与固定 SSH 转发可读，再查看图像 Topic"
        } else if vision?.status != "ok" || reading == nil {
            eventState = .unknown
            observation = reading?.reason ?? vision?.reason ?? "Agent 尚未提供此图像 Topic 的观测。"
            check = "系统诊断 → ROS graph 状态；确认 Agent 版本提供视觉只读接口"
        } else if status == "streaming" {
            if isYOLO, let age = reading?.frameAgeMs, age > imageTimeoutMs {
                eventState = .unknown
                observation = "YOLO 输出已超过常规帧时限（\(age) ms），但置信度发布门控可能暂停输出；不能据此判定故障。"
            } else if !isYOLO, let age = reading?.frameAgeMs, age > imageTimeoutMs {
                eventState = .unknown
                observation = "Agent 报告帧龄 \(age) ms，超过 \(imageTimeoutMs) ms 图像新鲜度阈值；模块级故障需有已确认运行预期和可归属的工作区证据。\(publisherText)"
            } else if let age = reading?.frameAgeMs {
                eventState = .normal
                observation = "最近帧龄 \(age) ms；\(publisherText)；ROS Header 时间 \(reading?.sourceStamp ?? "未知")。"
                if isYOLO { observation.append("输出流存活不代表识别精度已验收。") }
            } else {
                eventState = .unknown
                observation = "Agent 标记流正在更新，但没有可用帧龄；\(publisherText)。"
            }
            check = isYOLO ? "查看 YOLO 输入、节点日志与置信度门控配置；空输出本身不作为故障" : "测试驾驶舱 → 帧新鲜度；NUC ROS 日志与对应图像 Topic"
        } else if status == "stale" {
            if isYOLO {
                eventState = .unknown
                observation = "YOLO 输出帧已超时（帧龄 \(ageText)）；现有置信度发布门控可在没有高置信度结果时暂停发布，因此不据此判为故障。"
                check = "查看 YOLO 输入、节点日志与置信度发布门控；当前输出缺失不构成故障证据"
            } else {
                eventState = .unknown
                observation = "图像帧已超时；帧龄 \(ageText)，\(publisherText)；最后 ROS Header 时间 \(reading?.sourceStamp ?? "未知")。仅在显式确认相机运行预期及共享 Domain 后，由相机模块规则生成故障结论。"
                check = "检查系统健康中的相机模块结论，再核对 Agent Graph Domain、相机节点与对应 Topic"
            }
        } else if status == "unavailable" {
            if isYOLO {
                eventState = .unknown
                observation = "未发现 YOLO 输出发布者；置信度门控或模块未运行均可能导致无输出，当前没有故障结论。"
                check = "确认当前模式是否要求 YOLO 输出；查看输入流、节点日志与置信度发布门控"
            } else {
                if moduleContext.resolution == .unconfiguredNotRunning {
                    eventState = .notRunning
                    observation = "Topic 未发现发布者；Agent 完整进程扫描也未发现相机关联进程，且运行预期未配置。此状态不作为故障判定。"
                } else {
                    eventState = .unknown
                    observation = "Topic 未发现发布者；来源工作区或运行预期不足以确认相机模块应处于运行状态。"
                }
                check = "若当前模式要求该数据流，先核实工作区运行预期与 Domain，再查看 ROS graph"
            }
        } else if status == "waiting" {
            if isYOLO {
                eventState = .unknown
                observation = "Topic 有发布者但尚无输出帧；YOLO 置信度发布门控可抑制结果，不能据此判定故障。"
                check = "查看 YOLO 输入、节点日志与置信度发布门控；空输出本身不作为故障"
            } else {
                eventState = .unknown
                observation = "\(publisherText)，尚未收到首帧；当前没有足够证据判定预期状态。"
                check = "核对当前运行模式是否要求此数据流，再查看相机节点日志"
            }
        } else if status == "unsupported" {
            if isYOLO && moduleContext.processAndGraphVisible {
                eventState = .abnormal
                observation = "已配置的 YOLO 工作区进程与共享 Domain 可见，但标注图像 Topic 类型不兼容：\(reading?.topicType ?? "未知")；\(publisherText)。"
            } else {
                eventState = .unknown
                observation = "Topic 被观测到，但消息类型不兼容：\(reading?.topicType ?? "未知")；模块运行预期或发布者归属尚未确认。"
            }
            check = "系统诊断 → 运行时 Topic；核对消息类型、预期工作区和发布者归属"
        } else {
            eventState = .unknown
            observation = reading?.reason ?? "图像 Topic 状态未知。"
            check = "系统诊断 → ROS graph 与测试驾驶舱数据来源"
        }

        return HealthEvent(
            id: "vision-\(streamID.rawValue)",
            title: title,
            state: eventState,
            observation: observation,
            resource: "Topic · \(topic)",
            dataTime: receivedTime,
            rosDomain: domain,
            runMode: mode,
            suggestedCheck: check,
            observedAt: observedAt,
            source: source
        )
    }

    private static func runMode(agentOnline: Bool, snapshot: ConsoleSnapshot?, vision: VisionReading?, stream: VisionStreamID? = nil) -> HealthRunMode {
        if let vision {
            let candidates = stream.flatMap { vision.stream($0).map { [$0] } } ?? vision.streams
            if candidates.contains(where: isSynthetic) || vision.source.localizedCaseInsensitiveContains("synthetic") {
                return .synthetic
            }
        }
        if agentOnline && (snapshot?.source == "nuc-read-only-agent" || vision?.source == "nuc-ros2-read-only-subscriber") {
            return .live
        }
        return .unknown
    }

    private static func isSynthetic(_ reading: VisionStreamReading) -> Bool {
        reading.frameId?.hasPrefix("lunar-v0.2a-synthetic/") == true
    }

    private static func fullNodeName(_ node: ROSNode) -> String {
        let namespace = node.namespace.split(separator: "/").joined(separator: "/")
        let name = node.name.trimmingCharacters(in: CharacterSet(charactersIn: "/"))
        return "/" + ([namespace, name].filter { !$0.isEmpty }.joined(separator: "/"))
    }

    private static func normalizedNodeName(_ value: String) -> String {
        "/" + value.split(separator: "/").joined(separator: "/")
    }

    private static func domainLabel(actual: Int?, expected: Int?) -> String {
        let actualText = actual.map(String.init) ?? "未知"
        let expectedText = expected.map(String.init) ?? "未配置"
        return "实际 \(actualText) · 预期 \(expectedText)"
    }

    private static func formatted(_ date: Date?) -> String? {
        guard let date else { return nil }
        let formatter = ISO8601DateFormatter()
        formatter.formatOptions = [.withInternetDateTime, .withFractionalSeconds]
        return formatter.string(from: date)
    }

    private static func defaultTopic(_ stream: VisionStreamID) -> String {
        switch stream {
        case .rgb: return "/camera/color/image_raw"
        case .depth: return "/camera/depth/image_rect_raw"
        case .yolo: return "/yolo/annotated_image"
        }
    }
}
