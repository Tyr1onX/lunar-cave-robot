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

struct HealthEvent: Identifiable, Equatable {
    let id: String
    let title: String
    let state: HealthState
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
    static func resolve(_ events: [HealthEvent]) -> HealthState {
        if events.contains(where: { $0.state == .abnormal }) { return .abnormal }
        if events.isEmpty || events.contains(where: { $0.state == .unknown }) { return .unknown }
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
            domainID: ros?.domainId,
            expectedDomainID: ros?.expectedDomainId
        )
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
                downstreamUnknownReason: downstreamUnknownReason
            ))
        }
        return events
    }

    private static func rosGraphState(agentOnline: Bool, snapshot: ConsoleSnapshot?) -> HealthState {
        guard agentOnline, let ros = snapshot?.ros, ros.status == "ok", let age = ros.graphAgeMs, age >= 0 else { return .unknown }
        return age > rosGraphTimeoutMs ? .abnormal : .normal
    }

    private static func downstreamBlockReason(
        agentOnline: Bool,
        graphState: HealthState,
        domainState: HealthState,
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
                eventState = .abnormal
                observation = "Agent 报告帧龄 \(age) ms，超过 \(imageTimeoutMs) ms 图像超时阈值；\(publisherText)。"
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
                eventState = .abnormal
                observation = "RGB/深度帧已超时；帧龄 \(ageText)，\(publisherText)；最后 ROS Header 时间 \(reading?.sourceStamp ?? "未知")。"
                check = "测试驾驶舱 → 帧新鲜度；NUC ROS 日志、相机节点与对应 Topic"
            }
        } else if status == "unavailable" {
            if isYOLO {
                eventState = .unknown
                observation = "未发现 YOLO 输出发布者；置信度门控或模块未运行均可能导致无输出，当前没有故障结论。"
                check = "确认当前模式是否要求 YOLO 输出；查看输入流、节点日志与置信度发布门控"
            } else {
                eventState = .notRunning
                observation = "Topic 未发现发布者；当前未确认该相机模块应处于运行状态，因此不判定为故障。"
                check = "若当前模式要求该数据流，查看 ROS graph 与相机模块状态"
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
            eventState = .abnormal
            observation = "Topic 已被观测到，但消息类型不兼容：\(reading?.topicType ?? "未知")；\(publisherText)。"
            check = "系统诊断 → 运行时 Topic；核对该 Topic 的 ROS 消息类型"
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
