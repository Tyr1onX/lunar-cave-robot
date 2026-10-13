# Lunar Robot Console：Mac 可视化调试平台

## 第一版已实现范围

`tools/mac-console/` 提供一个无第三方包依赖的本机浏览器控制台。Mac 运行静态页面和 Node.js HTTP 服务，NUC 继续独占 ROS 2 运行时。页面的“刷新状态”只执行固定、只读的数据读取：

1. 复用 `tools/device_status.sh` 显示 GitHub `main`、ThinkBook/NUC SHA、分支、工作区改动、已有测试记录和显式部署 SHA。
2. 经已有 SSH 别名 `nuc` 运行固定的 `remote_snapshot.py`，读取 NUC CPU/内存/磁盘/温度区/网卡、ROS 节点与 Topic 名称、已构建工作区包以及构建日志末尾。
3. 页面分别展示 SSH、ROS 图、Topic 名称、Topic 数据采样和实体反馈；未采样的 Topic 流及未读取的物理反馈始终标注为“未采样/未核验”。

本地服务只绑定 `127.0.0.1`。直接快照 SSH 关闭 agent 与端口转发、禁止 TTY；设备状态脚本也显式关闭这些转发，并关闭 Git 可选索引写入。页面 API 只有 `GET /api/status`；命令、远端路径和日志文件均写死在项目源码中，不提供任意 Shell 执行、传感器启动、测试执行或电机控制入口。远端快照通过 SSH 标准输入执行，不在 NUC 安装或保存脚本。

## 逻辑分层

```text
Mac 浏览器界面（HTML/CSS/JS）
         │ localhost GET /api/status
Mac Node.js 只读协调器
         ├── tools/device_status.sh（版本、测试和部署记录）
         └── OpenSSH 固定 SSH 别名 + 固定 Python 快照
                         │ SSH 加密连接
                         ▼
NUC：主机资源 + ROS 2 graph introspection（--no-daemon）+ 已有构建/测试日志
```

ROS 2 的 DDS 不经过 Mac，也不监听公网端口。本地控制台没有直接建立 ROS Bridge。ROS 查询只枚举节点和 Topic 名称，不订阅消息、不测频率、不读取驱动器回执；SSH 成功、节点存在和硬件反馈是三个不同状态。

## 技术选型

| 方案 | 适用部分 | 本项目决策 |
| --- | --- | --- |
| 当前 Node + 本机网页 | 快速提供真实设备状态面板；不用为 Mac 安装 ROS 2，也不需额外 npm 依赖 | **第一版使用。** Mac 已有 Node 26；固定 SSH 任务由服务端执行，界面无法输入命令。当前是本机浏览器 UI，不是签名原生 `.app`。 |
| SwiftUI | Apple Silicon/macOS 原生窗口、菜单栏和系统通知 | 可作为纯 Mac 原生化选择。当前 Mac 的 `xcodebuild` 不可用，且安装的 Swift 6.3.3 与 macOS 27 SDK 要求的 Swift 6.4 不匹配，本轮不能实际构建验证；Apple 的 macOS App 创建流程以 Xcode target 为入口。[Apple macOS SwiftUI 教程](https://developer.apple.com/tutorials/swiftui/creating-a-macos-app)、[Xcode 系统要求](https://developer.apple.com/xcode/system-requirements) |
| Tauri + TypeScript | 后续把现有 Web UI 包成轻量原生窗口；能力面可收敛到固定命令 | 是可行的第二阶段候选，但本机缺少 Rust/Cargo，当前不适合作为首个可运行交付。若采用，shell capability 只放行固定 sidecar/参数；Tauri 支持按程序和参数限制 shell API。[Tauri Shell 权限范围](https://v2.tauri.app/reference/javascript/shell/) |
| Foxglove | RGB/深度图、点云、TF、地图、路径、规划轨迹和 ROS Bag/MCAP 回放 | **第二/三版复用。** 官方 ROS 2 集成支持实时 Foxglove Bridge 与记录文件；它比自研 3D/点云渲染合适。[Foxglove ROS 2](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2) |
| RViz | 开发机/NUC 上临时检查 TF、Nav2、机器人模型 | 可保留为 ROS 开发工具，不把 NUC GUI 转发到 Mac 作为主要产品界面；远程大点云体验与带宽会影响现场使用。 |

Foxglove Bridge 只有在数据可视化阶段通过单独审查后才安装/启动。优先使用 SSH 隧道或受控 Tailscale 网络并限制端口/认证；不得暴露原始 DDS 或无认证 WebSocket 到公网。第一版不部署 Foxglove Bridge。

## 分阶段路线

### 第 1 阶段：只读设备监控（当前）

- Mac 本机页面读取真实 SSH/NUC 状态、Git 与部署版本、ROS 节点/Topic 名称、已记录测试和日志。
- 不自动刷新，用户显式点“刷新状态”；显示查询时间和部分失败。
- 只列 Topic 名称，不声称消息流健康；不连接实体反馈。
- 没有任务启动/停止按钮。页面刷新不是实体急停。

### 第 2 阶段：传感器只读画面

- 由现场负责人批准并核对地址/型号后，部署 Foxglove Bridge 或受限的图像网关；只暴露白名单 Topic。
- 添加 RGB、深度、检测叠框、点云、IMU、TF 面板；记录带宽和端到端延迟。
- 先使用录制包验证布局，再做静态传感器采集；不加入速度/导航目标发布。

### 第 3 阶段：离线算法和导航/机械臂预览

- Foxglove layouts 展示地图、定位、Nav2 路径和代价地图；使用 Rosbag/MCAP 回放测试。
- 对机械臂模型、关节状态、末端位姿、IK/RRT 路径和碰撞结果只做软件规划预览。
- 页面显式标出仿真/回放/实机来源，禁止把目标关节值当成真实反馈。

### 第 4 阶段：受控测试任务

- 每个任务有代码审查的白名单入口、前置条件、权限角色、操作确认、超时、取消/故障退出、审计日志和结果记录。
- GUI 不接受脚本或自由命令字符串；服务端只执行预先登记的任务 ID。
- 实体执行器操作需硬件负责人、安全门槛和独立硬件急停审批。软件“停止”状态不表示物理停止、制动或机械臂防坠。

## 第一版使用

从仓库根目录进入 `tools/mac-console/` 后运行 `npm start`，浏览器打开 `http://127.0.0.1:4173`；也可在 Finder 双击 `Open Lunar Robot Console.command`。前置条件、SSH 行为和本机验证见该目录的 README。需要 Node.js 20+、SSH 别名 `nuc`/`thinkbook` 与仓库现有 GitHub 只读凭据。服务退出只关闭本机页面服务。

## 当前边界

- 第一版以本轮从 NUC 读取到的真实数据为准；启动页面后取新的实时快照。
- 传感器 Topic 消息流未采样；相机/雷达/底盘/机械臂物理连接不探测。
- 控制器反馈、急停、制动、负载保持和实体部署版本不由软件推断。
- macOS 当前 Xcode/Swift 工具链不匹配，因此交付为浏览器桌面面板，尚未打包/签名为原生应用。
