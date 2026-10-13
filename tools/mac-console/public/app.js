const byId = (id) => document.getElementById(id);

function setText(id, value) {
  byId(id).textContent = value == null || value === "" ? "—" : String(value);
}

function stateClass(element, kind) {
  element.classList.remove("good", "warning", "bad", "pending");
  element.classList.add(kind);
}

function shortSha(value) {
  return value && value !== "未记录" ? value.slice(0, 12) : (value || "—");
}

function formatBytesGiB(value) {
  return value == null ? "未知" : `${value.toFixed(1)} GiB`;
}

function formatDuration(seconds) {
  if (!Number.isFinite(seconds)) return "—";
  const days = Math.floor(seconds / 86400);
  const hours = Math.floor((seconds % 86400) / 3600);
  return days ? `${days} 天 ${hours} 小时` : `${hours} 小时`;
}

function addListItems(id, items, emptyText) {
  const list = byId(id);
  list.replaceChildren();
  if (!items?.length) {
    const empty = document.createElement("li");
    empty.className = "empty";
    empty.textContent = emptyText;
    list.append(empty);
    return;
  }
  for (const value of items) {
    const item = document.createElement("li");
    item.textContent = value;
    list.append(item);
  }
}

function renderResources(remote) {
  if (!remote) return;
  const { host, memory, disk } = remote;
  setText("host-name", `${host.hostname} · ${host.os}`);
  setText("os-label", `${host.kernel}`);
  setText("cpu-label", host.cpuModel);
  const load = host.loadAverage?.map((x) => x.toFixed(2)).join(" / ") || "—";
  setText("load-label", `${load} · ${host.logicalCores || "?"} 核`);
  setText("memory-label", `${memory.usedPercent}% · 可用 ${formatBytesGiB(memory.availableGiB)}`);
  setText("disk-label", `${disk.usedPercent}% · 剩余 ${formatBytesGiB(disk.freeGiB)}`);
  setText("uptime-label", formatDuration(host.uptimeSeconds));

  const temperatures = remote.thermalZones || [];
  const cpuZone = temperatures.find((zone) => /TCPU|x86_pkg_temp/i.test(zone.name));
  const maxZone = temperatures.reduce((max, zone) => Math.max(max, zone.celsius), -Infinity);
  setText("thermal-label", cpuZone ? `${cpuZone.name} ${cpuZone.celsius} °C` : (Number.isFinite(maxZone) ? `最高 ${maxZone} °C（传感器区）` : "无读数"));

  const links = byId("network-links");
  links.replaceChildren();
  for (const link of remote.networkLinks || []) {
    const chip = document.createElement("span");
    chip.className = `chip ${link.state === "up" ? "up" : "down"}`;
    chip.textContent = `${link.name}: ${link.state}`;
    links.append(chip);
  }
  if (!links.children.length) links.textContent = "未读取";
}

function renderRos(remote) {
  const ros = remote?.ros;
  const rosState = byId("ros-state");
  if (!ros) {
    setText("ros-state", "无法读取");
    stateClass(rosState, "bad");
    setText("ros-detail", "SSH 实时快照不可用");
    return;
  }
  setText("ros-distro", ros.distro ? `ROS 2 ${ros.distro}` : "ROS 环境未知");
  if (!ros.nodeQueryOk || !ros.topicQueryOk) {
    setText("ros-state", "查询失败");
    stateClass(rosState, "bad");
  } else if (ros.nodes.length) {
    setText("ros-state", `${ros.nodes.length} 个节点`);
    stateClass(rosState, "good");
  } else {
    setText("ros-state", "图可读 · 无节点");
    stateClass(rosState, "warning");
  }
  setText("ros-detail", `${ros.nodes.length} 节点 · ${ros.topics.length} 个 Topic 名称`);
  setText("node-count", ros.nodes.length);
  setText("topic-count", ros.topics.length);
  addListItems("node-list", ros.nodes, ros.nodeQueryOk ? "当前没有活动 ROS 节点" : "节点列表读取失败");
  addListItems("topic-list", ros.topics, ros.topicQueryOk ? "当前没有 ROS Topic" : "Topic 列表读取失败");
  const onlyBuiltins = ros.topics.length > 0 && ros.topics.every((topic) => ["/rosout", "/parameter_events"].includes(topic));
  const flow = byId("topic-flow-state");
  setText("topic-flow-state", "未采样");
  stateClass(flow, "warning");
  setText("topic-flow-detail", onlyBuiltins ? "仅发现 ROS 内置 Topic" : `${ros.topics.length} 个 Topic 名称可见`);
  setText("graph-note", "本页面只枚举节点和 Topic 名称，不订阅数据；消息频率、时间戳和传感器回执均未验证。物理控制器反馈始终单独标记为未核验。");
}

function renderVersions(snapshot) {
  const github = snapshot.status?.["GitHub main"] || {};
  const nuc = snapshot.status?.NUC || {};
  setText("github-sha", shortSha(github.SHA));
  setText("nuc-sha", shortSha(nuc.SHA || snapshot.remote?.repository?.sha));
  setText("nuc-worktree", `${nuc["分支"] || snapshot.remote?.repository?.branch || "—"} · ${nuc["未提交修改"] || "工作区状态未知"}`);
  setText("deployed-sha", shortSha(nuc["当前部署版本"]));

  const sync = byId("sync-state");
  const remoteSha = snapshot.remote?.repository?.sha || nuc.SHA;
  if (github.SHA && remoteSha && github.SHA === remoteSha) {
    setText("sync-state", "与 main 同 SHA");
    sync.style.color = "#267a55";
    sync.style.background = "#edf7f0";
  } else if (github.SHA && remoteSha) {
    setText("sync-state", "SHA 不同，待比较");
    sync.style.color = "#a36a10";
    sync.style.background = "#fff8e9";
  } else {
    setText("sync-state", "版本未知");
  }
}

function renderWorkspaces(remote) {
  const workspaceRoot = byId("workspace-list");
  workspaceRoot.replaceChildren();
  for (const workspace of remote?.workspaces || []) {
    const row = document.createElement("div");
    row.className = "workspace-row";
    const title = document.createElement("div");
    title.className = "workspace-name";
    const name = document.createElement("span");
    name.textContent = `${workspace.workspace} · ${workspace.overlay}`;
    const count = document.createElement("span");
    count.textContent = `${workspace.packages.length} 包`;
    title.append(name, count);
    const packages = document.createElement("div");
    packages.className = "workspace-packages";
    packages.textContent = workspace.packages.join(" · ") || "未发现已安装包";
    row.append(title, packages);
    workspaceRoot.append(row);
  }
  if (!workspaceRoot.children.length) {
    const empty = document.createElement("p");
    empty.className = "empty";
    empty.textContent = "未找到可读取的 p7 / p9b 安装叠加层";
    workspaceRoot.append(empty);
  }

  const componentList = byId("component-list");
  componentList.replaceChildren();
  const checks = [
    ["Livox SDK2 动态库", remote?.components?.livoxSdkLibrary ? "good" : "bad", remote?.components?.livoxSdkLibrary ? "存在" : "缺少"],
    ["历史 P8 MID-360 overlay", "warning", remote?.components?.legacyP8LivoxOverlay ? "旧版本存在；尚未核验" : "未找到"],
    ["Orbbec 驱动源码", remote?.components?.orbbecSourcePresent ? "good" : "bad", remote?.components?.orbbecSourcePresent ? "源码存在" : "缺少"],
    ["Orbbec ROS 包已构建", remote?.components?.orbbecBuiltInMainOverlay ? "good" : "bad", remote?.components?.orbbecBuiltInMainOverlay ? "已安装" : "未安装"],
    ["YOLO ROS 包已构建", remote?.components?.yoloBuiltInMainOverlay ? "good" : "bad", remote?.components?.yoloBuiltInMainOverlay ? "已安装" : "未安装"],
    ["位姿估计 ROS 包已构建", remote?.components?.poseEstimatorBuiltInMainOverlay ? "good" : "bad", remote?.components?.poseEstimatorBuiltInMainOverlay ? "已安装" : "未安装"],
    ["OpenVINO 权重文件", remote?.components?.openvinoXmlPresent && remote?.components?.openvinoWeightsBinPresent ? "good" : "bad", remote?.components?.openvinoXmlPresent && remote?.components?.openvinoWeightsBinPresent ? "XML + BIN" : "缺少 best.bin"],
    ["OpenVINO Python", remote?.components?.openvinoPythonPresent ? "good" : "bad", remote?.components?.openvinoPythonPresent ? "可导入" : "缺少依赖"],
    ["Ultralytics Python", remote?.components?.ultralyticsPythonPresent ? "good" : "bad", remote?.components?.ultralyticsPythonPresent ? "可导入" : "缺少依赖"],
    ["Orbbec Python SDK", remote?.components?.orbbecPythonSdkPresent ? "good" : "bad", remote?.components?.orbbecPythonSdkPresent ? "可导入" : "未安装"],
    ["ROS Bag 2", remote?.components?.rosbag2DebPresent ? "good" : "bad", remote?.components?.rosbag2DebPresent ? "已安装" : "未安装"],
  ];
  for (const [label, state, detail] of checks) {
    const row = document.createElement("div");
    row.className = "component-row";
    const dot = document.createElement("span");
    dot.className = `component-status ${state}`;
    const text = document.createElement("span");
    text.textContent = `${label}：${detail}`;
    row.append(dot, text);
    componentList.append(row);
  }
}

function renderTestRecords(snapshot) {
  const root = byId("test-records");
  root.replaceChildren();
  const github = snapshot.status?.["GitHub main"];
  const nuc = snapshot.status?.NUC;
  const records = [
    ["NUC · P7/P8 全套", nuc?.["P7/P8 全套"]],
    ["NUC · Nav2 工作区构建", nuc?.["ROS Nav2 工作区构建"]],
    ["NUC · 机械臂 ROS 工作区构建", nuc?.["ROS 机械臂工作区构建"]],
    ["NUC · 底盘 STM32 构建", nuc?.["底盘 STM32 构建"]],
    ["NUC · 机械臂 STM32 构建", nuc?.["机械臂 STM32 构建"]],
    ["NUC · 当前部署 SHA", nuc?.["当前部署版本"]],
    ["ThinkBook · 当前状态", snapshot.status?.ThinkBook?.SHA ? `${shortSha(snapshot.status.ThinkBook.SHA)} · ${snapshot.status.ThinkBook["未提交修改"]}` : null],
    ["GitHub · main", github?.SHA],
  ];
  for (const [label, value] of records) {
    const row = document.createElement("div");
    row.className = "record-row";
    const title = document.createElement("div");
    title.className = "record-title";
    title.textContent = label;
    const detail = document.createElement("div");
    detail.className = "record-value";
    detail.textContent = value || "未记录";
    row.append(title, detail);
    root.append(row);
  }
}

function renderLogs(remote) {
  const root = byId("build-logs");
  root.replaceChildren();
  for (const entry of remote?.recentBuildLogs || []) {
    const block = document.createElement("section");
    block.className = "log-block";
    const title = document.createElement("div");
    title.className = "log-heading";
    title.textContent = `${entry.title} · ${entry.updatedAt}`;
    const text = document.createElement("pre");
    text.className = "log-text";
    text.textContent = entry.text;
    block.append(title, text);
    root.append(block);
  }
  if (!root.children.length) {
    const empty = document.createElement("p");
    empty.className = "empty";
    empty.textContent = "当前没有可读取的构建日志片段。";
    root.append(empty);
  }
}

function render(snapshot) {
  const notice = byId("notice");
  const issues = [snapshot.statusError, snapshot.remoteError].filter(Boolean);
  notice.textContent = issues.join(" ");
  notice.classList.toggle("hidden", issues.length === 0);

  const ssh = byId("ssh-state");
  if (snapshot.remote) {
    setText("ssh-state", "已连接");
    stateClass(ssh, "good");
    renderResources(snapshot.remote);
  } else {
    setText("ssh-state", "不可达");
    stateClass(ssh, "bad");
  }
  renderRos(snapshot.remote);
  renderVersions(snapshot);
  renderWorkspaces(snapshot.remote);
  renderTestRecords(snapshot);
  renderLogs(snapshot.remote);

  const date = new Date(snapshot.generatedAt);
  setText("refresh-time", Number.isNaN(date.valueOf()) ? "读取时间未知" : `最近读取 ${date.toLocaleString("zh-CN")}`);
}

async function refresh() {
  const button = byId("refresh");
  button.disabled = true;
  button.innerHTML = "<span aria-hidden=\"true\">↻</span> 正在读取";
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    const payload = await response.json();
    render(payload);
  } catch {
    render({ generatedAt: new Date().toISOString(), status: {}, remote: null, remoteError: "无法连接本机状态服务。" });
  } finally {
    button.disabled = false;
    button.innerHTML = "<span aria-hidden=\"true\">↻</span> 刷新状态";
  }
}

byId("refresh").addEventListener("click", refresh);
refresh();
