# Lunar Robot Console

Mac 上运行的只读状态面板。页面由本机 Node.js 服务提供，数据通过现有 SSH 别名读取 NUC；它不会启动 ROS 节点、传感器、测试任务或执行器，也没有可输入任意 Shell 命令的功能。

## 启动

要求 Node.js 20+、可用的 SSH 配置别名 `nuc`，以及从仓库根目录运行的 `tools/device_status.sh` 所需的现有 GitHub/ThinkBook/NUC 访问配置。Mac 当前用户需要能够读取私有仓库 `main`。

```bash
cd tools/mac-console
npm start
```

然后打开 <http://127.0.0.1:4173>。也可从 Finder 双击 `Open Lunar Robot Console.command`，服务启动后会打开浏览器。按服务终端中的 Ctrl-C 关闭；关闭服务不会向 NUC 发送控制指令。

自定义 SSH 别名或本机端口：

```bash
LUNAR_NUC_ALIAS=nuc LUNAR_CONSOLE_PORT=4173 npm start
```

## 数据来源与安全边界

- GitHub SHA、设备分支/工作区状态、测试记录和部署 SHA 复用 [`tools/device_status.sh`](../device_status.sh) 的输出。
- NUC 资源、ROS 节点/Topic 名称、已安装工作区包和最近构建日志由 [`remote_snapshot.py`](remote_snapshot.py) 经 SSH 标准输入执行。远端不保存该脚本。
- SSH 禁用 agent 转发、转发通道和 TTY；本地 HTTP 服务只绑定 `127.0.0.1`。
- 服务只开放静态页面和 `GET /api/status`。其 SSH 命令和远端文件清单是固定的，页面不接受命令或路径输入。
- ROS 图查询使用 `ros2 node list --no-daemon` / `ros2 topic list --no-daemon`；不启动持久 ROS daemon。页面不订阅 Topic，因此“Topic 有数据”仍显示未采样。
- 控制器、传感器和执行器反馈不在本版本探测；“SSH 已连接”与“ROS 节点存在”不会表示真实设备健康。
- 第一次刷新可能需要 5～20 秒；刷新有短暂内存缓存，不会自动轮询。

## 本机检查

```bash
npm run check
```

当前界面仅用于只读监控和查看记录。传感器数据画面、地图、路径和机械臂模型计划后续由 Foxglove 等成熟 ROS 可视化组件承载；任何实体控制都不属于此界面。
