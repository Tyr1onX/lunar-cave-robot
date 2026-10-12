# 项目资产清单

更新日期：2026-10-12。原始工作目录和压缩包均未修改；本表记录实际找到的快照、整理结果和未导入内容。第一方固件已按团队确认归属和私有仓库内部归档授权纳入；第三方许可仍按各自条款保留。

## 已进入仓库

| 资产 | 位置 | 版本/来源 | 整理结果 |
| --- | --- | --- | --- |
| ROS 2 机械臂、视觉与规划 | [`tzb_ws/`](../tzb_ws/) | P8 主线 | 保留原工作空间边界；相机 ROS 源码和机器人 URDF/STL 模型已在主线 |
| ROS 2 导航、定位与底盘桥 | [`Nav2_rea_ws/`](../Nav2_rea_ws/) | P8 主线 | 保留原工作空间边界；P7/P8 安全流程未被历史分支覆盖 |
| 机械臂 STM32 | [`firmware/arm/`](../firmware/arm/) | 原包 Git `HEAD` `963fd0275ddbe2c759b35250842284c6d62e90c1`；LibXR `783b157fb3e9e5a7801edc9083834ba3ac49801f` | 保留当前 ZIP 工作树中的工程配置、源码、HAL/CMSIS、USB、FreeRTOS、LibXR 与许可证；已剔除 Git 元数据、构建输出、缓存、调试配置、轨迹 CSV 和执行文件 |
| 六轮底盘 STM32 | [`firmware/chassis/`](../firmware/chassis/) | 原始 ZIP SHA-256 `823ff6981859dbce1d05a3d4864b921bd0f9aa5165335d0caec7d12852701d2e` | 保留 CubeMX、CMake、Keil、Core/User、H7 HAL/CMSIS、FreeRTOS、启动/链接配置和所需 DSP 库；排除构建输出、其他 MCU 示例、缓存、日志、个人 IDE 设置 |
| 第三方许可摘要 | [`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md) | P9 固件归档新增 | 原始许可证文件保留在固件依赖目录 |

归档校验入口：`python3 tools/verify_project_assets.py`。归档源文件的逐文件 SHA-256 清单为 `firmware/SOURCE_SHA256SUMS`；文本按 Git 规范化后的 LF 字节计算，二进制按原始字节计算，因此跨平台换行符不会造成误报。源压缩包 SHA 与来源路径见下文。

## 固件原包和来源

### 机械臂

- 找到位置：Mac 下载目录 `~/Downloads/lunar_rover.zip`，大小 94,852,918 字节，SHA-256 `63ac07e173e148ab248b1689ccae64ba380b730d0c64b2b29d82b96307c48ab8`。该哈希与此前记录相同。
- 原包包含 4,754 个 ZIP 条目、约 253.6 MB 解压内容，以及 Git 元数据。Git `HEAD` 为 `963fd...`，但压缩时的工作树还包含未提交源码：`CMakeLists.txt`、`others/src/thread/variables.cpp` 的变化，以及 `User/`、`Modules/`、`.config.yaml`、LibXR CMake 接口等新增文件。本归档保留这些实际工程文件，不纳入原 `.git` 和 VS Code 本机任务。
- LibXR 工作树指向上游提交 `783b157...`；其 Apache-2.0 许可证和 Eigen 自带许可证一并保留。
- 本地和设备历史工程中均未找到 `User/xrobot_main.hpp`、`Modules/CMakeLists.txt`。`app_main.cpp` 和 LibXR CMake 配置要求这两个生成文件；机械臂源码因此还不能认定为可独立配置/编译。

### 六轮底盘

- 找到位置：用户本轮提供的微信附件 `DM-car-six-wheel.zip`，大小 249,683,285 字节，SHA-256 `823ff6981859dbce1d05a3d4864b921bd0f9aa5165335d0caec7d12852701d2e`。原包含 2,252 个文件、约 894 MB 解压内容，没有项目 Git 元数据。
- 工程包含 `CtrlBoard-H7_IMU.ioc`、CMake/Presets、Keil `.uvprojx`、C 与 FreeRTOS 源码、STM32H723 启动与链接配置。原工程使用的 DSP GCC 静态库和 Keil ARM 静态库均保留；其他架构版本未复制。
- 原 CMake 中的固定 Windows 用户/工具安装路径已移除，改为使用 `PATH` 和 `STM32CUBECLT_PATH`。原 Keil 工程还引用不存在于原包的 `User/Bsp/bsp_usart1.c`、`User/Bsp/rc_pwm.c`，且 CMake 源码列表没有使用它们；在整理副本中删除了这两个失效的 Keil 条目，没有合成替代源文件。现存 Keil `FilePath` 均解析到已归档源码。仅做工程配置清理；没有运行下载/烧录目标。

两台 Ubuntu 设备均没有 `arm-none-eabi-gcc`；本轮没有 STM32CubeIDE/Keil 命令行构建环境。底盘工程的源文件与构建引用已静态核验，但未报告编译成功。当前代码也不能证明实体控制板正在运行该版本。

## 其他发现与处理

### ROS 2 旧压缩包

- ThinkBook `~/teacher_robot_sources/imported/nav_ws.zip`，14,037,727 字节，SHA-256 `0d4d3cb04085fc6bd5f5a351e84124f33866ad643b224e2381f1027f7f7a925c`。与当前 `Nav2_rea_ws/src` 按相对路径比对：206 个源码/配置文件完全相同，20 个同路径文件不同，17 个旧路径无对应文件。旧快照包含直接导航序列节点、旧速度变换、横向速度配置及另一套 IMU/Point-LIO 参数；当前主线已由 P5b 差速桥、P6 任务协调及 P8 安全流程承担相关职责。旧 `fake_vel_transform` 会用固定角速度覆盖输入，旧导航参数还将底盘按全向模型配置。Point-LIO 参数和 terrain self-filter 是未实机验证的历史调参，本轮未恢复。
- ThinkBook `~/teacher_robot_sources/imported/serial_t.zip`，16,927 字节，SHA-256 `49f0cefbe386ab812ec38084d8d960fa4fe47776bcc16e54df6c8a6cb2328476`。其中 8 个源码/配置文件与当前 `tzb_ws/src/serial/driver/` 的同路径版本不同，另有一份旧协议说明。旧包将底盘和机械臂扩展放在同一串口帧，并含固定设备名和 `设备日志目录` 路径；P4e/P4f 当前主机安全驱动已取代该快照，未复制。
- NUC 上另有 `~/桌面/nav_ws.zip`、`~/serial_t.zip` 及回收站中的副本；未移动或删除。它们不改变当前主线代码判断。

### NUC MID-360 开发分支

`~/mid360_driver_dev/repo` 是干净的 `codex/mid360-driver-nuc-ready` 工作树，HEAD `189cf2f...`，此前在该目录对 `livox_ros_driver2` 记录过成功构建。其与当前 `main` 的树差异包含 25 个文件、约 1,873 行删除，其中包括 P7 离线验收脚本、安全文档和 P6 任务/串口代码变化。整分支不合入；驱动 SDK 查找修复已在 P8 主线上，唯一不同的 MID-360 launch 默认参数会直接选择设备配置，保持现有 P8 入口不变。

ThinkBook 的 P4f 历史工作树仍保留着本地未提交修改，部分差异会撤销 P8 的安全状态检查；NUC `~/tzb_ws/README.md` 也有本地修改。这些原目录均未覆盖或清理。

### 机械、电气和资料

- 可访问的 Mac、ThinkBook、NUC 项目路径中没有找到 `.step/.stp`、SolidWorks、DWG/DXF、KiCad/Altium 原理图或 PCB、BOM、接线图及电机/驱动器参数表。
- 当前机械臂 URDF/STL 模型和相机外壳网格已在 `tzb_ws`，无需重复复制。它们是描述/仿真网格，不是可编辑机械 CAD 源文件。
- 底盘 ZIP 内发现厂商资料 `HM系列关节伺服电机使用手册V1.0 (1).pdf`，577,880 字节，SHA-256 `ac9b418495fe7e56f73b2d40977557d3c24e5ca32c636fc377261a83bea86b87`。该手册来自电机厂商，未确认再分发条款，暂不上传；最后已知位置是上述 DM 压缩包内。
- 底盘 ZIP 的 `说明文件/六轮差速电机.txt` 是包含 Windows 用户目录、临时文件路径和工程操作记录的会话/日志文本，不是当前构建输入，已排除；源码和独立协议文档已保留。
- 用户提供的微信 `temp/drag/nav_ws.zip` 与 `serial_t.zip` 路径在本机不存在；同名包在 ThinkBook/NUC 已知归档路径找到并核对。机械臂 ZIP 当前找于 Mac 下载目录，不能把它误报为 ThinkBook 设备文件。

## 后续资产接收约定

可编辑 CAD、原理图、PCB、BOM、接线资料和获准分发的厂商数据表分别放入 [`hardware/mechanical/`](../hardware/mechanical/)、[`electronics/`](../hardware/electronics/)、[`bom/`](../hardware/bom/)、[`wiring/`](../hardware/wiring/) 和 [`datasheets/`](../hardware/datasheets/)。每次导入应记录来源、版本、SHA-256、依赖软件和共享权限；原始包保留在原位置，不把生成件或设备私密网络配置放入 Git。
