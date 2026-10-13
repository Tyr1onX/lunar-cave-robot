# Lunar Cave Robot｜月球洞穴机器人

吉甲大师战队的移动机器人与机械臂协作项目。GitHub 私有仓库 Tyr1onX/lunar-cave-robot 是团队源码的统一入口；ThinkBook 用于开发和离线测试，NUC 用于车载部署准备。当前 main 基线为 `b1cfe617209a06d69000a515af82b9b95add62ef`；最近完整 P7/P8 离线套件在代码 SHA `07a56c3a33882aa2866f1076d8597776e2579bcb` 上通过 12/12。PR #8 仍开放；P9-F 独立分支已修复其机械臂输出许可、租约和恢复时序缺陷，并补充集成主机测试。机械臂缺少真实 XRobot 产品输入；归档源码也不代表实体控制器当前固件版本。

## 源码地图

| 目录 | 内容 |
| --- | --- |
| [`tzb_ws/`](tzb_ws/) | 相机驱动包装、视觉检测与目标位姿、机械臂模型、运动规划、主机侧机械臂串口安全驱动 |
| [`Nav2_rea_ws/`](Nav2_rea_ws/) | Nav2 导航、MID-360 驱动、定位建图、底盘主机串口桥和 P6 任务协调 |
| [`firmware/arm/`](firmware/arm/) | STM32H723 机械臂工程；含 CubeMX、CMake、Keil、HAL/CMSIS、FreeRTOS、LibXR 与团队应用源码 |
| [`firmware/chassis/`](firmware/chassis/) | STM32H723 六轮底盘工程；含 CubeMX、CMake、Keil、HAL/CMSIS、FreeRTOS 与团队控制源码 |
| [`hardware/`](hardware/) | 机械 CAD、电气/PCB、接线、BOM 和设备资料归档入口；当前仅放资产接收说明，未找到可归档原始设计文件 |
| [`tools/`](tools/) | P4～P8 离线验证、源码完整性检查、三端状态查询与安全同步入口 |
| [docs/](docs/) | [新人技术总览](docs/TECHNICAL_OVERVIEW.md)、架构、验收、安全、硬件负责人清单与部署资料 |
| [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) | 第三方源码及许可证来源摘要；各依赖原始许可文件保留在对应源码目录 |

两个 ROS 2 工作空间保持独立。`tzb_ws` 与 `Nav2_rea_ws` 中存在历史重名消息包；不要把两者合并到同一个 colcon 工作空间，也不要在一个终端随意叠加两套 `install/setup.bash`。

## 系统链路

```text
MID-360 → 定位/建图 → Nav2 → 底盘主机桥 → UART 14 字节帧 → 六轮底盘 STM32
相机 → 检测/目标位姿 → 机械臂规划与任务协调 → 主机安全驱动 → USB CDC → 机械臂 STM32
```

底盘工程配置 USART1 115200/8N1，发送帧为 `A5 + 3 个小端 float32 + 4A`，固件随后经 FDCAN1/2 控制六个电机。ROS 2 P5b 主机桥的发送速率、帧格式与该源码静态对照一致；没有做真实 UART、CAN、电机或制动验收。

机械臂固件源码配置 STM32H723、FDCAN3 和 USB OTG HS CDC；源码中的主机帧为 46 字节下行、35 字节状态上行。ROS 2 主机接口与固件协议的静态帧定义已归档，但 USB 设备映射、实际端口和实机应答尚未核验。固件源码不等于实体控制器上正在运行的版本。

模块入口、接口边界和验证状态见[系统架构](docs/ARCHITECTURE.md)与[软硬件版本对应表](docs/HARDWARE_SOFTWARE_MATRIX.md)。

## 当前验证边界

- P0～P8 ROS 2 主线已进入 `main`。P8 记录 ThinkBook 和 NUC 各通过 12 个 PTY 离线场景；NUC 上 MID-360 驱动曾成功编译。记录不表示真实点云、IMU 或机器人运动验收通过。
- P9-C 离线稳定性修正确认后，ThinkBook 两轮完整 P7 场景各通过 12/12；P4f 的 19 项 PTY 驱动检查、10 项 P6 单测/静态合同、规划取消竞态及六个 P6 场景也通过。详情和源文件 SHA-256 见 [P9-C 离线验证](docs/P9C_OFFLINE_VALIDATION.md)。结果不等于实机验收。
- ThinkBook Ubuntu/GCC 11.4 已通过 P9-F 底盘解析器与状态机、机械臂状态机、Host/恢复集成、CAN 故障注入，以及生产 `Remote.c`→安全输出门→`Joint::ControlStep()` 快速模式切换集成测试；Humble 隔离构建也通过 `lite_motion_planner` 与两个消息依赖包。机械臂完整固件仍因 XRobot 生成输入缺失而未构建。测试细节与硬件边界见 [P9-F 机械臂安全集成记录](docs/P9F_ARM_SAFETY_INTEGRATION.md)；P9-E 问题清单保留为历史审查 [记录](docs/P9E_PR8_INTEGRATION_REVIEW.md)。
- 相机采集、传感器标定、真实 MCU 应答、物理急停、机械限位、底盘制动和实机抓取仍未验收。
- 本轮没有打开 STM32 串口、烧录固件或启动实体机器人运动。

新成员先读[技术总览](docs/TECHNICAL_OVERVIEW.md)。详见[P8 验收报告](docs/P8_VALIDATION_REPORT.md)、[项目状态](docs/PROJECT_STATUS.md)、[发布审计](docs/RELEASE_AUDIT.md)、[硬件负责人清单](docs/HARDWARE_OWNER_CHECKLIST.md)、[P9-F 安全集成记录](docs/P9F_ARM_SAFETY_INTEGRATION.md)、[PR #8 历史接入审查](docs/P9E_PR8_INTEGRATION_REVIEW.md)、[XRobot 构建恢复记录](docs/XROBOT_ARM_BUILD_RECOVERY.md)、[资产清单](docs/PROJECT_ASSETS.md)和[P9-B 安全审计](docs/STM32_SAFETY_AUDIT.md)。

## 开发与离线验证

推荐环境为 Ubuntu 22.04、ROS 2 Humble、colcon，以及各 ROS 包声明的系统依赖。首次构建按[P8 部署说明](docs/P8_DEPLOYMENT.md)检查依赖；两个工作空间分开构建、分开运行。

```bash
git clone --branch main git@github.com:Tyr1onX/lunar-cave-robot.git
cd lunar-cave-robot
P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator ./tools/p7_offline/run.sh
```

完整的 12 项离线故障场景可通过 `./tools/run_offline_validation.sh` 运行。脚本只调用仓库内隔离 PTY 测试，并把测试结果写入用户状态目录；不会启动设备驱动或实体机器人。源码和脚本自身的完整性检查入口为 `python3 tools/verify_project_assets.py`。
GitHub Actions 在 PR 与 main push 上强制运行底盘生产协议解析器、底盘安全状态机、机械臂安全状态机、机械臂 Host/恢复集成和真实 `Joint::ControlStep` 输出门测试；所需测试缺失时工作流失败。工作流不连接 STM32 或电机。

固件编译入口（构建目录和日志位于用户缓存目录，不会连接控制器）：

```bash
export STM32CUBECLT_PATH=/path/to/arm-gnu-toolchain
./tools/build_stm32_firmware.sh chassis
./tools/build_stm32_firmware.sh arm
```

固件需要 CMake 3.22+ 和 `arm-none-eabi-gcc`；Keil 工程还需要 MDK-ARM 及 STM32H7 设备包。脚本只配置和构建，不调用烧录目标，并将结果按源码 SHA、分支和范围写入设备状态。机械臂当前会因缺少 XRobot 生成文件而停止配置；两套工程的实际构建结论见[固件构建审计](docs/STM32_BUILD_AUDIT.md)。底盘主机侧协议测试可在 ThinkBook 上运行 `firmware/chassis/tests/host/run.sh`，它编译实际底盘解析器/命令代码并 mock HAL，不连接串口或电机。输入 ZIP、源码来源和逐文件 SHA-256 见 [`docs/PROJECT_ASSETS.md`](docs/PROJECT_ASSETS.md) 与 [`firmware/SOURCE_SHA256SUMS`](firmware/SOURCE_SHA256SUMS)，协议和安全缺口见 [`docs/STM32_PROTOCOL_SPEC.md`](docs/STM32_PROTOCOL_SPEC.md)、[`docs/STM32_SAFETY_AUDIT.md`](docs/STM32_SAFETY_AUDIT.md) 与 [`docs/STM32_SAFETY_REMEDIATION_PLAN.md`](docs/STM32_SAFETY_REMEDIATION_PLAN.md)。

## 三端版本管理

GitHub `main` 是源码版本来源。Mac 连接 ThinkBook 和 NUC 后，在仓库目录运行：

```bash
./tools/device_status.sh
./tools/sync_devices.sh
```

状态查询只读；同步脚本只克隆新目录或对干净的 `main` 工作树执行快进更新。遇到未提交修改、分支分叉或错误仓库时会停止，不重置工作区。它不会启动 ROS 节点或机器人程序。默认统一目录为 ThinkBook `~/lunar-cave-robot`、NUC `~/lunar-cave-robot`；原有 P0～P8 工作空间不移动、不覆盖。可以通过 `THINKBOOK_REPO`、`NUC_REPO` 环境变量更改目标路径。Mac 侧一键查询依赖 SSH 别名 `thinkbook`、`nuc` 和 GitHub 只读访问；同步还需要 Mac SSH agent 中已授权 GitHub 的密钥，密钥不会复制到设备。设备路径、备份策略和回滚步骤见[部署流程](docs/DEPLOYMENT_WORKFLOW.md)与[设备状态](docs/DEVICE_STATUS.md)。

状态查询会分别显示 P7/P8 全套、单场景、两个 ROS 工作区构建以及两套 STM32 固件构建记录。单场景通过 `./tools/run_p7_scenario.sh normal` 运行并单独记账，不会覆盖完整套件结果；两个 ROS 工作区可用 `./tools/build_ros2_offline_workspaces.sh` 独立构建并自动记录各自结果。测试记录附带执行时 SHA、分支、时间和测试范围；未运行显示“未记录”。

## 仓库及分享边界

仓库保持 Private，供战队内部开发与交接。团队已确认第一方工程可进入该私有仓库；这不自动授权公开发布或重新许可第三方代码。第三方原许可和来源保留；尚未确认可分发的电机厂商手册未上传，记录见[项目资产清单](docs/PROJECT_ASSETS.md)。
