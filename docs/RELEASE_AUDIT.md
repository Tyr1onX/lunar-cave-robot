# Pre-publication audit — 2026-10-11

Status: **private repository import authorized; not cleared for public release**. Staging: `~/lunar-cave-robot-staging/` on ThinkBook; NUC unchanged.

## Source snapshots

- `tzb_ws/` from branch `p2b/real-vision-data`, commit `b5bd744` (P0, P1, P2, P2b fixes carried as working-tree content, not Git history).
- `Nav2_rea_ws/` from branch `p0/offline-build`, commit `081ba74`.
- `Nav2_rea_ws/src/localization/dlio/` copies the **modified working tree**, not its nested Git directory or history (upstream `b446a3d`); it includes the restored local changes.
- `p1_offline/` includes only the Python probes; outdated absolute-path P1 YAML removed; excludes run logs and transient helper scripts.
- P2/P2b scripts under `tzb_ws/tools/` retained; README reflects incomplete real-sensor verification.

## P3–P4b private archive update

The integration branch imports only the four source patches in dependency order (P3 `f91bc6a`, P3b `e402434`, P4a `67359bd`, P4b `9d48d5d`), creating new commits from the current private `main`; the original Git history is not imported. See [P3–P4b validation](P3_P4B_VALIDATION.md) for test commands, observed results, and remaining hardware boundaries. Navigation contents were not changed.

The original STM32 firmware project and any firmware ZIP remain outside Git: permission to redistribute, third-party terms, and compiled firmware artifacts have not been cleared. The upper-computer protocol tests are not hardware acceptance evidence. Existing public-release ownership and license blockers below remain unchanged.

## Intentionally excluded

- All `.git` data, virtual environments, build/install/log outputs, Python caches, PID files.
- Local RGB-D bags, private PCD/PGM maps and map scans, large upstream demonstration images and saved Orbbec sample frames.
- Model `.bin` weights (`best.bin`, `yoloe-26l-seg.bin` and other OpenVINO binaries); their `*.xml` architecture and sanitized metadata may remain.
- Bundled Orbbec SDK `SDK/lib` precompiled objects; locally installed Livox SDK `vendor/livox_sdk` binaries.
- Any files not explicitly in the source workspaces or approved offline test resources.

## Open risks before publishing

1. **Ownership / permission:** NUC-recovered team code and DLIO modifications require project-owner publication approval.
2. **Licensing:** mixed GPL-2.0, Apache-2.0, MIT, BSD-like, AGPL-3.0; verify source-level attributions and distribution obligations. The absence of a repository-wide LICENSE is intentional.
3. **Binary provenance:** Orbbec proprietary/redistributable subcomponents and Ultralytics trained models are withheld.
4. **Portability:** legacy hardware-facing YAML/C++ contains previous-machine absolute paths; P2 offline launch resolves package paths, but real robot bringup requires separate review.
5. **Completeness:** P2b has no approved real-world camera bag, TF/calibration evidence, or closed-loop arm acceptance. Missing `tzb_state_test` prevents some runtime integration.
6. **Reproducibility:** full 14+18 package builds require hardware SDK dependencies omitted from this publishable snapshot. Verify safe offline subset separately.

**Do not make this repository public before resolving ownership, licenses, model rights and legacy configuration risks.**

## Local staging verification

- Arm: 13 packages compiled; the Orbbec camera driver was excluded because its binary SDK is not distributed. The pose estimator's optional camera runtime dependency was corrected in staging only.
- Navigation: 14 packages compiled; the Livox driver, FAST-LIO, Point-LIO and pointcloud_tools were excluded pending a separately supplied Livox SDK.
- Safe P2 test: synthetic grasp, detach, placement and return to observe position completed on ROS domain 187 with localhost-only transport. The process was then intentionally interrupted.
- Python and Bash syntax checks passed. No physical robot or sensor driver was activated.
- At least 24 source files still contain prior-machine absolute path references. Hardware bringup is not portable or approved for unattended operation.
- Exact candidate inventory and checksums are stored outside Git in the ThinkBook home directory, as lunar-cave-robot-preupload-manifest.txt and lunar-cave-robot-preupload-sha256.txt.


## 2026-10-11 统一归档分支审计

### 分支与历史边界

统一分支从 archive/p3-p4b-offline-validation 的提交 bb6e61f0ba330cc060bd2aaa92cdf73391ce7dbc 建立。没有改写或覆盖该分支，也没有合并 main、P5b 或旧源码仓库的 Git 历史。P5a 仅将同一目标仓库中的审计文档和离线模型测试文件作为普通文件纳入新树；来源提交分别为 1a5a467c012e268f0154e32cc09d2c544e03bb84 和 92b1203d6fccb70a874e2811596734f0d1565c0b。

P4c 核查结果：目标仓库当前分支列表没有 p4c/arm-control-safety；GitHub 对短 SHA 2964e87 返回 No commit found。仓库内无法确定其真实来源仓库、完整 SHA 或此前 HEAD 差异。统一归档只按任务提供的结论记录“安全审计完成，整改待办”，不伪造提交引用，也不声称导入了 P4c 代码。需要补齐来源仓库和完整提交 SHA 后，才能继续核对该历史成果。

### STM32 输入包审计与处置

| 输入包 | 结构与风险 | 处置 |
| --- | --- | --- |
| lunar_rover.zip，SHA-256 63ac07e173e148ab248b1689ccae64ba380b730d0c64b2b29d82b96307c48ab8 | 4037 个文件；含 Core/User、CubeMX IOC、CMake/CMakePresets、Keil 项目配置。根目录没有 LICENSE；包内 Git 配置指向 champ719/lunar_rover，HEAD 为 963fd0275ddbe2c759b35250842284c6d62e90c1。上游仓库未声明 GitHub license。包内还含 Git/submodule 元数据、LibXR/Eigen 等第三方源码、数百个构建/缓存文件、Keil 目标产物及一个无关可执行文件。许可文件只覆盖各自组件，不能授权整个机械臂工程；内部团队再分发授权未找到。 | 不上传源码、压缩包、Git 历史、子模块或目标文件。只保留 firmware/arm/README.md 审计说明。 |
| DM-car-six-wheel.zip，SHA-256 823ff6981859dbce1d05a3d4864b921bd0f9aa5165335d0caec7d12852701d2e | 与 P5a 审计报告记录的输入哈希一致。2252 个文件，约 894 MB 解压内容；含 STM32/CubeMX/CMake/Keil 配置。工程根目录没有 LICENSE；只发现 CMSIS、ST HAL、FreeRTOS 等组件许可。压缩包包含数百个 Keil/CMake 编译产物或二进制库，以及 J-Link/CMake 机器日志和乱码命名资料。未发现凭据文件名或私钥/常见访问令牌模式；绝对路径命中集中在日志、临时构建数据及测试/样例内容。团队内部再分发授权未找到。 | 不上传源码、压缩包、供应商二进制、日志或构建目录。只保留 firmware/chassis/README.md 和 P5a 已审计报告/模型测试。 |

机械臂包中的构建/IDE 日志与生成目录包含机器路径候选；Eigen 上游测试脚本也出现绝对路径候选。底盘包中的 JLinkLog.txt、build_log.txt 和 CMakeConfigureLog.yaml 有本机路径。最终归档不包含这些文件。两份包都没有项目根许可证或团队内部发布授权记录；Private 仓库权限不能替代许可或作者授权。

### .gitignore 与大文件检查

已补充 firmware 路径下的 CMake、Keil、调试缓存、日志和目标文件忽略规则。未加入任何固件 ZIP、可执行文件、静态库、HEX、ELF、map 或 object 文件。归档基线已有两个超过 10 MB 的资产（AstraPlus.stl 约 32.8 MB、DLIO 示例 GIF 约 14.4 MB），它们没有在本次修改中新增或更改；仍需在公开发布前另行审查。

### 离线验证

- 本机从 P5a 分支取出的 Python 标准库模型测试：13 项通过，0 项失败；覆盖 UART 帧、六轮分配、模式/超时边界及 ROS 速度路径模型，不访问 ROS、串口、CAN 或硬件。
- P3–P4b 的历史验收记录：23 tests，0 errors、0 failures、0 skipped，P3b 动态 TF 与合成抓放流程通过。本机为 macOS，未安装 ROS 2、colcon 或 arm-none-eabi-gcc，因此本次没有重跑这些 ROS 工作空间测试或编译 STM32 工程。
- P5a 测试不是底盘固件/ROS 集成测试；P3–P4b 的历史通过结果也不是实机验收。没有修改 ROS 工作空间源文件，没有连接、部署或烧录硬件。

本次分支只增加审计通过的文档、P5a 离线模型测试、固件缺口说明和忽略规则；P3–P4b 工作空间源码树与基础提交保持一致。
