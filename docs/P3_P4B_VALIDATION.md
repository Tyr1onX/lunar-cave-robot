# P3–P4b 离线归档与验证记录

验证日期：2026-10-11。测试环境：ThinkBook / Ubuntu 22.04 / ROS 2 Humble。归档以私有仓库 `main`（`b1173df`）为基线，采用**新提交移植补丁**，不合并原工作空间的 Git 历史。

## 阶段来源

| 阶段 | ThinkBook 原提交 | 解决的问题与归档内容 | 离线验证 |
| --- | --- | --- | --- |
| P3 | `f91bc6a79a827ea3f069ef4e92aa77448c9c7928` | 对视觉 `PoseStamped` 按观测时间查询 TF，处理 optical frame 转换，拒绝时间戳无效、过期、未来和 TF 缺失数据；新增 `test_pose_transform` | 4 个 C++ 用例通过 |
| P3b | `e40243444e561c3c167c4424be5c5892efdb6c5c` | 使用默认 `tzb_description`、`robot_state_publisher` 和合成反馈检查机械臂 URDF→TF→视觉→规划流程；新增隔离 launch / 脚本 | 动态 TF 测试 `P3B_TF_TEST=PASS`；合成抓放与返回完成 |
| P4a | `67359bd8856fbafec655f31f77415b6e6bacdf48` | 对照已分析的 STM32 帧布局实现机械臂命令/反馈解析、分片组帧、坏帧恢复、顺序和异常值校验；新增协议测试 | 12 个 C++ 用例通过 |
| P4b | `9d48d5ddc849a358acd058537c142fa4fdb69183` | 校验六关节名称顺序、弧度/角度和零位映射，与默认 URDF 关节轴、上下限和正向运动学比对；新增关节测试 | 3 个 C++ 用例及 1 个 URDF Python 检查通过 |

四次移植均基于前一阶段依次应用。原提交 SHA 作为来源元数据留在新提交信息中；机械臂 `tzb_ws/` 和导航 `Nav2_rea_ws/` 仍相互独立。此轮未修改导航代码。

## 从克隆的仓库离线复现

机器需安装 ROS 2 Humble、colcon 和项目原有 ROS / 运动规划开发依赖。进入**仓库根目录**，独立构建机械臂工作空间，不要叠加导航工作空间环境：

```bash
source /opt/ros/humble/setup.bash
cd tzb_ws
colcon --log-base log/archive build --base-paths src \
  --packages-up-to lite_motion_planner rm_serial_driver \
  --build-base build/archive --install-base install/archive \
  --parallel-workers 4 --cmake-args -DBUILD_TESTING=ON
colcon --log-base log/archive build --base-paths src \
  --packages-select tzb_description \
  --build-base build/archive --install-base install/archive --parallel-workers 2
colcon --log-base log/archive test --base-paths src \
  --packages-select lite_motion_planner rm_serial_driver \
  --build-base build/archive --install-base install/archive --parallel-workers 2
colcon --log-base log/archive test-result --test-result-base build/archive --verbose
```

P3b 动态 TF 测试需要两个终端，均使用：

```bash
source /opt/ros/humble/setup.bash
source tzb_ws/install/archive/setup.bash  # 在仓库根目录执行
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1
```

终端一：

```bash
ros2 launch lite_motion_planner p3b_offline.launch.py \
  offline_domain_id:=187 autostart:=false publish_synthetic_target:=false use_rviz:=false
```

终端二：

```bash
ros2 run lite_motion_planner p3b_urdf_tf_test.py
```

动态 TF 检查完成后停止终端一。合成全流程单独启动：

```bash
ros2 launch lite_motion_planner p3b_offline.launch.py \
  offline_domain_id:=187 autostart:=true publish_synthetic_target:=true use_rviz:=false
```

确认日志中出现 `tzb_catch sequence completed` 后停止测试。以上脚本限定本机 ROS domain 和合成反馈，不启动相机、底盘或机械臂串口驱动。**不要用默认的真实设备 bringup 替代上述命令。**

## 本次实际复验结果

将集成分支的受控快照拷贝到 ThinkBook 的 `/tmp/lunar-archive-validation-20261011/`，无原仓库 `.git` 或历史构建产物，进行新构建与测试：

- `arm2nav_msgs`、`rm_serial_msgs`、`lite_motion_msgs`、`rm_serial_driver`、`lite_motion_planner`：**5/5 构建成功**；`tzb_description` 单独构建成功。
- `colcon test-result`：**23 tests, 0 errors, 0 failures, 0 skipped**。其中 P3 视觉 TF 4 个、P4a 协议 12 个、P4b 关节映射 3 个和 URDF FK 1 个验证通过。
- P3b 动态：`P3B_TF_TEST=PASS`；J1–J3 改变相机位姿；J4–J6 不改变相机平移；找不到历史时间戳时拒绝转换。
- P3b 合成全流程：模拟 optical frame 目标转换至 `base_link`，完成抓取、附着、放置、分离和返回观察位姿，日志记录 `tzb_catch sequence completed`。
- 构建出现 Boost/eigenpy python310 的非致命 CMake 警告；ROS KDL 对根链接惯量给出警告，未阻塞上述测试。

## 代码/资料边界与未验证事项

- 本次移植的 20 个变更文件均为文本源码、ROS 配置或测试，无新增二进制/SDK/模型/压缩包，单个文件均低于 5 MB；新增内容未发现密钥特征或个人主目录绝对路径。原 `main` 已有两项超过 10 MB 的第三方资源：Orbbec `AstraPlus.stl`（32.8 MB）和 DLIO 示例 GIF（14.4 MB），本次未更动，日后公开发布仍需检查权利和仓库大小。旧版全仓路径与第三方分发风险仍按 [RELEASE_AUDIT](RELEASE_AUDIT.md) 管理。
- `package.xml`、CMake 测试入口、脚本执行权限、Python ROS 运行时依赖和安装规则已纳入本阶段检查。
- **STM32 下位机完整工程/原始压缩包未归档。** 本仓仅收录上位机协议实现与离线对照测试。STM32 原源码的发布授权、第三方代码许可和编译产物仍待独立审计；不可因此推断真实串口互通验收通过。
- 未连接真实机械臂、电机、夹爪、STM32、相机、雷达或底盘；未经测量的相机 optical TF 为**零平移测试夹具**，不代表手眼标定。
- 未验证真实串口收发、上下位机联调、通信丢包/断线、六关节实际方向与零位、限位和运动安全、实机视觉定位、完整抓取或导航—机械臂闭环。
- 未重跑完整导航及硬件 SDK / 模型依赖构建，未提供获批真实传感器 bag、PCD 地图、模型权重、SDK 二进制。全新电脑上的系统依赖安装/编译仍需接手成员验证。
