# MID-360 静态点云与 IMU 验收

本流程只启动 `livox_ros_driver2_node`，不启动 RViz、Nav2、底盘、机械臂、SLAM 或 Point-LIO。P8 不会在无人确认的情况下启动传感器驱动，也不会更改网络、路由或防火墙。

## P8 独立构建

在 NUC 的 P8 部署目录执行。此步骤只编译，不打开传感器或设备端口：

```bash
source /opt/ros/humble/setup.bash
cd /home/tzb/lunar-cave-p8
colcon --log-base log/mid360 build \
  --base-paths Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2 \
  --packages-select livox_ros_driver2 \
  --build-base build/mid360 \
  --install-base install/mid360
```

确认可执行文件、静态 launch 参数和动态库，不启动 ROS 驱动节点：

```bash
source /opt/ros/humble/setup.bash
source /home/tzb/lunar-cave-p8/install/mid360/setup.bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg executables livox_ros_driver2
ros2 launch --show-args livox_ros_driver2 mid360_static.launch.py
ldd /home/tzb/lunar-cave-p8/install/mid360/livox_ros_driver2/lib/liblivox_ros_driver2.so
```

launch 的 `config_file` 是必填项。它没有默认设备配置，避免误用仓库或历史工作空间中的地址。Point-LIO 不是原始 MID-360 点云和 IMU 验收的前置依赖。

## 现场前置检查

1. 由现场负责人从 MID-360 铭牌或批准的配置确认当前传感器 IP；确认 NUC 实际有线地址和连接的接口。两份历史 JSON 只作为待核对记录保留，不能据此推定当前网络参数。
2. 在接入传感器前运行只读检查：

   ```bash
   ip -br address
   ip route
   ip route get <现场确认的雷达地址>
   ```

   路由及源地址必须与已选配置和物理接线一致。若路由走 Wi-Fi、VPN 或地址不符，停止测试；本流程不修改接口地址、路由或防火墙。
3. 由有权限的现场负责人核对 UDP 端口方向和当前防火墙策略。此前 UFW 查询结果为 inactive；其它防火墙后端和规则未审计，因此不能据此判断现场 UDP 通路已准备好。
4. 核对设备供电、网线、雷达安装固定，并确认只连接 MID-360。不要连接或启动任何运动控制器。
5. 确认选用的 JSON 与现场地址、网络接口以及已标定外参相符；不覆盖原始配置，不将未验证的地址或外参当成有效参数。

## 单独启动驱动

仅当前置检查完成且现场负责人明确同意后执行。选用正确的本地配置路径作为必填参数：

```bash
source /opt/ros/humble/setup.bash
source /home/tzb/lunar-cave-p8/install/mid360/setup.bash
ros2 launch livox_ros_driver2 mid360_static.launch.py \
  config_file:=<已核实且批准的MID-360配置.json>
```

该 launch 仅启动 MID-360 驱动节点。停止时对该终端按 Ctrl-C。不要在同一测试中启动整机 bringup、导航、底盘或机械臂。

## 观测与通过标准

另开终端，使用同一个 ROS 2 环境：

```bash
source /opt/ros/humble/setup.bash
source /home/tzb/lunar-cave-p8/install/mid360/setup.bash
ros2 topic list -t
ros2 topic echo --once --qos-reliability best_effort /livox/imu
ros2 topic hz --qos-reliability best_effort /livox/lidar
```

先记录点云频率，再 Ctrl-C 停止 `hz`，单独检查 IMU 频率：

```bash
ros2 topic hz --qos-reliability best_effort /livox/imu
```

稳定观察后可另开终端录制短包：

```bash
ros2 bag record -o "$HOME/mid360_static_$(date +%Y%m%d_%H%M%S)" \
  /livox/lidar /livox/imu
```

通过要求：话题类型分别为 `sensor_msgs/msg/PointCloud2` 和 `sensor_msgs/msg/Imu`；两者均有持续非零频率、时间戳递增；点云空间形状及 IMU 数值合理；录制包可列出并回放。保存 ROS 日志、命令、所选配置的脱敏标识、时间戳与频率摘要。不要把设备地址或私有 JSON 上传到仓库。

此检查只证明静态数据流。雷达点云驱动使用标准 `PointCloud2` 时，不代表 Point-LIO 所需的 Livox `CustomMsg` 数据路径已经验证。IMU 坐标、时间同步、雷达安装外参及底盘/地图 TF 仍需单独标定和验收。
