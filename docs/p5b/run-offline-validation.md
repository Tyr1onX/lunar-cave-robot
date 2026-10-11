# P5b 离线运行说明

以下步骤只在 ThinkBook ROS 2 Humble 软件环境使用 PTY，不填入真实 tty 路径。

## 1. 构建桥接包

从统一仓库的 `p5b/chassis-ros2-integration` 检出后：

```bash
cd ~/robot_p0/Nav2_rea_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select chassis_serial_bridge
source install/setup.bash
```

## 2. 配置与 ROS 2 PTY 集成测试

脚本要求桥接包已经安装，并使用真实 `nav2_velocity_smoother` 节点。它会创建 PTY 对，安全话题前缀和独立 ROS Domain，测试设备参数不会指向实体串口。

```bash
source /opt/ros/humble/setup.bash
source ~/robot_p0/Nav2_rea_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
python3 tools/p5b_validate_configuration.py
python3 tools/p5b_pty_ros2_integration.py
```

预期最后一行：

```text
RESULT=PASS_ROS2_PTY_INTEGRATION
```

## 3. Nav2 launch 参数约定

三个启用导航的 launch 都要求显式 `chassis_device` 参数。离线时只能提供 PTY，例如：

```bash
ros2 launch bringup mapping_navigation_single_lidar.launch.py chassis_device:=/dev/pts/N
```

上面仅说明参数名和类型，不要把示例 `/dev/pts/N` 当作实车设备路径。完整 Point-LIO/Livox/Nav2 bringup 会启动传感器和导航栈，PTY 参数并不代表已启动硬件传感器；本阶段集成测试只拉起 smoother、桥接节点和模拟接收端。

实车设备路径须由现场核对后单独填写；桥接的默认配置路径为空，Nav2 bringup 不提供串口默认值。
