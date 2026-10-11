# P4e ROS 2 / PTY 离线验收

此目录中的 test_arm_host_safety_pty.py 启动实际编译的 rm_serial_driver ROS 2 节点，只通过测试创建的 PTY 设备与 35 字节反馈模拟器通信。脚本要求设置隔离的 ROS_DOMAIN_ID 和 ROS_LOCALHOST_ONLY=1。它不访问真实串口，不模拟机械臂物理行为。

## ThinkBook Humble 运行方法

先在 ThinkBook 上建立独立的临时测试目录。只复制本仓库的 tzb_ws/src/serial/driver 到 /tmp/p4e_ros_safety/src/rm_serial_driver，并复制本测试脚本到 /tmp/p4e_ros_safety。编译和测试均限定在该 /tmp 目录，不要把测试文件放进已有工作空间。

~~~bash
source /opt/ros/humble/setup.bash
source /tmp/robot_p4b_build/install/setup.bash
cd /tmp/p4e_ros_safety
colcon build --packages-select rm_serial_driver --cmake-args -DBUILD_TESTING=ON
source /tmp/p4e_ros_safety/install/setup.bash
export ROS_DOMAIN_ID=190
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
/usr/bin/python3 /tmp/p4e_ros_safety/test_arm_host_safety_pty.py
~~~

运行既有包回归时，测试需要 URDF 输入位于 /tmp/p4e_ros_safety/tzb_description/urdf/tzb_description.urdf。将仓库中的此单个 URDF 复制到临时目录后运行：

~~~bash
cd /tmp/p4e_ros_safety
colcon test --packages-select rm_serial_driver --event-handlers console_direct+
colcon test-result --verbose
~~~

PTY 注入脚本同时测量 5 秒合成 ROS 2 100 Hz 发布定时器和驱动 PTY 发帧周期。测试记录在报告中的样本和抖动仅代表隔离测试环境，不是完整规划负载或真实机械臂验收。脚本以进程组清理 ros2 run 包装程序和实际节点。

## 覆盖场景

- 上电 disabled 与 46 字节帧布局。
- 显式 enable + 新 JointState → 关节映射/夹爪 → 46 字节帧。
- 上游命令流停发、租约到期及目标/夹爪清除。
- fault reset / enable 对活动上游流量的拒绝。
- 旧/未来时间戳、NaN、Inf、限位外和关节数不完整目标。
- 有效反馈停更、无反馈时拒绝 reset、反馈恢复后仍需人工恢复。
- 活动状态下 STM32 反馈序号回退锁存。
- PTY 断开、同路径换接新 PTY、重连不自动运动。
- ROS 进程组突然终止及重启无缓存运动/夹爪重放；人工 enable 后仍需新鲜命令。

模拟接收端仅验证软件字节流、ROS 回调/服务和状态迁移。协议使能位清零不是电机急停；任何机械停止距离、保持力、防坠、失电抱闸、CAN 单轴健康或物理急停能力均不在该测试范围。

