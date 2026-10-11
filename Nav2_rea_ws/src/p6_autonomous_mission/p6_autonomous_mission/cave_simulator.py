"""Deterministic 2-D differential-drive base and lidar model for P6 offline runs."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class CaveSimulator(Node):
    def __init__(self):
        super().__init__('p6_cave_simulator')
        self.declare_parameter('command_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_beams', 180)
        self.declare_parameter('scan_rate_hz', 8.0)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('command_timeout_sec', 0.25)
        self.declare_parameter('start_x', 1.0)
        self.declare_parameter('start_y', 1.0)
        self.declare_parameter('start_yaw', 0.0)

        self.command_topic = self.get_parameter('command_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.beams = int(self.get_parameter('scan_beams').value)
        self.scan_rate = float(self.get_parameter('scan_rate_hz').value)
        self.control_rate = float(self.get_parameter('control_rate_hz').value)
        self.max_v = float(self.get_parameter('max_linear_speed').value)
        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.command_timeout = float(self.get_parameter('command_timeout_sec').value)

        self.resolution = 0.05
        self.width = 240
        self.height = 160
        self.occupancy = [0] * (self.width * self.height)
        self._make_world()
        self.x = float(self.get_parameter('start_x').value)
        self.y = float(self.get_parameter('start_y').value)
        self.yaw = float(self.get_parameter('start_yaw').value)
        self.v = 0.0
        self.w = 0.0
        self.last_command_time = 0.0
        self.last_update = time.monotonic()
        self.last_scan = 0.0

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, map_qos)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Twist, self.command_topic, self._on_command, 10)
        self.create_timer(1.0 / self.control_rate, self._tick)
        self._publish_map()
        self.get_logger().warning(
            'OFFLINE SIMULATION ONLY: /cmd_vel drives a mathematical differential-drive model; '
            'the map and /scan are generated in process. No sensor or device driver is started.')

    def _make_world(self):
        # OccupancyGrid rows grow in +y. Leave a one-cell solid boundary.
        for x in range(self.width):
            self._set_cell(x, 0, 100)
            self._set_cell(x, self.height - 1, 100)
        for y in range(self.height):
            self._set_cell(0, y, 100)
            self._set_cell(self.width - 1, y, 100)
        # A broken cross-cave wall forces a real planner to find the upper gap.
        self._fill_rect(4.8, 0.15, 5.05, 5.25)
        # Two isolated blocks make the route and return route visibly nontrivial.
        self._fill_rect(7.35, 2.0, 8.15, 3.8)
        self._fill_rect(8.8, 5.1, 9.35, 5.8)

    def _set_cell(self, x, y, value):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.occupancy[y * self.width + x] = value

    def _fill_rect(self, x0, y0, x1, y1):
        ix0 = max(0, int(x0 / self.resolution))
        iy0 = max(0, int(y0 / self.resolution))
        ix1 = min(self.width, int(math.ceil(x1 / self.resolution)))
        iy1 = min(self.height, int(math.ceil(y1 / self.resolution)))
        for iy in range(iy0, iy1):
            for ix in range(ix0, ix1):
                self._set_cell(ix, iy, 100)

    def _publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.occupancy
        self.map_pub.publish(msg)

    @staticmethod
    def _quaternion(yaw):
        return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

    def _on_command(self, msg):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        if not math.isfinite(vx) or not math.isfinite(wz):
            self.v = self.w = 0.0
            self.last_command_time = 0.0
            self.get_logger().error('Rejected non-finite base command and stopped the simulator.')
            return
        self.v = max(-self.max_v, min(self.max_v, vx))
        self.w = max(-self.max_w, min(self.max_w, wz))
        self.last_command_time = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        dt = min(0.1, max(0.0, now - self.last_update))
        self.last_update = now
        if now - self.last_command_time > self.command_timeout:
            self.v = self.w = 0.0
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.w * dt), math.cos(self.yaw + self.w * dt))
        self._publish_odom()
        if now - self.last_scan >= 1.0 / self.scan_rate:
            self.last_scan = now
            self._publish_scan()

    def _publish_odom(self):
        stamp = self.get_clock().now().to_msg()
        q = self._quaternion(self.yaw)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.x, transform.transform.rotation.y = q[0], q[1]
        transform.transform.rotation.z, transform.transform.rotation.w = q[2], q[3]
        self.tf_pub.sendTransform(transform)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.w
        self.odom_pub.publish(msg)

    def _occupied(self, x, y):
        ix = int(x / self.resolution)
        iy = int(y / self.resolution)
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return True
        return self.occupancy[iy * self.width + ix] >= 50

    def _publish_scan(self):
        stamp = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.base_frame
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / self.beams
        scan.angle_max = scan.angle_min + (self.beams - 1) * scan.angle_increment
        scan.range_min = 0.12
        scan.range_max = 8.0
        scan.scan_time = 1.0 / self.scan_rate
        scan.time_increment = 0.0
        ranges = []
        step = self.resolution * 0.5
        for i in range(self.beams):
            angle = self.yaw + scan.angle_min + i * scan.angle_increment
            distance = step
            while distance < scan.range_max:
                if self._occupied(self.x + math.cos(angle) * distance,
                                  self.y + math.sin(angle) * distance):
                    break
                distance += step
            ranges.append(min(distance, scan.range_max))
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = CaveSimulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
