import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

NAMES=['J1_joint','J2_joint','J3_joint','J4_joint','J5_joint','J6_joint']
INITIAL=[0.0,1.0,-2.61,0.0,0.0,0.0]
class SimulatedArmFeedback(Node):
    def __init__(self):
        super().__init__('p1_simulated_arm_feedback')
        self.values=dict(zip(NAMES,INITIAL))
        self.pub=self.create_publisher(JointState,'/p1_isolated/arm_joint_state',qos_profile_sensor_data)
        self.sub=self.create_subscription(JointState,'/p1_isolated/arm_joint_command',self.on_command,qos_profile_sensor_data)
        self.timer=self.create_timer(0.02,self.publish_state)
        self.received=0
        self.get_logger().info('SIMULATION ONLY: echoes isolated arm command samples as synthetic joint feedback; no hardware interfaces exist in this ROS domain.')
    def on_command(self,msg):
        for name,value in zip(msg.name,msg.position):
            if name in self.values: self.values[name]=float(value)
        self.received+=1
    def publish_state(self):
        msg=JointState(); msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=NAMES; msg.position=[self.values[n] for n in NAMES]
        self.pub.publish(msg)
def main():
    rclpy.init(); n=SimulatedArmFeedback()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
