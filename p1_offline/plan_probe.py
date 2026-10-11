import math
import rclpy
from rclpy.node import Node
from lite_motion_msgs.srv import PlanArmMotion, CheckStateCollision
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

ARM = [0.0, 1.0, -2.61, 0.0, 0.0, 0.0]
OBSERVE = [0.0, 0.0, -2.0, 0.0, 1.57, 0.0]
PLACE = [3.14, -0.25, -0.91, 0.0, 1.57, 0.0]
# FK of ARM through the checked URDF TCP fixed joint.
TARGET = (-0.015, 0.458184471, 0.255207188)
ROLL = -1.61
Q = (math.sin(ROLL / 2.0), 0.0, 0.0, math.cos(ROLL / 2.0))

class Probe(Node):
    def __init__(self):
        super().__init__('p1_plan_probe')
        self.plan = self.create_client(PlanArmMotion, '/plan_arm_motion')
        self.collision = self.create_client(CheckStateCollision, '/check_state_collision')
    def ready(self, client, name):
        if not client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError(f'{name} service not available')
    def call(self, client, req, label):
        f = client.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=50.0)
        if not f.done(): raise TimeoutError(label)
        return f.result()
    def joint_plan(self, label, start, goal, obstacle=False):
        r = PlanArmMotion.Request()
        r.group_name='arm'; r.ee_frame='tcp_link'; r.use_pose_goal=False
        r.start_joint_positions=start; r.goal_joint_positions=goal; r.plan_only=True
        if obstacle:
            r.obstacle_ids=['p1_deliberate_collision_box']
            box=SolidPrimitive(); box.type=SolidPrimitive.BOX; box.dimensions=[2.0,2.0,2.0]
            r.obstacle_shapes=[box]
            ps=PoseStamped(); ps.header.frame_id='base_link'; ps.pose.orientation.w=1.0
            r.obstacle_poses=[ps]
        out=self.call(self.plan,r,label)
        print(f'{label}: success={out.success}; points={len(out.trajectory.points)}; message={out.message}')
        return out
    def pose_plan(self):
        r=PlanArmMotion.Request(); r.group_name='arm'; r.ee_frame='tcp_link'
        r.use_pose_goal=True; r.start_joint_positions=OBSERVE; r.plan_only=True
        r.target_pose.header.frame_id='base_link'
        r.target_pose.pose.position.x, r.target_pose.pose.position.y, r.target_pose.pose.position.z=TARGET
        (r.target_pose.pose.orientation.x,r.target_pose.pose.orientation.y,
         r.target_pose.pose.orientation.z,r.target_pose.pose.orientation.w)=Q
        out=self.call(self.plan,r,'FK/IK pose plan')
        print(f'FK/IK pose plan: success={out.success}; points={len(out.trajectory.points)}; message={out.message}')
        return out

def main():
    rclpy.init(); n=Probe()
    try:
        n.ready(n.plan,'plan_arm_motion'); n.ready(n.collision,'check_state_collision')
        c=CheckStateCollision.Request(); c.group_name='arm'; c.joint_positions=ARM
        co=n.call(n.collision,c,'baseline collision query')
        print(f'baseline collision: success={co.success}; in_collision={co.in_collision}; category={co.category}; pair={co.object_a}/{co.object_b}; message={co.message}')
        n.joint_plan('joint path initial -> observe', ARM, OBSERVE)
        n.joint_plan('joint path observe -> place', OBSERVE, PLACE)
        n.pose_plan()
        bad=n.joint_plan('deliberate obstacle rejection', ARM, ARM, obstacle=True)
        if bad.success:
            print('WARNING: collision-box request was unexpectedly accepted')
    finally:
        n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
