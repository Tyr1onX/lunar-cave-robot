import math,rclpy
from rclpy.node import Node
from lite_motion_msgs.srv import PlanArmMotion
class P(Node):
 def __init__(self):super().__init__('p1_candidate_pose_probe');self.c=self.create_client(PlanArmMotion,'/plan_arm_motion')
def main():
 rclpy.init();n=P()
 try:
  if not n.c.wait_for_service(timeout_sec=8):raise RuntimeError('planner service unavailable')
  req=PlanArmMotion.Request();req.group_name='arm';req.ee_frame='tcp_link';req.use_pose_goal=True;req.plan_only=True
  req.start_joint_positions=[0.0,0.0,-2.0,0.0,1.57,0.0];req.target_pose.header.frame_id='base_link'
  req.target_pose.pose.position.x=-0.26168583161607833;req.target_pose.pose.position.y=-0.17536907187995862;req.target_pose.pose.position.z=0.35735090146410997
  roll,pitch,yaw=-2.73,0.0,2.5;cr,sr=math.cos(roll/2),math.sin(roll/2);cp,sp=math.cos(pitch/2),math.sin(pitch/2);cy,sy=math.cos(yaw/2),math.sin(yaw/2)
  q=req.target_pose.pose.orientation;q.x=sr*cp*cy-cr*sp*sy;q.y=cr*sp*cy+sr*cp*sy;q.z=cr*cp*sy-sr*sp*cy;q.w=cr*cp*cy+sr*sp*sy
  f=n.c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=50)
  if not f.done():raise TimeoutError('pose request')
  out=f.result();print(f'alternate target IK/path from observe: success={out.success}; points={len(out.trajectory.points)}; message={out.message}')
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
