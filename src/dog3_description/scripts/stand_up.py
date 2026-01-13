import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from kinematics import LegIK
import time

class StandUpNode(Node):
    def __init__(self):
        super().__init__('stand_up_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.ik = LegIK()
        
        # Target height (meters relative to hip)
        self.target_z = -0.04 # Start low
        self.target_x = 0.0
        
        self.timer = self.create_timer(1.0, self.publish_command)
        self.get_logger().info('Stand Up Node Started')

    def publish_command(self):
        # Calculate angles
        # knee_direction=1.0 gives "positive" knee angle
        theta1, theta2 = self.ik.solve(self.target_x, self.target_z, knee_direction=1.0)
        self.get_logger().info(f'Target: x={self.target_x}, z={self.target_z} -> Theta: {theta1:.3f}, {theta2:.3f}')
        
        msg = Float64MultiArray()

        # Mapping:
        # Front legs usually have knees bending backward (like elbow) -> knee_direction ?
        # Rear legs usually have knees bending forward (like knee) -> knee_direction ?
        # For simplicity, let's try same direction first.
        
        fl_hip = theta1
        fr_hip = theta1 
        rl_hip = theta1
        rr_hip = theta1
        
        fl_knee = theta2
        fr_knee = theta2 
        rl_knee = theta2
        rr_knee = theta2
        
        # Order must match controllers.yaml: 
        # Revolute 1, 2, 3, 4, 6, 7, 8, 9
        # 1: FL Hip, 2: FR Hip, 3: RL Hip, 4: RR Hip
        # 6: RR Knee, 7: RL Knee, 8: FL Knee, 9: FR Knee
        
        # Zero Pose Test
        # fl_hip = 0.0
        # fr_hip = 0.0
        # rl_hip = 0.0
        # rr_hip = 0.0
        
        # fl_knee = 0.0
        # fr_knee = 0.0
        # rl_knee = 0.0
        # rr_knee = 0.0

        # Reorder to match: 1, 2, 3, 4, 6, 7, 8, 9
        # [FL_Hip, FR_Hip, RL_Hip, RR_Hip, RR_Knee, RL_Knee, FL_Knee, FR_Knee]
        commands = [
            fl_hip, # 1
            fr_hip, # 2
            rl_hip, # 3
            rr_hip, # 4
            rr_knee, # 6
            rl_knee, # 7
            fl_knee, # 8
            fr_knee  # 9
        ]
        
        msg.data = commands
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StandUpNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
