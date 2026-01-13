import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from kinematics import LegIK
import math
import time

class WalkNode(Node):
    def __init__(self):
        super().__init__('walk_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.ik = LegIK()
        
        # Gait Parameters
        self.freq = 2.5 # Hz (Faster)
        self.base_step_length = 0.05 # meters (Longer steps)
        self.step_height = 0.008 # meters
        self.stand_height = -0.042 # meters
        self.width_scale = 0.1 # Distance from center to leg (adjust based on robot width)

        # Control State
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Timer
        self.timer_period = 0.02 # 50 Hz control loop
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.t = 0.0
        self.get_logger().info('Walk Node Started')

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def get_trajectory(self, phase_offset, step_len):
        """
        Returns (x, z) for a leg based on current time and phase offset.
        phase_offset: 0.0 or 0.5 (for trot)
        step_len: Custom step length for this leg (for turning)
        """
        # Cycle time: 0 to 1
        curr_cycle = (self.t * self.freq + phase_offset) % 1.0
        
        if curr_cycle < 0.5:
            # Swing Phase (0 to 0.5)
            # x: -L/2 -> +L/2
            # z: stand_h -> stand_h + step_h -> stand_h
            
            # Normalized swing time (0 to 1)
            swing_t = curr_cycle / 0.5 
            
            # Simple linear interpolation for x
            x = -step_len/2 + step_len * swing_t
            
            # Sine wave for z (lift)
            # Only lift if we are actually moving (step_len > 0), OR if we want to step in place?
            # Ideally step in place if linear_x=0 but angular!=0.
            # For simplicity, we lift if step_len is non-zero.
            if abs(step_len) > 0.001:
                z = self.stand_height + self.step_height * math.sin(swing_t * math.pi)
            else:
                z = self.stand_height
            
        else:
            # Stance Phase (0.5 to 1.0)
            # x: +L/2 -> -L/2
            # z: stand_h
            
            # Normalized stance time (0 to 1)
            stance_t = (curr_cycle - 0.5) / 0.5
            
            x = step_len/2 - step_len * stance_t
            z = self.stand_height
            
        return x, z

    def control_loop(self):
        self.t += self.timer_period
        
        # Differential Steering:
        # Left side step = linear_x - angular_z * width
        # Right side step = linear_x + angular_z * width
        # (Assuming positive angular Z is Left Turn, so Left side slows down/reverses, Right side speeds up)
        
        # NOTE: base_step_length is the max step. We scale it by linear_x (which is usually 0.5 or 1.0 from cmd_vel)
        # We can treat self.linear_x AS the requested speed in m/s? No, usually cmd_vel is m/s.
        # But our gait engine is primitive. Let's map linear_x (0-1) to base_step_length.
        
        # Better approach:
        # linear_x is multiplier (0.0 to 1.0).
        # step_len_L = base * (linear_x - angular_z)
        # step_len_R = base * (linear_x + angular_z)
        
        # To strictly follow cmd_vel (m/s) requires mapping freq * step_len.
        # Let's keep it simple: linear_x controls stride length directly relative to base.
        
        # Simplified Logic for Demo:
        # if linear_x > 0: walk forward
        # if angular_z > 0: turn left (Right side big step, Left side small/neg step)
        
        # Stride calculation
        left_stride = self.base_step_length * (self.linear_x - self.angular_z)
        right_stride = self.base_step_length * (self.linear_x + self.angular_z)
        
        # Limit strides to avoid IK issues
        max_stride = 0.04
        left_stride = max(-max_stride, min(max_stride, left_stride))
        right_stride = max(-max_stride, min(max_stride, right_stride))

        # Trot Gait:
        # Pair 1: FL, RR (Offset 0.0)
        # Pair 2: FR, RL (Offset 0.5)
        
        # FL (Left) uses left_stride
        x_fl, z_fl = self.get_trajectory(0.0, left_stride)
        
        # RR (Right) uses right_stride
        x_rr, z_rr = self.get_trajectory(0.0, right_stride)
        
        # FR (Right) uses right_stride
        x_fr, z_fr = self.get_trajectory(0.5, right_stride)
        
        # RL (Left) uses left_stride
        x_rl, z_rl = self.get_trajectory(0.5, left_stride)
        
        # Solve IK
        
        # FL (Pair 1 logic, Left)
        theta1_fl, theta2_fl = self.ik.solve(x_fl, z_fl, knee_direction=-1.0)
        
        # RR (Pair 1 logic, Right)
        theta1_rr, theta2_rr = self.ik.solve(x_rr, z_rr, knee_direction=-1.0)
        
        # FR (Pair 2 logic, Right)
        theta1_fr, theta2_fr = self.ik.solve(x_fr, z_fr, knee_direction=-1.0)
        
        # RL (Pair 2 logic, Left)
        theta1_rl, theta2_rl = self.ik.solve(x_rl, z_rl, knee_direction=-1.0)

        # Correct URDF Mapping:
        # Rev 1: FL Hip (Axis -Y)
        # Rev 2: FR Hip (Axis +Y) -> NEEDS INVERSION
        # Rev 3: RL Hip (Axis -Y)
        # Rev 4: RR Hip (Axis +Y) -> NEEDS INVERSION
        
        # Rev 8: FL Knee (Axis -Y)
        # Rev 9: FR Knee (Axis +Y) -> NEEDS INVERSION
        # Rev 7: RL Knee (Axis -Y)
        # Rev 6: RR Knee (Axis +Y) -> NEEDS INVERSION

        hip_offset = 1.57
        
        # Left Legs (Normal)
        fl_hip = theta1_fl + hip_offset
        fl_knee = theta2_fl
        
        rl_hip = theta1_rl + hip_offset
        rl_knee = theta2_rl
        
        # Right Legs (Inverted)
        # Invert the entire angle (theta + offset) because the axis is flipped
        fr_hip = -(theta1_fr + hip_offset)
        fr_knee = -theta2_fr
        
        rr_hip = -(theta1_rr + hip_offset)
        rr_knee = -theta2_rr

        # Reorder to match controller: [Revo 1, 2, 3, 4, 6, 7, 8, 9]
        # [FL, FR, RL, RR, RR_Knee, RL_Knee, FL_Knee, FR_Knee]
        
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
        
        msg = Float64MultiArray()
        msg.data = commands
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WalkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
