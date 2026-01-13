import math

class LegIK:
    def __init__(self, l1=0.026, l2=0.026):
        """
        l1: Length of upper leg (Hip to Knee)
        l2: Length of lower leg (Knee to Foot)
        """
        self.l1 = l1
        self.l2 = l2

    def solve(self, x, z, knee_direction=1.0):
        """
        Calculates joint angles.
        knee_direction: 1.0 or -1.0 to flip knee bend direction.
        """
        # Distance to target
        r = math.sqrt(x**2 + z**2)
        
        # Check reachability
        if r > (self.l1 + self.l2):
            # print(f"Target ({x}, {z}) is out of reach (max {self.l1 + self.l2})")
            r = self.l1 + self.l2
            
        try:
            cos_q2 = (x**2 + z**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
            cos_q2 = max(-1.0, min(1.0, cos_q2))
            
            q2_mag = math.acos(cos_q2)
            
            # Use knee_direction to determine sign
            theta2 = knee_direction * q2_mag 
            
            # Beta: Angle of vector to end effector
            beta = math.atan2(z, x)
            
            # Alpha: Angle between vector to end effector and upper leg
            cos_alpha = (self.l1**2 + x**2 + z**2 - self.l2**2) / (2 * self.l1 * math.sqrt(x**2 + z**2))
            cos_alpha = max(-1.0, min(1.0, cos_alpha))
            alpha = math.acos(cos_alpha)
            
            # Hip angle
            # If knee bends one way, hip angle changes
            if knee_direction > 0:
                 theta1 = beta - alpha
            else:
                 theta1 = beta + alpha

            return theta1, theta2
            
        except ValueError as e:
            print(f"IK Error: {e}")
            return 0.0, 0.0
            
        except ValueError as e:
            print(f"IK Error: {e}")
            return 0.0, 0.0

if __name__ == "__main__":
    ik = LegIK()
    print(ik.solve(0.0, -0.04))
