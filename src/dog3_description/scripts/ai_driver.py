import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

# Try importing YOLO, handle failure if not installed yet
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("YOLOv8 not found. Install with: pip install ultralytics")

class AIDriver(Node):
    def __init__(self):
        super().__init__('ai_driver')
        
        # Subscriptions
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Vision Tools
        self.cv_bridge = CvBridge()
        if YOLO_AVAILABLE:
            self.model = YOLO('yolov8n.pt') # Load nano model for speed
        
        # Control Logic
        self.obstacle_detected = False
        self.turn_cooldown = 0
        
        self.get_logger().info('AI Driver Started')

    def image_callback(self, msg):
        if not YOLO_AVAILABLE:
            return

        try:
            # Convert ROS Image to CV2
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run Inference
            results = self.model(cv_image, verbose=False)
            
            # Check for close obstacles
            # We look for ANY object with high confidence that is "large" (close)
            obstacle_found = False
            
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    conf = box.conf[0]
                    cls = box.cls[0]
                    
                    # Heuristic: If box width > 30% of screen, it's CLOSE.
                    # Image is 640 wide.
                    x1, y1, x2, y2 = box.xyxy[0]
                    width = x2 - x1
                    
                    if conf > 0.5 and width > 200: # > 200px width
                        obstacle_found = True
                        break
                
                # Render results for debugging (Optional, can view in GUI if desired)
                #annotated_frame = results[0].plot()
                #cv2.imshow("YOLO Vision", annotated_frame)
                #cv2.waitKey(1)

            # Control Logic
            cmd = Twist()
            
            if obstacle_found:
                self.get_logger().info('OBSTACLE DETECTED! Turning...')
                self.turn_cooldown = 20 # Keep turning for a bit
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 # Turn Left
                
            elif self.turn_cooldown > 0:
                self.get_logger().info(f'Turning away... {self.turn_cooldown}')
                self.turn_cooldown -= 1
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 
                
            else:
                self.get_logger().info('Path Clear. Walking Forward.')
                cmd.linear.x = 1.0 # Max speed
                cmd.angular.z = 0.0
                
            self.cmd_vel_pub.publish(cmd)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AIDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
