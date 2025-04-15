import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class MazeNavigator(Node):
    def __init__(self):
        super().__init__('maze_navigator')

        self.state = "IDLE"
        self.state_timer = 0  # counts how long we've been in a state

        # Subscriber for LiDAR scan data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan_raw',  # Topic where LiDAR data is published
            self.lidar_callback,
            10
        )

        # Subscriber for camera image frames
        self.cam_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Topic where camera data is published
            self.image_callback,
            10
        )

        # Publisher for movement commands (Twist messages)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',  # Topic where we send velocity commands
            10
        )

        # Bridge to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Store latest LiDAR scan data
        self.lidar_data = None

        # Flag to stop movement when goal is detected
        self.found_goal = False

        # Timer to run control loop every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def lidar_callback(self, msg):
        """Callback to receive and store LiDAR scan data."""
        self.lidar_data = msg.ranges

    def image_callback(self, msg):
        """Callback to receive and process camera frames."""
        # Convert ROS image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert BGR to HSV for better color filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the HSV range for detecting the color blue
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        # Create a mask that isolates blue areas in the frame
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Count the number of non-zero (blue) pixels
        blue_area = cv2.countNonZero(mask)

        # If there's a significant blue area, we assume it's the goal
        if blue_area > 1000:  # Threshold may need tuning
            self.get_logger().info("Blue wall detected! Goal reached.")
            self.found_goal = True

    def control_loop(self):
        if self.found_goal or self.lidar_data is None:
            self.stop_robot()
            return

        twist = Twist()

        # Handle state transitions
        if self.state == "TURNING_RIGHT":
            twist.angular.z = -0.5
            self.state_timer += 1
            if self.state_timer >= 10:  # ~1 second turn
                self.state = "FORWARD_AFTER_TURN"
                self.state_timer = 0

        elif self.state == "FORWARD_AFTER_TURN":
            twist.linear.x = 0.2
            self.state_timer += 1
            if self.state_timer >= 10:  # move forward ~1 second
                self.state = "IDLE"
                self.state_timer = 0

        elif self.state == "IDLE":
            # Normal decision making
            n = 50
            section = int(len(self.lidar_data)//4)
            front = min(min(self.lidar_data[0:10] + self.lidar_data[-10:]), 10.0)
            right = min(self.lidar_data[3*section - n: 3*section + n])
            left = min(self.lidar_data[1*section - n:1*section + n])

            wall_threshold = 0.15

            self.get_logger().info(f"Front: {front:.2f}, Right: {right:.2f}, Left: {left:.2f}")

            if right > wall_threshold:
                self.get_logger().info("Turning RIGHT")
                self.state = "TURNING_RIGHT"
                self.state_timer = 0
                return  # don't publish here, wait for next loop
            elif front > wall_threshold:
                self.get_logger().info("Moving FORWARD")
                twist.linear.x = 0.2
            elif left > wall_threshold:
                self.get_logger().info("Turning LEFT")
                twist.angular.z = 0.5
            else:
                self.get_logger().info("Dead-end! Rotating to find path.")
                twist.angular.z = -0.6

        self.cmd_pub.publish(twist)

    def stop_robot(self):
        """Stop the robot by sending zero velocity."""
        twist = Twist()  # Zero velocities
        self.cmd_pub.publish(twist)


def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args) 
    navigator = MazeNavigator()  
    rclpy.spin(navigator)  
    navigator.destroy_node()  
    rclpy.shutdown()  
  

if __name__ == '__main__':
    main()
