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

        # Extract sectors
        n = 50
        section = int(len(self.lidar_data)//4) # divide into sections
        front = min(min(self.lidar_data[0:10] + self.lidar_data[-10:]), 10.0)
        # right = min(self.lidar_data[260:280])  # ~270° ±10
        right = min(self.lidar_data[3*section - n: 3*section + n])
        # left = min(self.lidar_data[80:100])    # ~90° ±10
        left = min(self.lidar_data[1*section - n:1*section + n])

        # Threshold for wall detection
        wall_threshold = 0.15

        twist = Twist()

        # Debug printout
        self.get_logger().info(f"Front: {front:.2f}, Right: {right:.2f}, Left: {left:.2f}")


        twist.angular.z = 0.0
        twist.linear.x = 0.0
        # Right-hand rule: priority is RIGHT > FORWARD > LEFT > TURN AROUND
        if right > wall_threshold:
            # Path clear to the right
            self.get_logger().info("Turning RIGHT")
            twist.angular.z = -0.5
            # twist.linear.x = 0.1
        elif front > wall_threshold:
            # Go straight if front is clear
            self.get_logger().info("Moving FORWARD")
            twist.linear.x = 0.2
        elif left > wall_threshold:
            # Path only clear to the left
            self.get_logger().info("Turning LEFT")
            twist.angular.z = 0.5
            # twist.linear.x = 0.1
        else:
            # Dead-end, rotate in place
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
