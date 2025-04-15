import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        self.prevDir = 0 # Each number represents the absolute direction baymax was just moving in. 
        # 0 = forward, 1 = right, 2 = backward, 3 = left. (clockwise and just like the wall_distances topic is published in)
        self.maxSpeed = 0.3
        self.possibleOpenings = []

        # # Subscriber for camera image frames
        # self.cam_sub = self.create_subscription(
        #     Image,
        #     '/camera/image_raw',  # Topic where camera data is published
        #     self.image_callback,
        #     10
        # )

        # Publisher for movement commands (Twist messages)
        self.cmd_pub = self.create_publisher(
            Twist,
            '/controller/cmd_vel',  # Topic where we send velocity commands
            10
        )

        self.wallDist_sub = self.create_subscription(
            Float32MultiArray,
            '/wall_distances', 
            self.getPossibleOpenings, 
            10)

        # Bridge to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Flag to stop movement when goal is detected
        self.found_goal = False
        
        # Timer to run control loop every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.controller)

    def lidar_callback(self, msg):
        """Callback to receive and store LiDAR scan data."""
        self.lidar_data = msg.ranges
    
    def getPossibleOpenings(self, wallDist):
        # as long as no new openings, continue going self.prevDir.
        # If crossroad is detected, use switch statement
        # There are 4 possible directions at any time: forward, right, back, left.
        # Whatever direction is opposite his prevDir should never be considered for a possible opening because he just came from there. 
        # For instance if prevDir is forward then obviously he will have an opening behind but he shouldnt follow it. 
        possibleOpenings = [-1, -1, -1, -1]
        
        for i, dist in enumerate(wallDist.data):
            if dist < 0.25 or ((i + 2) % 4) == self.prevDir:
                possibleOpenings[i] = -1
            else:
                possibleOpenings[i] = dist
            print(possibleOpenings)
        self.possibleOpenings = possibleOpenings

    def controller(self):
        crossroadFound = False
        # Check if there is a crossroad. 
        for dist in self.possibleOpenings:
            print(dist)
            # if (i + 2 % 4) == self.prevDir: # Skip the direction we just came from
            #     # print(f"Skip counting {i}")
            #     continue
            if dist > 0:
                crossroadFound = True

        if crossroadFound:
            dirToMove = self.possibleOpenings.index(max(self.possibleOpenings))
        else:
            dirToMove = self.prevDir

        if dirToMove == 0:
            self.moveFoward()
        elif dirToMove == 1:
            self.moveRight()
        elif dirToMove == 2:
            self.moveBack()
        elif dirToMove == 3:
            self.moveLeft()
        else:
            print("Something has gone horribly wrong. I'm stuck in a box.")

    def moveFoward(self):
        print("forward\n")
        twist = Twist()
        twist.linear.x = self.maxSpeed
        self.cmd_pub.publish(twist)
        self.prevDir = 0

    def moveRight(self):
        print("right\n")
        twist = Twist()
        twist.linear.y = -self.maxSpeed
        self.cmd_pub.publish(twist)
        self.prevDir = 1

    def moveBack(self):
        print("back\n")
        twist = Twist()
        twist.linear.x = -self.maxSpeed
        self.cmd_pub.publish(twist)
        self.prevDir = 2

    def moveLeft(self):
        print("left\n")
        twist = Twist()
        twist.linear.y = self.maxSpeed
        self.cmd_pub.publish(twist)
        self.prevDir = 3

    def stop_robot(self):
        """Stop the robot by sending zero velocity."""
        twist = Twist()  # Zero velocities
        self.cmd_pub.publish(twist)

def main(args=None):
    """Entry point for the node."""
    rclpy.init(args=args) 
    node = Navigator()  
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()  

if __name__ == '__main__':
    main()
