import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray  # Assuming opponent position is published as an array
import time
class Charge(Node):
    def __init__(self, name):
        super().__init__('robot_charger')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)

        # Subscribe to the sumo wrestling master topic to get instructions.
        self.master_controller_subscription = self.create_subscription(
            Point, 
            '/master_controller',
            self.checkInstruction,
            10
            )

        # Subscriber for opponent position (expects [x, y] coordinates)
        # assuming this will only receive data every 5 seconds (or similar) and not more
        self.enemy_coords_subscription = self.create_subscription(
            Point,
            '/opponent_position',
            self.control_loop,
            10
        )

        self.target_x = None
        self.target_y = None
        self.attack = False
        self.defend = False
        self.lastX = 0

    def checkInstruction(self, msg):
        if msg.x == 1.0:
            self.attack = True
            self.defend = False

        if msg.y == 1.0:
            self.attack = False
            self.defend = True
        # Only msg.x or msg.y should be 1 at a time. The other needs to be 0. 
        

    # function called when the enemy coords are retreived
    # determines whether we should charge at the enemy or not
    # depends on whether the target is visible
    # more charges -> more points! :D
    # only charges once every 5 seconds max
    def control_loop(self, msg = None):
        #  If the robot is not told to attack by the master class, immediately return
        if not self.attack:
            return

        # if orange is not detected, the robot cannot be seen
        # spin in place to detect opponent
        twist = Twist()

        # Katie
        # If no message is passed, return
        if msg == None:
            # print("Target_x is none")
            return
        
        linearSpeed = 0.6
        angularSpeed = 2.0

        # Otherwise we have a point to target
        self.target_x = msg.x
        self.target_y = msg.y
        if self.target_x == 0.0:
            # print("Spin to look")
            # The camera's resolution is 640x480 so if x is less than 320, then it was on the left side. If x > 320 it must have been on the right.
            if (self.lastX < 320):
                twist.angular.z = 1.0  # Slow spin left. It can't be any higher than this or it overshoots. It spins faster than it processes the next frame
            else:
                twist.angular.z = -1.0  # Slow spin right
            twist.linear.x = 0.0 

        elif 200 <= self.target_x <= 450:
            # print("Moving forward")
            twist.angular.z = 0.0 
            twist.linear.x = linearSpeed  # Full speed forward
            self.lastX = self.target_x

        elif self.target_x < 200:
            # print("Turn left")
            twist.angular.z = angularSpeed  # Small left turn
            twist.linear.x = linearSpeed   # Move forward
            self.lastX = self.target_x

        elif self.target_x > 450:
            # print("Turn right")
            twist.angular.z = -angularSpeed  # Small right turn
            twist.linear.x = linearSpeed    # Move forward   
            self.lastX = self.target_x

        # Publish movement command
        self.cmd_vel_pub.publish(twist)


def main():
    charge = Charge('charge_node')
    rclpy.spin(charge)
    charge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    main()
