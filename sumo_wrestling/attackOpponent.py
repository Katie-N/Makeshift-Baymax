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

    # function called when the enemy coords are retreived
    # determines whether we should charge at the enemy or not
    # depends on whether the target is visible
    # more charges -> more points! :D
    # only charges once every 5 seconds max
    def control_loop(self, msg = None):
        # if orange is not detected, the robot cannot be seen
        # spin in place to detect opponent
        twist = Twist()

        # Katie
        # If no message is passed, return
        if msg == None:
            # print("Target_x is none")
            return

        # Otherwise we have a point to target
        self.target_x = msg.x
        self.target_y = msg.y
        if self.target_x == 0.0:
            print("Spin to look")
            twist.angular.z = 1.0  # Slow spin
        elif 200 <= self.target_x <= 450:
            print("Moving forward")
            twist.linear.x = 0.6  # Full speed forward
        elif self.target_x > 450:
            print("Turn left")
            twist.angular.z = 2.0  # Small left turn
            twist.linear.x = 0.6   # Move forward
        elif self.target_x < 200:
            print("Turn right")
            twist.angular.z = -2.0  # Small right turn
            twist.linear.x = 0.6    # Move forward    
        # Publish movement command
        self.cmd_vel_pub.publish(twist)
        # time.sleep(0.1)


def main():
    charge = Charge('charge_node')
    rclpy.spin(charge)
    charge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    main()
