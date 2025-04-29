import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray  # Assuming opponent position is published as an array
import time

class TryToScore(Node):
    def __init__(self, name):
        super().__init__('tryToScoreNode')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)

        # Subscriber for opponent position (expects [x, y] coordinates)
        # assuming this will only receive data every 5 seconds (or similar) and not more
        self.ball_coords_subscription = self.create_subscription(
            Float32MultiArray,
            '/ball_and_goal_position',
            self.control_loop,
            10
        )

        self.ball_x = None
        self.ball_y = None
        self.goal_x = None
        self.goal_y = None
        # self.attack = False
        # self.defend = False
        self.lastX = 0

    # function called when the enemy coords are retreived
    # determines whether we should charge at the enemy or not
    # depends on whether the target is visible
    # more charges -> more points! :D
    # only charges once every 5 seconds max
    def control_loop(self, msg = None):
        twist = Twist()

        # If no message is passed, return
        if msg == None:
            # print("Target_x is none")
            return
        
        linearSpeed = 0.6
        angularSpeed = 2.0

        # Otherwise we have a point to target
        self.ball_x = msg.data[0]
        self.ball_y = msg.data[1]
        self.goal_x = msg.data[2]
        self.goal_y = msg.data[3]
        # Cases
        # Ball is not in view -> Get it in view by spinning
        # Ball is in view -> 
            # Goal is not in view -> rotate around ball until goal is in view
            # Goal is in view -> Push the ball forward
        if self.ball_x == 0.0: # Then the ball is not in frame 
            # print("Spin to look")
            # The camera's resolution is 640x480 so if x is less than 320, then it was on the left side. If x > 320 it must have been on the right.
            if (self.lastX < 320):
                twist.angular.z = 1.0  # Slow spin left. It can't be any higher than this or it overshoots. It spins faster than it processes the next frame
            else:
                twist.angular.z = -1.0  # Slow spin right
            twist.linear.x = 0.0 
            ballInView = False
        else:
            ballInView = True

        if ballInView:
            if self.goal_x == 0.0:
                # Ball is in view but not the goal ->
                # rotate around ball

            elif 200 <= self.ball_x <= 450:
                # print("Moving forward")
                twist.angular.z = 0.0 
                twist.linear.x = linearSpeed  # Full speed forward
                self.lastX = self.ball_x

            elif self.ball_x < 200:
                # print("Turn left")
                twist.angular.z = angularSpeed  # Small left turn
                twist.linear.x = linearSpeed   # Move forward
                self.lastX = self.ball_x

            elif self.ball_x > 450:
                # print("Turn right")
                twist.angular.z = -angularSpeed  # Small right turn
                twist.linear.x = linearSpeed    # Move forward   
                self.lastX = self.ball_x

        # Publish movement command
        self.cmd_vel_pub.publish(twist)


def main():
    charge = TryToScore('tryToScoreNode')
    rclpy.spin(charge)
    charge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    main()
