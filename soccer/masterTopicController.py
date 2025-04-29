import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import subprocess

"""
Master class for running all processes
Determine if robot should be pushing the ball forward or circling the ball to see the goal
"""

class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')

        # Subscriber for opponent position (expects [x, y] coordinates)
        # assuming this will only receive data every 5 seconds (or similar) and not more
        self.ball_coords_subscription = self.create_subscription(
            Point,
            '/ball_position',
            self.control_loop,
            10
        )
        
        self.masterControllerPublisher = self.create_publisher(Point, '/master_controller', 10)

    def wall_callback(self, msg):
        """
        Callback function triggered when new wall distance data is received.
        Determines whether to switch to attack or defense mode.
        """
        
        #extract wall distances from the message
        front_dist, right_dist, back_dist, left_dist = msg.data

        #print distances for debugging
        # self.get_logger().info(f"Front: {front_dist:.2f}m, Right: {right_dist:.2f}m, Back: {back_dist:.2f}m, Left: {left_dist:.2f}m")

        msg = Point()

        #check if any wall is too close (less than threshold)
        if front_dist <= self.front_wall_threshold or right_dist <= self.wall_threshold or left_dist <= self.wall_threshold or back_dist <= self.front_wall_threshold:
            msg.x = 0.0 # x represents attack mode
            msg.y = 1.0 # y represents defense mode
        else:
            msg.x = 1.0 # x represents attack mode
            msg.y = 0.0 # y represents defense mode
        self.masterControllerPublisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)  
    node = MasterController()  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':
    main()
