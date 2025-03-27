import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import subprocess

"""
Master class for running all processes
Determine if robot should be in attack or defense mode
"""
#for now I am using wall distance as the parameter
#but this should be fine tuned to whatever criteria we decide

class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')

        #subscribe to the topic publishing wall distances from detectWalls.py
        self.subscription = self.create_subscription(
            Float32MultiArray,  #type
            '/wall_distances',  #topic name
            self.wall_callback, #callback function to process data
            10)  #queue size
        
        self.masterControllerPublisher = self.create_publisher(Point, '/master_controller', 10)

        #distance threshold to trigger defense mode (in meters)
        self.wall_threshold = 0.25  # Adjust as needed
        self.front_wall_threshold = 0.25


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
    rclpy.spin(node)  #
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':
    main()
