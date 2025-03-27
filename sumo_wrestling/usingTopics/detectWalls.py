import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class WallTracker(Node):
    def __init__(self):
        super().__init__('wall_tracker')
        
        #subscribe to the lidar scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',  #topic for LiDAR sensor readings
            self.lidar_callback,
            10)

        self.wall_distances = self.create_publisher(Float32MultiArray, '/wall_distances', 10)
        
        #Christina added these to use in defence.py
        self.left_dist = float('inf')
        self.right_dist = float('inf')
        self.front_dist = float('inf')
        self.back_dist = float('inf')

    def lidar_callback(self, msg):
        """
        callback function to process lidar data.
        calculates distances to the left, right, and front as the robot moves.
        """
        distances = np.array(msg.ranges)

        offset = 45
        self.left_dist = float(np.nanmin(distances[offset+45:135+offset]))   
        self.right_dist = float(np.nanmin(distances[offset+225:offset+315])) 
        self.front_dist = float(np.nanmin(np.concatenate((distances[0:offset+45], distances[offset+315:360]), axis=0)))
        self.back_dist = float(np.nanmin(distances[offset+135:offset+225])) 


        msg = Float32MultiArray()
        msg.data = [self.front_dist, self.right_dist, self.back_dist, self.left_dist]
        self.wall_distances.publish(msg)
        
        # Log detected distances for debugging and implementation
        # self.get_logger().info(f"Left Distance: {self.left_dist:.2f} m, Right Distance: {self.right_dist:.2f} m, Front Distance: {self.front_dist:.2f} m, Back Distance: {self.back_dist:.2f} m")


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    node = WallTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()