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
        """
        for if we decide to implement the feature where it moves away from
        wall if within designated distance
        """
        #desired distance from walls (meters)
        self.target_distance = 0.5

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
        #extract minimum distance from specific angle ranges
        self.left_dist = float(min(distances[50:130]))   # left
        self.right_dist = float(min(distances[250:310])) # right
        self.front_dist = float(min(distances[0:50] + distances[310:360]))  # front
        self.back_dist = float(min(distances[130:250])) # back
        
        msg = Float32MultiArray()
        msg.data = [self.front_dist, self.right_dist, self.back_dist, self.left_dist]
        self.wall_distances.publish(msg)
        
        #logging detected distances for debugging and implementation
        self.get_logger().info(f"Left Distance: {self.left_dist:.2f} m, Right Distance: {self.right_dist:.2f} m, Front Distance: {self.front_dist:.2f} m, Back Distance: {self.back_dist:.2f} m")
        
        #placeholder for future self-centering or avoidance logic
        #if left_dist < some_threshold or right_dist < some_threshold:
        #adjust movement to avoid walls


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