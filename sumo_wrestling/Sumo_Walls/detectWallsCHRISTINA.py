import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class WallTracker(Node):
    def __init__(self):
        super().__init__('wall_tracker')
        
        #subscribe to the lidar scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',  #topic for LiDAR sensor readings
            self.lidar_callback,
            10)
        print("subscribed to lidar")
        """
        for if we decide to implement the feature where it moves away from
        wall if within designated distance
        """
        #desired distance from walls (meters)
        self.target_distance = 0.5

        #Christina added these to use in defence.py
        self.left_dist = float('inf')
        self.right_dist = float('inf')
        self.front_dist = float('inf')

    def lidar_callback(self, msg):
        """
        callback function to process lidar data.
        calculates distances to the left, right, and front as the robot moves.
        """
        print("I heard" + str(msg))
        
        #extract minimum distance from specific angle ranges
        self.left_dist = min(msg.ranges[60:120])   #left side (60° to 120°)
        self.right_dist = min(msg.ranges[240:300]) #right side (240° to 300°)
        self.front_dist = min(msg.ranges[0:20] + msg.ranges[340:360])  #front (0°-20° & 340°-360°)

        #logging detected distances for debugging and implementation
        print(f"Left Distance: {self.left_dist:.2f}m, Right Distance: {self.right_dist:.2f}m, Front Distance: {self.front_dist:.2f}m")
        # self.get_logger().info(f"Left Distance: {left_dist:.2f}m, Right Distance: {right_dist:.2f}m, Front Distance: {front_dist:.2f}m")
        
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
