import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'lidar_topic', 10)
        self.timer_ = self.create_timer(0.1, self.publish_scan) # Publish every 0.1 seconds
    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_frame' # Replace with your LiDAR frame ID
        # ... (Fill in the scan data)
        scan.ranges = [0.0, 360.0] # Example ranges
        scan.intensities = [1.0, 1.0, 1.0] # Example intensities
        self.publisher_.publish(scan)
        self.get_logger().info('Publishing scan')

def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()