import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        #subscribe to lidar scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', #topic name for lidar data
            self.lidar_callback, #callback for when new data arrives
            10) #message queue size
        
        #initialize robots position and orientation
        self.robot_position = np.array([0, 0])  #robot starts at (0,0)
        self.robot_orientation = 0  #angle in radians

        #lists to store detected objects
        self.walls = []  #list of detected wall points
        self.opponent = None  #opponent's estimated position
    
    def lidar_callback(self, msg):
        """
        Callback function to process incoming lidar scan data.
        Converts polar coordinates to Cartesian 
        Classifies objects as walls or an opponent.
        """
        ranges = np.array(msg.ranges) #distance measurements from lidar
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges)) #corresponding angles
        
        #convert valid scan points from polar to cartesian 
        points = [self.polar_to_cartesian(r, theta) for r, theta in zip(ranges, angles) if r < msg.range_max]
        
        #classify detected objects
        self.walls, self.opponent = self.classify_objects(points)
        
        #adjust positions based on robot's movement
        self.update_map()
        
        #log detected walls and opponent positions
        self.get_logger().info(f"Walls: {self.walls}")
        self.get_logger().info(f"Opponent: {self.opponent}")
    
    def polar_to_cartesian(self, r, theta):
        """
        Converts polar coordinates (r, theta) to Cartesian (x, y).
        """
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return np.array([x, y])
    
    def classify_objects(self, points):
        """
        Distinguishes between walls and an opponent based on point clustering
        """
        walls = []
        opponent = None
        
        #if points match previous wall location, assume it is a wall
        #otherwise it is assumed an opponent
        for point in points:
            if self.is_wall(point):
                walls.append(point) #consistently detected objects are walls
            else:
                opponent = point  #assume any not wall object is opponent
        
        return walls, opponent
    
    def is_wall(self, point):
        """
        Determines if a given point belongs to a wall based on spatial consistency.
        """
        #if two points are less than 0.1 m away, they are same object
        #might cause problems as robot nears wall
        threshold_distance = 0.1  #distance consistency threshold for walls
        return any(np.linalg.norm(point - w) < threshold_distance for w in self.walls)
    
    def update_map(self):
        """
        Updates the detected wall and opponent positions relative to the robot's movement.
        """
        #create a rotation matrix based on robot orientation
        #rotates all objects as robot turns
        rotation_matrix = np.array([
            [np.cos(self.robot_orientation), -np.sin(self.robot_orientation)],
            [np.sin(self.robot_orientation), np.cos(self.robot_orientation)]
        ])
        
        #transform wall positions to align with new robot coordinates
        self.walls = [self.robot_position + rotation_matrix @ w for w in self.walls]
        
        #adjust opponent position if detected
        if self.opponent is not None:
            self.opponent = self.robot_position + rotation_matrix @ self.opponent

    def update_robot_position(self, new_position, new_orientation):
        """
        Updates the robot's position and orientation after movement.
        """
        self.robot_position = np.array(new_position)
        self.robot_orientation = new_orientation


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init()
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()