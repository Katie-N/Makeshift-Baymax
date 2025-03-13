import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarShapeRecognizer(Node):
    def __init__(self):
        super().__init__('lidar_shape_recognizer')
        
        #subscribe to the lidar scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  #lidar topic
            self.lidar_callback,
            10  #message queue size
        )
        
        #threshold to determine if points form a straight wall
        self.wall_threshold = 0.1  #distance tolerance for detecting straight lines
        
        #variable to store the last known position of the opponent
        self.opponent = None  

    def lidar_callback(self, msg):
        """
        Callback function that processes incoming LiDAR data.
        It converts the raw data into Cartesian coordinates, 
        groups the points manually, and identifies walls and the opponent.
        """
        
        #convert lidar data into an array
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        #convert polar coordinates (distance, angle) into Cartesian coordinates (x, y)
        points = np.array([self.polar_to_cartesian(r, theta) 
                           for r, theta in zip(ranges, angles) if 0.1 < r < msg.range_max])
        
        #if no valid points are detected, return early
        if points.size == 0:
            return
        
        #manually group points into clusters
        labels = self.manual_clustering(points)
        
        #analyze the clusters to differentiate walls from the opponent
        walls, opponent = self.identify_shapes(points, labels)
        
        #log detected walls and opponent position for debugging
        self.get_logger().info(f"Walls: {walls}")
        self.get_logger().info(f"Opponent: {opponent}")
        
        #update the last known opponent position
        self.opponent = opponent if opponent is not None else self.opponent
        
    def polar_to_cartesian(self, r, theta):
        """
        Converts a point from polar coordinates (distance, angle) to Cartesian coordinates (x, y).
        """
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return np.array([x, y])
    
    def manual_clustering(self, points):
        """
        Groups nearby points into clusters manually based on distance threshold.
        """
        labels = np.full(len(points), -1)  #initialize all points as unclassified
        cluster_id = 0

        for i in range(len(points)):
            if labels[i] != -1:
                continue  #skip already classified points
            
            #start a new cluster
            labels[i] = cluster_id
            
            for j in range(i + 1, len(points)):
                if np.linalg.norm(points[i] - points[j]) < 0.15:  #if points are close, assign to same cluster
                    labels[j] = cluster_id
            
            cluster_id += 1  #move to next cluster
        
        return labels
    
    def identify_shapes(self, points, labels):
        """
        Analyzes the clusters of points to determine which belong to 
        walls and which represent the opponent.
        """
        unique_labels = set(labels)
        walls = []  #list to store identified wall positions
        opponent = None  #variable to store opponent's estimated position
        
        for label in unique_labels:
            if label == -1:
                continue  #ignore noise points
            
            #extract all points that belong to the current cluster
            cluster_points = points[labels == label]
            
            #check if the cluster has a rectangular shape (walls) or a compact shape (opponent)
            if self.is_rectangular(cluster_points):
                walls.extend(cluster_points.tolist())  #add identified walls to the list
            else:
                #estimate the opponent's position as the centroid of the cluster
                opponent = np.mean(cluster_points, axis=0).tolist()
                
        return walls, opponent
    
    def is_rectangular(self, cluster_points):
        """
        Determines if a given cluster of points forms a rectangular shape (which represents walls).
        """
        #if the cluster has too few points, it cannot be a rectangle
        if len(cluster_points) < 4:
            return False
        
        #compute the bounding box dimensions of the cluster
        x_min, y_min = np.min(cluster_points, axis=0)
        x_max, y_max = np.max(cluster_points, axis=0)
        
        width = x_max - x_min
        height = y_max - y_min
        
        #calculate aspect ratio (width/height) to determine if it's roughly rectangular
        aspect_ratio = width / height if height != 0 else float('inf')
        
        #walls should have a rectangular aspect ratio (not a compact cluster)
        return 0.5 < aspect_ratio < 2.0  #allow some flexibility in shape dimensions
    

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