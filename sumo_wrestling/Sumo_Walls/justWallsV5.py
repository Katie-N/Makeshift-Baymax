import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class WallDetector(Node):
    def __init__(self):
        super().__init__('wall_detector')

        #subscribe to the lidar topic 
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',  # Default LiDAR topic, may need to change based on your setup
            self.lidar_callback,
            10)  #message queue size
        self.subscription  #prevent unused variable warning

    def lidar_callback(self, msg):
        """
        Callback function that processes incoming lidar scan data.
        Extracts distance readings at different angles and determines walls.
        """

        #convert lidar angle range from radians to degrees
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges)) * 180 / np.pi
        distances = np.array(msg.ranges)  #convert range data to a NumPy array

        #define wall data by analyzing LiDAR readings in specific angle ranges
        #each direction is associated with a small angle range to detect walls
        #angles are what google said they should be but may need to be changed
        wall_data = {
            "left": self.get_wall_info(distances, angles, 90, 110),  # Left side (90° to 110°)
            "right": self.get_wall_info(distances, angles, -90, -110),  # Right side (-90° to -110°)
            "front": self.get_wall_info(distances, angles, -10, 10),  # Front (-10° to 10°)
            "back": self.get_wall_info(distances, angles, 170, 190)  # Back (170° to 190°)
        }

        #format the output for clear readability
        distance_output = " | ".join([f"{key} wall: {dist:.2f}m" for key, (dist, _) in wall_data.items()])
        angle_output = " | ".join([f"{key} angle: {angle:.2f}°" for key, (_, angle) in wall_data.items()])

        #print the formatted wall detection data
        self.get_logger().info(f"{distance_output} | {angle_output}")

    def get_wall_info(self, distances, angles, angle_min, angle_max):
        """
        Extracts the closest distance and average angle of a wall within a given angle range.

        Parameters:
        - distances: NumPy array of distance readings from lidar
        - angles: NumPy array of corresponding angle values (in degrees)
        - angle_min, angle_max: The range of angles to filter (e.g., for left wall detection)

        Returns:
        - (closest_distance, average_angle): Tuple containing the nearest detected wall distance and its average angle
        """

        #create a mask to filter readings within the specified angle range
        mask = (angles >= angle_min) & (angles <= angle_max)

        #apply the mask to get valid distance and angle values
        valid_distances = distances[mask]
        valid_angles = angles[mask]

        if len(valid_distances) > 0:
            #find the closest detected point within this region
            closest_distance = np.min(valid_distances)
            #calculate the average angle of detected points in this region
            avg_angle = np.mean(valid_angles)
            return closest_distance, avg_angle
        else:
            #if no valid data points are found, return infinity for distance (no wall detected)
            # and 0 for angle (neutral/default value)
            return float('inf'), 0  

"""
it should hopefully print something like this so the data is easier to read:

left wall: 1.23m | right wall: 0.98m | front wall: 2.45m | back wall: inf
left angle: 95.6° | right angle: -94.2° | front angle: -3.4° | back angle: 175.2°

"""

def main(args=None):
    """
    Main function to initialize the ROS2 node and start wall detection.
    """
    rclpy.init(args=args)
    wall_detector = WallDetector()
    rclpy.spin(wall_detector)
    wall_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
