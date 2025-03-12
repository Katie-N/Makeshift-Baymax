import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

#second attempt at wall/oppenent tracking 
"""
In this version the walls are predefined in positions
The lidar scans and checks to make sure the walls are in the same position
Anything outside of the walls predefined position is assumed to be opponent
"""

class LidarTracker(Node):
    def __init__(self):
        super().__init__('lidar_tracker')
        
        #subscribe to the lidar scan topic to receive distance measurements
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  #topic where lidar sensor publishes readings
            self.lidar_callback,
            10)  #queue size for message buffering
        
        #define the assumed fixed walls of the playing field 
        #i forgot so just guessing (e.g., 2m x 2m square arena)
        self.field_size = 2.0  #assuming a 2m x 2m square arena
        
        #predefine the positions of the walls in Cartesian coordinates
        self.known_walls = [
            (-self.field_size / 2, y) for y in np.linspace(-self.field_size / 2, self.field_size / 2, 10)  #left wall
        ] + [
            (self.field_size / 2, y) for y in np.linspace(-self.field_size / 2, self.field_size / 2, 10)   #right wall
        ] + [
            (x, -self.field_size / 2) for x in np.linspace(-self.field_size / 2, self.field_size / 2, 10)  #bottom wall
        ] + [
            (x, self.field_size / 2) for x in np.linspace(-self.field_size / 2, self.field_size / 2, 10)   #top wall
        ]
        
        #dictionary to store detected objects (walls and opponent)
        self.detected_objects = {
            'walls': {},  #dictionary to store detected static wall positions
            'opponent': None  #stores last known position of the moving opponent
        }
        
        #store previous scan data for movement detection, to differentiate walls from a moving opponent
        self.previous_scan = {}
        
    def lidar_callback(self, msg):
        """
        Callback function triggered upon receiving new lidar scan data.
        Detects walls using predefined positions and identifies an opponent based on movement.
        """
        current_scan = {}  #stores the current frame's detected objects
        
        #iterate over each lidar scan angle and process distance data
        for i, distance in enumerate(msg.ranges):
            if 0.1 < distance < msg.range_max:  #filter out invalid readings (too close or out of range)
                angle = np.deg2rad(i)  #convert angle to radians
                x = distance * np.cos(angle)  #calculate Cartesian x-coordinate
                y = distance * np.sin(angle)  #calculate Cartesian y-coordinate
                
                #store current scan data for tracking object movements
                current_scan[i] = (x, y)
                
                #classify detected object as a wall or an opponent
                if self.is_known_wall((x, y)):
                    self.detected_objects['walls'][i] = (x, y)  #recognized as a wall
                elif self.is_moving_object(i, (x, y)):
                    self.detected_objects['opponent'] = (x, y)  #recognized as a moving opponent
        
        #log detected wall positions for debugging
        self.get_logger().info(f"Walls: {list(self.detected_objects['walls'].values())}")
        
        #log opponent position if detected
        if self.detected_objects['opponent']:
            self.get_logger().info(f"Opponent at: {self.detected_objects['opponent']}")
        
        #update stored previous scan data for motion tracking in the next frame
        self.previous_scan = current_scan
        
    def is_known_wall(self, position):
        """
        Checks if a detected object is within the predefined wall positions.
        Walls are assumed to remain fixed in the arena.
        """
        for wall_pos in self.known_walls:
            if np.linalg.norm(np.array(position) - np.array(wall_pos)) < 0.1:  #small tolerance threshold for wall detection
                return True
        return False
    
    def is_moving_object(self, angle_index, current_position):
        """
        Determines if an object is moving by comparing its position with previous scans.
        If the objects position has significantly changed, it is classified as an opponent.
        """
        if angle_index in self.previous_scan:
            prev_x, prev_y = self.previous_scan[angle_index]  #get the object's previous coordinates
            curr_x, curr_y = current_position  #get current coordinates
            movement_threshold = 0.05  #minimum movement required to classify as an opponent
            
            #if the object has moved beyond the threshold, it is likely an opponent
            if np.sqrt((curr_x - prev_x) ** 2 + (curr_y - prev_y) ** 2) > movement_threshold:
                return True
        return False

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    Initializes the LidarTracker node and keeps it running.
    """
    rclpy.init(args=args)
    node = LidarTracker()
    rclpy.spin(node)  #keep the node running to continuously process lidar data
    node.destroy_node()
    rclpy.shutdown()  #properly shut down the ROS2 node

if __name__ == '__main__':
    main()  #run the main function if the script is executed directly
