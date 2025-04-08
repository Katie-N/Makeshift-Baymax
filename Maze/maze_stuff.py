import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import heapq
import time

class MazeSolver(Node):
    def __init__(self):
        # Initialize 
        super().__init__('maze_solver')
        
        # Subscribe to LiDAR data and camera image data
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        
        # Publisher for robot movement commands
        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        
        # Convert ROS2 image messages to OpenCV format
        self.bridge = CvBridge()
        
        # Define grid parameters (1-inch resolution for 6x5 ft maze)
        self.grid_size = (72, 60)  # 6 feet = 72 inches, 5 feet = 60 inches
        self.grid = np.zeros(self.grid_size, dtype=int)  # 0: unexplored, 1: wall, 2: path
        
        # Dictionary to store detected wall colors at specific coordinates
        self.color_map = {}
        
        # Define start position (initially at bottom-left corner) and unknown goal position
        self.start_pos = (1, 1)  # Assume starting near bottom-left corner
        self.goal_pos = None  # Initially unknown, set when blue wall is detected
        self.robot_pos = self.start_pos  # Track current position
        self.robot_direction = 0  # 0: right, 1: down, 2: left, 3: up (clockwise)
        
        # Start maze exploration using wall-following
        self.explore_maze()
        
        # After exploration, compute the shortest path using Dijkstra's Algorithm
        self.shortest_path = self.compute_shortest_path()
        
        # Follow the computed shortest path to the goal
        self.follow_shortest_path()
    def stop_moving(self):
        """Stops the robot's movement."""
        print("stop_moving()")

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.5)  # Give time to stop
    
    def lidar_callback(self, msg):
        """Processes LiDAR data to update the grid with walls."""
        print("lidar_callback()")
        print(msg.ranges)
        for i, distance in enumerate(msg.ranges):
            if distance < 0.5:  # Threshold to detect walls (meters)
                x, y = self.get_grid_coordinates(i, distance)
                if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
                    self.grid[x, y] = 1  # Mark detected obstacle as a wall
    
    # Got mostly from Katie's Sumo code
    def camera_callback(self, msg):
        """Processes camera images to detect colored walls."""
        print("wall_distance()")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  # Convert ROS image message to OpenCV format
        detected_color = self.detect_color(cv_image)  # Detect color in the image
        
        if detected_color:
            self.get_logger().info(f"Detected color: {detected_color}")
            
            # If the detected color is blue, set it as the goal position
            if detected_color == 'blue':
                self.goal_pos = self.robot_pos  # Set goal to the robot's current position
            
            # Store the detected color in the color_map for future reference
            self.color_map[self.robot_pos] = detected_color
    
    def detect_color(self, image):
        """Detects color in an image using HSV filtering."""
        print("detect_color()")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # Convert image to HSV color space
        
        # Define color ranges for detection
        color_ranges = {
            'red': ((0, 120, 70), (10, 255, 255)),
            'blue': ((90, 50, 50), (130, 255, 255)),
            'green': ((40, 50, 50), (80, 255, 255))
            # Add more of course
        }
        
        # Check for each color in the defined ranges
        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))  # Create a mask for the color
            if np.any(mask):  # If any pixels match the color range
                return color  # Return the first detected color
        return None  # No color detected
    
    # Not good at the math stuff so this may need to be fixed
    def get_grid_coordinates(self, angle, distance):
        """Converts LiDAR angle and distance data into grid coordinates."""
        print("get_grid_coordinates()")

        # Calculate x, y coordinates based on robot's position and LiDAR data
        x = int(self.robot_pos[0] + distance * np.cos(np.radians(angle)))
        y = int(self.robot_pos[1] + distance * np.sin(np.radians(angle)))
        return x, y
    
    def explore_maze(self):
        """Wall-following algorithm to explore the maze."""
        print("explore_maze()")

        self.get_logger().info("Starting wall-following exploration...")
        
        # Continue exploration until the goal is found
        while self.goal_pos is None:
            # Keep the wall on the robot's right side using the right-hand rule
            if not self.wall_on_right():
                self.turn_right()
            
            # Move forward while the front is clear
            while self.front_clear():
                self.move_forward()
                if self.goal_pos:  # Stop if goal is reached
                    break
            self.turn_left()
        
        self.get_logger().info("Exploration complete!")
    
    def compute_shortest_path(self):
        """Finds the shortest path to the goal using Dijkstra's Algorithm."""
        print("computeShortestPath()")

        if self.goal_pos is None:
            return []  # No goal, no path
        
        # Define possible directions: right, down, left, up
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        pq = [(0, self.start_pos)]  # Priority queue for Dijkstra's (cost, position)
        visited = set()  # Set of visited nodes
        prev = {}  # Dictionary to track previous positions for path reconstruction
        
        # Dijkstra's algorithm
        while pq:
            cost, current = heapq.heappop(pq)  # Get the current node with the lowest cost
            if current in visited:
                continue  # Skip if already visited
            visited.add(current)
            
            if current == self.goal_pos:
                break  # Stop if goal is reached
            
            for d in directions:
                # Check all neighboring cells (right, down, left, up)
                neighbor = (current[0] + d[0], current[1] + d[1])
                
                # Ensure the neighbor is within bounds and not a wall
                if 0 <= neighbor[0] < self.grid_size[0] and 0 <= neighbor[1] < self.grid_size[1] and self.grid[neighbor] != 1:
                    heapq.heappush(pq, (cost + 1, neighbor))  # Add neighbor to the priority queue
                    prev[neighbor] = current  # Track the current node as the previous one for path reconstruction
        
        # Reconstruct the shortest path from the goal to the start
        path = []
        step = self.goal_pos
        while step in prev:
            path.append(step)
            step = prev[step]
        path.reverse()  # Reverse path to get from start to goal
        return path
    
    def follow_shortest_path(self):
        """Follows the shortest path to the goal."""
        print("follow_shortest_path()")


        for step in self.shortest_path:
            self.robot_pos = step  # Update robot's position to the next step in the path
            self.move_forward()  # Move robot forward by one step
        self.get_logger().info("Reached the goal following the shortest path!")
    
    def move_forward(self):
        """Moves the robot forward one step (1 inch)."""
        print("move_forward()")

        twist = Twist()
        twist.linear.x = 0.05
        self.cmd_pub.publish(twist)
        time.sleep(0.5)  # Give time to move
        self.stop_moving()
        self.robot_pos = self.update_position()

    def turn_right(self):
        """Turns the robot 90 degrees to the right."""
        print("turn_right()")

        twist = Twist()
        twist.angular.z = -1.57  # Approximate 90-degree turn
        self.cmd_pub.publish(twist)  # Publish the turn command
        time.sleep(0.5)  # Give time to move
        self.stop_moving()
        self.robot_direction = (self.robot_direction + 1) % 4  # Update robot's facing direction

    def turn_left(self):
        """Turns the robot 90 degrees to the left."""
        print("turn_left()")

        twist = Twist()
        twist.angular.z = 1.57  # Approximate 90-degree turn
        self.cmd_pub.publish(twist)  # Publish the turn command
        time.sleep(0.5)
        self.stop_moving()
        self.robot_direction = (self.robot_direction - 1) % 4  # Update robot's facing direction

    def front_clear(self):
        """Checks if the front is clear using LiDAR data."""
        print("front_clear()")

        x, y = self.robot_pos
        if self.robot_direction == 0:
            check_x, check_y = x + 1, y
        elif self.robot_direction == 1:
            check_x, check_y = x, y + 1
        elif self.robot_direction == 2:
            check_x, check_y = x - 1, y
        elif self.robot_direction == 3:
            check_x, check_y = x, y - 1
        else:
            return False

        if 0 <= check_x < self.grid_size[0] and 0 <= check_y < self.grid_size[1]:
            return self.grid[check_x, check_y] != 1
        return False

    
    def wall_on_right(self):
        """Checks if there is a wall to the right of the robot."""
        print("wall_on_right()")

        x, y = self.robot_pos
        # Adjust based on direction
        if self.robot_direction == 0:
            check_x, check_y = x, y + 1
        elif self.robot_direction == 1:
            check_x, check_y = x - 1, y
        elif self.robot_direction == 2:
            check_x, check_y = x, y - 1
        elif self.robot_direction == 3:
            check_x, check_y = x + 1, y
        else:
            return False  # Unknown direction

        if 0 <= check_x < self.grid_size[0] and 0 <= check_y < self.grid_size[1]:
            return self.grid[check_x, check_y] == 1
        return False

    
    def update_position(self):
        """Updates the robot's position based on its movement direction."""
        print("update_position()")

        # Update position based on the robot's current direction
        if self.robot_direction == 0:
            print("Moving right")
            return (self.robot_pos[0] + 1, self.robot_pos[1])  # Move right
        elif self.robot_direction == 1:
            print("Moving down")
            return (self.robot_pos[0], self.robot_pos[1] + 1)  # Move down
        elif self.robot_direction == 2:
            print("Moving left")
            return (self.robot_pos[0] - 1, self.robot_pos[1])  # Move left
        elif self.robot_direction == 3:
            print("Moving up")
            return (self.robot_pos[0], self.robot_pos[1] - 1)  # Move up
    
    def run(self):
        # Wait for a bit to ensure sensor data is coming in
        self.get_logger().info("Waiting for sensor data...")
        time.sleep(2)
        self.explore_maze()
        self.shortest_path = self.compute_shortest_path()
        self.follow_shortest_path()
        rclpy.spin(self)


    def print_grid(self):
        for row in self.grid.T[::-1]:  # Flip Y for visualization
            print(''.join(['#' if cell == 1 else '.' for cell in row]))


if __name__ == '__main__':
    rclpy.init()  
    node = MazeSolver()  
    node.run()  
    rclpy.shutdown()  
