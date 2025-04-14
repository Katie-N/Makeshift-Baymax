import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import os
from ament_index_python.packages import get_package_share_directory
from collections import deque

"""
Takes in a 12x10 grid as a map
Navigates based on map
"""

# Each cell in the grid represents 15 cm (0.15 meters) in the real world
CELL_SIZE = 0.15

# Define cardinal directions and their (dx, dy) vector representations
DIRECTIONS = {
    'N': (0, -1),  # North = up in grid (decrease row index)
    'E': (1, 0),   # East = right in grid (increase column index)
    'S': (0, 1),   # South = down in grid (increase row index)
    'W': (-1, 0)   # West = left in grid (decrease column index)
}

# Used to compute turn directions in clockwise order
DIR_ORDER = ['N', 'E', 'S', 'W']

class MazeNavigator(Node):
    def __init__(self):
        super().__init__('maze_navigator')

        # Create a ROS2 publisher to control robot velocity using Twist messages
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Load the map of the maze from a .txt file
        self.grid = self.load_grid('nav.txt')

        # Generate a hardcoded path (you'll replace this later with actual shortest path)
        self.path = self.generate_path()

        # Track the direction the robot is currently facing (N/E/S/W)
        self.current_dir = None

        # Begin following the path
        self.move_along_path()

    def load_grid(self, filename):
        """
        Reads the maze layout from a .txt file and returns it as a 2D list.
        Each line is a row of the maze, containing 0s (paths) and 1s (walls), space-separated.
        """
        package_dir = get_package_share_directory('Maze22')
        file_path = os.path.join(package_dir, filename)

        with open(file_path, 'r') as f:
            lines = f.readlines()
        grid = [list(map(int, line.strip().split())) for line in lines]
        return grid

    def generate_path(self):
        """
        Uses BFS to find the shortest path from (0, 0) to the nearest bottom-right 0.
        Returns a list of (row, col) tuples representing the path.
        """
        rows, cols = len(self.grid), len(self.grid[0])
        start = (0, 0)
        end = None

        # Find the last 0 in the grid (bottom-right-ish)
        for r in range(rows-1, -1, -1):
            for c in range(cols-1, -1, -1):
                if self.grid[r][c] == 0:
                    end = (r, c)
                    break
            if end:
                break

        if not end:
            self.get_logger().warn("No reachable end point found.")
            return []

        # BFS setup
        queue = deque()
        queue.append((start, [start]))  # (current_node, path_so_far)
        visited = set()
        visited.add(start)

        while queue:
            (r, c), path = queue.popleft()

            if (r, c) == end:
                return path  # Found goal

            # Explore 4 directions
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and self.grid[nr][nc] == 0 and (nr, nc) not in visited:
                    queue.append(((nr, nc), path + [(nr, nc)]))
                    visited.add((nr, nc))

        self.get_logger().warn("No path found.")
        return []

    def determine_initial_direction(self):
        """
        Looks at the first step in the path to figure out which direction the robot
        should be facing at the beginning (e.g., North, East, South, West).
        """
        if len(self.path) < 2:
            return None  # Not enough points to determine direction

        # Extract the coordinates of the first two points in the path
        x0, y0 = self.path[0] # Start
        x1, y1 = self.path[1] # Next

        # Calculate difference in coordinates between the first two points
        dx = x1 - x0 # How many to move right/left
        dy = y1 - y0 # How many to move up/down

        # Loop through the predefined direction vectors in the DIRECTIONS dictionary.
        # It compares (dx, dy) to see which direction the robot is moving toward:
            # (1, 0) = East (right)
            # (-1, 0) = West (left)
            # (0, -1) = North (up)
            # (0, 1) = South (down)
        for dir_label, (dx_ref, dy_ref) in DIRECTIONS.items():
            if (dx, dy) == (dx_ref, dy_ref):
                return dir_label  # Found matching direction vector

        return None  # Couldn't match direction

    def angle_between(self, from_dir, to_dir):
        """
        Calculates how much to rotate (in radians) to change the robot's heading
        from one direction to another.
        Positive = turn left (counter-clockwise), negative = turn right (clockwise).
        Used in the angle_between() function
        """
        from_idx = DIR_ORDER.index(from_dir) # Find index of the current direction the robot is facing
        to_idx = DIR_ORDER.index(to_dir) # Find index of the direction the robot needs to face next
        diff = (to_idx - from_idx) % 4 # Compute how many 90-degree turns we need to go from from_dir to to_dir (clockwise)

        if diff == 0:
            return 0.0  # Already facing the right way
        elif diff == 1:
            print(f"Turning left from {from_dir} to {to_dir}")
            return math.pi / 2  # 90 degrees left
        elif diff == 2:
            print(f"U-turn from {from_dir} to {to_dir}")
            return math.pi      # 180 degrees (U-turn)
        elif diff == 3:
            print(f"Turning right from {from_dir} to {to_dir}")
            return -math.pi / 2  # 90 degrees right

    def move_forward(self, distance):
        """
        Commands the robot to move forward a specific distance (in meters).
        Assumes a constant forward speed of 0.1 m/s and calculates how long to move.
        """
        speed = 0.1  # meters/second
        duration = distance / speed  # Time = Distance / Speed

        twist = Twist()
        twist.linear.x = speed  # Set forward speed

        self.publisher_.publish(twist)  # Start moving
        print(f"Moving forward {distance} meters for {duration:.2f} seconds")
        time.sleep(duration)           # Wait while robot moves

        twist.linear.x = 0.0           # Stop movement
        self.publisher_.publish(twist)

    def rotate(self, angle):
        """
        Rotates the robot in place by a given angle (in radians).
        Positive angle = left turn, Negative angle = right turn.
        """
        speed = 0.5  # rad/s
        duration = abs(angle) / speed  # Time = Angle / Speed

        twist = Twist()
        twist.angular.z = speed if angle > 0 else -speed  # Set rotation direction

        self.publisher_.publish(twist)  # Begin rotation
        print(f"Rotating {'left' if angle > 0 else 'right'} {math.degrees(abs(angle)):.1f} degrees for {duration:.2f} seconds")
        time.sleep(duration)           # Rotate for the required time

        twist.angular.z = 0.0          # Stop rotation
        self.publisher_.publish(twist)

    def move_along_path(self):
        """
        Core navigation function: moves the robot cell-by-cell through the maze
        using the path from self.path, rotating as needed to face the right direction.
        """
        if len(self.path) < 2:
            self.get_logger().info("Path too short or empty.")
            return

        # Start at the first point in the path
        current = self.path[0]

        # Determine which direction the robot should be facing initially
        self.current_dir = self.determine_initial_direction()
        if not self.current_dir:
            self.get_logger().warn("Could not determine initial direction.")
            return

        # Iterate through each step in the path
        for next_point in self.path[1:]:
            x0, y0 = current
            x1, y1 = next_point
            dx = x1 - x0
            dy = y1 - y0

            # Determine the direction this step is moving toward
            target_dir = None
            for label, (dx_ref, dy_ref) in DIRECTIONS.items():
                if (dx, dy) == (dx_ref, dy_ref):
                    target_dir = label
                    break

            if target_dir is None:
                self.get_logger().warn(f"Invalid step from {current} to {next_point}")
                continue

            print(f"\nNext cell: {next_point}")
            print(f"Current direction: {self.current_dir}, Target direction: {target_dir}")

            # Rotate robot if needed to face the new direction
            angle = self.angle_between(self.current_dir, target_dir)
            if angle != 0.0:
                self.rotate(angle)

            # Move forward one grid cell (15 cm)
            self.move_forward(CELL_SIZE)

            # Update direction and current position
            self.current_dir = target_dir
            current = next_point


def main(args=None):
    # Node startup
    rclpy.init(args=args)
    navigator = MazeNavigator()
    rclpy.spin(navigator)

    # Clean up resources
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()