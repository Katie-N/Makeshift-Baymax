import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from math import ceil

# Instead of creating a subscription to wall_distances we will use multiprocessing 
# to allow us to access the global variable from detectWalls when WE want to
# (NOT every single time it publishes)
import time

# Originally Defence.py
class mapWalls(Node):
    def __init__(self, shared_wall_distances):
        super().__init__('wall_avoider')
        self.cell_size = 0.15  # Size of each cell in the grid (in meters)
        # Fixed-size grid
        self.widthInMeters = 1.8288 # 6 feet
        self.heightInMeters = 1.524 # 5 feet
        self.grid_size = (ceil(self.widthInMeters/self.cell_size), ceil(self.heightInMeters/self.cell_size))  # (x, y) dimensions
        # Initialize the grid with -1 (unmapped)
        self.grid = [[-1 for i in range(self.grid_size[0])] for i in range(self.grid_size[1])]
        self.robot_direction = 0  # 0: up, 1: right, 2: down, 3: left

        self.robot_pos = [self.grid_size[0] // 2,self.grid_size[1] // 2] # Starting position around the center
        self.shared_wall_distances = shared_wall_distances

        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10) #Movement Commands

        # Start the controller node to handle all of the logic and functions
        self.controller()

    # The printed grid will have the orientation:
    #        back
    #   right     left
    #        front
    # 
    def print_grid(self):
        for row in self.grid:
            rowString = ''
            for cell in row:
                if cell == 0: rowString += '.'
                elif cell == 1: rowString += '#'
                else: rowString += " "
            print(rowString)

    def mapNearestWalls(self):        
        fdist = self.shared_wall_distances[0]
        rdist = self.shared_wall_distances[1]
        bdist = self.shared_wall_distances[2]
        ldist = self.shared_wall_distances[3]
        print(f"{fdist}, {rdist}, {bdist}, {ldist}")

        linearSpeed = 0.6
        goForward = linearSpeed
        goBack = -linearSpeed
        goRight = -linearSpeed
        goLeft = linearSpeed

        # Check the nearby cells so we don't try to overwrite them
        cellOnLeft = self.grid[self.robot_pos[0]][self.robot_pos[1]+1]
        cellInBack = self.grid[self.robot_pos[0]-1][self.robot_pos[1]]
        cellOnRight = self.grid[self.robot_pos[0]][self.robot_pos[1]-1]
        cellInFront = self.grid[self.robot_pos[0]+1][self.robot_pos[1]]

        distToNextCell = self.cell_size
        print(f"Cells frbl: {cellInFront} {cellOnRight} {cellInBack} {cellOnLeft}")

        # If the cell on the left is unmapped
        if cellOnLeft == -1: 
            # If the closest wall on the left is less than the safe distance
            if ldist < distToNextCell:
                # There is a wall in the cell on the left
                self.grid[self.robot_pos[0]][self.robot_pos[1]+1] = 1
            else:
                # Mark cell on the left as empty
                self.grid[self.robot_pos[0]][self.robot_pos[1]+1] = 0                
                # # Otherwise the closest wall in front is far enough away to map some empty cells.
                # # Loop through the distance to the wall in the front and mark each cell as empty
                # for i in range(int(ldist // distToNextCell)):
                #     # Map the cell in front to be empty
                #     self.grid[self.robot_pos[0]][self.robot_pos[1]+1+i] = 0
                # # Finish by mapping the last cell to be a wall (because we looped through all the empty cells already)
                # if self.grid[self.robot_pos[0]][self.robot_pos[1]+1+int(ldist // distToNextCell)] == -1:
                #     self.grid[self.robot_pos[0]][self.robot_pos[1]+1+int(ldist // distToNextCell)] = 1
        
        
        # if the cell in the back is unmapped
        if cellInBack == -1:
            print("Undefined right cell")
            if bdist < distToNextCell:
                print("MARK AS WALL")
                # There is a wall in the cell behind
                self.grid[self.robot_pos[0]-1][self.robot_pos[1]] = 1
            else:
                # Mark cell immediately behind as empty
                self.grid[self.robot_pos[0]-1][self.robot_pos[1]] = 0
                # # There is NOT a wall in the cell on the right
                # for i in range(int(bdist // distToNextCell)):
                #     # Map the cells on the right to be empty
                #     self.grid[self.robot_pos[0]-1-i][self.robot_pos[1]] = 0
                # # Finish by mapping the last cell to be a wall (because we looped through all the empty cells already)
                # # if self.grid[self.robot_pos[0]-1-int(bdist // distToNextCell)][self.robot_pos[1]] == -1:
                # #     self.grid[self.robot_pos[0]-1-int(bdist // distToNextCell)][self.robot_pos[1]] = 1
        
        # if the cell on the right is unmapped
        if cellOnRight == -1:
            if rdist < distToNextCell:
                # There is a wall in the cell on the right
                self.grid[self.robot_pos[0]][self.robot_pos[1]-1] = 1
            else:
                # Mark cell to the right as empty
                self.grid[self.robot_pos[0]][self.robot_pos[1]-1] = 0
                # for i in range(int(rdist // distToNextCell)):
                #     # Map the cells in the back to be empty
                #     self.grid[self.robot_pos[0]][self.robot_pos[1]-1-i] = 0
                # # Finish by mapping the last cell to be a wall (because we looped through all the empty cells already)
                # if self.grid[self.robot_pos[0]][self.robot_pos[1]-1-int(rdist // distToNextCell)] == -1:
                #     self.grid[self.robot_pos[0]][self.robot_pos[1]-1-int(rdist // distToNextCell)] = 1
        
        # if the cell in front is unmapped
        if cellInFront == -1:
            if fdist < distToNextCell:
                # There is a wall in the cell in front
                self.grid[self.robot_pos[0]+1][self.robot_pos[1]] = 1
            else:
                # Mark cell in front as empty
                self.grid[self.robot_pos[0]+1][self.robot_pos[1]] = 0
                # for i in range(int(fdist // distToNextCell)):
                #     # Map the cells on the left to be empty
                #     self.grid[self.robot_pos[0]+1+i][self.robot_pos[1]] = 0
                # # Finish by mapping the last cell to be a wall (because we looped through all the empty cells already)
                # # if self.grid[self.robot_pos[0]+1+int(fdist // distToNextCell)][self.robot_pos[1]] == -1:
                # #     self.grid[self.robot_pos[0]+1+int(fdist // distToNextCell)][self.robot_pos[1]] = 1
        self.print_grid()

    
    def stop_moving(self):
        """Stops the robot's movement."""
        print("stop_moving()")

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.5)  # Give time to stop
    
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
                # AFTER MOVING WE HAVE TO UPDATE THE GRID!!
                self.mapNearestWalls()
                if self.goal_pos:  # Stop if goal is reached
                    break
            self.turn_left()
        
        self.get_logger().info("Exploration complete!")
    
    # Move forward by 1 cell
    # Going this speed for 0.5 seconds should result in traveling 0.15 meters.
    # Figure out what speed to use.
    def move_forward(self):
        """Moves the robot forward one cell."""
        print("move_forward()")

        twist = Twist()
        twist.linear.x = 0.1 
        self.cmd_pub.publish(twist)
        time.sleep(1)  # Give time to move
        self.stop_moving()
        self.update_position()
        

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
        """Checks if the front is clear BASED ON THE GRID. """
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
        """Checks if there is a wall to the right of the robot BASED ON THE GRID"""
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
            self.robot_pos[0] += 1
            return # (self.robot_pos[0] + 1, self.robot_pos[1])  # Move right
        elif self.robot_direction == 1:
            print("Moving down")
            self.robot_pos[1] += 1
            return # (self.robot_pos[0], self.robot_pos[1] + 1)  # Move down
        elif self.robot_direction == 2:
            print("Moving left")
            self.robot_pos[0] -= 1
            return # (self.robot_pos[0] - 1, self.robot_pos[1])  # Move left
        elif self.robot_direction == 3:
            print("Moving up")
            self.robot_pos[1] -= 1
            return # (self.robot_pos[0], self.robot_pos[1] - 1)  # Move up
    
    def controller(self):
        # The position of the robot is only updated after move_forward(). 
        # So we do not need to call it here.
        # Just rely on all of the linear movement functions to update the position after running!
        # self.update_position()

        # Then we can map the walls near the robot using the lidar data
        while(not self.shared_wall_distances):
            print("Not ready yet")
            time.sleep(0.5)


        self.mapNearestWalls()

        # This just maps the walls and moves forward repeatedly.
        # It is only here to show how to update the grid after movement. 
        # In the future instead of move_forward it will be following the wall and updating the map every second or so.
        for i in range(4):
            self.move_forward()
            self.mapNearestWalls()

        # self.turn_right()
        # self.move_forward()
        # self.mapNearestWalls()

def main(shared_wall_distances):
    rclpy.init()
    node = mapWalls(shared_wall_distances)
    rclpy.spin_once(node) # We only want the node to spin once because we want it to map the walls once.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()