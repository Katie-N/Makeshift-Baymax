#This is meant to help the robot find the shortest path possible.
#import rclpy
#from rclpy.node import Node
from collections import deque

class ShortestPath:
    def __init__(self, mapFile, start, stop, finalMap = "finalMap.txt"):
        self.mapFile = mapFile
        self.start = start
        self.stop = stop
        self.finalMap = finalMap
        self.maze = self.loadMaze()

    def loadMaze(self):
        with open(self.mapFile, "r") as f:
            return [[int(cell) for cell in line.strip().split()] for line in f.readlines()]

    def visited(self, path):
        for x, y in path[1:-1]:
            self.maze[y][x] = "*"  
  

    def spa(self):
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),     # cardinal
            #(-1, -1), (-1, 1), (1, -1), (1, 1)    # diagonal
        ]

        queue = deque([(self.start, [self.start])])
        visit = set([self.start])

        while queue:
            (x, y), path = queue.popleft()

            if (x, y) == self.stop:
                return path

            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if (0 <= ny < len(self.maze) and 0 <= nx < len(self.maze[0]) and 
                    self.maze[ny][nx] == 0 and (nx, ny) not in visit):
                    visit.add((nx, ny))
                    queue.append(((nx, ny), path + [(nx, ny)]))
        return None
    
    def saveMaze(self):
        with open(self.finalMap, 'w') as f:
            for row in self.maze:
                f.write(' '.join(str(cell) for cell in row) + '\n')

    def pathToInstructions(self, path, initial_facing="E"):
        DIRECTIONS = {
        (1, 0): "E",   # right
        (0, -1): "N",  # up
        (-1, 0): "W",  # left
        (0, 1): "S"    # down
        }

        ROTATION = {
            ("N", "N"): "forward",
            ("N", "E"): "right",
            ("N", "S"): "backward",
            ("N", "W"): "left",
            
            ("E", "E"): "forward",
            ("E", "S"): "right",
            ("E", "W"): "backward",
            ("E", "N"): "left",

            ("S", "S"): "forward",
            ("S", "W"): "right",
            ("S", "N"): "backward",
            ("S", "E"): "left",

            ("W", "W"): "forward",
            ("W", "N"): "right",
            ("W", "E"): "backward",
            ("W", "S"): "left",
        }

        if not path or len(path) < 2:
            return []

        instructions = []
        current_facing = initial_facing

        i = 1
        while i < len(path):
            x0, y0 = path[i - 1]
            x1, y1 = path[i]
            dx, dy = x1 - x0, y1 - y0

            new_dir = DIRECTIONS[(dx, dy)]
            move = ROTATION[(current_facing, new_dir)]

            if move == "forward":
                instructions.append("forward")
                i += 1  # move to the next step
            else:
                instructions.append(move)
                current_facing = new_dir

        return instructions
    
    def saveInstructions(self, instructions, filename="instructions.txt"):
        with open(filename, "w") as f:
            for step in instructions:
                f.write(step + "\n")
    
    def solve(self):
        path = self.spa()
        if path:
            self.visited(path)
            self.saveMaze()
            instructions = self.pathToInstructions(path)
            self.saveInstructions(instructions)
            print("Instructions:", instructions)
            return path
        else:
            return None
        
    def print_maze(self):
        for row in self.maze:
            print(' '.join(str(cell) for cell in row))

        
if __name__ == "__main__":
    solver = ShortestPath("map.txt", start=(0, 0), stop=(9, 11))
    path = solver.solve()
    solver.print_maze()
        