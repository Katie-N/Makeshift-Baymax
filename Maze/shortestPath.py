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
            (-1, -1), (-1, 1), (1, -1), (1, 1)    # diagonal
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
    
    def solve(self):
        path = self.spa()
        if path:
            self.visited(path)
            self.saveMaze()
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
        