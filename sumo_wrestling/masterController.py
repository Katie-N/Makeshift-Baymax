import detectOpponent.py # Katie
import detectWalls.py # Elizabeth
import attackOpponent.py
import defence.py
import time

from Sumo_Walls.detectWalls import WallTracker
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class masterController(Node):
    def __init__(self):        
        super().__init__('sumo_wrestling_controller')
        self.detectWalls = WallTracker()

        self.wall_distances = self.create_subscription(
            Float32MultiArray,
            '/wall_distances',
            self.newWallData,
            10
        )
        
        # Subscriber for opponent position (expects [x, y] coordinates)
        # assuming this will only receive data every 5 seconds (or similar) and not more
        self.enemy_coords_subscription = self.create_subscription(
            Point,
            '/opponent_position',
            self.newOpponentData,
            10
        )
        self.wallDistances
    
    def newWallData(self, msg):
        self.wallDistances = msg

    def newOpponentData(self, msg):
        self.opponentDistances = msg

    def controlRobot(self):
        print(self.wallDistances)
        print(self.opponentDistances)


def main():
    masterControllerNode = masterController('Master-Controller-Node')
    try:
        rclpy.spin(masterControllerNode)
    except:
        masterControllerNode.destroy_node()
        rclpy.shutdown()
        
# def main():
#     while True:
#         # Poll the positions of the walls and of the opponent on a regular interval.
#         sleep 0.5
        
#         # Start publishing wall positions by starting detectWalls node
#         # Start publishing opponent positions by starting detectOpponent node
#         # Start defence node
#         # Start attackOpponent node
#         opponentPosition = detectOpponent()
#         closest2WallsPositions = detectWalls()

#         # Top priority is always avoiding the walls
#         if (closest2WallsPositions[0].distance < opponentPosition.distance 
#         or closest2WallsPositions[1].distance < opponentPosition.distance):
#             avoidWalls(closest2WallsPositions)
#         else: # If the opponent is closer than the walls, we focus on them
#             attackOpponent() or avoidOpponent()
#             # Behavior Strategies:
#                 # Maybe half the time the robot attacks the other half it avoids? Pure randomness would be hard for the other team to predict.
#                 # Maybe we have multiple modes for the robot and we pick the mode to use based on the opponents behavior in other matches:
#                     # 1. Aggressive mode. Used when the other robot does a lot of avoiding. Maybe this mode is good at charging directly at the other robot.
#                     # 2. Defensive mode. Used when the other robot does a lot of charging. Maybe this mode is really good at dodging quickly.