# This is all pseudocode for now
import detectOpponent.py # Katie
import detectWalls.py # Elizabeth
import attackOpponent.py
import defence.py
import time

def main():
    while True:
        # Poll the positions of the walls and of the opponent on a regular interval.
        sleep 0.5
        time.sleep(0.5)
        
        # Start publishing wall positions by starting detectWalls node
        # Start publishing opponent positions by starting detectOpponent node
        # Start defence node
        # Start attackOpponent node
        opponentPosition = detectOpponent()
        closest2WallsPositions = detectWalls()

        # Top priority is always avoiding the walls
        if (closest2WallsPositions[0].distance < opponentPosition.distance 
        or closest2WallsPositions[1].distance < opponentPosition.distance):
            avoidWalls(closest2WallsPositions)
        else: # If the opponent is closer than the walls, we focus on them
            attackOpponent() or avoidOpponent()
            # Behavior Strategies:
                # Maybe half the time the robot attacks the other half it avoids? Pure randomness would be hard for the other team to predict.
                # Maybe we have multiple modes for the robot and we pick the mode to use based on the opponents behavior in other matches:
                    # 1. Aggressive mode. Used when the other robot does a lot of avoiding. Maybe this mode is good at charging directly at the other robot.
                    # 2. Defensive mode. Used when the other robot does a lot of charging. Maybe this mode is really good at dodging quickly.