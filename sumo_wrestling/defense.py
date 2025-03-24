import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Sumo_Walls.detectWalls import WallTracker

class DefenceWalls(Node):
    def __init__(self):
        super().__init__('wall_avoider')
        self.detectWalls = WallTracker()
        self.timer = self.create_timer(0.1, self.avoidWalls) #Checks distances and changes movement every 0.1s to avoid walls
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) #Movement Commands

    def avoidWalls(self):
        ldist = self.detectWalls.left_dist
        rdist = self.detectWalls.right_dist
        fdist = self.detectWalls.front_dist

        move = Twist()

        safe = 0.35
        turnSpeed = 0.5
        forwardSpeed = 0.2

        if fdist < safe or fdist != float('inf'):
            print("Too close!")
            move.linear.x = 0.0
            if rdist > ldist:
                print("Turning Right")
                move.angular.z = -turnSpeed #turns right
            else:
                print("Turning Left")
                move.angular.z = turnSpeed #turns left
        else:
            print("Moving forward")
            move.linear.x = forwardSpeed
            move.angular.z = 0.0
        
        self.cmd_pub.publish(move)

def main(args=None):
    rclpy.init(args=args)
    node = DefenceWalls()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()