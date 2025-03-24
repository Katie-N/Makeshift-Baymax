import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Sumo_Walls.detectWalls import WallTracker
from std_msgs.msg import Float32MultiArray

class DefenceWalls(Node):
    def __init__(self):
        super().__init__('wall_avoider')
        self.detectWalls = WallTracker()
        # self.timer = self.create_timer(0.1, self.avoidWalls) #Checks distances and changes movement every 0.1s to avoid walls
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) #Movement Commands

        self.wall_distances = self.create_subscription(
            Float32MultiArray,
            '/wall_distances',
            self.avoidWalls,
            10
        )

    def avoidWalls(self, msg):
        fdist = msg.data[0]
        rdist = msg.data[1]
        bdist = msg.data[2]
        ldist = msg.data[3]

        move = Twist()

        safe = 0.20
        turnSpeed = 0.5
        forwardSpeed = 0.2
        # print(f"f={fdist} r={rdist} l={ldist} b={bdist}")
        if fdist < safe:
            print("Too close! 2")
            move.linear.x = 0.0
            if rdist > ldist:
                print("Turning Right 2")
                move.angular.z = -turnSpeed #turns right
            else:
                print("Turning Left 2")
                move.angular.z = turnSpeed #turns left
        else:
            print("Moving forward")
            print(f"{fdist}, {rdist}, {ldist}")
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