import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from Sumo_Walls.detectWalls import WallTracker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class DefenceWalls(Node):
    def __init__(self):
        super().__init__('wall_avoider')
        self.attack = False
        self.defend = False

        # self.detectWalls = WallTracker()
        # self.timer = self.create_timer(0.1, self.avoidWalls) #Checks distances and changes movement every 0.1s to avoid walls
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) #Movement Commands

        self.wall_distances = self.create_subscription(
            Float32MultiArray,
            '/wall_distances',
            self.avoidWalls,
            10
        )

        # Subscribe to the sumo wrestling master topic to get instructions.
        self.master_controller_subscription = self.create_subscription(
            Point, 
            '/master_controller',
            self.checkInstruction,
            10
            )

    def checkInstruction(self, msg):
        if msg.x == 1.0:
            self.attack = True
            self.defend = False

        if msg.y == 1.0:
            self.attack = False
            self.defend = True
        # Only msg.x or msg.y should be 1 at a time. The other needs to be 0. 
        

    def avoidWalls(self, msg):

        if not self.defend:
            return
        
        move = Twist()
        move.linear.x = 0.0
        move.linear.y = 0.0
        move.linear.z = 0.0
        move.angular.x = 0.0
        move.angular.y = 0.0
        move.angular.z = 0.0
        self.cmd_pub.publish(move)
        return
        

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