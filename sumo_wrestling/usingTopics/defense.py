import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class DefenceWalls(Node):
    def __init__(self):
        super().__init__('wall_avoider')
        self.attack = False
        self.defend = False

        self.cmd_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10) #Movement Commands

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

        fdist = msg.data[0]
        rdist = msg.data[1]
        bdist = msg.data[2]
        ldist = msg.data[3]

        twist = Twist()

        safe = 0.25
        frontSafe = 0.25
        
        turnSpeed = 1.0
        turnLeft = turnSpeed
        turnRight = -turnSpeed
        linearSpeed = 0.6
        goForward = linearSpeed
        goBack = -linearSpeed
        goRight = -linearSpeed
        goLeft = linearSpeed

        twist.angular.z = 0.0

        if (fdist <= frontSafe):
            if (rdist<=safe):
                # move backwards and left
                twist.linear.x = goBack
                twist.linear.y = goLeft
                # twist.angular.z = turnLeft
                # print("back left")
            elif (ldist <= safe):
                # move backwards and right
                twist.linear.x = goBack
                twist.linear.y = goRight
                # twist.angular.z = turnRight
                # print("back right")
            else:
                # move backwards
                twist.linear.x = goBack
                twist.linear.y = 0.0
                # twist.angular.z = 0.0
                # print("back")
        elif (bdist <= frontSafe):
            if (rdist<=safe):
                # move forward and left
                twist.linear.x = goForward
                twist.linear.y = goLeft
                # twist.angular.z = turnLeft
                # print("forward left")

            elif (ldist <= safe):
                # move forward and right
                twist.linear.x = goForward
                twist.linear.y = goRight
                # twist.linear.y = turnRight
                # print("forward right")

            else:
                # Move forward
                twist.linear.x = goForward
                twist.linear.y = 0.0
                # twist.angular.z = 0.0
                # print("forward")

        elif(rdist <= safe):
            # Move left
            # twist.angular.z = turnLeft
            twist.linear.x = 0.0
            twist.linear.y = goLeft
            # print("left")


        elif(ldist <= safe):
            # move right
            # twist.angular.z = turnRight
            twist.linear.x = 0.0            
            twist.linear.y = goRight
            # print("right")

        else:
            # stop
            twist.linear.x = 0.0            
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            # print("stop")

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DefenceWalls()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()