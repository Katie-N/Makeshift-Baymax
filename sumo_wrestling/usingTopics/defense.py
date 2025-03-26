import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

# from Sumo_Walls.detectWalls import WallTracker
from std_msgs.msg import Float32MultiArray

class DefenceWalls(Node):
    def __init__(self):
        super().__init__('wall_avoider')
        self.attack = False
        self.defend = False

        # self.detectWalls = WallTracker()
        # self.timer = self.create_timer(0.1, self.avoidWalls) #Checks distances and changes movement every 0.1s to avoid walls
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
        # frontSafe = 0.5
        turnSpeed = 1.5
        forwardSpeed = 0.6

        if (fdist <= safe):
            if (rdist<=safe):
                # move backwards and left
            elif (ldist <= safe)
                # move backwards and right
            else:
                # move backwards
        elif (bdist <= safe):
            if (rdist<=safe):
                # move forward and left
            elif (ldist <= safe)
                # move forward and right
            else:
                # move forward'
        elif( rdist <= safe):
            # Move left
        elif(ldist <= safe):
            # move right
        else:
            # stop


        # if fdist <= safe:
        #     # drive backwards
        #     print("going backward")
        #     twist.linear.x = 0
        #     twist.angular.z = 0
        #     # twist.linear.x = -0.6
        # elif bdist <= safe:
        #     # drive forward
        #     print("going forward")
        #     twist.linear.x = 0.6
        # elif rdist <= safe:
        #     # turn left
        #     print("going left")
        #     twist.angular.z = 2.0 
        #     twist.linear.x = 0.6 
        # elif ldist <= safe:
        #     # turn right
        #     print("going right")
        #     twist.angular.z = -2.0
        #     twist.linear.x = 0.6
        
        
        # # print(f"f={fdist} r={rdist} l={ldist} b={bdist}")
        # if fdist < safe: #Too close in the front
        #     print("Too close! 2")
        #     move.linear.x = 0.0
        #     if rdist > ldist:
        #         print("Turning Right 2")
        #         move.angular.z = -turnSpeed #turns right
        #     else:
        #         print("Turning Left 2")
        #         move.angular.z = turnSpeed #turns left
        # elif rdist < safe: #Too close on the right
        #     print("On the right!")
        #     move.angular.z = turnSpeed
        # elif ldist < safe: #Too close on the left
        #     print("On the left!")
        #     move.angular.z = -turnSpeed
        # elif bdist < safe: #Too close on in the back
        #     print("Behind you!")
        #     move.linear.x = 0.0
        #     if rdist > ldist:
        #         print("Turning Right 2")
        #         move.angular.z = -turnSpeed #turns right
        #     else:
        #         print("Turning Left 2")
        #         move.angular.z = turnSpeed #turns left
        # else:
        #     print("All good!")
        #     print(f"{fdist}, {rdist}, {ldist}")
        #     #move.linear.x = forwardSpeed
        #     #I don't want it to move forward anymore because that's the attack's job to move forward.
        #     move.angular.z = 0.0       
        
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DefenceWalls()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()