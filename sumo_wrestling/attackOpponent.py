import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray  # Assuming opponent position is published as an array

class Charge(Node):
    def __init__(self, name):
        super().__init__('robot_charger')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for opponent position (expects [x, y] coordinates)
        # assuming this will only receive data every 5 seconds (or similar) and not more
        self.enemy_coords_subscription = self.create_subscription(
            Point,
            '/opponent_position',
            self.control_loop,
            10
        )

        # Robot's max speed parameters (assumed values, adjust if necessary)
        self.declare_parameter('max_linear_speed', 1.0)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('slow_spin_speed', 0.3)  # rad/s when searching

        self.linear_speed = self.get_parameter('max_linear_speed').value
        self.angular_speed = self.get_parameter('max_angular_speed').value
        self.spin_speed = self.get_parameter('slow_spin_speed').value

        self.target_x = None
        self.target_y = None

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz control loop

    # helper function to send movement instructions to wheels
    def axes_callback(self, axes):
        lx = axes[0]
        ly = axes[1]
        duration = axes[2]
        twist = Twist()

        twist.linear.y = val_map(lx, -1, 1, -self.max_linear, self.max_linear) 
        twist.linear.x = val_map(ly, -1, 1, -self.max_linear, self.max_linear)
        twist.angular.z = val_map(0, -1, 1, -self.max_angular, self.max_angular)
        self.mecanum_pub.publish(twist)
        time.sleep(duration) # Amount of time to draw the line
        self.stop_robot()
    
    # helper function to 
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.mecanum_pub.publish(msg)

    # function called when the enemy coords are retreived
    # determines whether we should charge at the enemy or not
    # depends on whether the target is visible
    # more charges -> more points! :D
    # only charges once every 5 seconds max
    def control_loop(self):
        # if orange is not detected, the robot cannot be seen
        # spin in place to detect opponent
        twist = Twist()

        if 200 <= self.target_x <= 450:
            twist.linear.x = 1.0  # Full speed forward

        elif self.target_x > 450:
            twist.angular.z = 0.2  # Small left turn
            twist.linear.x = 0.8   # Move forward

        elif self.target_x < 200:
            twist.angular.z = -0.2  # Small right turn
            twist.linear.x = 0.8    # Move forward

        else:
            twist.angular.z = 1.0  # Slow spin
            # sleep(3)

        # Publish movement command
        self.cmd_vel_pub.publish(twist)

def main():
    charge = Charge('charge_node')
    rclpy.spin(charge)
    charge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    main()
