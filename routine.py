import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class RoutineController(Node):
    def __init__(self):
        super().__init__('routine_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.run_routine()

    def publish_velocity(self, linear_x, angular_z, duration):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        time.sleep(duration)
        self.stop_robot()

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def run_routine(self):
        self.get_logger().info("Starting routine...")

        self.publish_velocity(0.2, 0.0, 2)  # Move forward for 2 seconds
        self.publish_velocity(0.0, 1.0, 1)  # Turn right for 1 second
        self.publish_velocity(0.2, 0.0, 2)  # Move forward for 2 seconds
        self.publish_velocity(0.0, -1.0, 1) # Turn left for 1 second
        self.publish_velocity(0.2, 0.0, 2)  # Move forward for 2 seconds

        self.get_logger().info("Routine complete.")
        rclpy.shutdown()

def main():
    rclpy.init()
    RoutineController()
    rclpy.spin(RoutineController())

if __name__ == '__main__':
    main()
