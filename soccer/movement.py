#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityController(Node):
    def __init__(self, linear_speed, angular_speed):
        super().__init__('velocity_controller')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        self.get_logger().info(f'Node started with linear={linear_speed}, angular={angular_speed}')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)

    # Set your speed values here
    linear_speed = 0.5   # meters per second
    angular_speed = 0.0  # radians per second

    node = VelocityController(linear_speed, angular_speed)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
