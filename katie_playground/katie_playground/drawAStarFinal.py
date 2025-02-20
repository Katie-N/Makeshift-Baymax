#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from enum import Enum
from rclpy.node import Node
from sdk.common import val_map
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import BuzzerState
from ros_robot_controller_msgs.msg import BuzzerState, SetPWMServoState, PWMServoState

import time

class Star(Node):
    def __init__(self, name, speed):
        super().__init__(name)

        self.min_value = 0.1
        self.speed = speed
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 3.0)
        self.declare_parameter('disable_servo_control', True)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.joy_sub = self.create_subscription(Joy, '/joy', self.full_glam, 1)
        self.mecanum_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)

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

    def full_glam(self, x):
        if (self.speed == 1):
            self.starPath(0.5)
            self.starPath(1)
            self.starPath(1.5)
        else:
            self.starPath(0.25)
            self.starPath(5)
            self.starPath(1)

    def starPath(self, duration):
        # The star has 5 line movements.
        # Each line is of the form: lx, ly (for linear x and linear y)
        # lx: Positive to go left, negative to go right
        # ly: Positive to go up, negative to go down
        scalingFactor = 0.25 # To slow the robot down, we can adjust this scaling factor without affecting the actual path taken.
        duration = 1.5 # FIXME: This is the time to draw the lines. This needs to be calculated for each line.
        path=[
            [0.32492 * -scalingFactor, scalingFactor, duration], # Slower going right than going up
            [0, 0, 0.1], # Brief stop
            [0.32492 * -scalingFactor, -scalingFactor, duration], # Slower going right than going backward
            [0, 0, 0.1], # Brief stop
            [scalingFactor, 0.72654 * scalingFactor, duration], # faster going left than going up
            [0, 0, 0.1], # Brief stop
            [-scalingFactor, 0, duration], # Straight right
            [0, 0, 0.1], # Brief stop
            [scalingFactor, 0.72654 * -scalingFactor, duration], # faster going left than going backward
        ]

        for line in path:
            try:
                self.axes_callback(line)
            except Exception as e:
                self.get_logger().error(str(e))
        self.stop_robot()

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.mecanum_pub.publish(msg)

def main(speed):
    myStar = Star('makeAStar', speed)
    rclpy.spin_once(myStar)

if __name__ == "__main__":
    rclpy.init()
    main(1)