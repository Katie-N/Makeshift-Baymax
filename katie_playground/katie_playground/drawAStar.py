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
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.min_value = 0.1
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 3.0)
        self.declare_parameter('disable_servo_control', True)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        # self.disable_servo_control = self.get_parameter('disable_servo_control').value
        # self.machine = os.environ['MACHINE_TYPE']
        # self.get_logger().info('\033[1;32m%s\033[0m' % self.max_linear)
        # self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.starPath, 1)
        # self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.mecanum_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)

        # self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        # self.last_buttons = dict(zip(BUTTON_MAP, [0.0, ] * len(BUTTON_MAP)))
        # self.mode = 0
        # self.create_service(Trigger, '~/init_finish', self.get_node_state)
        # self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        # self.starPath(name)

    # def get_node_state(self, request, response):
    #     response.success = True
    #     return response

    def axes_callback(self, axes):
        lx = axes[0]
        ly = axes[1]
        duration = axes[2]
        twist = Twist()
        # if abs(axes['lx']) < self.min_value:
        #     axes['lx'] = 0
        # if abs(axes['ly']) < self.min_value:
        #     axes['ly'] = 0
        # if abs(axes['rx']) < self.min_value:
        #     axes['rx'] = 0

        twist.linear.y = val_map(lx, -1, 1, -self.max_linear, self.max_linear) 
        twist.linear.x = val_map(ly, -1, 1, -self.max_linear, self.max_linear)
        twist.angular.z = val_map(0, -1, 1, -self.max_angular, self.max_angular)
        self.mecanum_pub.publish(twist)
        time.sleep(duration) # Amount of time to draw the line
        self.stop_robot()

    def starPath(self, joy_msg):
        # axes = dict(zip(AXES_MAP, joy_msg.axes))
        # axes_changed = False
        # hat_x, hat_y = axes['hat_x'], axes['hat_y']
        # hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        # hat_yu, hat_yd = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0
        # buttons = list(joy_msg.buttons)
        # buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        # buttons = dict(zip(BUTTON_MAP, buttons))

        # axes = {
        #     # Positive to go left, negative to go right
        #     "lx": 0.25,
        #     # Positive to go up, negative to go down
        #     "ly": 0,
        #     # Rotation
        #     "rx": 0
        # }
        
        # The star has 5 line movements.
        # Each line is of the form: lx, ly (for linear x and linear y)
        # lx: Positive to go left, negative to go right
        # ly: Positive to go up, negative to go down
        scalingFactor = 0.25 # To slow the robot down, we can adjust this scaling factor without affecting the actual path taken.
        duration = 1 # FIXME: This is the time to draw the lines. This needs to be calculated for each line.
        path=[
            [-scalingFactor, scalingFactor * 3.07768, duration], # Slower going right than going up
            [0.32492 * -scalingFactor, -scalingFactor, duration], # Slower going right than going backward
            [scalingFactor, scalingFactor * 0.726543, duration], # Slower going up than going left
            [-scalingFactor, 0, duration], # Straight right
            [scalingFactor * 1.37638, -scalingFactor, duration], # Slower to go backwards than left            
        ]

        # for key, value in axes.items(): 
        #     if self.last_axes[key] != value:
        #         axes_changed = True

        for line in path:
            try:
                self.axes_callback(line)
            except Exception as e:
                self.get_logger().error(str(e))
        self.stop_robot()

        # if axes_changed:
        #     try:
        #         self.axes_callback(axes)
        #     except Exception as e:
        #         self.get_logger().error(str(e))
        # for key, value in buttons.items():
        #     if value != self.last_buttons[key]:
        #         new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
        #     else:
        #         new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
        #     callback = "".join([key, '_callback'])
        #     if new_state != ButtonState.Normal:
        #         self.get_logger().info(str(new_state))
        #         if  hasattr(self, callback):
        #             try:
        #                 getattr(self, callback)(new_state)
        #             except Exception as e:
        #                 self.get_logger().error(str(e))
        # self.last_buttons = buttons
        # self.last_axes = axes

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.mecanum_pub.publish(msg)

def main():
    myStar = Star('makeAStar')
    rclpy.spin(myStar)
    myStar.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()