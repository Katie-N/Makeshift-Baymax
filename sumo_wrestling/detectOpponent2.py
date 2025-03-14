#!/usr/bin/env python3
''' This node will require the /ros_robot_controller and /ascamera to be running '''

''' To see image for debugging, use self.display_with_matplotlib(<image_var>)
    Be sure to only use one of these'''

'''--------------------------------------------------------------------------------------------------
    NOTE How this node will work: 
    1) Detect color and measure the center of it
    2) Measure how far center is from middle of frame
    3) Turn so that center of color is in center of frame, turning faster if it error is larger 
--------------------------------------------------------------------------------------------------'''

'''Fill in blocks of code noted by TODO'''

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# This is needed for movement
from sdk.common import val_map
import time

# TODO Create a proportional constant applied to speed when turning based on how far centroid is from center (set to around 0.5/100 to start)

class ColorTracking(Node):
    def __init__(self, name):

        # Basic node setup
        super().__init__(name)

        self.cmd_vel = self.create_publisher(Twist, 'controller/cmd_vel', 1)

        self.subscription = self.create_subscription(
            Image,
            'ascamera/camera_publisher/rgb0/image',
            self.listener_callback,
            10)

        # DONE: Initialize conneciton between ROS 2 and OpenCV
        self.bridge = CvBridge()

        # -- Matplotlib Setup for debugging --
        
        # Comment this out for final implementation, as it will be much more performant
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()
        # ------------------------------------


        # FROM katie_playground/drawAStarFinal2.py
        self.min_value = 0.1
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 3.0)
        self.declare_parameter('disable_servo_control', True)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        # self.joy_sub = self.create_subscription(Joy, '/joy', self.full_glam, 1)
        self.mecanum_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)


    # FROM katie_playground/drawAStarFinal2.py
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
    
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.mecanum_pub.publish(msg)
        # time.sleep(5)


    def listener_callback(self, data):
        # DONE: Convert ROS2 Image msg type to OpenCV img
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        # NOTE: Call this function to display input image for debugging
        # self.display_with_matplotlib( <image_var> )

        # DONE? Convert BGR to LAB for easier processing
        # current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2LAB)

        # Optional: Add blurring step to remove noise from image
        gaussian = cv2.GaussianBlur(current_frame, (15, 15), 0)
        self.ax.clear()
        self.ax.imshow(gaussian)
        self.ax.set_title("Real-time Image")
        plt.draw()
        plt.pause(0.001)  # A brief pause so the figure actually updates

        height, width, channels = gaussian.shape # For color images
        print(f"{width} x {height}")

        # TODO Define range of color in LAB with lower and upper thresholds
        # process image
        # This extracts the orange tape from the robot
        lower_bound = np.array([33, 47, 153])  # Lower bound for the orange (BGR)
        upper_bound = np.array([111, 153, 253]) # Upper bound for the orange (BGR)
        orange_mask = cv2.inRange(gaussian, lower_bound, upper_bound)

        # This extracts the gray color of the robot
        lower_bound = np.array([31, 24, 21])
        upper_bound = np.array([71, 71, 71])
        gray_mask = cv2.inRange(gaussian, lower_bound, upper_bound)

        # TODO Create a binary mask of the selected color range
        # Apply the mask to the original image (we techincally only need this for our viewing purposes)
        # Combine the masks using bitwise OR
        combined_mask = cv2.bitwise_or(orange_mask, gray_mask)

        # Get the color's centroid x and y components ( we compute this in a custom function below )
        centroid_x, centroid_y = self.get_color_centroid( orange_mask )
        # The camera's resolution is 


        # TODO Move depending on centroid location
        scalingFactor = 0.25 # To slow the robot down, we can adjust this scaling factor without affecting the actual path taken.
        # angle = self.cameraCoordsToAngle(centroid_x, centroid_y)
        
        # self.axes_callback([angle*scalingFactor, 0, 1]) 
        if (centroid_x < 320):
            self.axes_callback([(centroid_x / 320)*scalingFactor, abs((centroid_y / 480) - 1) * scalingFactor, 0.25])
        else:
            self.axes_callback([-(centroid_x / 320)*scalingFactor, abs((centroid_y / 480) - 1) * scalingFactor, 0.25])

          

    def cameraCoordsToAngle(x, y):
        FOV = 31.608
        # The camera has a resolution of 640x480
        # Assuming 640 / 2 = 320 is 0 degrees then we can determine the angle
        # xToAngleMap = {
        #     0: -FOV,
        #     160: -FOV / 2
        #     320: 0,
        #     480: FOV / 2
        #     640: FOV
        # }
        if (x < 320):
            return -(x / 320) * FOV
        else:
            return (320 % x) / 320 * FOV
        # x > 340 is positive degree
        # x < 340 is negative degrees

        # f(x) =
        #     -(x / 320) * FOV          : x < 320
        #     (320 % x) / 320 * FOV     : x > 320

    def get_color_centroid(self, mask):
        """
        Return the centroid of the largest contour in the binary image 'mask'
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        maxX = maxY = maxW = maxH = largestContourWidth = 0
        
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if (w > largestContourWidth): # This will get rid of the noise anyways. No need for a second check!
                largestContourWidth = w
                maxX = x
                maxY = y
                maxW = w
                maxH = h
        
        centroid_x = maxX + maxW/2
        centroid_y = maxY + maxH/2
        print("Coords of largest center of color" + str(centroid_x) + " " + str(centroid_y))

        # TODO Set a minimum area required for contours to be considered (value around 100 - 200 might be a good start)

        # TODO Get a list of contours and compute the centroid location of the largest contour
        # NOTE: Be sure to check that there are any contours in the mask at all!
        
        # Function should end with something like
        centroid_y = abs(centroid_y - 480) # Invert the y pixel axis. y = 0 now means the bottom of the screen not the top
        return centroid_x, centroid_y

    def display_with_matplotlib(self, bgr_frame):
        """
        Display the given BGR frame in the existing matplotlib figure, in real time.
        Since only the headless OpenCV version is installed, we visualize using matplotlib
        """
        # Convert BGR -> RGB for matplotlib
        rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        self.ax.clear()
        self.ax.imshow(rgb_frame)
        self.ax.set_title("Image for Debugging")
        plt.draw()
        plt.pause(0.001)  # A brief pause so the figure actually updates

def main(args=None):
    
    rclpy.init(args=args)
    color_tracking_node = ColorTracking('color_tracking_node')
    try:
        rclpy.spin(color_tracking_node)
    except KeyboardInterrupt:
        color_tracking_node.get_logger().info("Keyboard Interrupt (Ctrl+C): Stopping node.")
        color_tracking_node.stop_robot()

    finally:
        color_tracking_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()