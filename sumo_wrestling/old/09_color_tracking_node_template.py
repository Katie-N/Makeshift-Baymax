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


    def listener_callback(self, data):
        # DONE: Convert ROS2 Image msg type to OpenCV img
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # NOTE: Call this function to display input image for debugging
        self.display_with_matplotlib( current_frame )

        # DONE? Convert BGR to LAB for easier processing
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2LAB)

        # Optional: Add blurring step to remove noise from image
        gaussian = cv2.GaussianBlur(current_frame, (15, 15), 0)
        # self.ax.clear()
        # self.ax.imshow(gaussian)
        # self.ax.set_title("Real-time Image")
        # plt.draw()
        # plt.pause(0.1)  # A brief pause so the figure actually updates

        # TODO Define range of color in LAB with lower and upper thresholds

        # TODO Create a binary mask of the selected color range
        # Apply the mask to the original image (we techincally only need this for our viewing purposes)

        # Get the color's centroid x and y components ( we compute this in a custom function below )
        # centroid_x, centroid_y = self.get_color_centroid( <mask var> )
        
        # TODO Move depending on centroid location

        # This section shuold end with something like 
        # self.cmd_vel.publish(twist)        
    
    def get_color_centroid(self, mask):
        """
        Return the centroid of the largest contour in the binary image 'mask'
        """ 

        # TODO Set a minimum area required for contours to be considered (value around 100 - 200 might be a good start)

        # TODO Get a list of contours and compute the centroid location of the largest contour
        # NOTE: Be sure to check that there are any contours in the mask at all!
        
        # Function should end with something like
        # return centroid_x, centroid_y
    


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
    finally:
        color_tracking_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()