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
from geometry_msgs.msg import Point
from sdk.common import val_map
import time

class ColorTracking(Node):
    def __init__(self, name):

        # Basic node setup
        super().__init__(name)
        
        self.enemy_coords_publisher = self.create_publisher(Point, '/ball_position', 10)
        
        self.subscription = self.create_subscription(
            Image,
            'ascamera/camera_publisher/rgb0/image',
            self.listener_callback,
            10)


        self.bridge = CvBridge()

        # -- Matplotlib Setup for debugging --
        
        # Comment this out for final implementation, as it will be much more performant
        plt.ion()  # Enable interactive mode
        self.fig, self.ax = plt.subplots()
        # ------------------------------------


    def listener_callback(self, data):
        # Convert ROS2 Image msg type to OpenCV img
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Optional: Add blurring step to remove noise from image
        gaussian = cv2.GaussianBlur(current_frame, (15, 15), 0)

        # # Uncomment to see the masked image for debugging
        self.ax.clear()
        self.ax.imshow(gaussian)
        self.ax.set_title("Real-time Image")
        plt.draw()
        plt.pause(0.001)  # A brief pause so the figure actually updates

        height, width, channels = gaussian.shape # For color images
        # print(f"{width} x {height}")

        # This extracts the red ball from the robot's field of view
        lower_bound = np.array([33, 47, 153])  # BGR format
        upper_bound = np.array([111, 153, 253])  # BGR format
        red_mask = cv2.inRange(gaussian, lower_bound, upper_bound)

        # Get the color's centroid x and y components ( we compute this in a custom function below )
        centroid_x, centroid_y = self.get_color_centroid( red_mask )

        # publish Point message to attack opponent node to quickly send coords
        msg = Point()
        msg.x, msg.y = centroid_x, centroid_y
        msg.z = 0.0  

        self.enemy_coords_publisher.publish(msg)
        # Our camera's resolution is 640x480 
        
    # helper function to determine central point of color
    # these coords represent the middle of the enemy as we see them in the camera
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
        # NOTE: comment this out for the actual competition. for debugging only
        print(f"Coords of ball center of color: {centroid_x}, {centroid_y}")

        # centroid_y = abs(centroid_y - 480) # Invert the y pixel axis. y = 0 now means the bottom of the screen not the top
        return centroid_x, centroid_y


    # helper function for debugging
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
                

def main():
    rclpy.init()
    color_tracking_node = ColorTracking('color_tracking_node')
    rclpy.spin(color_tracking_node)
    color_tracking_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
