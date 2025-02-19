#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import time

from ros_robot_controller_msgs.msg import BuzzerState

# This is my python script with available songs
import songs

class BasicBuzzer(Node):
    def __init__(self, name):
        super().__init__(name)
        print("basic buzzer")
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
    
        # Use this to make sure code does not start executing before subscribers are listening
        # This prevents skipping instructions
        while self.buzzer_pub.get_subscription_count() == 0:
            self.get_logger().info("Waiting for subscribers to connect...")
            time.sleep(0.1)  # Check every 100ms

        buzzer_msg = BuzzerState()
        buzzer_msg.off_time = 0.01
        buzzer_msg.on_time = 0.25
        buzzer_msg.repeat = 1

        # Choose the song here. No other lines need to be changed
        song = songs.twinkleTwinkle

        for i in range(len(song.notes)):
            # Setup the next note to sing
            note = round(song.notes[i])
            buzzer_msg.freq = note
            buzzer_msg.on_time = song.noteDurations[i]
            
            # Sing the note
            self.buzzer_pub.publish(buzzer_msg)
            # Sleep so that the next iteration does not run until the current note has finished
            time.sleep(song.noteDurations[i] + song.timeBetween)

def main(args=None):
    rclpy.init(args=args)
    BasicBuzzerNode = BasicBuzzer('Basic_Buzzer')
    BasicBuzzerNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
