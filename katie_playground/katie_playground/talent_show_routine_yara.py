import basic_buzzer
import drawAStar
import threading
import rclpy
from rclpy.node import Node

# Thread that handles singing Twinkle Twinkle
thread1 = threading.Thread(target=basic_buzzer.main)
# Thread that handles drawing the star
thread2 = threading.Thread(target=drawAStar.main)

thread1.start()
thread2.start()

thread1.join()
thread2.join()

# Since we want the star to be drawn while the song is sung, 
# we need to use threading to launch them at the same time.
# Tested using testingTiming.py