import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import subprocess

"""
Master class for running all processes
Determine if robot should be in attack or defense mode
"""
#for now I am using wall distance as the parameter
#but this should be fine tuned to whatever criteria we decide

class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')

        #subscribe to the topic publishing wall distances from detectWalls.py
        self.subscription = self.create_subscription(
            Float32MultiArray,  #type
            '/wall_distances',  #topic name
            self.wall_callback, #callback function to process data
            10)  #queue size
        
        #variables to track active processes (attack or defense)
        self.attack_process = None
        self.defense_process = None

        #distance threshold to trigger defense mode (in meters)
        self.wall_threshold = 0.5  # Adjust as needed

    def wall_callback(self, msg):
        """
        Callback function triggered when new wall distance data is received.
        Determines whether to switch to attack or defense mode.
        """
        #extract wall distances from the message
        front_dist, right_dist, back_dist, left_dist = msg.data

        #print distances for debugging
        self.get_logger().info(f"Front: {front_dist:.2f}m, Right: {right_dist:.2f}m, Back: {back_dist:.2f}m, Left: {left_dist:.2f}m")

        #check if any wall is too close (less than threshold)
        if front_dist < self.wall_threshold or right_dist < self.wall_threshold or left_dist < self.wall_threshold:
            self.switch_to_defense()
        else:
            self.switch_to_attack()

    def switch_to_attack(self):
        """
        Switches the robot to attack mode by stopping defense mode (if running)
        and starting the attackOpponent.py script.
        """
        #stop defense mode if it's running
        if self.defense_process:
            self.get_logger().info("Stopping defense mode")
            self.defense_process.terminate()
            self.defense_process = None

        #start attack mode if it's not already running
        if not self.attack_process:
            self.get_logger().info("Starting attack mode")
            self.attack_process = subprocess.Popen(["python3", "attackOpponent.py"])

    def switch_to_defense(self):
        """
        Switches the robot to defense mode by stopping attack mode (if running)
        and starting the defense.py script.
        """
        #stop attack mode if it's running
        if self.attack_process:
            self.get_logger().info("Stopping attack mode")
            self.attack_process.terminate()
            self.attack_process = None

        #start defense mode if it's not already running
        if not self.defense_process:
            self.get_logger().info("Starting defense mode")
            self.defense_process = subprocess.Popen(["python3", "defense.py"])

def main(args=None):
    rclpy.init(args=args)  
    node = MasterController()  
    rclpy.spin(node)  #
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':
    main()