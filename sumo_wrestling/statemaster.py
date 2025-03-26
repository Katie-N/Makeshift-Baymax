import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Bool

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        #publisher for the state
        self.state_pub = self.create_publisher(String, '/state', 10)

        #subscribers for wall distances and orange detection
        self.wall_sub = self.create_subscription(Float32MultiArray, '/wall_distances', self.wall_callback, 10)
        self.orange_sub = self.create_subscription(Bool, '/orange_detected', self.orange_callback, 10)

        #state Variables
        self.current_state = "START"
        self.wall_threshold = 0.20  #minimum distance from the wall , can change
        self.orange_detected = False
        self.wall_distance = None  #will store the front wall distance

        #timer to publish state regularly
        self.timer = self.create_timer(0.5, self.update_state)

    def wall_callback(self, msg):
        """ Updates the wall distance from lidar data. """
        self.wall_distance = msg.data[0]  #front wall distance

    def orange_callback(self, msg):
        """ Updates whether orange is detected. """
        self.orange_detected = msg.data

    def update_state(self):
        """ Updates the robot's state based on conditions. """
        new_state = self.current_state

        #in start: if in start, goes into attack if wall is not in threshold
        #otherwise goes to defense
        if self.current_state == "START":
            if self.wall_distance is not None:
                if self.wall_distance < self.wall_threshold:
                    new_state = "DEFENSE"
                else:
                    new_state = "ATTACK"
        
        #in defense: if wall is not within threshold, goes to attack
        #stays in defense otherwise
        elif self.current_state == "DEFENSE":
            if self.wall_distance is not None and self.wall_distance > self.wall_threshold:
                new_state = "ATTACK"

        #in attack: if wall is in threshold goes to defense
        #if no orange seen, spin
        elif self.current_state == "ATTACK":
            if self.wall_distance is not None and self.wall_distance < self.wall_threshold:
                new_state = "DEFENSE"
            elif not self.orange_detected:
                new_state = "SPIN"

        #in spin: if wall outside threshold and orange is seen, attack
        #if wall within, defense
        #if wall outside threshold and no orange, spin
        elif self.current_state == "SPIN":
            if self.wall_distance is not None:
                if self.orange_detected and self.wall_distance > self.wall_threshold:
                    new_state = "ATTACK"
                elif self.wall_distance < self.wall_threshold:
                    new_state = "DEFENSE"
                else:
                    new_state = "SPIN"  #keeps spinning if the wall is outside the threshold

        #update current state and publish
        if new_state != self.current_state:
            self.current_state = new_state
            self.publish_state()

    def publish_state(self):
        """ Publishes the current state. """
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        self.get_logger().info(f"State changed to: {self.current_state}")

def main():
    rclpy.init()
    state_manager = StateManager()
    rclpy.spin(state_manager)
    state_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
