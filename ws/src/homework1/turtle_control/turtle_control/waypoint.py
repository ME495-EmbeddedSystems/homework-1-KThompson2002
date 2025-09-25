from enum import Enum, auto
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim_msgs.srv import Reset, Spawn

class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    MOVING = auto(),
    STOPPED = auto(),

class Waypoint(Node):
    def __init__(self):
        super().__init__("waypoint")
        self.freq = 90 # hz
        self.timer = self.create_timer(1/self.freq, self.timer_callback)
        self.toggle = self.create_service(Empty, "empty",  self.toggle_callback)
        self.load = self.create_service(Waypoints, "waypoints", self.load_callback)
        self.state = State.STOPPED
        self.reset = self.create_client(Reset,"reset")

    def timer_callback(self):
        if self.state == State.MOVING:
            msg = f"moving"
            self.get_logger().debug(msg)

    def toggle_callback(self, request, response):
        if self.state == State.MOVING:
            self.get_logger().info("Stopping")
            self.state = State.STOPPED
        else:
            self.state == State.MOVING
        return response

    def load_callback(self, request, response):
        self.get_logger().info("Resetting and adding waypoints")
        self.reset_future = self.reset.call_async(Reset.Request(name="turtle1"))




def main(args=None):
    rclpy.init(args=args)
    node = Waypoint()
    rclpy.spin(node) 
    node.shutdown()

