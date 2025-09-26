from enum import Enum, auto
import rclpy
import math
from rclpy.node import Node
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoint
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from turtlesim_msgs.srv import SetPen, TeleportAbsolute
from turtlesim_msgs.msg import Pose
from geometry_msgs.msg import Twist, Vector3


class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    MOVING = auto(),
    STOPPED = auto(),

class WaypointNode(Node):
    def __init__(self):
        super().__init__("waypoint")
        self.freq = 90 # hz
        self._velocity = 2.0
        self.timer = self.create_timer(1/self.freq, self.timer_callback)
        self.my_callback_group = MutuallyExclusiveCallbackGroup()
        #Create services
        self.toggle = self.create_service(Empty, "toggle",  self.toggle_callback)
        self.load = self.create_service(Waypoint, "load", self.load_callback)
        # Establish initial state
        self.state = State.STOPPED
        self.curr_waypoint = None

        #Creating clients
        self.reset = self.create_client(Empty, "/reset", callback_group=self.my_callback_group)
        self.set_pen = self.create_client(SetPen, "turtle1/set_pen", callback_group=self.my_callback_group)
        self.teleport = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute", callback_group=self.my_callback_group)

        #Establish future client storage
        self.set_pen_msg = SetPen.Request()
        self.teleport_msg = TeleportAbsolute.Request()

        #Create Subscribers
        self.pose = Pose()
        self._feedback = self.create_subscription(Pose, "turtle1/pose", self.feedback_callback, 10)

        #Create Publishers
        self._pub = self.create_publisher(Twist, "cmd_vel", 10)

        if not self.reset.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "kill" service to become available')

        if not self.set_pen.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "spawn" service to become available')

        if not self.teleport.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "spawn" service to become available')


    def timer_callback(self):
        if self.state == State.MOVING:
            msg = f"moving"
            self.get_logger().debug(msg)
            twist = self.turtle_twist()
            self._pub.publish(twist)

    def toggle_callback(self, request, response):
        if self.state == State.MOVING:
            self.get_logger().info("Stopping")
            self.state = State.STOPPED
        elif self.state == State.STOPPED:
            self.state = State.MOVING
        return response

    async def load_callback(self, request, response):
        self.get_logger().info("Resetting and adding waypoints")
        self.state = State.STOPPED
        response.distance = 0
        if request.waypoints == None:
            return response
        
        await self.reset.call_async(Empty.Request())

        self.set_pen_msg.off = 1
        await self.set_pen.call_async(self.set_pen_msg)

        i = 1
        for position in request.waypoints:
            corner1_x = position.x + 0.5
            corner1_y = position.y + 0.5

            corner2_x = position.x - 0.5
            corner2_y = position.y - 0.5

            self.teleport_msg.x, self.teleport_msg.y = corner1_x, corner1_y
            await self.teleport.call_async(self.teleport_msg)

            self.set_pen_msg.off = 0
            await self.set_pen.call_async(self.set_pen_msg)

            self.teleport_msg.x, self.teleport_msg.y = corner2_x, corner2_y
            await self.teleport.call_async(self.teleport_msg)

            self.set_pen_msg.off = 1
            await self.set_pen.call_async(self.set_pen_msg)

            corner1_x = position.x - 0.5
            corner1_y = position.y + 0.5

            corner2_x = position.x + 0.5
            corner2_y = position.y - 0.5

            self.teleport_msg.x, self.teleport_msg.y = corner1_x, corner1_y
            await self.teleport.call_async(self.teleport_msg)

            self.set_pen_msg.off = 0
            await self.set_pen.call_async(self.set_pen_msg)

            self.teleport_msg.x, self.teleport_msg.y = corner2_x, corner2_y
            await self.teleport.call_async(self.teleport_msg)

            self.set_pen_msg.off = 1
            await self.set_pen.call_async(self.set_pen_msg)

            if i < len(request.waypoints):
                x = i - 1
                response.distance += self.get_distance(request.waypoints[0], request.waypoints[1])



        self.teleport_msg.x, self.teleport_msg.y = request.waypoints[0].x, request.waypoints[0].y
        await self.teleport.call_async(self.teleport_msg)
        if len(request.waypoints) > 1:
            self.curr_waypoint = request.waypoints[1]
        return response
    
    def feedback_callback(self, pose):
        self.pose = pose


    def get_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
    
    def turtle_twist(self):
        """ Create a twist suitable for a turtle

            Args:
            xdot (float) : the forward velocity
            omega (float) : the angular velocity

            Returns:
            Twist - a 2D twist object corresponding to linear/angular velocity
        """
        if self.state == State.MOVING:
            dx = self.curr_waypoint.x - self.pose.x
            dy = self.curr_waypoint.y - self.pose.y
            theta_dir = math.atan(dy/dx)
            self.get_logger().info(f"Angle is: {theta_dir}")
            
            omega = theta_dir - self.pose.theta
        
            return Twist(linear = Vector3(x = self._velocity, y = 0.0, z = 0.0),
                    angular = Vector3(x = 0.0, y = 0.0, z = omega))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    rclpy.spin(node) 
    rclpy.shutdown()

