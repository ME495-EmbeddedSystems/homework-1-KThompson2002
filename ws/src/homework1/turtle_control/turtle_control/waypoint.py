from enum import Enum, auto
import rclpy
import math

from rclpy.node import Node
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoint
from turtle_interfaces.msg import ErrorMetric

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from turtlesim_msgs.srv import SetPen, TeleportAbsolute
from turtlesim_msgs.msg import Pose
from geometry_msgs.msg import Twist, Vector3, Point


class State(Enum):
    """ Current state of the system.
        Determines what the main timer function should be doing on each iteration
    """
    MOVING = auto(),
    LAST = auto()
    STOPPED = auto(),

class WaypointNode(Node):
    def __init__(self):
        super().__init__("waypoint")
        # declare parameters
        self.declare_parameter("tolerance", 1.0)
        self.declare_parameter("frequency", 90.0)

        # Create Parameter
        self.tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        self.freq = self.get_parameter("frequency").get_parameter_value().double_value

        # Establish constant variables
        self._velocity = 2.0
        self._vel_ang = 4.0
        self.timer = self.create_timer(1/self.freq, self.timer_callback)
        self.my_callback_group = MutuallyExclusiveCallbackGroup()
        # Create services
        self.toggle = self.create_service(Empty, "toggle",  self.toggle_callback)
        self.load = self.create_service(Waypoint, "load", self.load_callback)
        # Establish initial state
        self.state = State.STOPPED
        self.curr_waypoint = None
        self.curr_waypoint_idx = 0
        self.waypoints = None
        self.distance_actual = 0
        self.distance = 0
        self.complete_loops = 0
        self.curr_waypoint_loop = 0
        self.curr_loop_distance = 0
        
        
        # Creating clients
        self.reset = self.create_client(Empty, "/reset", callback_group=self.my_callback_group)
        self.set_pen = self.create_client(SetPen, "turtle1/set_pen", callback_group=self.my_callback_group)
        self.teleport = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute", callback_group=self.my_callback_group)

        # Establish future client storage
        self.set_pen_msg = SetPen.Request()
        self.teleport_msg = TeleportAbsolute.Request()

        # Create Subscribers
        self.pose = Pose()
        self._feedback = self.create_subscription(Pose, "turtle1/pose", self.feedback_callback, 10)

        # Create Publishers
        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.metric = self.create_publisher(ErrorMetric, "loop_metrics", 10)

        if not self.reset.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "reset" service to become available')

        if not self.set_pen.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "SetPen" service to become available')

        if not self.teleport.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "Teleport Absolute" service to become available')


    def timer_callback(self):
        if self.state == State.MOVING or self.state == State.LAST:
            msg = f"moving"
            self.get_logger().debug(msg)
            if self.curr_waypoint is not None and self.waypoints is not None:
                if self.get_distance(self.pose, self.curr_waypoint) < self.tolerance:
                    if self.state != State.LAST:
                        self.curr_waypoint_idx += 1
                        if self.curr_waypoint_idx >= len(self.waypoints):
                            self.curr_waypoint_idx = 0
                            self.curr_waypoint = self.waypoints[0]
                            self.state = State.LAST
                        else:
                            self.curr_waypoint = self.waypoints[self.curr_waypoint_idx]
                    else:
                        self.complete_loops += 1
                        self.curr_waypoint_loop += 1
                        self.state = State.STOPPED
                        self.curr_waypoint = self.waypoints[1]
                        self.curr_waypoint_idx = 1
                        if self.curr_waypoint_loop > 1:
                            self.distance += self.curr_loop_distance
                        msg = ErrorMetric()
                        msg.complete_loops = self.complete_loops
                        msg.actual_distance = self.distance_actual
                        msg.error = self.distance_actual - self.distance
                        self.metric.publish(msg)
                        self.get_logger().info(f"Publishing: {msg.complete_loops}, {msg.actual_distance}, and {msg.error}")
                twist = self.turtle_twist()
                self._pub.publish(twist)
            else:
                self.state = State.STOPPED
                    
            

    async def toggle_callback(self, request, response):
        if self.state == State.MOVING or self.state == State.LAST:
            self.get_logger().info("Stopping")
            self.state = State.STOPPED
            self.set_pen_msg.off = 1
            await self.set_pen.call_async(self.set_pen_msg)
            
        elif self.state == State.STOPPED:
            self.state = State.MOVING
            self.set_pen_msg.off = 0
            await self.set_pen.call_async(self.set_pen_msg)
        return response

    async def load_callback(self, request, response):
        self.get_logger().info("Resetting and adding waypoints")
        self.state = State.STOPPED
        response.distance = 0
        if request.waypoints == None:
            return response
        self.waypoints = request.waypoints.copy()
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
                response.distance += self.get_distance(request.waypoints[x], request.waypoints[i])
                i += 1



        response.distance += self.get_distance(request.waypoints[0], request.waypoints[i-1])
        self.curr_waypoint_loop = 0
        self.teleport_msg.x, self.teleport_msg.y = request.waypoints[0].x, request.waypoints[0].y
        await self.teleport.call_async(self.teleport_msg)
        self.get_logger().info(f"Straight line distance: {response.distance}")
        if len(request.waypoints) > 1:
            self.curr_waypoint = request.waypoints[1]
            self.curr_waypoint_idx = 1
        self.curr_loop_distance = response.distance
        self.distance += response.distance
        return response
    
    def feedback_callback(self, pose):
        if self.state == State.MOVING or self.state == State.LAST:
            self.distance_actual += self.get_distance(self.pose, pose)
        self.pose = pose


    def get_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
    
    def turtle_twist(self):
        """ Create a twist which moves the turtle proportionally towards the next waypoint """
        if self.state == State.MOVING or self.state == State.LAST:
            #Establish initial important variables
            vel_msg = Twist()
            dist = self.get_distance(self.pose, self.curr_waypoint)
            linear = dist * self._velocity

            #Calculate relative angle
            dx = self.curr_waypoint.x - self.pose.x
            dy = self.curr_waypoint.y - self.pose.y
            theta_dir = math.atan2(dy, dx)
            omega = theta_dir - self.pose.theta
            # self.get_logger().info(f"Angle is: {omega}")
            # if (omega > math.pi):

            angle = omega * self._vel_ang

            if omega > 0.01 or omega < -0.01:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = angle
                return vel_msg
            else:
                vel_msg.linear.x = linear
                vel_msg.angular.z = angle
                return vel_msg
            
        else:
            return Twist()

            




def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    rclpy.spin(node) 
    rclpy.shutdown()

