from ty_autopilot_core.base_node  import BaseNode
import rclpy
import threading
import time
import mavros
import rclpy.qos
from math import *
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from std_msgs.msg import Header
sensor_qos = rclpy.qos.qos_profile_sensor_data


class SetpointPosition(BaseNode):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        # publisher for mavros/setpoint_position/local
        self.pub = BaseNode.Publisher(self, PoseStamped, "/mavros/setpoint_position/local", sensor_qos)

        # subscriber for mavros/local_position/local
        self.sub = BaseNode.subscribe_topic(self, "/mavros/local_position/pose",PoseStamped, sensor_qos)

        self.done = False
        self.done_evt = threading.Event()

    def navigate(self, x, y, z):
        msg = PoseStamped(
            header=Header(
                frame_id="base_link",  
                stamp=self.get_clock().now().to_msg()),   
        )
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        yaw_degrees = 0 
        yaw = radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = Quaternion(*quaternion)
        if not self.done:
            self.pub.publish(msg)
            time.sleep(0.1)

    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z
        time.sleep(0.1)
        msg = PoseStamped(
            header=Header(
                frame_id="base_link",  
                stamp=self.get_clock().now().to_msg()),   
        )        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        yaw_degrees = 0 
        yaw = radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.get_logger().info(str(quaternion[0]))
        msg.pose.orientation.x = quaternion[0] 
        msg.pose.orientation.y = quaternion[1] 
        msg.pose.orientation.z = quaternion[2] 
        msg.pose.orientation.w = quaternion[3] 
        if not self.done:
            self.pub.publish(msg)
            time.sleep(0.1)
        pose = BaseNode.read_topic(self, '/mavros/local_position/pose')
        def is_near(msg, x, y):
            return abs(x - y) < 1.5
        if is_near('X', pose.pose.position.x, self.x) and is_near('Y', pose.pose.position.y, self.y):
            self.done = True
            self.done_evt.set()
        else:
            self.done = False
        if self.done:
            return True


class SetpointVelocity(BaseNode):
    def __init__(self):
        self.velocity_pub = BaseNode.Publisher(self, Twist,"sp_vel/cmd_vel", 10)

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

    def set(self,vx,vy,vz,yaw_rate):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = yaw_rate

        self.velocity_pub.publish(twist)
        time.sleep(0.1)
