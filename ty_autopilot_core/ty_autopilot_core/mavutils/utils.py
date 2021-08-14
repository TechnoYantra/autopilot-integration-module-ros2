#!/usr/bin/env python3
from ty_autopilot_core.ty_autopilot_core.base_node import BaseNode
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, ManualControl
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.duration import Duration

state_qos = rclpy.qos.QoSProfile(depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
sensor_qos = rclpy.qos.qos_profile_sensor_data
parameters_qos = rclpy.qos.qos_profile_parameters

class Status(BaseNode):
    def __init__(self):
        BaseNode.__init__(self)
        self.subscribe_topic(topic="/mavros/state", type=State, )
        self.subscribe_topic(topic="mavros/local_position/velocity_body", type=TwistStamped, qos_profile=state_qos)
        self.subscribe_topic(topic="mavros/local_position/pose", type=PoseStamped, qos_profile=state_qos)
        self.subscribe_topic(topic="mavros/global_position/global", type=NavSatFix, qos_profile=state_qos)
        self.subscribe_topic(topic="mavros/battery", type=BatteryState, qos_profile=sensor_qos)
        self.subscribe_topic(topic="/mavros/manual_control/control", type=ManualControl, qos_profile=state_qos)

    def timeout(self ,stamp, timeout):
        return self.get_clock().now() - stamp > Duration(seconds=timeout)

    def checkManualControl(self):
        error = False
        message = "Manual Control Healthy"
        if self.timeout(self.topic.read("/mavros/manual_control/control").header.stamp, 3):
            error = True
            message = "Manual control timeout, RC is switched off?"
            self.get_logger().info("Manual control timeout, RC is switched off?")
        return error, message

    def checkState(self):
        error = False
        message = "Mavros Connected"
        if self.timeout(self.topic.read("/mavros/state").header.stamp, 3):
            error = True
            message = "State timeout, check mavros settings"
            self.get_logger().info("State timeout, check mavros settings")
        if not self.topic.read("/mavros/state").connected:
            error = True
            message = "No connection to FCU"
            self.get_logger().info("No connection to FCU")
        return error, message

    def checkLocalPosition(self):
        error = False
        message = "Local Position Received"
        if self.timeout(self.topic.read("mavros/local_position/pose").header.stamp, 3):
            error = True
            message = "No local position, check settings"
            self.get_logger().info("No local position, check settings")
        return error, message

    def checkGlobalPosition(self):
        error = False
        message = "Global Position Received"
        if self.timeout(self.topic.read("mavros/global_position/global").header.stamp, 3):
            error = True
            message = "No Global position, check GPS"
            self.get_logger().info("No Global position, check GPS")
        return error, message

