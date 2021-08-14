import rclpy
from rclpy.node import Node
from ty_autopilot_core.mavutils.mavenum import *
from ty_autopilot_core.node.controller import Controller
from ty_autopilot_msgs.srv import *
from ty_autopilot_core.base_node import BaseNode

class ServiceNavigate():
    def __init__(self):
        BaseNode.get_logger(self).info("Starting ServiceNavigate")
        BaseNode.create_service(self, SetVelocity, "ty_autopilot/set_velocity", Controller.set_velocity)
        BaseNode.create_service(self, SetPositionLocal, "ty_autopilot/set_position_local", Controller.set_position_local)
        BaseNode.create_service(self, SetPositionGlobal, "ty_autopilot/set_position_global", Controller.set_position_global)
        BaseNode.create_service(self, SetWaypoints, "ty_autopilot/set_waypoints", Controller.set_waypoints)

