#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from services.service_get_vehicle_state import ServiceGetVehicleState
from services.service_navigate import ServiceNavigate
from services.service_send_movebase_goal import ServiceSendMovebaseGoal
from services.service_switch_gps_mode import ServiceGpsMode
from services.service_set_ekf_origin import ServiceSetEKFOrigin

class ServiceFactory:
    def __init__(self):
        super().__init__("ty_autopilot_services")
        ServiceGetVehicleState()
        ServiceNavigate()
        ServiceSendMovebaseGoal()
        ServiceGpsMode()
        ServiceSetEKFOrigin()

if __name__ == "__main__":
    ServiceFactory()
    rospy.spin()