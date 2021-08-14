from typing import Callable
from ty_autopilot_core.base_node import BaseNode
from ty_autopilot_core.mavutils.mavenum import RosFrame, SetpointType, MavrosTopics, MavFrames
from ty_autopilot_core.mavutils.param_util import RosParam
import rclpy.qos
# from ty_autopilot_core.mavutils.kml_parser import parse_json
from geometry_msgs.msg import Twist
from ty_autopilot_msgs.msg import CommandRequest
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import WaypointPush
from rclpy.duration import Duration
from ty_autopilot_core.mavutils.mavenum import *



class Controller:
    def __init__(self):
        self.local_target_pub = BaseNode.Publisher(self, PositionTarget, 'mavros/setpoint_raw/local', rclpy.qos.qos_profile_sensor_data)
        self.srv_client_mission_push = BaseNode.create_client(self, srv_name="/mavros/mission/push", srv_type= WaypointPush)
        self.command_request_pub = BaseNode.Publisher(self, CommandRequest, "/ty_autopilot/com_request_in", 10)
        BaseNode.subscribe_topic(self, State, "/mavros/state",10)
        self.default_cmd()

    def control(self, sp_type, frame_id, auto_arm, arm_vehicle, enable_rc, use_joy, mode, waypoints, x, y, z, yaw, vx, vy, vz, yaw_rate, use_ros_planner, ctrl_wait_timeout, switch_gps):
        comm_req = CommandRequest()
        comm_req.vehicle_id = RosParam.get_param(self, "target_system_id", 1)
        comm_req.service_request = True
        RosParam.set_param(self, "com_requested", True)
        comm_req.requested_mode = mode
        comm_req.auto_arm = auto_arm
        if auto_arm or arm_vehicle:
            comm_req.requested_arming = True
        else:
            comm_req.requested_arming = False

        comm_req.use_ros_planner = use_ros_planner
        if switch_gps == None:
            comm_req.switch_gps = RosParam.get_param(self, "gps_enable",False)
        else:
            comm_req.switch_gps = switch_gps
        comm_req.velocity_target.x = 0.0
        comm_req.velocity_target.y = 0.0
        comm_req.velocity_target.z = 0.0
        comm_req.yaw_rate = 0.0
        comm_req.sp_type = SetpointType.POSITION_LOCAL.value
        comm_req.position_target.x = 0.0
        comm_req.position_target.y = 0.0
        comm_req.position_target.z = 0.0
        comm_req.yaw = 0.0

        if frame_id is None:
            frame_id = RosFrame.LOCAL_FRAME.value

        if sp_type == SetpointType.VELOCITY.value:
            comm_req.sp_type = SetpointType.VELOCITY.value
            comm_req.velocity_target.x = vx
            comm_req.velocity_target.y = 0.0
            comm_req.velocity_target.z = 0.0
            comm_req.yaw_rate = yaw_rate

        if sp_type == SetpointType.POSITION_LOCAL.value:
            RosParam.set_param(self, "wp_reached", False)
            RosParam.set_param(self, "new_goal", True)
            comm_req.sp_type = SetpointType.POSITION_LOCAL.value
            comm_req.position_target.x = x
            comm_req.position_target.y = y
            comm_req.position_target.z = 0.0
            comm_req.yaw = yaw

        if sp_type == SetpointType.POSITION_GLOBAL.value:
            RosParam.set_param("wp_reached", False)
            RosParam.set_param("new_goal", True)
            comm_req.sp_type = SetpointType.POSITION_GLOBAL.value
            comm_req.position_target.x = x
            comm_req.position_target.y = y
            comm_req.position_target.z = 0.0
            comm_req.yaw = yaw
        

        if sp_type == SetpointType.WAYPOINTS.value:
            if BaseNode.read_topic(self, "/mavros/state").armed:
                comm_req.arming = False
                self.srv_client_mission_push(waypoints)

        for i in range(3):
            self.command_request_pub.publish(comm_req)

    def default_cmd(self):
        comm_req = CommandRequest()
        comm_req.vehicle_id = 1
        comm_req.service_request = False
        comm_req.requested_mode = ""
        comm_req.requested_arming = False
        comm_req.auto_arm = False
        comm_req.use_ros_planner = False
        comm_req.switch_gps = False
        comm_req.sp_type = ""
        comm_req.ctrl_wait_timeout = 0.0
        comm_req.yaw = 0.0
        comm_req.yaw_rate = 0.0
        for i in range(3):
            self.command_request_pub.publish(comm_req)

    def set_velocity(self, request, response):
        print("setpoint service called :====",request)
        self.control(self,
            sp_type=SetpointType.VELOCITY.value, 
            frame_id=MavFrames.BODY_NED, 
            auto_arm=request.auto_arm, 
            arm_vehicle = None,
            enable_rc=False, 
            use_joy=False, 
            mode=Mode.GUIDED.value, 
            waypoints=None, 
            x=None, y=None, z=None, yaw=None, 
            vx=request.vx, vy=request.vy, vz=request.vz, yaw_rate=request.yaw_rate, 
            use_ros_planner=False,
            ctrl_wait_timeout=request.command_life,
            switch_gps= None)
        response.success = True
        response.message = "Setting Velocity"
        return response
        
    def set_position_local(self, request, response):
        self.control(
            sp_type=SetpointType.POSITION_LOCAL.value, 
            frame_id=request.frame_id, 
            auto_arm=request.auto_arm, 
            arm_vehicle = None,
            enable_rc=False, 
            use_joy=False, 
            mode=Mode.GUIDED.value, 
            waypoints=None,  
            x=request.x, y=request.y, z=request.z, yaw=request.heading, 
            vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, 
            use_ros_planner=request.use_ros_planner, 
            ctrl_wait_timeout=0.0,
            switch_gps= None)
        response.success = True
        response.message = "Setting Position local"
        return response

    def set_position_global(self, request,response):
        self.control(
            sp_type=SetpointType.POSITION_GLOBAL.value, 
            frame_id=MavFrames.GLOBAL, 
            auto_arm=request.auto_arm, 
            enable_rc=False, use_joy=False, 
            mode=Mode.GUIDED.value, 
            waypoints=None, 
            x=request.x_lat, y=request.y_long, z=request.z_lat, yaw=request.heading, 
            vx=request.vx, vy=request.vy, vz=request.vz, yaw_rate=request.yaw_rate, 
            use_ros_planner=request.use_ros_planner, 
            ctrl_wait_timeout=None )
        response.success = True
        response.message = "Setting Position global"
        return response

    def set_waypoints(self, request,response):
        self.control(
            sp_type=SetpointType.WAYPOINTS, 
            frame_id=None, 
            auto_arm=request.auto_arm, 
            enable_rc=False, 
            use_joy=False, 
            mode=None, 
            waypoints=request.waypoints, 
            x=None, y=None, z=None, yaw=None, 
            vx=None, vy=None, vz=None, yaw_rate=None, 
            use_ros_planner=False, 
            ctrl_wait_timeout=None )
        response.success = True
        response.message = "waypoint stored"
        return response