from ty_autopilot_core.node.setpoint import SetpointVelocity, SetpointPosition
from ty_autopilot_core.node.rc_override import MavrosRC
from ty_autopilot_core.node.obstacle_detection import ObstacleDetector
from .node.cmd_vel_mux_selector import MuxSelectorNode
import rclpy
from .base_node import BaseNode
import rclpy.node
import rclpy.qos
import rclpy.parameter
import time
import math
import re
import sys
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from sensor_msgs.msg import NavSatFix
from ty_autopilot_msgs.srv import * 
# from topic_tools.srv import MuxAdd, MuxDelete, MuxList, MuxSelect

# from .mavutils.topictools import Topic as topics
from .mavutils.mavenum import (
                    MavrosTopics, MavFrames, RosFrame,
                    SetpointType, Mode, 
                    VehicleClass, VehicleType, 
                    ServoOutputFunction )
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.srv import CommandInt, WaypointPull, SetMode, CommandBool
from mavros_msgs.msg import State, WaypointList, OverrideRCIn, HomePosition
from rcl_interfaces.srv import ListParameters
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from ty_autopilot_msgs.msg import CommandRequest

from .mavutils import alvinxy, param_util, geoutil
from .mavutils.param_util import RosParam, MavParam
from .node.frame_transformer import  FrameTransformer
from .node.command_republisher import CommandRepublisher
from .services.service_navigate import ServiceNavigate

import threading
from typing import Callable
from ty_autopilot_core.base_node import BaseNode
from ty_autopilot_core.mavutils.mavenum import RosFrame, SetpointType, MavrosTopics, MavFrames
from ty_autopilot_core.mavutils.param_util import RosParam
import rclpy.qos
from geometry_msgs.msg import Twist
from ty_autopilot_msgs.msg import CommandRequest
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import WaypointPush
from rclpy.duration import Duration
from ty_autopilot_core.mavutils.mavenum import *
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes

state_qos = rclpy.qos.QoSProfile(depth=10, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
sensor_qos = rclpy.qos.qos_profile_sensor_data
parameters_qos = rclpy.qos.qos_profile_parameters


class MissionController(BaseNode):
    def __init__(self):
        BaseNode.__init__(self)
        FrameTransformer.__init__(self)
        CommandRepublisher.__init__(self)
        ObstacleDetector.__init__(self)
        MuxSelectorNode.__init__(self)
        MavrosRC.__init__(self)
        SetpointVelocity.__init__(self)
        SetpointPosition.__init__(self)
        self.log = self.get_logger().info("Started ty_autopilot_core_node")
        self._allow_undeclared_parameters = True
        self.automatically_declare_parameters_from_overrides=True

        self.srv_client_mission_push = self.create_client( srv_name="/mavros/mission/push", srv_type= WaypointPush)
        self.command_request_pub = self.Publisher( CommandRequest, "/ty_autopilot/com_request_in", 10)
        self.nav_goal_pub = self.Publisher( PoseStamped, "/goal_update", 10)
        self.default_cmd()

        self.autopilot_param_config = RosParam.get_param(self, parameter_name='autopilot_param_config_file', get_value=True)
        self.ext_nav_topic = RosParam.get_param(self, 'autopilot_ros_config.twist_mux_topics.ext_nav')
        self.key_teleop_topic = RosParam.get_param(self, 'autopilot_ros_config.twist_mux_topics.key_teleop')
        self.sp_vel_topic = RosParam.get_param(self, 'autopilot_ros_config.twist_mux_topics.sp_vel')

        self.create_timer(0.0, self.transformer_timer_callback)  
        self.create_timer(0.1, self.comm_republisher_timer_callback)   
        self.create_timer(0.2, self.mux_publisher_timer_callback)   
        self.create_timer(0.2, self.obstacle_detection_timer_callback) 
        self.create_timer(0.2, self.rc_override_timer_callback)  

        self.subscribe_topic(topic="/mavros/state", type=State, qos_profile=sensor_qos)
        self.subscribe_topic(topic="/diagnostics", type=DiagnosticArray, qos_profile=sensor_qos)
        self.subscribe_topic(topic="/move_base_simple/goal", type=PoseStamped, qos_profile=sensor_qos)
        self.subscribe_topic(topic="/mavros/mission/waypoints", type=WaypointList, qos_profile=sensor_qos)
        self.subscribe_topic(topic="/ty_autopilot/com_request_out", type=CommandRequest, qos_profile=sensor_qos)
        self.subscribe_topic(topic="/mavros/global_position/global", type=NavSatFix, qos_profile=sensor_qos)
        self.subscribe_topic(topic="/mavros/local_position/pose", type=PoseStamped, qos_profile=sensor_qos)
        self.subscribe_topic(topic="/mux/selected", type=String, qos_profile=sensor_qos)
        self.subscribe_topic(topic="/mavros/home_position/home", type=HomePosition, qos_profile=sensor_qos)
        
        self.list_parameters_cli = self.create_client(ListParameters, "/ty_autopilot_core_node/list_parameters")
        self.change_mode = self.create_client(SetMode, "/mavros/set_mode" )
        self.waypoint_pull = self.create_client(WaypointPull, "/mavros/mission/pull")

        self.create_service( SetVelocity, "ty_autopilot/set_velocity", self.set_velocity)
        self.create_service( SetPositionLocal, "ty_autopilot/set_position_local", self.set_position_local)
        self.create_service( SetPositionGlobal, "ty_autopilot/set_position_global", self.set_position_global)
        self.create_service( SetWaypoints, "ty_autopilot/set_waypoints", self.set_waypoints)
        self.goal_handle = None
        self.rc_channels = [0]*8
        self.init_goal = True
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        t = threading.Thread(target=self.main_loop, daemon = True).start()

    def navigate_to(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        self.get_logger().info(str(self.init_goal))
        if RosParam.get_param(self, parameter_name="init_goal", default_value= True):
            RosParam.set_param(self, parameter_name="init_goal", parameter_value= False)
            self.get_logger().info('Navigating to goal: ' + str(goal_pose.pose.position.x) + ' ' + str(goal_pose.pose.position.y))
            send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self.feedbackCallback)
            self.goal_handle = send_goal_future.result()
            time.sleep(1)
        try:
            self.result_future = self.goal_handle.get_result_async()
            self.get_logger().info(str(self.result_future.result().status))
        except:
            time.sleep(1)
        self.nav_goal_pub.publish(goal_pose)
        

    def feedbackCallback(self, msg):
        self.get_logger().info('Received action feedback message')
        self.feedback = msg.feedback
        self.get_logger().info(str(msg.feedback))

    def transformer_timer_callback(self):
        try:
            FrameTransformer.transform(self)
        except:
            self.get_logger().error('transformer')

    def comm_republisher_timer_callback(self):
        try:
            CommandRepublisher.republish_command(self)
        except:
            self.get_logger().error('comm_republisher')

    def mux_publisher_timer_callback(self):
        try:
            MuxSelectorNode.mux_publisher(self)
        except:
            self.get_logger().error('mux_publisher')
    
    def obstacle_detection_timer_callback(self):
        try:
            ObstacleDetector.obstacle_detection(self)
        except:
            self.get_logger().error('obstacle_detection')

    def rc_override_timer_callback(self):
        try:
            MavrosRC.publish_rc_channels(self, self.rc_channels)
        except:
            self.get_logger().error('rc_override')
            
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
        self.control(
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
            sp_type=SetpointType.WAYPOINTS.value, 
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


    def execute_mode(self,mode):
        modes = {
                    Mode.GUIDED.value: self.guided,
                    Mode.AUTO.value: self.auto,
                    Mode.HOLD.value: self.hold,
                    # Mode.MAVCOMMAND.value: self.mavcommand
                }
        function = modes.get(mode, lambda: "Invalid Mode")
        function()

    def hold(self):
        pass

    def guided(self):
        if self.command_req.sp_type == SetpointType.POSITION_LOCAL.value:
            position_target = self.command_req.position_target
            self.get_logger().info("guided sp position local")
            if self.command_req.use_ros_planner:
                if RosParam.get_param(self, parameter_name="mux_select") != self.ext_nav_topic:
                    RosParam.set_param(self, parameter_name="mux_select", parameter_value=self.ext_nav_topic)
                self.get_logger().info(str(RosParam.get_param(self, parameter_name="continue_mission",default_value= False)))
                if RosParam.get_param(self, parameter_name="new_goal", default_value= False):
                    self.navigate_to(position_target.x, position_target.y)
                    RosParam.set_param(self, parameter_name="new_goal", parameter_value= False)
            else:
                if not RosParam.get_param(self, parameter_name="wp_reached", default_value=False):
                    reached = SetpointPosition.set(self, position_target.x, position_target.y, position_target.z)
                    if reached:
                        RosParam.set_param(self, parameter_name="wp_reached", parameter_value=True)
        elif self.command_req.sp_type == SetpointType.POSITION_GLOBAL.value:
            self.get_logger().info("guided sp position global")
            position_target = self.command_req.position_target
            if RosParam.get_param(self, parameter_name="mux_select") != self.ext_nav_topic:
                RosParam.set_param(self, parameter_name="mux_select", parameter_value=self.ext_nav_topic)
              
            if RosParam.get_param(self, parameter_name="new_goal",default_value= False):
                self.navigate_to(position_target.x, position_target.y)
                RosParam.set_param(self, parameter_name="new_goal", parameter_value=False)
        elif RosParam.get_param(self, parameter_name="continue_mission",default_value= False):
            if RosParam.get_param(self, parameter_name="mux_select") != self.key_teleop_topic:
                RosParam.set_param(self, parameter_name="mux_select", parameter_value=self.key_teleop_topic)
        
        if self.command_req.sp_type == SetpointType.VELOCITY.value:
            self.get_logger().info("guided sp velocity")
            velocity_target = self.command_req.velocity_target
            yaw_rate = self.command_req.yaw_rate
            self.get_logger().info(str(velocity_target))
            if RosParam.get_param(self, parameter_name="mux_select") != self.sp_vel_topic:
                RosParam.set_param(self, parameter_name="mux_select", parameter_value=self.sp_vel_topic)
            SetpointVelocity.set(self, velocity_target.x, velocity_target.y, velocity_target.z, yaw_rate)

        if RosParam.get_param(self, parameter_name="waypoint_pulled", default_value=False) == True:
            self.monitor_mission_progress()
            self.get_logger().info("guided waypoint_pulled")
            if not RosParam.get_param(self, parameter_name="mission_complete", default_value= False):
                if RosParam.get_param(self, parameter_name="obstacle_detected", default_value= False):
                    current_target_wp_seq = self.read_topic("/mavros/mission/waypoints").current_seq
                    launch_waypoint = self.read_topic("/mavros/mission/waypoints").waypoints[0]
                    current_target_wp = self.read_topic("/mavros/mission/waypoints").waypoints[current_target_wp_seq]
                    lat_origin = self.read_topic("/mavros/mission/waypoints").waypoints[0].x_lat
                    long_origin = self.read_topic("/mavros/mission/waypoints").waypoints[0].y_long
                    start_wp = self.read_topic("/mavros/mission/waypoints").waypoints[current_target_wp_seq - 2 if current_target_wp_seq==2 else 1]
                    next_wp  = self.read_topic("/mavros/mission/waypoints").waypoints[current_target_wp_seq]
                    start_y,start_x = alvinxy.ll2xy(start_wp.x_lat, start_wp.y_long, lat_origin, long_origin)
                    next_y,next_x = alvinxy.ll2xy(next_wp.x_lat, next_wp.y_long, lat_origin, long_origin)
                    local_x, local_y = geoutil.get_target_goal_on_path(
                        start_x, start_y,
                        next_x, next_y,
                        (self.read_topic("/mavros/local_position/pose").pose.position.y), (self.read_topic("/mavros/local_position/pose").pose.position.x),
                        RosParam.get_param(self, 'rosnav_goal_distance',3.0)) 
                    
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    goal_pose.pose.position.x = local_x
                    goal_pose.pose.position.y = -local_y
                    goal_pose.pose.orientation.w = 1.0
                    goal_msg = NavigateToPose.Goal()
                    goal_msg.pose = goal_pose
                    
                    self.get_logger().info(str(self.init_goal))
                    if RosParam.get_param(self, parameter_name="init_goal", default_value= True):
                        RosParam.set_param(self, parameter_name="init_goal", parameter_value= False)
                        self.get_logger().info('Navigating to goal: ' + str(goal_pose.pose.position.x) + ' ' + str(goal_pose.pose.position.y))
                        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self.feedbackCallback)
                        # self.goal_handle = send_goal_future.result()
                        time.sleep(1)
                    try:
                        self.result_future = self.goal_handle.get_result_async()
                        self.get_logger().info("status :"+str(self.result_future.result().status))
                    except:
                        time.sleep(0.1)
                    self.nav_goal_pub.publish(goal_pose)
                    self.get_logger().info(str(RosParam.get_param(self, parameter_name="init_goal", default_value= True)))
                    self.get_logger().info("Handover control to ROS")
                    time.sleep(1.0)
                elif not RosParam.get_param(self, parameter_name="obstacle_detected"):
                    self.change_mode.call(SetMode.Request(custom_mode=Mode.AUTO.value))


    def auto(self):
        if not RosParam.get_param(self,parameter_name='waypoint_pulled', default_value= False):
            resp = self.waypoint_pull.call(WaypointPull.Request())
            RosParam.set_param(self, parameter_name="waypoint_pulled", parameter_value= resp.success)
            RosParam.set_param(self, parameter_name="mission_complete", parameter_value= False)
            time.sleep(1)
            for waypoint in range(len(self.read_topic("/mavros/mission/waypoints").waypoints)):
                if self.read_topic("/mavros/mission/waypoints").waypoints[waypoint].command == 92:
                    RosParam.set_param(self, parameter_name="guided_enable",parameter_value=  True)
                    RosParam.set_param(self, parameter_name="new_goal",parameter_value=  True)

            self.pose_seq = list()
            self.waypoint_goal = list()
            for waypoint in range(len(self.read_topic("/mavros/mission/waypoints").waypoints)):
                if self.read_topic("/mavros/mission/waypoints").waypoints[waypoint].command == 16:
                    if not waypoint == 0:
                        target_waypoint = self.read_topic("/mavros/mission/waypoints").waypoints[waypoint]
                        lat_orgin = self.read_topic("/mavros/global_position/global").latitude
                        long_orgin = self.read_topic("/mavros/global_position/global").longitude
                        local_y,local_x = alvinxy.ll2xy(target_waypoint.x_lat, target_waypoint.y_long, lat_orgin, long_orgin)
                        self.pose_seq.extend([local_x,-local_y,0.0])
                    time.sleep(0.5)
            self.waypoint_goal = [self.pose_seq[i:i+3] for i in range(0, len(self.pose_seq), 3)]
            waypoint_count = len(self.waypoint_goal)
            self.get_logger().info(str(waypoint_count))
            
            if RosParam.get_param(self, parameter_name="mux_select") != self.ext_nav_topic:
                RosParam.set_param(self, parameter_name="mux_select", parameter_value=self.ext_nav_topic)
                self.get_logger().info('audto_guided ext')
        
        self.monitor_mission_progress()
        if RosParam.get_param(self, parameter_name="guided_enable",default_value= False):
            if RosParam.get_param(self, parameter_name="new_goal", default_value= False):
                self.navigate_to(self.waypoint_goal[self.waypoint_numb][0], self.waypoint_goal[self.waypoint_numb][1])
                time.sleep(3)
                self.get_logger().info(str(self.waypoint_numb))
                RosParam.set_param(self, parameter_name="new_goal",parameter_value=False)
            distance = math.sqrt( (self.read_topic("/mavros/local_position/pose").pose.position.x + self.waypoint_goal[self.waypoint_numb][1])**2 + (self.read_topic("/mavros/local_position/pose").pose.position.y - self.waypoint_goal[self.waypoint_numb][0])**2 )
            if distance<1.0:
                RosParam.set_param(self, parameter_name="new_goal", parameter_value= True)
                self.waypoint_numb = self.waypoint_numb+1
                if self.waypoint_numb == len(self.waypoint_goal):
                    RosParam.set_param(self, parameter_name="new_goal", parameter_value= False)
                    self.change_mode.call(SetMode.Request(custom_mode="HOLD"))

        if RosParam.get_param(self, parameter_name="obstacle_detected", default_value=False):
            self.get_logger().info("Obstacle detected, Switching to GUIDED")
            RosParam.set_param(self, parameter_name="init_goal", parameter_value= True)
            self.change_mode.call(SetMode.Request(custom_mode=Mode.GUIDED.value))

    def monitor_mission_progress(self):
        current_target_wp_seq = self.read_topic("/mavros/mission/waypoints").current_seq
        total_mission_items = len(self.read_topic("/mavros/mission/waypoints").waypoints)
        if current_target_wp_seq == total_mission_items:
            RosParam.set_param(self, parameter_name="mission_complete", parameter_value=True)


    def run_once(function):
        def wrapper(*args, **kwargs):
            if not wrapper.has_run:
                wrapper.has_run = True
                return function(*args, **kwargs)
        wrapper.has_run = False
        return wrapper


    @run_once
    def initialization(self):
        rc_channels = [0]*8
        param_json = RosParam.parse_yaml(self,self.autopilot_param_config)
        self.get_logger().info(str(param_json))
        while not self.read_topic("/mavros/state").connected == True:
            self.get_logger().info("Waiting for FCU connection...")
            self.log = self.get_logger().info(str(self.read_topic("/mavros/state").connected))
            time.sleep(1)
        self.get_logger().info("FCU connected !")
        MavParam.mav_param_pull(self,force=True)

        rc_aux_function = param_json["auxilary_channel_config"]
        servo_out_function = param_json["output_channel_config"]
        ek3_source_list = param_json["ek3_sources_config"]
        self.get_logger().info(str(servo_out_function))  

        for servo_function in servo_out_function:
            self.get_logger().info(str(MavParam.mav_param_get(self,servo_function) ))  
            if MavParam.mav_param_get(self, servo_function) != servo_out_function[servo_function]:
                ret = MavParam.mav_param_set(self, servo_function,servo_out_function[servo_function])


        for rc_option in rc_aux_function:
            self.rc_channels[int(re.findall(r"\d+", rc_option)[0])-1] = rc_aux_function[rc_option]
            self.get_logger().info(str(rc_channels))
            if MavParam.mav_param_get(self, rc_option) != rc_aux_function[rc_option]:
                ret = MavParam.mav_param_set(self, rc_option, rc_aux_function[rc_option])
                self.get_logger().info(str(ret))

        for ek3_source in ek3_source_list:
            if MavParam.mav_param_get(self, ek3_source) != ek3_source_list[ek3_source]:
                ret = MavParam.mav_param_set(self, ek3_source, ek3_source_list[ek3_source])
                self.get_logger().info(str(ret))  
        self.get_logger().info("Autopilot Configuration complete")      

    def main_loop(self):
        self.initialization()
        while rclpy.ok():
            state = self.read_topic("/mavros/state")
            self.command_req = self.read_topic("/ty_autopilot/com_request_out")        
            if RosParam.get_param(self,parameter_name='com_requested', default_value= False) == True:
                RosParam.set_param(self, parameter_name='com_requested', parameter_value=False)
                if not self.command_req.sp_type == Mode.MAVCOMMAND.value:
                    self.get_logger().info(str(state.mode))
                    if state.mode != self.command_req.requested_mode:
                        self.get_logger().info( str(self.command_req.requested_mode))
                        self.change_mode.call(SetMode.Request(custom_mode=self.command_req.requested_mode))
     
                    RosParam.set_param(self, parameter_name='com_requested', parameter_value=False)
                    self.execute_mode(self.command_req.requested_mode)
                else:
                    self.execute_mode(Mode.MAVCOMMAND.values)
            else:
                self.execute_mode(state.mode)
        

def main(args=sys.argv):
    rclpy.init(args=args)
    mission_ontroller_node = MissionController()
    rclpy.spin(mission_ontroller_node)
    mission_ontroller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()