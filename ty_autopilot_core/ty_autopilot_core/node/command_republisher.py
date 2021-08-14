import rclpy.qos
from ty_autopilot_core.base_node import BaseNode
from ty_autopilot_msgs.msg import CommandRequest

sensor_qos = rclpy.qos.qos_profile_sensor_data

class CommandRepublisher:
    def __init__(self):
        BaseNode.get_logger(self).info("Starting ComRepub as comm_repub.")
        BaseNode.subscribe_topic(self,"/ty_autopilot/com_request_in", CommandRequest, sensor_qos)
        self.command_req_pub = BaseNode.Publisher(self,CommandRequest, "/ty_autopilot/com_request_out", sensor_qos)
        self.com = CommandRequest()
    
    def republish_command(self):
        # BaseNode.get_logger(self).info("republisher handle:   "+str(BaseNode.read_topic(self,'/ty_autopilot/com_request_in')))
        cmd_req = BaseNode.read_topic(self,'/ty_autopilot/com_request_in')
        # BaseNode.get_logger(self).info(str(cmd_req.vehicle_id))
        self.com.vehicle_id = cmd_req.vehicle_id
        self.com.service_request = cmd_req.service_request
        self.com.requested_mode = cmd_req.requested_mode
        self.com.requested_arming = cmd_req.requested_arming
        self.com.auto_arm = cmd_req.auto_arm
        self.com.use_ros_planner = cmd_req.use_ros_planner
        self.com.switch_gps = cmd_req.switch_gps
        self.com.sp_type = cmd_req.sp_type
        self.com.ctrl_wait_timeout = cmd_req.ctrl_wait_timeout
        self.com.position_target = cmd_req.position_target
        self.com.velocity_target = cmd_req.velocity_target
        self.com.yaw = cmd_req.yaw
        self.com.yaw_rate = cmd_req.yaw_rate
        self.command_req_pub.publish(cmd_req)
        
