#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ty_autopilot_msgs.srv import GetVehicleState, GetVehicleStateResponse, GeoFenceSetResponse
from ty_autopilot_core.mavutils import mavenum, utils
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import Twist, Pose, TwistStamped
import tf2_ros

STATUS = utils.Status()

class ServiceGetVehicleState(Node):
    def __init__(self):
        super().__init__('get_vehicle_state_service')
        self.get_logger().info("Starting Service GetVehicleState.")
        srv_server_get_transform = self.create_service(GetVehicleState, "ty_autopilot/get_vehicle_state", self.service_callback)
        self.local_frame = "map"
        self.fcu_frame = "base_link"
        self.topic = Topic()
        self.topic.subscribe_topic(topic="/mavros/state", type=State)
        self.topic.subscribe_topic(topic="/mavros/local_position/velocity_body", type=TwistStamped)
        self.topic.subscribe_topic(topic="/mavros/global_position/global", type=NavSatFix)
        self.topic.subscribe_topic(topic="/mavros/battery", type=BatteryState)
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

    def service_callback(self,request, response):
        if request.frame_id == None:
            request.frame_id = self.local_frame
        response.frame_id = request.frame_id
        response.x = 0.0
        response.y = 0.0
        response.z = 0.0
        response.lat = 0.0
        response.lon = 0.0
        response.alt = 0.0
        response.vx = 0.0
        response.vy = 0.0
        response.vz = 0.0
        response.pitch = 0.0
        response.roll = 0.0
        response.yaw = 0.0
        response.pitch_rate = 0.0
        response.roll_rate = 0.0
        response.yaw_rate = 0.0
        response.voltage = 0.0
        response.cell_voltage = 0.0

        if not STATUS.timeout(self.topic.read("/mavros/state").header.stamp, 3):
            response.connected = self.topic.read("/mavros/state").connected
            response.armed = self.topic.read("/mavros/state").armed
            response.mode = self.topic.read("/mavros/state").mode

        try:
            trans,rot = self.tf_buffer.lookup_transform(request.frame_id, self.fcu_frame, self.get_clock(0.5))
            response.x = trans[0]
            response.y = trans[1]
            response.z = trans[2]

            rot_euler = tf2_ros.transformations.euler_from_quaternion(rot[0], rot[1], rot[2], rot[3])
            response.roll = rot_euler[0]
            response.pitch = rot_euler[1]
            response.yaw = rot_euler[2]
        except:
            tf2_ros.LookupError
        if not STATUS.timeout(self.topic.read("mavros/local_position/velocity_body").header.stamp, 3):
            response.vx = 0.0 # todo 
            response.vy = 0.0 # todo 
            response.vz = 0.0 # todo 
        if not STATUS.timeout(self.topic.read("mavros/global_position/global").header.stamp, 3):
            response.lat = self.topic.read("mavros/global_position/global").latitude
            response.lon = self.topic.read("mavros/global_position/global").longitude
            response.alt = self.topic.read("mavros/global_position/global").altitude
        if not STATUS.timeout(self.topic.read("mavros/battery").header.stamp, 3):
            response.voltage = self.topic.read("mavros/battery").voltage
            response.cell_voltage = self.topic.read("mavros/battery").cell_voltage[0]
        return response


def main(args=None):
    rclpy.init(args=args)
    service = ServiceGetVehicleState()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()