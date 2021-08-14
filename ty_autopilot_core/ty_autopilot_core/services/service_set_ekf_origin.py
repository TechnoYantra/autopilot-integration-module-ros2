#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from ty_autopilot_msgs.srv import SetEkfOrigin
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink
from ty_autopilot_core.mavutils.utils import Status, Param

param = Param()
class ServiceSetEKFOrigin(Node):
    def __init__(self):
        super().__init__('set_ekf_origin_service')
        self.set_ekf_origin      = self.create_service(SetEkfOrigin, '/ty_autopilot/set_ekf_origin', self.set_ekf_origin)
        self.mavlink_pub         = self.create_publisher(Mavlink, "/uas1/mavlink_sink", queue_size=20)
        self.f = self.fifo()

    class fifo:
        def __init__(self):
            self.buf = []
        def write(self, data):
            self.buf += data
            return len(data)
        def read(self):
            return self.buf.pop(0)

    def set_ekf_origin(self,request, response):
        lat             = param.get('geo_origin_lat', 0.0)
        lon             = param.get('geo_origin_lon', 0.0)
        alt             = param.get('geo_origin_alt', 0.0)
        srcSystem       = param.get('system_id', 1)
        srcComponent    = param.get('component_id', 254)
        mav = MAV_APM.MAVLink(self.f, srcSystem, srcComponent)
        while self.mavlink_pub.get_num_connections() <= 0:
            pass
        for _ in range(4):
            time.sleep(1)
            self.set_global_origin(mav, self.mavlink_pub, lat, lon, alt)
            self.set_home_position(mav, self.mavlink_pub, lat, lon, alt)
        response.success = True
        return response

    def send_mav_message(self, msg, mav, pub):
        msg.pack(mav)
        rosmsg = convert_to_rosmsg(msg)
        pub.publish(rosmsg)
        self.get_logger().info("sent message %s" % msg)

    def set_global_origin(self, mav, pub, lat, lon, alt):
        target_system = mav.srcSystem
        #target_system = 0 
        lattitude = lat
        longitude = lon
        altitude = alt
        msg = MAV_APM.MAVLink_set_gps_global_origin_message(
                target_system,
                lattitude, 
                longitude,
                altitude)

        self.send_mav_message(msg, mav, pub)

    def set_home_position(self, mav, pub, lat, lon, alt):
        target_system = mav.srcSystem
        #target_system = 0
        lattitude = lat
        longitude = lon
        altitude = alt
        x = 0
        y = 0
        z = 0
        q = [1, 0, 0, 0]   # w x y z
        approach_x = 2
        approach_y = 2
        approach_z = 1
        msg = MAV_APM.MAVLink_set_home_position_message(
                target_system,
                lattitude,
                longitude,
                altitude,
                x,
                y,
                z,
                q,
                approach_x,
                approach_y,
                approach_z)
        self.send_mav_message(msg, mav, pub)
        
def main(args=None):
    rclpy.init(args=args)
    set_ekf_origin_service = ServiceSetEKFOrigin()
    rclpy.spin(set_ekf_origin_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()