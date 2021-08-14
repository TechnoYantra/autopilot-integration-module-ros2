#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ty_autopilot_msgs.srv import SwitchGpsMode
from ty_autopilot_msgs.msg import CommandRequest


class ServiceGpsMode(Node):
    def __init__(self):
        super().__init__("gps_mode_service")
        self.get_logger().info("Starting ServiceGpsMode service server.")
        self.create_service( SwitchGpsMode,"/ty_autopilot/toggle_gps", self.toggle_gps)
        self.message_pub = self.create_publisher(CommandRequest, "/ty_autopilot/com_request_out", queue_size=10)

    def toggle_gps(self,req):
        self.message_pub.publish(CommandRequest(toggle_gps=req.gps))

def main(args=None):
    rclpy.init(args=args)
    gps_mode_service = ServiceGpsMode()
    rclpy.spin(gps_mode_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()