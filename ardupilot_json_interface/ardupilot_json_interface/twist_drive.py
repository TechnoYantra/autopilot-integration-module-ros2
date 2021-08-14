import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCOut
from geometry_msgs.msg import Twist
import math
class TwistDrive(Node):

    def __init__(self):
        super().__init__('twist_drive_node')
        self.autopilot_pwm_sub = self.create_subscription(RCOut, '/mavros/rc/out', self.autopilot_pwm_cb, 10)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def autopilot_pwm_cb(self, msg):
        steer_max = 1.0
        throttle_max = 0.5
        twist_out = Twist()
        twist_out.linear.x = ( self.clamp((msg.channels[0] - 1500.0)/500.0, -1, 1) + self.clamp((msg.channels[2] - 1500.0)/500.0, -1, 1) ) * throttle_max
        twist_out.angular.z = (self.clamp((msg.channels[0] - 1500.0)/500.0, -1, 1) - self.clamp((msg.channels[2] - 1500.0)/500.0, -1, 1) ) * math.radians(steer_max) * -1 
        twist_out.linear.y = 0.0
        twist_out.linear.z = 0.0
        twist_out.angular.x = 0.0
        twist_out.angular.y = 0.0
        self.twist_pub.publish(twist_out)
    
    def map_to_limit(self, x, in_min=1000.0, in_max=2000.0, out_min=-1.0, out_max=1.0):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def clamp(self,v,min_v,max_v):
        if v < min_v:
            v = min_v
        if v > max_v:
            v = max_v
        return v


def main(args=None):
    rclpy.init(args=args)
    node = TwistDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()