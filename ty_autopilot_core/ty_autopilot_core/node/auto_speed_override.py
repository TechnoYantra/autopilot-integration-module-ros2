from geometry_msgs.msg import Twist 
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State


class AutoSpeedOverride(Node):

    def __init__(self):
        super().__init__('auto_mode_speed_override_node')
        self.auto_override_linvel = self.create_subscription(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', self.auto_override_linvel_cb, 10)
        self.auto_current_linvel = self.create_publisher(Twist,'/auto/current/linvel',10)
        self.autopilot_state = self.create_subscription(State, '/mavros/state', self.autopilot_state_cb, 10)
        self.target_vel = Twist()
        self.current_vel = Twist()
        self.state = State()
        self.cli = self.create_client(ParamSet, '/mavros/param/set')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ParamSet.Request()


    def autopilot_state_cb(self, msg):
        self.state = msg

    def send_request(self):
        self.req.param_id = "WP_SPEED" 
        if not self.target_vel.linear.x == 0.0:
            self.req.value = ParamValue(integer=0, real=self.target_vel.linear.x)
            self.future = self.cli.call_async(self.req)
    
    def auto_override_linvel_cb(self,msg):
        self.target_vel = msg
        self.current_vel = msg


def main(args=None):
    rclpy.init(args=args)

    speed_override_client = AutoSpeedOverride()
    

    while rclpy.ok():
        rclpy.spin(speed_override_client)
        if speed_override_client.state.mode == "AUTO":
            speed_override_client.send_request()
            if speed_override_client.future.done():
                try:
                    response = speed_override_client.future.result()
                except Exception as e:
                    speed_override_client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    speed_override_client.get_logger().info('Success override AUTO speed'+ str(response.success))
                    speed_override_client.current_vel.linear.x = response.value
                    speed_override_client.auto_current_linvel.publish(speed_override_client.current_vel)
        else:
            pass
    speed_override_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
