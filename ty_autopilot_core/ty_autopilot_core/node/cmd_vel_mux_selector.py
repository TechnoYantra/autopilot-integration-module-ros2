from ty_autopilot_core.base_node import BaseNode
from geometry_msgs.msg import Twist
from ty_autopilot_core.mavutils.param_util import RosParam

class MuxSelectorNode():
    def __init__(self):
        BaseNode.get_logger(self).info("Starting MuxSelectorNode as mux_selector_node.")
        self.ext_nav_topic = RosParam.get_param(self, 'autopilot_ros_config.twist_mux_topics.ext_nav')
        self.key_teleop_topic = RosParam.get_param(self, 'autopilot_ros_config.twist_mux_topics.key_teleop')
        self.sp_vel_topic = RosParam.get_param(self, 'autopilot_ros_config.twist_mux_topics.sp_vel')
        BaseNode.get_logger(self).info(str(type(self.ext_nav_topic)))
        BaseNode.subscribe_topic(self, self.ext_nav_topic, Twist, 10)
        BaseNode.subscribe_topic(self, self.key_teleop_topic, Twist, 10)
        BaseNode.subscribe_topic(self, self.sp_vel_topic, Twist, 10)
        BaseNode.subscribe_topic(self, "/cmd_vel/stop", Twist, 10)
        self.mux_vel_publisher = BaseNode.Publisher(self, Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.twist_out = Twist()

    def mux_publisher(self):
        twist_out = Twist()

        if RosParam.get_param(self,parameter_name='mux_select') == self.ext_nav_topic :
            BaseNode.get_logger(self).info("mux_selector using ext_nav velocity.")
            ext_nav_vel = BaseNode.read_topic(self, self.ext_nav_topic)
            twist_out.linear.x = ext_nav_vel.linear.x
            twist_out.linear.y = ext_nav_vel.linear.y
            twist_out.linear.z = ext_nav_vel.linear.z
            twist_out.angular.x = ext_nav_vel.angular.x
            twist_out.angular.y = ext_nav_vel.angular.y
            twist_out.angular.z = ext_nav_vel.angular.z
            self.mux_vel_publisher.publish(twist_out)

        elif RosParam.get_param(self,parameter_name='mux_select') == self.key_teleop_topic:
            key_teleop_vel = BaseNode.read_topic(self, self.key_teleop_topic)
            twist_out.linear.x = key_teleop_vel.linear.x
            twist_out.linear.y = key_teleop_vel.linear.y
            twist_out.linear.z = key_teleop_vel.linear.z
            twist_out.angular.x = key_teleop_vel.angular.x
            twist_out.angular.y = key_teleop_vel.angular.y
            twist_out.angular.z = key_teleop_vel.angular.z
            self.mux_vel_publisher.publish(twist_out)
            BaseNode.get_logger(self).info("mux_selector using key_teleop velocity.")

        elif RosParam.get_param(self,parameter_name='mux_select') == self.sp_vel_topic:
            sp_vel = BaseNode.read_topic(self, self.sp_vel_topic)
            twist_out.linear.x = sp_vel.linear.x
            twist_out.linear.y = sp_vel.linear.y
            twist_out.linear.z = sp_vel.linear.z
            twist_out.angular.x = sp_vel.angular.x
            twist_out.angular.y = sp_vel.angular.y
            twist_out.angular.z = sp_vel.angular.z
            self.mux_vel_publisher.publish(twist_out)
            BaseNode.get_logger(self).info("mux_selector using sp_vel velocity.")

        elif RosParam.get_param(self,parameter_name='mux_select') == 'vel_zero':
            BaseNode.get_logger(self).info("mux_selector using vel_zero velocity.")
            twist_out.linear.x = 0.0
            twist_out.linear.y = 0.0
            twist_out.linear.z = 0.0
            twist_out.angular.x = 0.0
            twist_out.angular.y = 0.0
            twist_out.angular.z = 0.0
            self.mux_vel_publisher.publish(twist_out)
        
