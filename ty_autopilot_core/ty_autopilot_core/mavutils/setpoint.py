import rospy
import mavros

from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, PoseWithCovarianceStamped, \
        Vector3, Vector3Stamped, Point, Quaternion


def get_pub_accel_accel(**msg):
    return rospy.Publisher(mavros.get_topic('setpoint_accel', 'accel'), Vector3Stamped, **msg)


def get_pub_attitude_cmd_vel(**msg):
    return rospy.Publisher(mavros.get_topic('setpoint_attitude', 'cmd_vel'), PoseStamped, **msg)


def get_pub_attitude_throttle(**msg):
    return rospy.Publisher(mavros.get_topic('setpoint_attitude', 'att_throttle'), Float64, **msg)


def get_pub_attitude_pose(**msg):
    return rospy.Publisher(mavros.get_topic('setpoint_attitude', 'attitude'), PoseStamped, **msg)


def get_pub_position_local(**msg):
    return rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, **msg)


def get_pub_velocity_cmd_vel(**msg):
    return rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel_unstammped'), TwistStamped, **msg)
