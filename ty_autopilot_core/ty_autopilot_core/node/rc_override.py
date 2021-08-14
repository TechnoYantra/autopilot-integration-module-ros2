#! /usr/bin/python

import rospy
import sys
import time

from mavros_msgs.msg import OverrideRCIn
from ty_autopilot_core.mavutils.mavenum import RcOption

class MavrosRC():

    def __init__(self):
        rospy.init_node("mavros_rc_override_node")
        self.rc_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)
        self.rate = rospy.Rate(1)

    def publish(self):
        while not rospy.is_shutdown():
            channels=[0]*8
            rc_options = rospy.get_param('rc_options', channels)
            disable_gps = rospy.get_param('disable_gps', False)
            pose_source = rospy.get_param('mav_pose_source',1)
            msg = OverrideRCIn()
            msg.channels = channels 
            if RcOption.EKF_POS_SOURCE.value in rc_options:
                msg.channels[rc_options.index(RcOption.EKF_POS_SOURCE.value)] = 1000+(500*(pose_source-1))
            if RcOption.GPS_DISABLE.value in rc_options:
                msg.channels[rc_options.index(RcOption.GPS_DISABLE.value)] = 1000+(1000*int(disable_gps))
            print(msg)
            self.rc_pub.publish(msg)
            rospy.sleep(0.5)

if __name__ == '__main__':
    mavros_rc = MavrosRC()
    mavros_rc.publish()

