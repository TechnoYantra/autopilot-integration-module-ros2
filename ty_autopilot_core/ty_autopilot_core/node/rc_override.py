import sys
import time
from ty_autopilot_core.mavutils.param_util import RosParam
from ty_autopilot_core.base_node import BaseNode

from mavros_msgs.msg import OverrideRCIn
from ty_autopilot_core.mavutils.mavenum import RcOption

class MavrosRC(BaseNode):
    def __init__(self):
        BaseNode.get_logger(self).info("Starting mavros_rc_override")
        self.rc_pub = BaseNode.Publisher(self,OverrideRCIn, "/mavros/rc/override", 10)

    def publish_rc_channels(self,channels = [0]*8):
        BaseNode.get_logger(self).info(str(channels))
        rc_options = channels
        disable_gps = RosParam.get_param(self, 'disable_gps', False)
        pose_source = RosParam.get_param(self, 'mav_pose_source',1)
        msg = OverrideRCIn()
        msg.channels = channels 
        if RcOption.EKF_POS_SOURCE.value in rc_options:
            msg.channels[rc_options.index(RcOption.EKF_POS_SOURCE.value)] = 1000+(500*(pose_source-1))
        if RcOption.GPS_DISABLE.value in rc_options:
            msg.channels[rc_options.index(RcOption.GPS_DISABLE.value)] = 1000+(1000*int(disable_gps))
        self.rc_pub.publish(msg)


