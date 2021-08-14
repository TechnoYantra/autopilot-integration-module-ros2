#!/usr/bin/env python
import rospy
import threading
import time
import csv
import os
import sys
import actionlib
import math
from ty_autopilot_core.mavutils import alvinxy
from mavutils.topictools import Topic
from ty_autopilot_core.mavutils.mavenum import (
    RosFrame, SetpointType, MavrosTopics, MavFrames)
reload(alvinxy)
from ty_autopilot_msgs.srv import SendMovebaseGoal, SendMovebaseGoalResponse
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import NavSatFix
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
from tf.transformations import quaternion_from_euler


class ServiceSendMovebaseGoal():
    def __init__(self):
        # rospy.init_node('service_send_movebase_goal')
        rospy.loginfo("Starting ServiceGetVehicleState.")
        self.set_nav_goal       = rospy.Service('/ty_autopilot/set_nav_movebase_goal', SendMovebaseGoal, self.set_nav_movebase_goal_service)
        self.topic = Topic()
        self.topic.subscribe_topic(topic="/mavros/global_position/global", type=NavSatFix)
        # self.global_pose_sub    = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.global_pose_callback)
        
        self.mb_action_client   = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        self.ORIGIN_SET         = False 
        self.lat_orgin          = 0.0
        self.long_orgin         = 0.0
        self.global_pose        = NavSatFix()

        # rospy.spin()
            
    def global_pose_callback(self, msg):
        self.global_pose = msg

    def set_nav_movebase_goal_service(self,req):
        resp = SendMovebaseGoalResponse()
        goal = MoveBaseGoal()
        local_x = 0.0
        local_y = 0.0
        target_frame = req.frame
        if target_frame == RosFrame.GLOBAL_FRAME.value:

            if not self.ORIGIN_SET:

                self.lat_orgin = self.topic.read("/mavros/global_position/global").latitude
                self.long_orgin = self.topic.read("/mavros/global_position/global").longitude
                self.ORIGIN_SET = True

            local_x,local_y = alvinxy.ll2xy(req.x_lat, req.y_long, self.lat_orgin, self.long_orgin)
            rospy.loginfo("global_to_local" + str([local_x, local_y]))

        elif target_frame == RosFrame.LOCAL_FRAME.value or target_frame == RosFrame.BODY_FRAME.value:
            local_x = req.x_lat
            local_y = req.y_long

            global_x,global_y = alvinxy.xy2ll(req.x_lat, req.y_long, self.lat_orgin, self.long_orgin)
            rospy.loginfo("local_to_global" + str( [global_x, global_y]) )

        else:
            rospy.loginfo("Please provide frame type of given coordinate, frames:(global/local)")

        self.mb_action_client.wait_for_server()
        goal.target_pose.header.frame_id    = "map"
        goal.target_pose.header.stamp       = rospy.Time.now()
        goal.target_pose.pose.position.x    = local_x
        goal.target_pose.pose.position.y    = local_y

        quaternion                          = quaternion_from_euler(0, 0, math.radians(req.yaw))
        goal.target_pose.pose.orientation   = Quaternion(*quaternion)

        print(self.mb_action_client.send_goal(goal))
        rospy.loginfo(goal)
        """
        # wait for the client to send goal
        wait = self.client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not up
        if not wait:
            resp.success = False
            resp.message = "Action server not available!"
        else:
            resp.success = True
            resp.message = "Goal sent"   
        """
        resp.success = True
        resp.message = "Goal Sent"
        return resp



# if __name__ == "__main__":
#     service_server = ServiceSendMovebaseGoal()