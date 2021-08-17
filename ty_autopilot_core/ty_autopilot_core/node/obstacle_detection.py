from ty_autopilot_core.mavutils.param_util import RosParam
from ty_autopilot_core.base_node import BaseNode
import rclpy
from sensor_msgs.msg import LaserScan
from rclpy.node import Node


class ObstacleDetector(BaseNode):
    def __init__(self):
        BaseNode.get_logger(self).info('obsticle_detection_node')
        sub = BaseNode.subscribe_topic(self, '/laser', LaserScan, 10) #We subscribe to the laser's topic
        # print("Obsticle: ",RosParam.set_param("obstacle_detected", False))


    def obstacle_detection(self):
        sample = list()
        # self.get_logger().info("Obsticle: "+ str(RosParam.get_param(self, "obstacle_detected", False)))

        for angle in range(RosParam.get_param(self, "start_angle", 90)*2,RosParam.get_param(self, "end_angle", 270)*2, 2):
            if BaseNode.read_topic(self,'/laser').ranges[angle] < RosParam.get_param(self, "safety_range_max", 2) and BaseNode.read_topic(self,'/laser').ranges[angle] > RosParam.get_param(self, "safety_range_min", 0.5):
                # print(angle/2, ": " ,msg.ranges[angle])
                sample.append(BaseNode.read_topic(self,'/laser').ranges[angle])
            else:
                pass
            
        if sample:
            if max(sample) > RosParam.get_param(self, "safety_range_min", 0.5) and max(sample) < RosParam.get_param(self, "safety_range_max", 5):
                RosParam.set_param(self, "obstacle_detected", True)
                # print("Obsticle: ",True)
            else:
                RosParam.set_param(self, "obstacle_detected", False)
                # print("Obsticle: ",False)
        else:
            RosParam.set_param(self, "obstacle_detected", False)


