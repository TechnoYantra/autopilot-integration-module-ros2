import threading
from ty_autopilot_core.base_node import BaseNode
import rclpy.qos
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import tf_transformations
from ty_autopilot_core.mavutils.param_util import RosParam

class FrameTransformer:
    def __init__(self):
        self.log = BaseNode.get_logger(self)
        self.log.info("Started FrameTransformer")
        self.viso_roll = RosParam.get_param(self, parameter_name='autopilot_ros_config.ext_pose_estimation.odometry.frame_pose.orientation.x', default_value=0.0)  # apply_r
        self.viso_pitch = RosParam.get_param(self, parameter_name='autopilot_ros_config.ext_pose_estimation.odometry.frame_pose.orientation.y', default_value=0.0)  # apply_p
        self.viso_yaw = RosParam.get_param(self, parameter_name='autopilot_ros_config.ext_pose_estimation.odometry.frame_pose.orientation.z', default_value=0.0)  # apply_y
        self.gamma_world = RosParam.get_param(self, parameter_name='autopilot_ros_config.ext_pose_estimation.source_frame_orinetation.z', default_value=1.576)  # apply_wgamma
        self.pose_out = PoseStamped()
        self.external_odom_topic = RosParam.get_param(self, 'autopilot_ros_config.ext_pose_estimation.odometry.topic')
        self.log.info(str(type(self.external_odom_topic)))
        self.pub_transform = BaseNode.Publisher(self, PoseStamped, "/mavros/vision_pose/pose", rclpy.qos.qos_profile_sensor_data)
        BaseNode.subscribe_topic(self, topic=self.external_odom_topic, type=Odometry, qos_profile=rclpy.qos.qos_profile_sensor_data) ## use base_node for subscribers


    def transform(self):
        pose_msg = BaseNode.read_topic(self, self.external_odom_topic)
        # BaseNode.get_logger(self).info(str(pose_msg))
        pose = pose_msg.pose.pose
        # BaseNode.get_logger(self).info(str(pose.position))

        # Rotation from original world frame having z forward to world frame with y forward.
        pose_x = math.cos(self.gamma_world) * pose.position.x + \
            math.sin(self.gamma_world) * pose.position.y
        pose_y = -math.sin(self.gamma_world) * pose.position.x + \
            math.cos(self.gamma_world) * pose.position.y
        pose_z = pose.position.z

        #  Rotation from camera to body frame.
        quat_viso_to_body_x = tf_transformations.quaternion_from_euler(self.viso_roll, 0.0, 0.0)
        quat_viso_to_body_y = tf_transformations.quaternion_from_euler(0.0, self.viso_pitch, 0.0)
        quat_viso_to_body_z = tf_transformations.quaternion_from_euler(0.0, 0.0, self.viso_yaw)

        # # Rotate body frame 90 degree (align body x with world y at launch)
        quat_rot_z = tf_transformations.quaternion_from_euler(0.0, 0.0, -self.gamma_world)
        rot_x = pose.orientation.x
        rot_y = pose.orientation.y
        rot_z = pose.orientation.z
        rot_w = pose.orientation.w

        # quat_body = quat_rot_z * [rot_x, rot_y, rot_z, rot_w]  * quat_viso_to_body_x * quat_viso_to_body_y * quat_viso_to_body_z
        quat_body = tf_transformations.quaternion_multiply(quat_rot_z, [rot_x, rot_y, rot_z, rot_w])
        quat_body = tf_transformations.quaternion_multiply(quat_body, quat_viso_to_body_x)
        quat_body = tf_transformations.quaternion_multiply(quat_body, quat_viso_to_body_y)
        quat_body = tf_transformations.quaternion_multiply(quat_body, quat_viso_to_body_z)

        self.pose_out.header.stamp = BaseNode.get_clock(self).now().to_msg()
        self.pose_out.header.frame_id = "map"
        self.pose_out.pose.position.x = pose_x
        self.pose_out.pose.position.y = pose_y
        self.pose_out.pose.position.z = pose_z

        self.pose_out.pose.orientation.x = quat_body[0]
        self.pose_out.pose.orientation.y = quat_body[1]
        self.pose_out.pose.orientation.z = quat_body[2]
        self.pose_out.pose.orientation.w = quat_body[3]
        self.pub_transform.publish(self.pose_out)

        RosParam.set_param(self, parameter_name="frame_transformer_beat", parameter_value=True)



