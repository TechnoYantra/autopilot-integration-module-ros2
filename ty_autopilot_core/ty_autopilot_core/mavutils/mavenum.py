#!/usr/bin/env python3

from enum import Enum

class MavFrames(Enum):
    LOCAL_NED = "LOCAL_NED"
    GLOBAL = "GLOBAL"
    MISSION = "MISSION"
    GLOBAL_RELATIVE_ALT = "GLOBAL_RELATIVE_ALT"
    LOCAL_ENU = "LOCAL_ENU"
    GLOBAL_INT = "GLOBAL_INT"
    GLOBAL_RELATIVE_ALT_INT = "GLOBAL_RELATIVE_ALT_INT"
    LOCAL_OFFSET_NED = "LOCAL_OFFSET_NED"
    BODY_NED = "BODY_NED"
    BODY_OFFSET_NED = "BODY_OFFSET_NED"
    GLOBAL_TERRAIN_ALT = "GLOBAL_TERRAIN_ALT"
    GLOBAL_TERRAIN_ALT_INT = "GLOBAL_TERRAIN_ALT_INT"
    BODY_FRD = "BODY_FRD"
    LOCAL_FRD = "LOCAL_FRD"
    LOCAL_FLU = "LOCAL_FLU"

class SetpointType(Enum):
    POSITION_LOCAL = "POSITION_LOCAL"
    POSITION_GLOBAL = "POSITION_GLOBAL"
    VELOCITY = "VELOCITY"
    WAYPOINTS = "WAYPOINTS"

class Mode(Enum):
    MANUAL = "MANUAL"
    GUIDED = "GUIDED"
    AUTO = "AUTO"
    ACRO = "ACRO"
    HOLD = "HOLD"
    RTL = "RTL"
    OFFBOARD = "OFFBOARD"
    MISSION = "MISSION"
    MAVCOMMAND = "MAVCOMMAND"

class VehicleClass(Enum):
    ROVER = 1

class VehicleType(Enum):
    SKID_STEER = 0
    DIFFERENTIAL = 0
    OMNI_3 = 1
    OMNI_X = 2
    OMNI_PLUS = 3

class ServoOutputFunction(Enum):
    THROTTLE_LEFT = 73
    THROTTLE_RIGHT = 74

    MOTOR1 = 33
    MOTOR2 = 34
    MOTOR3 = 35
    MOTOR4 = 36

class MavrosTopics(Enum):
    MAV_VISION_POSE_STAMPED = "/mavros/vision_pose/pose"
    MAV_VISION_POSE_STAMPED_COV = "/mavros/vision_pose/pose_cov"
    MAV_VISION_SPEED_STAMPED = "/mavros/vision_speed/speed_twist"
    MAV_ODOMETRY_IN = "/mavros/odometry/in"
    MAV_ODOMETRY_OUT = "/mavros/odometry/out"
    MAV_SETPOINT_VEL_UNSTAMPED = "/mavros/setpoint_velocity/cmd_vel_unstamped"
    
class RosFrame(Enum):
    LOCAL_FRAME = "map"
    BODY_FRAME = "base_link"
    GLOBAL_FRAME = "global"

class RcOption(Enum):
    EKF_POS_SOURCE = 90
    GPS_DISABLE = 65