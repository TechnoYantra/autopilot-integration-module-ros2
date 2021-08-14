#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import struct
import json
import math
import time
import threading
import tf2_ros as tf
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu
from mavros_msgs.msg import RCOut
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from pymavlink.rotmat import Vector3, Matrix3
from pymavlink.quaternion import Quaternion
import message_filters


class RobotState(Node):
    def __init__(self):
        super().__init__('robot_state')
        robot_state_sub = message_filters.Subscriber(self,Odometry, "/odom")
        imu_sub         = message_filters.Subscriber(self, Imu, "/imu")
        self.apm_cmd_vel_pub = self.create_publisher(Twist, "/apm/cmd_vel/out", 5)
        self.pose_pub = self.create_publisher(PoseStamped, "/ned/pose/out", 5)
        self.imu_pub = self.create_publisher(Imu, "/ned/imu/out", 5)
        ts = message_filters.ApproximateTimeSynchronizer([robot_state_sub, imu_sub], 1, 0.1)
        ts.registerCallback(self.callback)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 9002))
        self.sock.settimeout(5)
        self.timestamp  = 0.0
        self.imu        = [[0]*3 , [0]*3]
        self.position   = [0]*3
        self.attitude   = [0]*3
        self.velocity   = [0]*3
        self.quaternion = Quaternion([0,0,0,1])
        self.msg_pack   = {}
        self.pwm        = [0]*16
        self.RATE_HZ = 200
        self.multiplier = 100.0
        self.time_start = self.get_clock().now().nanoseconds/1000000000

    def callback(self, robot_state, imu):
        self.robot_state = robot_state
        self.imu_raw = imu
        # try:
        data,address = self.sock.recvfrom(100)
        parse_format    = 'HHI16H'
        magic           = 18458
        if len(data) != struct.calcsize(parse_format):
            print("got packet of len %u, expected %u" % (len(data), struct.calcsize(parse_format)))
        decoded = struct.unpack(parse_format,data)
        if magic != decoded[0]:
            print("Incorrect protocol magic %u should be %u" % (decoded[0], magic))
        frame_rate_hz   = decoded[1]
        frame_count     = decoded[2]
        self.pwm        = decoded[3:]
        self.motor_pwm_pub(self.pwm)
        self.position = [ self.robot_state.pose.pose.position.x, -self.robot_state.pose.pose.position.y, -self.robot_state.pose.pose.position.z ]
        self.attitude = [ self.robot_state.pose.pose.orientation.x, -self.robot_state.pose.pose.orientation.y, -self.robot_state.pose.pose.orientation.z, self.robot_state.pose.pose.orientation.w ] ##
        self.velocity = [ self.robot_state.twist.twist.linear.x, -self.robot_state.twist.twist.linear.y, -self.robot_state.twist.twist.linear.z ]
        dcm = self.quaternion.dcm
        accel = dcm.transposed() * Vector3(self.imu_raw.linear_acceleration.x, self.imu_raw.linear_acceleration.y, self.imu_raw.linear_acceleration.z)
        gyro = dcm.transposed() * Vector3(self.imu_raw.angular_velocity.x, -self.imu_raw.angular_velocity.y, -self.imu_raw.angular_velocity.z)
        self.imu[0] = [ self.imu_raw.linear_acceleration.x, self.imu_raw.linear_acceleration.y, self.imu_raw.linear_acceleration.z ]
        self.imu[1] = [ self.imu_raw.angular_velocity.x, self.imu_raw.angular_velocity.y, self.imu_raw.angular_velocity.z ]
        self.ned_pose_pub(self.position, self.attitude ,self.imu[0], self.imu[1])
        stamp = self.get_clock().now().nanoseconds/1000000000 - self.time_start
        self.msg_pack = {
            "timestamp" : int(stamp*10),
            "imu" : {
                "accel_body" : self.imu[0],
                "gyro" : self.imu[1]
            },
            "position" : self.position,
            "quaternion" : self.attitude,
            "velocity" : self.velocity
        }
        JSON_string = "\n" + json.dumps(self.msg_pack,separators=(',', ':')) + "\n"
        # print(type(JSON_string.encode("ascii", "ignore")))
        self.sock.sendto(bytes(JSON_string.encode("ascii", "ignore")), address)

        # except Exception as ex:
        #     time.sleep(0.01)

    def motor_pwm_pub(self,pwm):
        steer_max = 45.0
        throttle_max = 200.0
        twist_out = Twist()
        twist_out.linear.x = ( self.clamp((pwm[0] - 1500.0)/500.0, -1, 1) + self.clamp((pwm[2] - 1500.0)/500.0, -1, 1) ) * throttle_max
        twist_out.angular.z = (self.clamp((pwm[0] - 1500.0)/500.0, -1, 1) - self.clamp((pwm[2] - 1500.0)/500.0, -1, 1) ) * math.radians(steer_max) * -1 
        self.apm_cmd_vel_pub.publish(twist_out)
    
    def clamp(self,v,min_v,max_v):
        if v < min_v:
            v = min_v
        if v > max_v:
            v = max_v
        return v

    def ned_pose_pub(self,position,attitude, acc, gyro):
        pose = PoseStamped()
        imu = Imu()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.x = attitude[0]
        pose.pose.orientation.y = attitude[1]
        pose.pose.orientation.z = attitude[2]
        pose.pose.orientation.w = attitude[3]
        self.pose_pub.publish(pose)

        imu.linear_acceleration.x = acc[0]
        imu.linear_acceleration.y = acc[1]
        imu.linear_acceleration.z = acc[2]
        imu.angular_velocity.x = gyro[0]
        imu.angular_velocity.y = gyro[1]
        imu.angular_velocity.z = gyro[2]
        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    robot_state = RobotState()
    rclpy.spin(robot_state)
    robot_state.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()