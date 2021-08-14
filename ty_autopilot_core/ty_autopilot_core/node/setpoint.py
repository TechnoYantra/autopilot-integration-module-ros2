from ty_autopilot_core.base_node  import BaseNode

import threading
import time
import mavros

from math import *
from ty_autopilot_core.mavutils import setpoint as SP
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Twist
sensor_qos = rclpy.qos.qos_profile_sensor_data


class SetpointPosition:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        # publisher for mavros/setpoint_position/local
        # self.pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10) #SP.get_pub_position_local(queue_size=10)        
        self.pub = BaseNode.Publisher(self, PoseStamped, "/mavros/setpoint_position/local", sensor_qos)

        # subscriber for mavros/local_position/local
        # self.sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.reached)
        BaseNode.subscribe_topic(self,"/mavros/local_position/pose", PoseStamped, sensor_qos)
        # try:
        #     thread.start_new_thread(self.navigate, ())
        # except:
        #     pass

        self.done = False
        self.done_evt = threading.Event()

    def navigate(self, x, y, z):
        rate = rospy.Rate(10) 

        msg = PoseStamped(
            header=SP.Header(
                frame_id="base_link",  
                stamp=rospy.Time.now()),   
        )

        # while not rospy.is_shutdown() or not self.done:
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        yaw_degrees = 0 
        yaw = radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = SP.Quaternion(*quaternion)
        if not self.done:
            self.pub.publish(msg)
            rate.sleep()

    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z
        print("setting")
        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                # print("sp loop")
                rate.sleep()
        time.sleep(delay)
        self.navigate(x,y,z)

        if self.done:
            # print("True")
            return True


    def reached(self, topic):
        def is_near(msg, x, y):
            #rospy.loginfo("Position %s: local: %d, target: %d, abs diff: %d", msg, x, y, abs(x - y))
            return abs(x - y) < 1.5

        if is_near('X', topic.pose.position.x, self.x) and \
           is_near('Y', topic.pose.position.y, self.y):
            self.done = True
            self.done_evt.set()

class SetpointVelocity:
    def __init__(self):
        self.velocity_pub = rospy.Publisher("sp_vel/cmd_vel", Twist , queue_size=5)
        thread.start_new_thread(self.navigate, ())

        self.twist = Twist()
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

    def set(self,vx,vy,vz,yaw_rate):
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.yaw_rate = yaw_rate
    
    def navigate(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            self.twist.linear.x = self.vx
            self.twist.linear.y = self.vy
            self.twist.linear.z = self.vz

            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = self.yaw_rate
            self.velocity_pub.publish(self.twist)
            rate.sleep()






# def rover_test():
#     rospy.init_node('setpoint_position_demo')
#     rate = rospy.Rate(10)

#     setpoint = SetpointPosition()

#     rospy.loginfo("driving to the right")
#     setpoint.set(5.0, 0.0, 0.0, 5)

#     rospy.loginfo("driving to the left")
#     setpoint.set(0.0, -5.0, 0.0, 5)

#     rospy.loginfo("driving to the right")
#     setpoint.set(-5.0, -5.0, 0.0, 5)

#     rospy.loginfo("driving to the left")
#     setpoint.set(0.0, 5.0, 0.0, 5)

#     offset_x = 0.0
#     offset_y = 0.0
#     offset_z = 0.0
#     sides = 360
#     radius = 20


# if __name__ == '__main__':
#     try:
#         rover_test()
#     except rospy.ROSInterruptException:
#         pass