import rclpy
import sys
from rclpy.node import Node
from mavros_msgs.msg import WaypointList, HomePosition
from ty_autopilot_core.mavutils import alvinxy, param_util, geoutil
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PublishPath(Node):

    def __init__(self):
        super().__init__('path_publisher_node')
        waypoint_sub = self.create_subscription(WaypointList, "/mavros/mission/waypoints", self.waypoint_cb, 10)
        home_sub = self.create_subscription(HomePosition, "/mavros/home_position/home", self.home_cb,10)
        self.mission_path = self.create_publisher(Path, "/mission/path_enu", 10 )
        self.home = HomePosition()
    
    def home_cb(self,msg):
        self.home = msg
        
    def waypoint_cb(self,msg):
        waypoints_list = msg
        self.get_logger().info(str(waypoints_list)) 
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        point_array = list()
        pose = PoseStamped()
        points = []
        for waypoint in range(len(waypoints_list.waypoints)):
            target_waypoint = waypoints_list.waypoints[waypoint]
            print(waypoint)
            lat_orgin = self.home.geo.latitude
            long_orgin = self.home.geo.longitude
            local_y,local_x = alvinxy.ll2xy(target_waypoint.x_lat, target_waypoint.y_long, lat_orgin, long_orgin)
            if local_x > 1000.0 or local_x < -1000.0:
                local_x = 0.0
            if local_y > 1000.0 or local_y < -1000.0:
                local_y = 0.0
            points.append([local_x, local_y])
            self.get_logger().info(str(points))
        
        for point in range(len(points[:-1])):
            point_array.extend(geoutil.get_pose_between_wp(points[point][0], points[point][1], points[point+1][0], points[point+1][1]))
        
        for point in point_array:
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)         

        self.mission_path.publish(path)

            
            





def main(args=sys.argv):
    rclpy.init(args=args)
    path_publisher_node = PublishPath()
    rclpy.spin(path_publisher_node)
    path_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()