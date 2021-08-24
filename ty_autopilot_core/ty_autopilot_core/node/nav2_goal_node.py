from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class NavGoal(Node):
    def __init__(self):
        super().__init__('minimal_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseStamped, '/nav/target_goal', self.target_goal_callback, 10)
        self.nav_goal_pub = self.create_publisher( PoseStamped, "/goal_update", 10)
        self.target_goal = PoseStamped()

    def target_goal_callback(self, msg):
        self.target_goal = msg
        self.nav_goal_pub.publish(msg)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected resending goal')
            self.send_goal()
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info('Goal Result: {0}'.format(result))
    
        self.get_logger().info('Goal status: {0}'.format(status))
        if status == 6:
            self.send_goal()


    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.target_goal.pose.position.x
        goal_pose.pose.position.y = self.target_goal.pose.position.y
        goal_pose.pose.orientation.w = 1.0
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    action_client = NavGoal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()