#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.duration import Duration

class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Define the waypoints
        self.waypoints = []

        # Waypoint 1
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 1.0
        pose.pose.position.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        self.waypoints.append(pose)

        # # Waypoint 2
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 1.0
        pose.pose.position.y = -1.0
        pose.pose.orientation.z = -0.7071
        pose.pose.orientation.w = 0.7071
        
        self.waypoints.append(pose)

        # # Waypoint 3
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = -1.0
        pose.pose.position.y = -1.0
        pose.pose.orientation.z = 1.0
        pose.pose.orientation.w = 0.0  
        self.waypoints.append(pose)

        # # Waypoint 4
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = -1.0
        pose.pose.position.y = 1.0
        pose.pose.orientation.z = 0.7071
        pose.pose.orientation.w = 0.7071  
        self.waypoints.append(pose)

        # # Return to starting point
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 1.0
        pose.pose.position.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        self.waypoints.append(pose)

        self.send_waypoints()

    def send_waypoints(self):
        self.get_logger().info('Waiting for "follow_waypoints" action server...')
        self._action_client.wait_for_server()

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        self.get_logger().info('Sending waypoints to the waypoint follower...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Currently at waypoint {feedback.current_waypoint + 1}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server.')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Waypoints navigation completed.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)

if __name__ == '__main__':
    main()
