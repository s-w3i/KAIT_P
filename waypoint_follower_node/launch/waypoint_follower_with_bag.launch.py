import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.events import Shutdown as ShutdownEvent

def generate_launch_description():
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='cmd_vel_bag',
        description='Name of the rosbag output directory'
    )

    bag_name = LaunchConfiguration('bag_name')

    # Node to run the waypoint follower
    waypoint_follower_node = Node(
        package='waypoint_follower_node',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
    )

    # Command to start ros2 bag recording with multiple topics
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/cmd_vel', '/imu', '/scan', '/odom',
            '-o', bag_name
        ],
        output='screen',
    )

    # Event handler to stop rosbag recording when waypoint follower node exits
    stop_rosbag_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=waypoint_follower_node,
            on_exit=[
                EmitEvent(event=ShutdownEvent(
                    reason='Waypoint follower node has exited.'
                )),
            ],
        )
    )

    return LaunchDescription([
        bag_name_arg,
        rosbag_record,
        waypoint_follower_node,
        stop_rosbag_on_exit,
    ])
