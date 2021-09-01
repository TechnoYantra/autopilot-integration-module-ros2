#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config_locks = os.path.join(get_package_share_directory('ty_autopilot_core'),'twist_mux_locks.yaml')
    default_config_topics = os.path.join(get_package_share_directory('ty_autopilot_core'),'twist_mux_topics.yaml')
    default_config_joystick = os.path.join(get_package_share_directory('ty_autopilot_core'),'twist_mux_joystick.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_locks',
            default_value=default_config_locks,
            description='Default locks config file'),
        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'),
        DeclareLaunchArgument(
            'config_joy',
            default_value=default_config_joystick,
            description='Default joystick config file'),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/mavros/setpoint_velocity/cmd_vel_unstamped',
            description='cmd vel output topic'),
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', LaunchConfiguration('cmd_vel_out'))},
            parameters=[
                LaunchConfiguration('config_locks'),
                LaunchConfiguration('config_topics'),
                LaunchConfiguration('config_joy')]
        ),

        Node(
            package='twist_mux',
            executable='twist_marker',
            output='screen',
            remappings={('/twist', LaunchConfiguration('cmd_vel_out'))},
            parameters=[{
                'frame_id': 'base_link',
                'scale': 1.0,
                'vertical_position': 1.0}])
            ])