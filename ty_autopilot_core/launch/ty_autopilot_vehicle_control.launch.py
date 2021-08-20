from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
import yaml

def generate_launch_description():
    param_file_name = 'autopilot_integrator.yaml'
    
    configFilepath = os.path.join(get_package_share_directory('ty_autopilot_core'), param_file_name)

    config_param_dir = LaunchConfiguration(
        'config_param_dir',
        default=os.path.join(
            get_package_share_directory('ty_autopilot_core'),
            param_file_name)
            )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_param_dir',
            default_value=config_param_dir,
            description='Full path to autopilot parameter file to load'),

        Node(
            package='ty_autopilot_core',
            executable='vehicle_controller_node',
            name='ty_autopilot_core_node',
            output='screen',
            parameters=[config_param_dir]
        )
    ])

