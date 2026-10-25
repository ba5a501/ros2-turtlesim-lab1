"""Starts turtlesim + every server with their parameters (R1, R6, R7)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('turtlesim_control'), 'config', 'params.yaml')

    max_lin = LaunchConfiguration('max_linear_speed')
    max_ang = LaunchConfiguration('max_angular_speed')

    return LaunchDescription([
        DeclareLaunchArgument('max_linear_speed', default_value='2.0',
                              description='Max linear speed (m/s)'),
        DeclareLaunchArgument('max_angular_speed', default_value='2.0',
                              description='Max angular speed (rad/s)'),

        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),

        Node(
            package='turtlesim_control',
            executable='pose_action_server',
            name='pose_action_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'max_linear_speed': ParameterValue(max_lin, value_type=float),
                    'max_angular_speed': ParameterValue(max_ang, value_type=float),
                },
            ],
        ),
        Node(
            package='turtlesim_control',
            executable='reset_service_node',
            name='reset_service_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='turtlesim_control',
            executable='square_action_server',
            name='square_action_server',
            output='screen',
            parameters=[params_file],
        ),
    ])
