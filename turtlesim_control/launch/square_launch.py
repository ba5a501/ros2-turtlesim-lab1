"""Starts the whole system, then sends one /draw_square goal automatically."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlesim_control'), 'launch', 'lab1_launch.py')),
        launch_arguments={
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
        }.items(),
    )

    send_square = ExecuteProcess(
        cmd=['ros2', 'action', 'send_goal', '/draw_square',
             'turtlesim_interfaces/action/DrawSquare',
             ['{side_length: ', LaunchConfiguration('side_length'), '}'],
             '--feedback'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('side_length', default_value='4.0'),
        DeclareLaunchArgument('max_linear_speed', default_value='2.0'),
        DeclareLaunchArgument('max_angular_speed', default_value='2.0'),
        system,
        TimerAction(period=4.0, actions=[send_square]),
    ])
