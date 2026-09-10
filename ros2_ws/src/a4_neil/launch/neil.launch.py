"""Bring up the Neil side: sensor simulator + grader.

Loads scenario parameters from share/a4_neil/config/params.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('a4_neil'), 'config', 'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='a4_neil',
            executable='sensor_sim_node',
            name='sensor_sim_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='a4_neil',
            executable='grader',
            name='grader',
            output='screen',
            parameters=[params_file],
        ),
    ])
