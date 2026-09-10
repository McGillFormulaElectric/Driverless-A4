"""Launch A4.1 dead-reckoning node under the user's GitHub-username namespace.

Loads parameters from share/a4_new_member/config/params.yaml.

Usage:
    ros2 launch a4_new_member dr.launch.py github_user:=<your-handle>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    github_user = LaunchConfiguration('github_user')
    params_file = os.path.join(
        get_package_share_directory('a4_new_member'), 'config', 'params.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'github_user',
            description='Your GitHub username; used as the ROS namespace.',
        ),
        Node(
            package='a4_new_member',
            executable='dead_reckoning_node',
            name='dead_reckoning_node',
            namespace=github_user,
            output='screen',
            parameters=[params_file],
        ),
    ])
