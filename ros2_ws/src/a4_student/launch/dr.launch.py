"""Launch A4.1 dead-reckoning node under the student's GitHub-username namespace.

Usage:
    ros2 launch a4_student dr.launch.py github_user:=<your-handle>
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    github_user = LaunchConfiguration('github_user')
    return LaunchDescription([
        DeclareLaunchArgument(
            'github_user',
            description='Your GitHub username; used as the ROS namespace.',
        ),
        Node(
            package='a4_student',
            executable='dead_reckoning_node',
            name='dead_reckoning_node',
            namespace=github_user,
            output='screen',
        ),
    ])
