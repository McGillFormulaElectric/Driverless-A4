"""Composed bring-up: Neil stack + new_member DR + new_member EKF nodes.

Includes:
  - a4_neil/neil.launch.py       (NOT namespaced — it owns /neil/*)
  - a4_new_member/dr.launch.py     (pushed under /<github_user>)
  - a4_new_member/ekf.launch.py    (pushed under /<github_user>)

Usage:
    ros2 launch a4_new_member bringup.launch.py github_user:=<your-handle>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    github_user = LaunchConfiguration('github_user')

    neil_launch = os.path.join(
        get_package_share_directory('a4_neil'), 'launch', 'neil.launch.py'
    )
    dr_launch = os.path.join(
        get_package_share_directory('a4_new_member'), 'launch', 'dr.launch.py'
    )
    ekf_launch = os.path.join(
        get_package_share_directory('a4_new_member'), 'launch', 'ekf.launch.py'
    )

    neil_group = GroupAction([
        # Neil publishes on absolute topics (/neil/imu, /neil/gps,
        # /neil/truth, /neil/feedback) and must NOT be namespaced.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(neil_launch)),
    ])

    new_member_group = GroupAction([
        PushRosNamespace(github_user),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dr_launch),
            launch_arguments={'github_user': github_user}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            launch_arguments={'github_user': github_user}.items(),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'github_user',
            description='Your GitHub username; used as the ROS namespace for the new_member nodes.',
        ),
        neil_group,
        new_member_group,
    ])
