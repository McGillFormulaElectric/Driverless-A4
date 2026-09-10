"""Composed bring-up: Neil stack + solution DR + solution EKF nodes.

Includes:
  - a4_neil/neil.launch.py       (NOT namespaced — it owns /neil/*)
  - a4_solution/dr.launch.py     (pushed under /<github_user>)
  - a4_solution/ekf.launch.py    (pushed under /<github_user>)

Usage:
    ros2 launch a4_solution bringup.launch.py github_user:=<your-handle>
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
        get_package_share_directory('a4_solution'), 'launch', 'dr.launch.py'
    )
    ekf_launch = os.path.join(
        get_package_share_directory('a4_solution'), 'launch', 'ekf.launch.py'
    )

    neil_group = GroupAction([
        # Neil publishes on absolute topics (/neil/imu, /neil/gps,
        # /neil/truth, /neil/feedback) and must NOT be namespaced.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(neil_launch)),
    ])

    solution_group = GroupAction([
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
            description='Your GitHub username; used as the ROS namespace for the solution nodes.',
        ),
        neil_group,
        solution_group,
    ])
