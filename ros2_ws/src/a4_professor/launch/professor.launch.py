"""Bring up the professor side: sensor simulator + grader."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='a4_professor',
            executable='sensor_sim_node',
            name='sensor_sim_node',
            output='screen',
        ),
        Node(
            package='a4_professor',
            executable='grader',
            name='grader',
            output='screen',
        ),
    ])
