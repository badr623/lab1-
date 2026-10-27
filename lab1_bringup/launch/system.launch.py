import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('lab1_bringup'), 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
        ),
        Node(
            package='lab1_control',
            executable='goto_pose_server',
            name='goto_pose_server',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='lab1_control',
            executable='reset_service',
            name='reset_service',
            output='screen',
        ),
        Node(
            package='lab1_control',
            executable='square_server',
            name='square_server',
            output='screen',
        ),
    ])
