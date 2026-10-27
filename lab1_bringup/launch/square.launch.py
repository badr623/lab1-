import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('lab1_bringup'),
            'launch', 'system.launch.py')))

    trigger_square = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal', '/draw_square',
                 'lab1_interfaces/action/DrawSquare',
                 '{side_length: 3.0}', '--feedback'],
            output='screen')])

    return LaunchDescription([system, trigger_square])
