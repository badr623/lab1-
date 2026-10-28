import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

GOTO = 'ros2 action send_goal /goto_pose lab1_interfaces/action/GoToPose '
SPEED = 'ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits '


def step(title, script):
    return ExecuteProcess(
        cmd=['bash', '-c', 'echo; echo "===== ' + title + ' ====="; ' + script],
        output='screen')


def generate_launch_description():
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('lab1_bringup'),
            'launch', 'system.launch.py')))

    steps = [
        step('R1: turtlesim, nodes and topics',
             'ros2 node list; ros2 topic list'),
        step('R2: custom message and interfaces',
             'ros2 interface list | grep lab1_interfaces; '
             'ros2 interface show lab1_interfaces/msg/TargetPose'),
        step('R3: goToPose action',
             'ros2 action list; '
             + GOTO + '"{target: {x: 8.0, y: 3.0, yaw: 1.57}}" --feedback; '
             'ros2 topic echo /turtle1/pose --once; '
             + GOTO + '"{target: {x: 2.0, y: 9.0, yaw: 3.0}}"; '
             + GOTO + '"{target: {x: 15.0, y: 3.0, yaw: 0.0}}"'),
        step('R4: reset service',
             'ros2 service call /reset_turtle std_srvs/srv/Trigger; '
             'ros2 topic echo /turtle1/pose --once'),
        step('R5: square action (4 goToPose goals)',
             'ros2 action send_goal /draw_square '
             'lab1_interfaces/action/DrawSquare "{side_length: 3.0}" --feedback'),
        step('R6: everything started by launch files',
             'echo "demo.launch.py includes system.launch.py"; '
             'ros2 node list'),
        step('R7: parameters and speed service',
             'ros2 param list /goto_pose_server; '
             'ros2 param get /goto_pose_server max_linear_speed; '
             + SPEED + '"{max_linear: 1.5, max_angular: 1.5}"; '
             'ros2 param get /goto_pose_server max_linear_speed; '
             + SPEED + '"{max_linear: -1.0, max_angular: 1.0}"; '
             'ros2 param set /goto_pose_server max_angular_speed 0.0; '
             + SPEED + '"{max_linear: 0.5, max_angular: 0.5}"; '
             + GOTO + '"{target: {x: 9.0, y: 9.0, yaw: 0.0}}"; '
             + SPEED + '"{max_linear: 2.0, max_angular: 2.0}"; '
             'echo "===== DONE ====="'),
    ]

    chain = [
        RegisterEventHandler(OnProcessExit(target_action=prev, on_exit=[nxt]))
        for prev, nxt in zip(steps, steps[1:])
    ]

    return LaunchDescription(
        [system, TimerAction(period=4.0, actions=[steps[0]])] + chain)
