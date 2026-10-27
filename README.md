# Lab 1: ROS 2, Turtlesim, Actions & Services

Design of Mechatronics Systems, Faculty of Engineering, Alexandria National University.
ROS 2 Jazzy on Ubuntu 24.04.

## Team

| Name | ID |
|---|---|
| Badr Ali Seddik | 2304142 |
| Youssef El-Sayed Gaber Abu El-Dahab | 2304230 |

## Packages

| Package | Type | Description |
|---|---|---|
| `lab1_interfaces` | ament_cmake | Custom interfaces: message `TargetPose` (x, y, yaw), actions `GoToPose` and `DrawSquare`, service `SetSpeedLimits`. |
| `lab1_control` | ament_python | Nodes: `goto_pose_server` (goToPose action, speed parameters, speed service), `reset_service` (clear and teleport using turtlesim services), `square_server` (draws a square using 4 goToPose calls). |
| `lab1_bringup` | ament_python | Launch files and the parameter file `config/params.yaml`. |

### Interfaces

- `lab1_interfaces/msg/TargetPose`: `float32 x, y, yaw`
- `lab1_interfaces/action/GoToPose`: goal = `TargetPose target`, result = `success, message`, feedback = `distance_remaining`
- `lab1_interfaces/action/DrawSquare`: goal = `side_length`, result = `success, message`, feedback = `current_side` (1 to 4)
- `lab1_interfaces/srv/SetSpeedLimits`: request = `max_linear, max_angular`, response = `success, message`

### Nodes, actions and services

| Node | Provides |
|---|---|
| `goto_pose_server` | action `/goto_pose`, service `/set_speed_limits`, parameters `max_linear_speed` and `max_angular_speed` |
| `reset_service` | service `/reset_turtle` (`std_srvs/srv/Trigger`) |
| `square_server` | action `/draw_square` |

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy
- `sudo apt install ros-jazzy-turtlesim python3-colcon-common-extensions`

## Build

```bash
mkdir -p ~/lab1_ws/src
cd ~/lab1_ws/src
git clone https://github.com/badr623/lab1-.git
cd ~/lab1_ws
colcon build
source install/setup.bash
```

Run `source ~/lab1_ws/install/setup.bash` in every new terminal.

## Run

Start the whole system (turtlesim and all nodes) with one command:

```bash
ros2 launch lab1_bringup system.launch.py
```

Start the system and draw a square automatically after 4 seconds:

```bash
ros2 launch lab1_bringup square.launch.py
```

## Commands to try every requirement

Run these in a second terminal while `system.launch.py` is running.

**R1: Turtlesim**
```bash
ros2 node list
ros2 topic list
```

**R2: Custom message and interfaces**
```bash
ros2 interface list | grep lab1_interfaces
ros2 interface show lab1_interfaces/msg/TargetPose
ros2 interface show lab1_interfaces/action/GoToPose
ros2 interface show lab1_interfaces/action/DrawSquare
ros2 interface show lab1_interfaces/srv/SetSpeedLimits
```

**R3: goToPose action server** (targets must be inside 0 to 11)
```bash
ros2 action list
ros2 action send_goal /goto_pose lab1_interfaces/action/GoToPose "{target: {x: 8.0, y: 3.0, yaw: 1.57}}" --feedback
ros2 topic echo /turtle1/pose --once
```

**R4: Reset service**
```bash
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 topic echo /turtle1/pose --once
```
The service turns the pen off, calls turtlesim's `teleport_absolute` to (5.54, 5.54, 0), calls turtlesim's `clear`, then turns the pen on again.

**R5: Square action server**
```bash
ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare "{side_length: 3.0}" --feedback
```
The server calls the reset service, then sends exactly 4 goals to `/goto_pose`, one after the other. Side lengths that do not fit in the window (more than about 5.2) or are not positive are rejected.

**R6: Launch files**
```bash
ros2 launch lab1_bringup system.launch.py
ros2 launch lab1_bringup square.launch.py
```

**R7: ROS parameters and speed service**
```bash
ros2 param list
ros2 param get /goto_pose_server max_linear_speed
ros2 param set /goto_pose_server max_linear_speed 1.0
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear: 1.5, max_angular: 1.0}"
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear: -1.0, max_angular: 1.0}"
```
The node reads the limits every control cycle, so changes take effect while the turtle is moving. Zero or negative values are rejected by both the service and `ros2 param set`. The launch file loads the default values from `lab1_bringup/config/params.yaml`.

**R8: Packages**

The repository contains only the three package folders and this README.
