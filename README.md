# Lab 1 — ROS 2, Turtlesim, Actions & Services

Design of Mechatronics Systems · Alexandria University, Faculty of Engineering
ROS 2 Humble · Ubuntu 22.04

## Team

| Name | ID |
|------|----|
| Ziad Ahmed | 9446 |
| <Member 2> | <ID> |
| <Member 3> | <ID> |

## Packages

| Package | Type | Contents |
|---------|------|----------|
| `turtlesim_interfaces` | ament_cmake | `msg/TargetPose.msg` (x, y, yaw) · `action/GoToPose.action` · `action/DrawSquare.action` · `srv/SetSpeedLimits.srv` |
| `turtlesim_control` | ament_python | `pose_action_server` (goToPose + speed params + `/set_speed_limits`) · `reset_service_node` (`/reset_turtle`) · `square_action_server` (`/draw_square`) · launch files · `config/params.yaml` |

| Interface | Name | Type |
|-----------|------|------|
| Action | `/go_to_pose` | `turtlesim_interfaces/action/GoToPose` |
| Action | `/draw_square` | `turtlesim_interfaces/action/DrawSquare` |
| Service | `/set_speed_limits` | `turtlesim_interfaces/srv/SetSpeedLimits` |
| Service | `/reset_turtle` | `std_srvs/srv/Trigger` |
| Params | `/pose_action_server` | `max_linear_speed`, `max_angular_speed`, `control_rate` |

## Build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this-repo-url> .
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch turtlesim_control lab1_launch.py                       # whole system
ros2 launch turtlesim_control lab1_launch.py max_linear_speed:=1.0  # custom speed limits
ros2 launch turtlesim_control square_launch.py side_length:=3.0     # system + draws a square
```

## Try every requirement (new terminal, `source ~/ros2_ws/install/setup.bash` first)

```bash
# R2 interfaces
ros2 interface show turtlesim_interfaces/msg/TargetPose
ros2 interface show turtlesim_interfaces/action/GoToPose

# R3 goToPose
ros2 action send_goal /go_to_pose turtlesim_interfaces/action/GoToPose "{target_pose: {x: 8.0, y: 8.0, yaw: 1.57}}" --feedback

# R4 reset
ros2 service call /reset_turtle std_srvs/srv/Trigger

# R5 square (4 goToPose calls)
ros2 action send_goal /draw_square turtlesim_interfaces/action/DrawSquare "{side_length: 4.0}" --feedback

# R7 parameters + service
ros2 param get /pose_action_server max_linear_speed
ros2 param set /pose_action_server max_linear_speed 1.0
ros2 service call /set_speed_limits turtlesim_interfaces/srv/SetSpeedLimits "{max_linear_speed: 0.5, max_angular_speed: 1.0}"
ros2 service call /set_speed_limits turtlesim_interfaces/srv/SetSpeedLimits "{max_linear_speed: -1.0, max_angular_speed: 1.0}"   # refused
```

## Bonus: design patterns

| Pattern | Where | Why |
|---------|-------|-----|
| **State** | `controllers.py` — `RotateToTarget → DriveToTarget → RotateToYaw → Done` | Each phase of a goal owns its own command and transition logic, replacing a nested if/else loop. Adding a phase means adding one class. |
| **Strategy** | `controllers.py` — `ControlStrategy` / `ProportionalStrategy` | Gains and saturation are swappable without touching the states or the ROS node. |
| **Facade** | `reset_service_node.py` — `TurtlesimFacade` | Hides the three raw turtlesim clients (`set_pen`, `teleport_absolute`, `clear`) behind simple methods. |

The motion logic has no ROS imports, so it is unit tested with `colcon test --packages-select turtlesim_control`.

## Bonus: bag file

```bash
# record while the square runs
ros2 bag record -o ~/ros2_ws/bags/square_run /turtle1/cmd_vel /turtle1/pose
ros2 bag info ~/ros2_ws/bags/square_run
# replay: reset first so the turtle starts from the same pose
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 bag play ~/ros2_ws/bags/square_run --topics /turtle1/cmd_vel
```

Only `/turtle1/cmd_vel` is played back — replaying `/turtle1/pose` would fight the live pose published by turtlesim.
