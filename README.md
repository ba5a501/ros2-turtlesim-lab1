# Lab 1 — ROS 2, Turtlesim, Actions & Services

Design of Mechatronics Systems · Alexandria University, Faculty of Engineering
ROS 2 Humble · Ubuntu 22.04

## Team

| Name | ID |
|------|----|
| Yasine Bekhiet | 9337 |
| Anas Refaat | 9482 |
| Ziad | 9446 |

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

Use Ubuntu 22.04 with ROS 2 Humble installed. Install the build tools with
`sudo apt install python3-colcon-common-extensions python3-rosdep ros-humble-turtlesim`.
If rosdep has not been initialized, run `sudo rosdep init`, then `rosdep update`.
Replace `<this-repo-url>` below with this repository's GitHub URL.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this-repo-url> ros2-lab1
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

## Run

Choose one launch command at a time; stop it with Ctrl+C before starting another.
In every new terminal, source `/opt/ros/humble/setup.bash` and
`~/ros2_ws/install/setup.bash`.

```bash
ros2 launch turtlesim_control lab1_launch.py                       # whole system
ros2 launch turtlesim_control lab1_launch.py max_linear_speed:=1.0  # custom speed limits
ros2 launch turtlesim_control square_launch.py side_length:=3.0     # system + draws a square
```

## Try every requirement (new terminal, `source ~/ros2_ws/install/setup.bash` first)

```bash
# R1 / R6: turtlesim and all three server nodes
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 topic echo /turtle1/pose --once

# R2 interfaces
ros2 interface show turtlesim_interfaces/msg/TargetPose
ros2 interface show turtlesim_interfaces/action/GoToPose
ros2 interface show turtlesim_interfaces/action/DrawSquare
ros2 interface show turtlesim_interfaces/srv/SetSpeedLimits

# R3 goToPose
ros2 action send_goal /go_to_pose turtlesim_interfaces/action/GoToPose "{target_pose: {x: 8.0, y: 8.0, yaw: 1.57}}" --feedback

# R4 reset
ros2 service call /reset_turtle std_srvs/srv/Trigger

# R5 square (4 goToPose calls)
ros2 action send_goal /draw_square turtlesim_interfaces/action/DrawSquare "{side_length: 4.0}" --feedback

# R7 parameters + service
ros2 param list /pose_action_server
ros2 param get /pose_action_server max_angular_speed
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

## Optional bonus: bag file (not included or verified)

A recording is not supplied. Run these steps on the ROS machine before claiming this bonus.
Start recording in one terminal, send the square goal in another, wait for completion,
then stop recording with Ctrl+C before inspecting or playing it. Keep controllers idle
during playback. The recording stays outside the source repository.

```bash
# record while the square runs
ros2 bag record -o ~/ros2_ws/bags/square_run /turtle1/cmd_vel /turtle1/pose
ros2 bag info ~/ros2_ws/bags/square_run
# replay: reset first so the turtle starts from the same pose
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 bag play ~/ros2_ws/bags/square_run --topics /turtle1/cmd_vel
```

Only `/turtle1/cmd_vel` is played back — replaying `/turtle1/pose` would fight the live pose published by turtlesim.


## Verification and acceptance checks

The controller unit tests can run without ROS:

```bash
cd turtlesim_control
python3 -m pytest test -q
```

For ROS integration, from `~/ros2_ws` after building:

```bash
colcon test --packages-select turtlesim_control
colcon test-result --verbose
```

Check a fresh clone and build on Ubuntu before submission. Send goals to the left,
right, above and below the start, and verify final position error below 0.1 and yaw
error below 0.05 radians. Change both speed limits during a long goal and verify the
new limits take effect. Zero and negative speeds must be rejected. Run reset twice
while idle: the screen must be clear and the pose approximately (5.544445, 5.544445, 0).
Run the square twice: each result must report success and `goals_sent: 4`.
Wait for actions to finish before manually resetting; do not send overlapping square
goals or drive the turtle with teleop during an action.

Preparation checks on Windows: controller unit tests and Python syntax checks only.
A clean colcon build, live ROS actions/services, GUI drawing, and bag replay still
require validation on a ROS 2 Humble machine. No bag bonus is claimed.

## Repository contents and publishing

The repository contains the two source packages, this README, and `.gitignore`.
Generated `build/`, `install/`, `log/`, Python caches, and bag recordings are excluded.
The declared package maintainer email is inherited from the supplied project and is
a placeholder; replace it in both package.xml files and setup.py with a real team
contact before public distribution.

After creating an empty GitHub repository for the team, publish the local repository:

```bash
git remote add origin <your-github-repository-url>
git push -u origin main
```

Submit the resulting repository URL using the form linked in the assignment PDF.
