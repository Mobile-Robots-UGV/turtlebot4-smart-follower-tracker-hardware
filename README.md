# TurtleBot 4 Smart Follower Tracker Hardware

ROS 2 Jazzy hardware implementation of a TurtleBot 4 Lite smart follower and tracker system.

This repository runs on a VM connected to a real TurtleBot 4 Lite. The robot publishes its OAK-D camera, LiDAR, odometry, TF, and command topics under `/robot_09`. The VM runs ArUco board pose estimation, KF/PF target tracking, recovery-aware follower control, SLAM Toolbox mapping, and RViz visualization.

---

## Project Summary

The system detects an ArUco board in the TurtleBot 4 OAK-D camera stream, estimates the board pose, tracks the board using a selectable Kalman Filter or Particle Filter, and commands the robot to follow the board safely.

The tracker publishes three states:

| State       | Meaning                         | Robot Behavior                                 |
| ----------- | ------------------------------- | ---------------------------------------------- |
| `measured`  | Board is visible                | Follow the live tracked board pose             |
| `predicted` | Board was recently lost         | Move cautiously using short-horizon prediction |
| `lost`      | Board has been missing too long | Stop the robot                                 |

The follower uses LiDAR as a front safety guard. SLAM runs separately from the follower and builds a map while the robot follows the board.

---

## Tested Hardware Setup

```text
Robot: TurtleBot 4 Lite
Robot namespace: /robot_09
Host: Ubuntu 24.04 LTS VM
ROS 2: Jazzy
Camera: TurtleBot 4 OAK-D RGB stream
LiDAR: TurtleBot 4 RPLiDAR scan
Control topic: /robot_09/cmd_vel
Command type: geometry_msgs/msg/TwistStamped
Command QoS: BEST_EFFORT
Command frame_id: robot_09/base_link
```

Important hardware discovery:

```text
Robot topics are namespaced under /robot_09.
TF is published on /robot_09/tf and /robot_09/tf_static.
The TF frame names inside the tree are odom and base_link, not robot_09/odom and robot_09/base_link.
SLAM must remap /tf -> /robot_09/tf and /tf_static -> /robot_09/tf_static.
```

---

## Repository Layout

```text
turtlebot4-smart-follower-tracker-hardware/
├── README.md
├── docs/
│   ├── hardware_runbook.md
│   ├── troubleshooting.md
│   ├── rviz_notes.md
│   └── testing_checklist.md
├── board_pose_ros/
│   ├── board_pose_ros/
│   │   ├── __init__.py
│   │   └── board_pose_node.py
│   ├── config/
│   │   ├── board_config.json
│   │   └── camera_calib_oak.npz
│   ├── launch/
│   │   └── board_pose.launch.py
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── sft_hardware_tracker/
│   ├── sft_hardware_tracker/
│   │   ├── __init__.py
│   │   ├── board_tracker_node.py
│   │   └── recovery_follower_node.py
│   ├── config/
│   │   ├── sft_hardware_recovery.yaml
│   │   └── sft_slam_toolbox_hardware.yaml
│   ├── launch/
│   │   ├── sft_hardware_recovery.launch.py
│   │   ├── sft_hardware_slam.launch.py
│   │   └── sft_hardware_full.launch.py
│   ├── rviz/
│   │   └── sft_turtlebot_hardware.rviz
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
└── scripts/
    ├── check_robot_topics.sh
    ├── run_board_pose.sh
    ├── run_slam_hardware.sh
    ├── run_tracker_follower.sh
    └── stop_robot.sh
```

---

## System Architecture

```text
Real TurtleBot 4 Lite
        |
        | /robot_09/oakd/rgb/image_raw/compressed
        v
board_pose_node
        |
        | /robot_09/board_pose
        | /robot_09/board_visible
        | /robot_09/board_debug_image
        | /robot_09/board_markers
        v
board_tracker_node
        |
        | /robot_09/tracked_board_pose
        | /robot_09/tracker_status
        | /robot_09/predicted_board_path
        v
recovery_follower_node
        |
        | /robot_09/cmd_vel
        v
Real TurtleBot 4 Lite Base

LiDAR safety:
/robot_09/scan -> recovery_follower_node

SLAM:
/robot_09/scan + /robot_09/odom + /robot_09/tf -> slam_toolbox -> /map
```

---

## ROS Topics

### Robot Inputs

| Topic                                     | Type                              | Purpose                   |
| ----------------------------------------- | --------------------------------- | ------------------------- |
| `/robot_09/oakd/rgb/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Hardware OAK-D RGB input  |
| `/robot_09/scan`                          | `sensor_msgs/msg/LaserScan`       | LiDAR for safety and SLAM |
| `/robot_09/odom`                          | `nav_msgs/msg/Odometry`           | Robot odometry            |
| `/robot_09/tf`                            | `tf2_msgs/msg/TFMessage`          | Robot dynamic TF          |
| `/robot_09/tf_static`                     | `tf2_msgs/msg/TFMessage`          | Robot static TF           |

### Perception Outputs

| Topic                         | Type                                 | Purpose                             |
| ----------------------------- | ------------------------------------ | ----------------------------------- |
| `/robot_09/board_pose`        | `geometry_msgs/msg/PoseStamped`      | Raw ArUco board pose                |
| `/robot_09/board_visible`     | `std_msgs/msg/Bool`                  | Board detection flag                |
| `/robot_09/board_used_ids`    | `std_msgs/msg/Int32MultiArray`       | Marker IDs used for pose estimation |
| `/robot_09/board_debug_image` | `sensor_msgs/msg/Image`              | Annotated camera image              |
| `/robot_09/board_markers`     | `visualization_msgs/msg/MarkerArray` | Board visualization markers         |

### Tracking and Control

| Topic                            | Type                             | Purpose                            |
| -------------------------------- | -------------------------------- | ---------------------------------- |
| `/robot_09/tracked_board_pose`   | `geometry_msgs/msg/PoseStamped`  | Filtered or predicted board pose   |
| `/robot_09/tracker_status`       | `std_msgs/msg/String`            | `measured`, `predicted`, or `lost` |
| `/robot_09/predicted_board_path` | `nav_msgs/msg/Path`              | Short predicted target path        |
| `/robot_09/cmd_vel`              | `geometry_msgs/msg/TwistStamped` | Velocity command to TurtleBot 4    |

---

## Dependencies

Install the hardware dependencies on the VM:

```bash
sudo apt update
sudo apt install -y \
  python3-opencv \
  ros-jazzy-cv-bridge \
  ros-jazzy-rqt-image-view \
  ros-jazzy-rviz2 \
  ros-jazzy-slam-toolbox \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-ros \
  ros-jazzy-xacro \
  ros-jazzy-nav2-map-server
```

Confirm OpenCV has ArUco support:

```bash
python3 -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"
```

The second value must be:

```text
True
```

---

## Build

Clone into a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Mobile-Robots-UGV/turtlebot4-smart-follower-tracker-hardware.git
```

Build:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select board_pose_ros sft_hardware_tracker
source install/setup.bash
```

Use this in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=9
```

---

## Hardware Run Order

Use separate terminals. Start conservatively and verify each stage before starting robot motion.

---

### Terminal 0: Verify Robot Topics

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9

ros2 topic list | grep -E '/robot_09/(scan|odom|cmd_vel|oakd|tf|tf_static)'
ros2 topic hz /robot_09/scan
ros2 topic hz /robot_09/odom
```

Expected rates:

```text
/robot_09/scan: about 7-10 Hz
/robot_09/odom: about 20 Hz
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link --ros-args \
  -r /tf:=/robot_09/tf \
  -r /tf_static:=/robot_09/tf_static
```

---

### Terminal 1: Start SLAM

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9

ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file ~/ros2_ws/src/turtlebot4-smart-follower-tracker-hardware/sft_hardware_tracker/config/sft_slam_toolbox_hardware.yaml \
  -r /tf:=/robot_09/tf \
  -r /tf_static:=/robot_09/tf_static
```

If lifecycle activation is required:

```bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

Verify map:

```bash
ros2 topic hz /map
ros2 topic echo --once /map
```

---

### Terminal 2: Start Board Pose Detection

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9

ros2 launch board_pose_ros board_pose.launch.py
```

Place the ArUco board in front of the OAK-D camera.

Check detection in another terminal:

```bash
ros2 topic echo /robot_09/board_visible
ros2 topic echo --once /robot_09/board_pose
```

Expected:

```text
data: true
```

---

### Terminal 3: Start Tracker and Follower

Only start this after `/robot_09/board_visible` is `true`.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9

ros2 launch sft_hardware_tracker sft_hardware_recovery.launch.py \
  virtual_robot:=false \
  use_sim_time:=false \
  start_tracker:=true \
  start_follower:=true \
  rviz:=false
```

Expected output:

```text
Board tracker node started
Recovery follower node started
Publishing cmd_vel: /robot_09/cmd_vel
```

---

### Terminal 4: Start RViz

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9

rviz2 -d ~/ros2_ws/src/turtlebot4-smart-follower-tracker-hardware/sft_hardware_tracker/rviz/sft_turtlebot_hardware.rviz
```

RViz configuration:

```text
Fixed Frame: map
RobotModel: disabled
SLAM Map: enabled
Robot 09 Scan: enabled
Board Debug Image: enabled
Board Pose Markers: enabled
Predicted Board Path: enabled
```

---

### Terminal 5: Monitor System

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9

ros2 topic echo /robot_09/tracker_status
```

In another terminal:

```bash
ros2 topic echo /robot_09/cmd_vel
```

Check command QoS:

```bash
ros2 topic info -v /robot_09/cmd_vel | grep -A12 recovery_follower_node
```

Expected:

```text
Reliability: BEST_EFFORT
```

---

## Emergency Stop

Use this command to publish zero velocity continuously for one second:

```bash
timeout 1 ros2 topic pub --rate 10 \
  --qos-reliability best_effort \
  /robot_09/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: robot_09/base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

You can also stop the follower launch with:

```text
Ctrl+C
```

---

## Manual Motion Test

Before testing follower control, verify that the robot accepts command velocity:

```bash
timeout 3 ros2 topic pub --rate 10 \
  --qos-reliability best_effort \
  /robot_09/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: robot_09/base_link}, twist: {linear: {x: 0.05}, angular: {z: 0.0}}}"
```

Stop:

```bash
timeout 1 ros2 topic pub --rate 10 \
  --qos-reliability best_effort \
  /robot_09/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: robot_09/base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

---

## Configuration

Main follower/tracker config:

```text
sft_hardware_tracker/config/sft_hardware_recovery.yaml
```

Recommended conservative hardware settings:

```yaml
board_tracker_node:
  ros__parameters:
    tracker_backend: kf
    pf_num_particles: 300
    process_noise: 0.5
    measurement_noise: 0.1
    fresh_threshold_s: 0.5
    prediction_timeout_s: 3.0
    prediction_horizon_s: 1.5
    prediction_dt_s: 0.1
    publish_rate_hz: 20.0

recovery_follower_node:
  ros__parameters:
    desired_distance_m: 0.70
    min_distance_m: 0.30
    kp_linear: 0.35
    kp_angular: 0.90
    max_linear_measured: 0.15
    max_angular_measured: 0.45
    max_linear_predicted: 0.02
    max_angular_predicted: 0.12
    pose_timeout_s: 3.0
    publish_rate_hz: 20.0
    scan_topic: /robot_09/scan
    front_stop_distance_m: 0.45
    front_slow_distance_m: 0.80
```

SLAM config:

```text
sft_hardware_tracker/config/sft_slam_toolbox_hardware.yaml
```

Important hardware SLAM frame settings:

```yaml
use_sim_time: false
map_frame: map
odom_frame: odom
base_frame: base_link
scan_topic: /robot_09/scan
```

---

## Follower Control Law

The follower uses the tracked board pose in the camera frame:

```text
x = lateral board offset
z = forward board distance
```

Control:

```text
distance_error = z - desired_distance_m
linear.x = kp_linear * distance_error
angular.z = -kp_angular * x
```

The command is clamped by measured/predicted speed limits and the LiDAR front safety guard.

---

## Tracking Modes

### `measured`

The board is visible and fresh pose measurements are available.

```text
Robot follows the live tracked pose.
```

### `predicted`

The board was recently lost, but the tracker still has a valid short-horizon prediction.

```text
Robot moves very slowly and cautiously.
Forward motion is blocked if LiDAR detects a front obstacle.
```

### `lost`

The board has been missing longer than `prediction_timeout_s`.

```text
Robot stops.
```

---

## Testing Checklist

1. Confirm `/robot_09/scan` is publishing.
2. Confirm `/robot_09/odom` is publishing.
3. Confirm `odom -> base_link` TF works using `/robot_09/tf` remap.
4. Confirm manual `/robot_09/cmd_vel` motion works with BEST_EFFORT QoS.
5. Start SLAM and confirm `/map` publishes.
6. Start board pose and confirm `/robot_09/board_visible` becomes `true`.
7. Start tracker and confirm `/robot_09/tracker_status` becomes `measured`.
8. Start follower and confirm `/robot_09/cmd_vel` publishes continuously.
9. Let the robot follow the board slowly.
10. Hide the board briefly and confirm `predicted`.
11. Hide the board longer and confirm `lost` and zero velocity.
12. Save the map if needed.

---

## Save Map

After SLAM mapping:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/sft_hardware_map \
  --ros-args -p map_subscribe_transient_local:=true
```

This saves:

```text
~/sft_hardware_map.pgm
~/sft_hardware_map.yaml
```

---

## Troubleshooting

### Robot does not move, but `/robot_09/cmd_vel` is publishing

Check the publisher QoS:

```bash
ros2 topic info -v /robot_09/cmd_vel
```

The follower publisher must use:

```text
Reliability: BEST_EFFORT
```

Also check that commands use:

```text
frame_id: robot_09/base_link
```

### Manual `--once` velocity command does not move the robot

Use a continuous stream instead:

```bash
timeout 3 ros2 topic pub --rate 10 \
  --qos-reliability best_effort \
  /robot_09/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: robot_09/base_link}, twist: {linear: {x: 0.05}, angular: {z: 0.0}}}"
```

### SLAM does not publish `/map`

Activate SLAM lifecycle:

```bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

Confirm TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link --ros-args \
  -r /tf:=/robot_09/tf \
  -r /tf_static:=/robot_09/tf_static
```

### RViz RobotModel is red

Disable RobotModel. The hardware RViz config should use:

```text
Fixed Frame: map
RobotModel: disabled
```

### RViz drops LaserScan messages

If you see:

```text
Message Filter dropping message: frame 'rplidar_link'
```

this is usually visualization-only. The follower can still use `/robot_09/scan` for safety.

---

## Known Limitations

* The recovery prediction is target-state prediction, not global obstacle-aware planning.
* LiDAR is used as a local front safety guard only.
* If the board leaves the camera view for too long, the robot stops.
* RViz RobotModel is disabled because the real robot TF tree does not use `robot_09/`-prefixed frame names.
* SLAM requires TF remapping from `/tf` to `/robot_09/tf` and `/tf_static` to `/robot_09/tf_static`.

---

## Future Work

* Add a clean `sft_hardware_slam.launch.py` that wraps the tested SLAM command.
* Add a conservative `sft_hardware_full.launch.py` with follower disabled by default.
* Add scripts for robot topic checks, emergency stop, and standard hardware launch.
* Add rosbag recording for measured/predicted/lost trials.
* Add quantitative comparison of KF and PF tracking.
* Transform board pose into the map frame for global visualization.
* Connect predicted target pose to Nav2 or a costmap-based recovery planner.

---

## Authors

* Tatwik Meesala
* Prajjwal
* Lu Yan Tan
