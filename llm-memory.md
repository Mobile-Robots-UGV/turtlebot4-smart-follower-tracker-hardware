
# Handoff Summary — TurtleBot 4 Smart Follower Tracker Hardware

## 1. Main Goal

Create a clean hardware-focused ROS 2 Jazzy repository named:

```text
turtlebot4-smart-follower-tracker-hardware
````

The repo should contain the working TurtleBot 4 Lite hardware stack for:

* ArUco board detection from the real OAK-D camera
* KF/PF target tracking
* short target-loss prediction
* recovery-aware following control
* LiDAR front safety stop/slow behavior
* SLAM mapping
* RViz visualization, including optional real robot model visualization

The current working source repo is:

```text
~/ros2_ws/src/sim-to-real-integration
```

The older repo `turtlebot4-sft-aruco-kf-pf-recovery` is no longer needed for the new hardware repo.

---

## 2. Key Background / Context

Hardware platform:

```text
Robot: TurtleBot 4 Lite
Robot namespace: /robot_09
Host: Ubuntu 24.04 LTS VM
ROS 2: Jazzy
Camera: TurtleBot 4 OAK-D
LiDAR: TurtleBot 4 RPLiDAR
```

The working perception/control pipeline is:

```text
/robot_09/oakd/rgb/image_raw/compressed
  -> board_pose_node
  -> /robot_09/board_pose
  -> board_tracker_node
  -> /robot_09/tracked_board_pose
  -> recovery_follower_node
  -> /robot_09/cmd_vel
```

Working tracker states:

```text
measured  = board visible, live tracked pose
predicted = board recently lost, KF/PF predicts short horizon
lost      = board missing too long, robot stops
```

Important discovered hardware details:

```text
Robot topics are under /robot_09.
Robot publishes TF on /robot_09/tf and /robot_09/tf_static.
TF frame names are odom and base_link, not robot_09/odom and robot_09/base_link.
SLAM and RViz need TF remaps:
  /tf := /robot_09/tf
  /tf_static := /robot_09/tf_static
```

Manual TF check that works:

```bash
ros2 run tf2_ros tf2_echo odom base_link --ros-args \
  -r /tf:=/robot_09/tf \
  -r /tf_static:=/robot_09/tf_static
```

---

## 3. Important Decisions Already Made

### New repo scope

The new repo should be hardware-only/minimal.

Include:

```text
board_pose_ros/
sft_hardware_tracker/
docs/
scripts/
README.md
```

Do not include simulation-heavy folders from `sim-to-real-integration`:

```text
create3_sim/
irobot_create_msgs/
turtlebot4/
turtlebot4_simulator/
worlds/
urdf/
meshes/
materials/
simulation launch files
```

### New repo name

```text
turtlebot4-smart-follower-tracker-hardware
```

### Feature status table to preserve

```markdown
| Feature                      | Status |
| ---------------------------- | ------ |
| ArUco detection              | Yes    |
| KF/PF target tracking        | Yes    |
| Short target-loss prediction | Yes    |
| Following control            | Yes    |
| SLAM mapping                 | Yes    |
| LiDAR front safety stop/slow | Yes    |
```

### README already drafted

A README draft was created in canvas with these sections:

```text
Project Summary
Tested Hardware Setup
Repository Layout
System Architecture
ROS Topics
Dependencies
Build
Hardware Run Order
Emergency Stop
Manual Motion Test
Configuration
Follower Control Law
Tracking Modes
Testing Checklist
Save Map
Troubleshooting
Known Limitations
Future Work
Authors
```

It was later updated to include:

```text
Feature/status table
RViz real TurtleBot model instructions
TF remap requirement for RViz
RobotModel settings with empty TF Prefix
Troubleshooting update for red RobotModel
```

---

## 4. User Preferences / Constraints / Requirements

Environment assumptions:

```text
Ubuntu 24.04 LTS Desktop
ROS 2 Jazzy
Python preferred
Workspace: ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=9
```

Build command preference:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select board_pose_ros sft_hardware_tracker
source install/setup.bash
```

The user wants step-by-step instructions and exact terminal commands.

Do not use the old repo going forward unless explicitly requested:

```text
~/ros2_ws/src/turtlebot4-sft-aruco-kf-pf-recovery
```

Use current working repo as source:

```text
~/ros2_ws/src/sim-to-real-integration
```

---

## 5. Current Status

Everything is currently working on hardware.

Confirmed working:

```text
/robot_09/scan publishes around 7.5 Hz
/robot_09/odom publishes around 20 Hz
odom -> base_link TF works with /robot_09/tf remap
SLAM publishes /map at 1 Hz after lifecycle configure/activate
board_pose_node detects ArUco board
board_tracker_node reports status=measured
recovery_follower_node publishes /robot_09/cmd_vel
robot moves/follows after QoS and frame_id fix
RViz works with map, scan, board debug image, board markers
RViz can show real robot model when launched with TF remaps and empty RobotModel TF Prefix
```

Working hardware run order:

### Terminal 1 — SLAM

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9

ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file ~/ros2_ws/src/sim-to-real-integration/sft_hardware_tracker/config/sft_slam_toolbox_hardware.yaml \
  -r /tf:=/robot_09/tf \
  -r /tf_static:=/robot_09/tf_static
```

If needed:

```bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

### Terminal 2 — board pose

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9

ros2 launch board_pose_ros board_pose.launch.py
```

### Terminal 3 — tracker + follower

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

### Terminal 4 — RViz

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9

rviz2 -d ~/ros2_ws/src/sim-to-real-integration/sft_hardware_tracker/rviz/sft_turtlebot_hardware.rviz --ros-args \
  -r /tf:=/robot_09/tf \
  -r /tf_static:=/robot_09/tf_static
```

RViz settings:

```text
Fixed Frame: map
RobotModel Description Topic: /robot_09/robot_description
RobotModel TF Prefix: empty
Map enabled
Robot 09 Scan enabled
Board Debug Image enabled
Board Markers enabled
TF enabled
```

---

## 6. Important Fixes Made / Must Preserve

### Follower command publishing fix

The robot did not move until this was fixed.

Manual test that moved the robot:

```bash
timeout 3 ros2 topic pub --rate 10 \
  --qos-reliability best_effort \
  /robot_09/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: robot_09/base_link}, twist: {linear: {x: 0.05}, angular: {z: 0.0}}}"
```

Therefore `recovery_follower_node.py` must publish:

```text
Topic: /robot_09/cmd_vel
Type: geometry_msgs/msg/TwistStamped
QoS: BEST_EFFORT
frame_id: robot_09/base_link
Publish rate: continuous, about 20 Hz
```

The follower file was patched to include:

```python
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
```

and:

```python
cmd_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

self.pub_cmd = self.create_publisher(TwistStamped, cmd_vel_topic, cmd_qos)
```

and:

```python
cmd.header.frame_id = 'robot_09/base_link'
```

The user accidentally deleted `from rclpy.node import Node` once; preserve that import.

### Hardware SLAM config

Working file:

```text
sft_hardware_tracker/config/sft_slam_toolbox_hardware.yaml
```

Important settings:

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: false
    mode: mapping
    map_frame: map
    odom_frame: odom
    base_frame: base_link
    scan_topic: /robot_09/scan
```

SLAM command must remap TF:

```bash
-r /tf:=/robot_09/tf
-r /tf_static:=/robot_09/tf_static
```

### RViz model visualization

To visualize the real robot model with the hardware:

```bash
rviz2 -d ~/ros2_ws/src/sim-to-real-integration/sft_hardware_tracker/rviz/sft_turtlebot_hardware.rviz --ros-args \
  -r /tf:=/robot_09/tf \
  -r /tf_static:=/robot_09/tf_static
```

RobotModel display:

```text
Description Source: Topic
Description Topic: /robot_09/robot_description
TF Prefix: empty
Enabled: true
```

Do not use:

```text
TF Prefix: robot_09
```

because the real TF frames are not prefixed.

---

## 7. Open Questions / Next Steps

### Repository creation

Need to create new repo folder:

```bash
cd ~/ros2_ws/src
mkdir turtlebot4-smart-follower-tracker-hardware
cd turtlebot4-smart-follower-tracker-hardware
git init
```

Then copy only hardware files from:

```text
~/ros2_ws/src/sim-to-real-integration
```

Suggested copy list:

```text
board_pose_ros/
sft_hardware_tracker/sft_hardware_tracker/board_tracker_node.py
sft_hardware_tracker/sft_hardware_tracker/recovery_follower_node.py
sft_hardware_tracker/config/sft_hardware_recovery.yaml
sft_hardware_tracker/config/sft_slam_toolbox_hardware.yaml
sft_hardware_tracker/launch/sft_hardware_recovery.launch.py
sft_hardware_tracker/rviz/sft_turtlebot_hardware.rviz
```

Need to add new launch files:

```text
sft_hardware_tracker/launch/sft_hardware_slam.launch.py
sft_hardware_tracker/launch/sft_hardware_full.launch.py
```

`sft_hardware_slam.launch.py` should wrap the tested `slam_toolbox` command with TF remaps.

`sft_hardware_full.launch.py` should be conservative. Recommended defaults:

```text
start_slam:=false
start_board_pose:=true
start_tracker:=true
start_follower:=false
rviz:=false
```

Follower should be disabled by default to prevent accidental robot motion.

### Scripts to add

Suggested scripts:

```text
scripts/check_robot_topics.sh
scripts/run_slam_hardware.sh
scripts/run_board_pose.sh
scripts/run_tracker_follower.sh
scripts/stop_robot.sh
```

### Docs to add

Suggested docs:

```text
docs/hardware_runbook.md
docs/troubleshooting.md
docs/rviz_notes.md
docs/testing_checklist.md
```

### README update needed after repo creation

The README currently references paths using:

```text
~/ros2_ws/src/turtlebot4-smart-follower-tracker-hardware/...
```

Make sure those paths match the final repo name exactly.

---

## 8. Exact Details / Commands to Preserve

### Workspace setup

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=9
```

### Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select board_pose_ros sft_hardware_tracker
source install/setup.bash
```

### Robot topic check

```bash
ros2 topic list | grep -E '/robot_09/(scan|odom|cmd_vel|oakd|tf|tf_static|robot_description|joint_states)'
ros2 topic hz /robot_09/scan
ros2 topic hz /robot_09/odom
```

### TF check

```bash
ros2 run tf2_ros tf2_echo odom base_link --ros-args \
  -r /tf:=/robot_09/tf \
  -r /tf_static:=/robot_09/tf_static
```

### Manual motion test

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

### Emergency stop

```bash
timeout 1 ros2 topic pub --rate 10 \
  --qos-reliability best_effort \
  /robot_09/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: robot_09/base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

### Save map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/sft_hardware_map \
  --ros-args -p map_subscribe_transient_local:=true
```

### Key validated status examples

Follower working example:

```text
status=measured x≈0.12 z≈1.32 front≈1.56 linear=0.150 angular≈-0.11
```

Tracker working example:

```text
backend=kf status=measured x≈0.11 z≈0.97
```

Board pose working example:

```text
visible=True ids=[4, 3, 2, 1] x≈-0.026 z≈1.42
```

---

## 9. Notes for Future ChatGPT Continuation

Start by asking the user whether they want to:

```text
1. create the new repo folder and copy files
2. write the cleaned launch files
3. finalize README.md from the drafted canvas
4. add scripts
5. test the new repo on hardware
```

Use step-by-step terminal commands. The user prefers exact commands and incremental validation.

Do not reintroduce the old hardware repo unless explicitly asked.

Primary current repo:

```text
~/ros2_ws/src/sim-to-real-integration
```

Target new repo:

```text
~/ros2_ws/src/turtlebot4-smart-follower-tracker-hardware
```

```
```
