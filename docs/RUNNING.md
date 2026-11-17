# Run the robot and the decision nodes

This guide shows how we started the nodes during the project. I reconstructed it from the project notes and recorded sessions.

## Set up the environment

These variables come from the systemd unit that we used on the robot:

```bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export MACHINE_TYPE=MentorPi_Mecanum
source /opt/ros/jazzy/setup.bash && source ~/workspace/ros2_ws/install/local_setup.bash
```

On the laptop, set the same `ROS_DOMAIN_ID`.

## Start the robot

`robot_bringup_launch.py` starts the nodes in three stages:

1. At once: the controller, the LiDAR, the camera, the image pipeline, and the AprilTag node.
2. After 10 s: slam_toolbox and the object detector.
3. After 20 s: Nav2.

On the robot, the bring-up ran as the systemd service `deploy/ros2_searchforce_slam.service`:

```bash
sudo systemctl start ros2_searchforce_slam.service
```

To start the bring-up without systemd, run the launch file directly:

```bash
ros2 launch searchforce robot_bringup_launch.py
```

## Start a task on the laptop

The goal sequencer and the explorer wait until Nav2 is available. You can start them before the bring-up on the robot has finished.

For task 1, run:

```bash
ros2 run searchforce task1_maze_requester
ros2 run searchforce task1_goal_sequencer
```

For task 2, run the explorer and the map reconstruction. The map reconstruction draws the maze plot when the status is `finished`.

```bash
ros2 run searchforce task2_frontier_explorer
ros2 run searchforce task2_map_reconstruction
```

For task 3, run the explorer. The object detector already runs on the robot, because it is part of the bring-up in every run.

```bash
ros2 run searchforce task2_frontier_explorer
```

## Inspect and debug

To view the compressed camera stream on the laptop:

```bash
ros2 run image_decompressor decompress_image_node
```

```bash
ros2 run rqt_image_view rqt_image_view
```

To see the AprilTag detections:

```bash
ros2 topic echo /apriltag_detections
```

To end a run by hand, publish `finished`:

```bash
ros2 topic pub /searchforce_status std_msgs/msg/String "{data: 'finished'}" --once
```

To open RViz with the Nav2 view:

```bash
ros2 launch nav2_bringup rviz_launch.py
```
