# Run notes

Reconstructed from the project's notes and recorded sessions. Sources are given per section.

## Environment (from the systemd unit used on the robot)
```bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export MACHINE_TYPE=MentorPi_Mecanum
source /opt/ros/jazzy/setup.bash && source ~/workspace/ros2_ws/install/local_setup.bash
```
The laptop uses the same `ROS_DOMAIN_ID`.

## Robot
`robot_bringup_launch.py` starts the controller, LiDAR, camera, image pipeline and AprilTag node immediately, slam_toolbox and the object detector after 10 s, and Nav2 after 20 s. It ran as a systemd service (`deploy/ros2_searchforce_slam.service`):

```bash
sudo systemctl start ros2_searchforce_slam.service      # or: ros2 launch searchforce robot_bringup_launch.py
```

## Laptop
The goal sequencer and the explorer wait until Nav2 is available, so they can be started before the robot's bring-up has finished.

```bash
# Task 1
ros2 run searchforce task1_maze_requester
ros2 run searchforce task1_goal_sequencer
# Task 2 (the plot of the maze is rebuilt by task2_map_reconstruction once the status is 'finished')
ros2 run searchforce task2_frontier_explorer
ros2 run searchforce task2_map_reconstruction
# Task 3 (the detector already runs on the robot; it is part of the bring-up in every run)
ros2 run searchforce task2_frontier_explorer
```
Order of the original `launch_moritz.txt`: `maze_requester`, `goal_sequencer_threshold_moritz`, `random_goal_with_threshold_moritz_explorer_frontier_cluster`, `MapProcessorNode3` (now `task2_map_reconstruction`), `rviz_launch.py`.

## Useful extras used during the project
```bash
ros2 run image_decompressor decompress_image_node   # view the compressed camera stream on the laptop
ros2 run rqt_image_view rqt_image_view
ros2 topic echo /apriltag_detections
ros2 topic pub /searchforce_status std_msgs/msg/String "{data: 'finished'}" --once   # manual trigger
ros2 launch nav2_bringup rviz_launch.py
```
