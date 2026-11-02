# Solution write-up

## Robot and software stack

- Hiwonder MentorPi, mecanum drive (`MACHINE_TYPE=MentorPi_Mecanum`), ROS 2 Jazzy, `rmw_fastrtps_cpp`
- Sensors: LDLidar 2-D LiDAR, IMU, wheel odometry fused with an EKF, USB camera on a 2-servo pan/tilt mount
- slam_toolbox (online async, mapping mode) provides `map → odom`
- Nav2: NavFn global planner, Regulated Pure Pursuit controller, custom replanning behaviour tree
- All planning logic runs on a laptop in the same ROS 2 network and talks to Nav2 through the `NavigateToPose` action

## Task 1: known maze

1. `task1_maze_requester` calls the `/get_ros_maze` service, which returns the maze as a grid of wall bitmasks plus start cell, goal cell and start orientation. It writes them to `rosfile.maze`.
2. `maze_solver` (class `MazeSolverSlam`) turns the maze into an adjacency matrix and a networkx graph and runs Dijkstra from start to goal.
3. The cell path is converted into poses. SLAM starts with the robot at the map origin, so the maze frame is rotated into the SLAM frame according to the start orientation (`bottom`, `left`, `right`, `top`). Every goal is the **centre of a cell**.
4. `task1_goal_sequencer` sends the poses to Nav2 one at a time. It watches the action feedback and sends the next goal as soon as the robot is within a distance threshold (0.4 m) of the current one, so the robot keeps moving instead of stopping in each cell.

## Task 2: unknown maze, hybrid exploration

`task2_frontier_explorer` keeps a visit counter per 0.254 m cell and works in two modes.

**Mode 1, cell walking.** From the current cell it proposes the four neighbouring cells as goals. A neighbour is accepted when the share of blocked cells in a strip across the path in the SLAM map stays below 30 % (`give_test_width`, `blocked_threshold`). Among the accepted neighbours it picks the one with the lowest visit count. A timer re-checks the next planned goal while the robot drives, and replans if new walls appeared.

**Mode 2, frontier search.** When no neighbour is free, or only visited ones remain, the explorer switches to frontier search:
1. `find_frontiers`: free cells (0) with at least one unknown neighbour (-1) in the occupancy grid.
2. `cluster_frontiers_cc`: 8-connected components, discarding clusters smaller than 20 cells, one centroid per cluster.
3. `filter_frontiers_by_occupancy`: drop centroids that fall on an occupied cell (`RADIUS = 0`, so only the centroid's own cell is checked).
4. `choose_frontier`: pick the nearest centroid that is farther than the goal tolerance and not within 0.1 m of an already visited frontier.
5. Send it to Nav2. **The run ends when no new centroid is left**: the node publishes `finished` right away, saves the robot track and sends one last goal to its previous goal position.

The `explore` flag switches from cell walking to frontier mode the first time no free unvisited neighbour exists, and it never switches back. The cell walk covers the start of the run, frontier search does the rest.

**Output.** The task requires a 2-D plot of the maze that shows which walls exist, with the driven path. `task2_map_reconstruction` (laptop) listens for `finished` (the explorer repeats it once per second), takes the SLAM map and the stored robot track, and rebuilds the maze cells: Otsu threshold, Canny edges, minimum-area rectangle, perspective warp and a cell-grid overlay of the path.

## Task 3: object search

Same explorer, plus on the robot:

1. `usb_cam` → image pipeline → `apriltag_node` publishes detections. The pan/tilt servos are set to fixed angles at start-up. `task3_object_detector` accepts tag IDs 992–997.
2. It estimates distance and bearing from the tag's pixel width and the camera intrinsics and transforms the point into the map frame with TF. Detections are accumulated.
3. The colour of the object is classified in LAB space with a white balance correction and the majority vote over all frames wins.
4. On the first detection it publishes `found`. The explorer cancels its goal, waits 10 s, then returns to the start (0, 0) and publishes `finish_detection` once per second.
5. The detector averages the stored positions, rotates them into the maze's axis-aligned frame (angle from a best-fit rectangle of the occupancy map) and writes position and colour to `object.txt`.


## Nav2 and SLAM parameters that deviate from the defaults

The maze cells are only 0.254 m wide, far smaller than what the stock tuning (made for larger robots in rooms) assumes. The values are in `config/`. The "stock" values are from the Jazzy files shipped with the packages (`nav2_bringup/params/nav2_params.yaml`, `slam_toolbox/config/mapper_params_online_async.yaml`). The stock controller there is MPPI, so Regulated Pure Pursuit parameters have no stock entry. The notes are my reading of the config, not documented intent.

**Controller (Regulated Pure Pursuit)**

| Parameter | Used | Stock | Note |
|---|---|---|---|
| `desired_linear_vel` | 0.1 m/s | n/a | Slow driving next to walls |
| `lookahead_dist` / `min` / `max` | 0.15 / 0.15 / 0.3 m | n/a | Short lookahead for cell-sized paths |
| `use_velocity_scaled_lookahead_dist` | false | n/a | Fixed lookahead |
| `use_collision_detection` | false | n/a | Obstacle handling is left to the costmap |
| `allow_reversing` | true | n/a | Reversing allowed |
| `rotate_to_heading_angular_vel` / `max_angular_accel` | 1.3 / 1.3 rad/s(²) | n/a | Gentler turns |
| `curvature_lookahead_dist`, `approach_velocity_scaling_dist` | 0.15 / 0.2 m | n/a | Scaled to the small maze |
| `xy_goal_tolerance` | 0.15 m | 0.25 | Cell-centre accuracy |
| `yaw_goal_tolerance` | 3.4 rad | 0.25 | Final heading is ignored, the next goal sets it |
| `required_movement_radius` | 0.05 m | 0.5 | Progress checker for slow movement |
| `transform_tolerance` | 10 s | 0.1 | Very tolerant of late TF data |
| `controller_frequency` | 20 Hz | 20 Hz | |

**Costmaps and planner**

| Parameter | Used | Stock |
|---|---|---|
| Layers | static + inflation (obstacle layer disabled), costmap built from the SLAM map | local: voxel + inflation, global: static + obstacle + inflation |
| `inflation_radius` / `cost_scaling_factor` | 0.3 m / 30 | 0.7 m / 3.0 |
| Resolution (local / global) | 0.01 / 0.02 m (the static layer resizes the global costmap to the map's 0.01 m) | 0.05 m |
| Local costmap | 1 × 1 m rolling window | 3 × 3 m |
| Footprint | ~2 cm square (clearance via inflation) | `robot_radius: 0.22` |
| Planner | NavFn, `allow_unknown: true`, `tolerance 0.05` | `allow_unknown: true`, `tolerance 0.5` |
| Behaviour tree | custom: replan at 1 Hz, clear global costmap as recovery, 6 retries | stock replanning tree |

**slam_toolbox**

| Parameter | Used | Default |
|---|---|---|
| `resolution` | 0.01 m | 0.05 |
| `max_laser_range` | 1.3 m | 20.0 |
| `minimum_travel_distance` / `minimum_travel_heading` | 0.05 m / 0.01 rad | 0.5 / 0.5 |
| `correlation_search_space_smear_deviation` | 0.08 | 0.1 |
| `minimum_angle_penalty` | 5.0 (a floor on the multiplier, so heading is effectively not penalised) | 0.9 |
| `distance_variance_penalty` / `angle_variance_penalty` | 0.5 / 0.8 | 0.5 / 1.0 |

The short laser range (1.3 m) maps only the walls around the robot and keeps the map free of far noise. The small travel thresholds update the map after every few centimetres, which gives the explorer and the cell-walk logic a current map.

## Status topic protocol

`/searchforce_status` (std_msgs/String) synchronises the nodes: `found` (detector sees the object) → `finish_detection` (the explorer has started its return to the start and repeats the message once per second) → `finished` (exploration stopped, the track is saved, repeated once per second).

## Behaviour worth knowing

- The object detector is part of the robot bring-up in every run, so a tag seen during task 2 also publishes `found` and ends the exploration. It exits after one run and has to be restarted.
- A watchdog also publishes `finished` after 20 s without robot motion, even if frontiers remain. If a goal ends at the moment `found` arrives, `finished` can be sent before the robot is back at the start.
- `task1_maze_requester` requests a fixed maze number (29), and the other nodes write their files to `SEARCHFORCE_CACHE` on the machine they run on, so `object.txt` stays on the robot.
- Once the robot has started moving, the goal sequencer skips the current goal after 5 s without motion. This includes a goal it is holding back because it is still outside the SLAM map.
- Some comments and log messages are in German or Chinese.

