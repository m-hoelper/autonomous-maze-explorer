# Architecture and algorithms

This document explains how the navigation stack is set up, what we changed for this maze, and how the three tasks work. The parameter files are in [`config/`](../ros2_ws/src/searchforce/config/). [PARAMETERS.md](PARAMETERS.md) compares every tuned value with the stock value.

A sentence that starts with *Interpretation:* says why a choice makes sense. It is not recorded project intent. I checked the descriptions of Nav2 and slam_toolbox mechanisms against their Jazzy source code.

## Navigation stack

```
 LiDAR ──► slam_toolbox ──► /map (0.01 m)  and  map → odom
 wheels + IMU ──► EKF ───► odom → base_footprint
                                  │
        /map ──► Nav2 costmaps (static + inflation layers: local 1×1 m rolling window, global)
                                  │
 NavigateToPose goal ──► bt_navigator ──► planner_server (NavFn) ──► global path
                              │                                          │
                              └──► controller_server (Regulated Pure Pursuit)
                                        │
                                        ▼
                     velocity_smoother ──► collision_monitor ──► controller/cmd_vel ──► wheels
```

| Part | Role in this robot |
|---|---|
| slam_toolbox, online async, mapping mode | Builds the occupancy grid from the LiDAR scans while the robot drives, and publishes the `map` to `odom` transform. There is no prior map and no AMCL. The robot starts in a known cell, and the map grows as it moves. |
| EKF, part of the robot bring-up | Fuses wheel odometry and the IMU into the `odom` to `base_footprint` transform. |
| Nav2 `bt_navigator` | Runs the custom behavior tree for each `NavigateToPose` goal. The tree replans the global path at 1 Hz and follows it. On failure, it clears the global costmap and tries again, up to 6 times. |
| Nav2 `planner_server` | Runs the NavFn global planner. NavFn uses Dijkstra on the costmap grid, because `use_astar` is `false`. |
| Nav2 `controller_server` | Runs Regulated Pure Pursuit at 20 Hz to follow the global path. |
| Velocity smoother and collision monitor | The smoother limits the command to 0.32 m/s and 1.0 rad/s, with no sideways velocity. The scan source and the polygon of the collision monitor are both disabled, so the monitor only forwards the command. |
| Costmaps | Are built only from the SLAM map, with a static layer and an inflation layer. The LiDAR obstacle layer is configured but disabled, because the maze has no moving obstacles. |
| Decision nodes on the laptop | Pick goals and send them to `NavigateToPose` over ROS 2 and DDS. Nav2 only receives "go to this pose". |

The robot has mecanum wheels. The controller and the velocity smoother command only forward motion and turns, so the robot drives like a differential-drive robot.

## Controller: Regulated Pure Pursuit

### Pure Pursuit

In every control cycle, the controller picks a *lookahead point* on the global path at the distance `L` from the robot. It transforms that point into the robot frame and takes its lateral offset `y`. Then it commands the curvature `κ = 2·y / L²`. The robot drives at the linear speed `v` with the angular speed `ω = v·κ`, so it moves along a circular arc through the lookahead point. When the heading error is above 0.785 rad, the robot turns in place first (`use_rotate_to_heading`).

### Regulation in this configuration

Regulated Pure Pursuit can slow down in curves, near obstacles, and on approach to the end of a path. This configuration does the following:

- **Curves.** Curvature scaling is enabled. But `regulated_linear_scaling_min_speed` is 0.1 m/s, which is also the top speed, so the scaling has no effect.
- **Obstacles.** Cost-based scaling is off.
- **Approach.** Within 0.2 m of the end of a path, the speed drops towards 0.05 m/s. Task 1 and the cell walk usually send the next goal before that point. So the approach scaling mostly applies to frontier goals and to the final goal.

### Why Pure Pursuit

The team tried the MPPI controller first. It did not work well in the maze and had too many parameters for this use. Pure Pursuit follows the planned path directly, samples no candidate trajectories, and needs far fewer parameters.

### Parameters with the most effect

The values are in `nav2_params_bringup_moritz.yaml`. [PARAMETERS.md](PARAMETERS.md) compares them with stock.

| Parameter | Value | Effect |
|---|---|---|
| `desired_linear_vel` | 0.1 m/s | Top speed. *Interpretation:* slow driving keeps SLAM scan matching stable and leaves time to react in narrow cells. |
| `lookahead_dist` | 0.15 m | A short lookahead follows the path closely. A long lookahead cuts corners and can clip the maze walls. The lookahead is fixed, because `min_lookahead_dist` and `max_lookahead_dist` apply only with velocity scaling, and velocity scaling is off. |
| `xy_goal_tolerance` | 0.15 m | The distance at which a goal counts as reached. Half a cell is 0.127 m, so the robot can count as arrived while it is slightly outside the target cell. |
| `yaw_goal_tolerance` | 3.4 rad | Larger than π, so the controller ignores the final heading. |
| `use_rotate_to_heading` | true | Turns in place when the path direction is far from the current heading. |
| `rotate_to_heading_angular_vel` | 1.3 rad/s | Turn speed in place. |
| `max_angular_accel` | 1.3 rad/s² | Gentler angular acceleration. |
| `allow_reversing` | true | Allows driving backwards when the lookahead point is behind the robot. Nav2 rejects this combination with `use_rotate_to_heading` as a parameter change at runtime, but both values load as `true` at start. We did not record how the two settings interact on the robot. |
| `use_collision_detection` | false | The controller does not predict collisions. The costmap and the planner keep the robot away from the walls. |
| `transform_tolerance` | 10 s | Accepts late TF data. |
| `required_movement_radius` | 0.05 m | The progress checker needs at least 5 cm of motion within its 10 s allowance. This fits the slow speed. |

## Costmaps and planner for 0.254 m cells

| Parameter | Value | Effect |
|---|---|---|
| `inflation_radius` | 0.3 m | The inflation layer raises the cost near walls up to this distance. |
| `cost_scaling_factor` | 30 | The cost is `252 · exp(−30 · (d − r))`, where `d` is the distance to the wall and `r` is about 0.01 m. The cost is about 187 at 2 cm, 76 at 5 cm, 17 at 10 cm, and 7.5 at the corridor center at 12.7 cm. The steep curve keeps the center cheap, so paths stay near the center and the corridor stays passable. |
| `footprint` | About 2 × 2 cm | The tiny footprint keeps the body of the robot from blocking the corridor. The inflation gives the wall clearance instead. |
| Local resolution | 0.01 m | Fine enough for thin maze walls. |
| Global resolution | 0.02 m configured | The static layer resizes the global costmap to the 0.01 m of the SLAM map, so 0.01 m applies in practice. |
| Local costmap | 1 × 1 m, rolling | Covers about four cells around the robot. |
| Planner `tolerance` | 0.05 m | How far from the goal the planned path may end. It is small, because goals are cell centers. |
| `track_unknown_space` | false | The costmap treats unknown map cells as free space, so the planner can route through unmapped space. Together with `allow_unknown: true`, this makes goals in unexplored areas plannable. A goal must still be inside the current map bounds, so the decision nodes wait and try again when a goal is outside the map. |

## SLAM with slam_toolbox

slam_toolbox maps online and asynchronously. It matches each scan against the map and optimizes a pose graph with Ceres, with loop closure. These parameters differ most from the stock file:

| Parameter | Value | Effect |
|---|---|---|
| `resolution` | 0.01 m | The map cell size. It is much finer than the usual 0.05 m, so thin walls and gaps stay visible. The cost is a larger map to process. |
| `max_laser_range` | 1.3 m | slam_toolbox ignores scan points beyond 1.3 m. *Interpretation:* the robot maps only the walls close to it, which keeps distant noise out of the map. |
| `minimum_travel_distance` | 0.05 m | One of three triggers for a new scan. |
| `minimum_travel_heading` | 0.01 rad | One of three triggers for a new scan. |
| `minimum_time_interval` | 0.5 s | One of three triggers for a new scan. slam_toolbox processes a new scan when any trigger is reached, so at least every 0.5 s, also when the robot stands still. The explorer gets frequent map updates. |
| `minimum_angle_penalty` | 5.0, stock 0.9 | The floor of the heading penalty multiplier in scan matching, `max(penalty, floor)`. The penalty is at most 1, so with a floor of 5.0 the multiplier is always 5.0. Scan matching therefore does not penalize a heading that differs from the odometry, and `angle_variance_penalty` (0.8) has no effect. The position penalty stays active. We intended the opposite: a strong angle penalty that keeps walls straight in a maze with only 90° corners. A stronger penalty needs a floor below 1 and a smaller variance. |
| `correlation_search_space_smear_deviation` | 0.08 | Smooths the search space of the scan matcher. |

Task 1 converts maze coordinates into the SLAM frame, because the map frame starts at the start pose of the robot.

## Task 1: known maze

1. **Download.** `task1_maze_requester` calls the maze service `/get_ros_maze` and writes the maze to `rosfile.maze`. The node requests the fixed maze number 29. The maze is a grid of cells, and each cell is a 4-bit wall mask: left 8, right 4, top 2, bottom 1. The file also holds the start cell, the goal cell, and the start orientation.
2. **Graph.** `calculate_adj_matrix` turns the grid into an adjacency matrix. A move to a neighboring cell costs 1 when the wall between the cells is open. A diagonal move costs `√2`, and only when all four wall segments around the shared corner are open.
3. **Shortest path.** `networkx.dijkstra_path` finds the cheapest route from start to goal. The edges have two weights, 1 and `√2`, so Dijkstra is the correct algorithm. Breadth-first search is correct only for equal weights.
4. **Poses.** `MazeSolverSlam.path_nodes_to_xy` converts each cell on the path to its cell center, relative to the start cell. SLAM puts the robot at the map origin, so the method rotates the coordinates into the SLAM frame according to the start orientation: `bottom`, `left`, `right`, or `top`. The heading at each pose points to the next pose.
5. **Execution.** `task1_goal_sequencer` sends the poses to Nav2 one at a time. After a hold-off of 2 s on each goal, it sends the next goal as soon as `distance_remaining` is between 0.05 and 0.4 m. One cell (0.254 m) and one diagonal (0.359 m) are already shorter than 0.4 m. In practice, the hold-off sets the pace, and the robot follows a rolling goal instead of stopping in each cell. A replaced goal returns status 6, aborted. The node logs this status as an error and continues the sequence.

Only the solver runs before the drive. The sequencer runs on the laptop during the drive, and Nav2 plans a global path to the current cell center and controls the robot there. So there are two Dijkstra searches: one on the maze graph and one inside NavFn on the costmap.

The conversion assumes that the robot is aligned with the maze at the start, because the SLAM frame takes its orientation from the start pose.

## Tasks 2 and 3: exploration of an unknown maze

The explorer keeps a visit counter for each 0.254 m cell on a virtual grid that starts at the start cell of the robot. It works in two modes.

### Mode 1: cell walk

1. The explorer proposes the four neighbors of the current cell, in the order straight, left, right, and back, relative to the last direction of travel.
2. For each neighbor, `give_test_width` samples a strip of the SLAM map along the path. The strip has 9 parallel lanes, ±4 cm at 1 cm resolution. `is_test_width_free` accepts the neighbor when fewer than 30 % of the lanes are blocked. Unknown lanes do not count as blocked.
3. The explorer picks the accepted neighbor with the lowest visit count. A tie goes to the earliest neighbor in the order of step 1, which gives a bias to the left.
4. While the robot drives, a timer checks the next planned goal every 2 s. If new walls are in the map, the explorer plans again.
5. If no neighbor is free, or the best neighbor is already visited, the explorer switches to mode 2. It never switches back.

As in task 1, the cell walk sends the next goal early: after a hold-off of 3 s, when the distance is between 0.15 and 0.4 m.

### Mode 2: frontier search

Frontier goals run to the end. The explorer does not send the next goal early.

1. `find_frontiers` finds the free map cells (0) that have at least one unknown neighbor (−1).
2. `cluster_frontiers_cc` groups the frontier cells into 8-connected components with `scipy.ndimage.label`. It drops clusters with fewer than 20 cells and computes one centroid for each remaining cluster.
3. `filter_frontiers_by_occupancy` drops centroids on an occupied map cell. `RADIUS` is 0, so the function checks only the cell of the centroid.
4. `choose_frontier` picks the nearest centroid by straight-line distance. The centroid must be farther away than the goal tolerance and more than 0.1 m from every frontier visited before.
5. The explorer sends the centroid to Nav2. When no centroid is left, the explorer publishes `finished` at once, saves the robot track, and sends one last goal to its previous goal position.

### Why the explorer is a hybrid

The cell walk came first. Its goals are on an absolute grid that assumes that the SLAM map is aligned with the maze. On the robot, small rotations of the SLAM map moved those goals away from the real cells. We added frontier search to fix this. Frontier goals come from the SLAM map itself, so they do not depend on the alignment. The cell walk stayed for the start of a run, because the early map is too small for frontier search.

The hybrid grew out of runs on the robot. It is not a design from scratch, and it does not guarantee full coverage. [Known limitations](#known-limitations-of-the-algorithms) lists the gaps.

### Maze plot for task 2

Task 2 requires a plot of the walls and the driven path. `task2_map_reconstruction` runs on the laptop and waits for `finished`. Then it takes the SLAM map and the stored robot track and rebuilds the maze cells in five steps: Otsu threshold, Canny edges, a minimum-area rectangle, a perspective warp, and a cell grid with the path drawn on it.

## Task 3: object search

The robot explores as in task 2. In the background, a camera pipeline on the robot does the following:

1. `usb_cam` sends images through the image pipeline to `apriltag_node`, which publishes detections. The pan-tilt servos are set to fixed angles at start. `task3_object_detector` accepts only the tag IDs 992 to 997.
2. The detector estimates distance and bearing from the pixel width of the tag and the camera intrinsics. It transforms the point into the map frame with TF and collects the detections.
3. The detector classifies the color of the object in LAB space, with a white balance correction. The majority vote over all frames decides the color.
4. On the first detection, the detector publishes `found`. The explorer cancels its goal, waits 10 s while more detections arrive, and drives back to the start at (0, 0).
5. The detector averages the stored positions and rotates them into the axis-aligned frame of the maze. It takes the angle from a best-fit rectangle of the occupancy map. Then it writes the position and the color to `object.txt` on the robot.

## Status topic

The nodes synchronize over `/searchforce_status`, a `std_msgs/String` topic. The messages come in this order:

1. `found`: the detector sees the object.
2. `finish_detection`: the explorer has started to drive back to the start. It repeats the message once per second.
3. `finished`: the exploration has stopped, and the track is saved. The explorer repeats the message once per second.

## Failure behavior

| Situation | What happens |
|---|---|
| Planning or path following fails | The behavior tree clears the global costmap and tries again, up to 6 times. The spin, wait, and backup recoveries are commented out. |
| The goal is outside the current map | Task 1 and the explorer check this before they send a goal, and try again every 2 s. |
| Nav2 rejects the goal while it starts | Task 1 and the explorer send the same goal again every 2 s. |
| The robot does not move for 5 s after it first moved | Task 1 skips to the next goal, also a goal it holds back because the goal is outside the SLAM map. The explorer picks a new goal. Waiting for the map to grow also counts as not moving. |
| The robot does not move for 20 s, in the explorer | The run ends and the explorer publishes `finished`, even if frontiers are left. If a goal ends at the moment `found` arrives, `finished` can come before the robot is back at the start. |
| Nav2 returns `NO_VALID_PATH`, error code 208, in the explorer | The node starts the two no-motion timers and picks a new goal at once. |
| Something moves into the path | Nothing reacts. The obstacle layer, the collision monitor, and `use_collision_detection` are all off. A new obstacle appears only through the SLAM map, which slam_toolbox publishes every 2 s. |
| The detector sees a tag during task 2 | The detector is part of the bring-up in every run, so it publishes `found`, and the exploration ends. The detector exits after one run and must be restarted. |

## Known limitations of the algorithms

- Frontier search does not guarantee full coverage. It drops clusters with fewer than 20 cells and centroids on walls. The 0.1 m list of visited frontiers can exclude centroids that are left. The 20 s no-motion watchdog can end the run early.
- The centroid of a cluster is not always reachable or free. For a curved frontier, the centroid can lie away from the free cells of the cluster.
- Frontier selection uses straight-line distance, not path length. It can pick a frontier that is close but behind a wall.
- `find_frontiers` is a double loop in plain Python over the map. At 0.01 m resolution, it is slow compared with a vectorized numpy version.
- Frontier mode is one-way. The visit counter is on a virtual grid that starts at the start pose, so it assumes that SLAM drift stays smaller than one cell.
- Task 1 goals and cell-walk goals assume that the SLAM map is aligned with the maze. A rotation error grows with the distance from the start. An error of 3° is about 16 cm at 3 m, which is more than half a cell.
- The maze number for task 1 is fixed in the code at 29.
- Each node writes its files to `SEARCHFORCE_CACHE` on its own machine, so `object.txt` stays on the robot.
