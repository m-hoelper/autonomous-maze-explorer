# Architecture and algorithms

How the navigation stack is set up, what is specific to this maze, and how the three tasks work. Parameter values are in [`config/`](../ros2_ws/src/searchforce/config/). A shorter parameter comparison against the stock files is in [SOLUTION.md](SOLUTION.md).

Statements marked *(interpretation)* describe why a choice makes sense, not the project's recorded intent. Mechanism descriptions for Nav2 and slam_toolbox were checked against their Jazzy source code.

## 1. The navigation stack

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

| Part | What it does here |
|---|---|
| **slam_toolbox** (online async, mapping mode) | Builds the occupancy grid from the LiDAR scans while driving and publishes `map → odom`. There is **no prior map and no AMCL**: the robot starts in a known cell and the map grows as it moves. |
| **EKF** (robot bring-up) | Fuses wheel odometry and the IMU into `odom → base_footprint`. |
| **Nav2 `bt_navigator`** | Runs the custom behaviour tree for each `NavigateToPose` goal: replan the global path at 1 Hz, follow it, and on failure clear the global costmap and retry (up to 6 times). |
| **Nav2 `planner_server`** | NavFn global planner (Dijkstra on the costmap grid, `use_astar: false`). |
| **Nav2 `controller_server`** | Regulated Pure Pursuit follows the global path at 20 Hz. |
| **Velocity smoother, collision monitor** | The smoother limits the command to 0.32 m/s and 1.0 rad/s, with no sideways velocity. The collision monitor's scan source and polygon are both disabled, so it only forwards the command. |
| **Costmaps** | Built **only from the SLAM map** (static layer plus inflation). The LiDAR obstacle layer is configured but disabled. |
| **Decision nodes** (laptop) | Pick goals and send them to `NavigateToPose` over ROS 2 / DDS. Nav2 itself only ever sees "go to this pose". |

Why the costmaps use only the SLAM map: the maze is a static environment with no dynamic obstacles, so the obstacle layer was turned off.

The robot has mecanum wheels, but the controller and the velocity smoother only command forward and turning motion, so it drives like a differential robot.

## 2. Controller: Regulated Pure Pursuit

**How it works.** Every control cycle the controller picks a *lookahead point* on the global path at distance `L` from the robot. It transforms that point into the robot frame, takes its lateral offset `y`, and commands the curvature `κ = 2·y / L²`. The robot drives at linear speed `v` with angular speed `ω = v·κ`, so it moves along a circular arc that passes through the lookahead point. The robot rotates in place when the heading error is large (`use_rotate_to_heading`, above 0.785 rad here).

**What "regulated" does in this configuration.** Regulated Pure Pursuit can reduce speed in curves, near obstacles and on approach to the end of a path. Here:

- **Curves:** curvature scaling is enabled, but `regulated_linear_scaling_min_speed` is 0.1 m/s, the same as the top speed, so it has no effect.
- **Obstacles:** cost-based scaling is off.
- **Approach:** speed drops towards 0.05 m/s within 0.2 m of the end of a path. Task 1 and the cell walk usually replace the goal before that point, so this mostly applies to frontier goals and the final goal.

**Why Pure Pursuit for this project.** A more complex controller (MPPI) was tried first, but it did not work well here and had too many parameters to configure for this use case, so the team moved to a simpler one. Pure Pursuit follows the planned path directly, with no sampling of candidate trajectories, and needs far fewer parameters.

**Parameters that matter most** (values in `nav2_params_bringup_moritz.yaml`, stock comparison in SOLUTION.md):

| Parameter | Value | What it changes |
|---|---|---|
| `desired_linear_vel` | 0.1 m/s | Top speed. *(interpretation)* Slow driving keeps SLAM scan matching stable and leaves time to react in narrow cells. |
| `lookahead_dist` | 0.15 m | A short lookahead follows the path tightly. A long one cuts corners and could clip the maze walls. The lookahead is fixed: `min_`/`max_lookahead_dist` only apply with velocity scaling, which is off. |
| `xy_goal_tolerance` | 0.15 m | When a goal counts as reached. Half a cell is 0.127 m, so the robot can count as arrived while slightly outside the target cell. |
| `yaw_goal_tolerance` | 3.4 rad | Larger than π, so the final heading is ignored. |
| `use_rotate_to_heading`, `rotate_to_heading_angular_vel`, `max_angular_accel` | true, 1.3, 1.3 | Turn in place when the path direction is far from the current heading, with gentler acceleration. |
| `allow_reversing` | true | Permits driving backwards when the lookahead point is behind the robot. Nav2 rejects this combination with `use_rotate_to_heading` as a runtime parameter change, but both load as true at start-up. Their interaction on the robot is not documented. |
| `use_collision_detection` | false | The controller does not predict collisions itself, so the costmap and the planner keep the robot off the walls. |
| `transform_tolerance` | 10 s | Tolerates late TF data. |
| `required_movement_radius` | 0.05 m | Progress checker: the robot has to move at least 5 cm within the 10 s time allowance, which fits the slow speed. |

## 3. Costmaps and planner tuned for 0.254 m cells

| Parameter | Value | What it changes |
|---|---|---|
| `inflation_radius`, `cost_scaling_factor` | 0.3 m, 30 | The inflation layer raises the cost near walls: `cost = 252 · exp(−30 · (d − r))`, with `d` the distance to the wall and `r` about 0.01 m. That gives about 187 at 2 cm, 76 at 5 cm, 17 at 10 cm and 7.5 at the corridor centre (12.7 cm). The steep slope keeps the centre cheap, so paths stay near it without making the corridor impassable. |
| `footprint` | about 2 × 2 cm | A tiny footprint so the corridor is not blocked by the robot's body. Wall clearance comes from the inflation instead. |
| Resolution local / global | 0.01 / 0.02 m configured | Fine enough for thin maze walls. The global value is overridden in practice: the static layer resizes the global costmap to the SLAM map's 0.01 m. |
| Local costmap | 1 × 1 m rolling | Covers about four cells around the robot. |
| Planner `tolerance` | 0.05 m | How far from the goal the planner may end. Small, because goals are cell centres. |
| `track_unknown_space` | false | Unknown map cells become free space in the costmap, so the planner can route through unmapped space. This is what makes goals in unexplored areas plannable (`allow_unknown: true` is also set). Goals still have to be inside the current map bounds, which is why the decision nodes wait and retry when a goal is outside the map. |

## 4. SLAM: slam_toolbox

Online asynchronous mapping with scan matching and a pose graph optimised by Ceres, plus loop closure. The parameters that differ most from the stock file:

| Parameter | Value | What it changes |
|---|---|---|
| `resolution` | 0.01 m | Map cell size. Much finer than the usual 0.05 m, so thin walls and gaps stay resolved, at the cost of a larger map to process. |
| `max_laser_range` | 1.3 m | Scan points beyond 1.3 m are ignored. *(interpretation)* The robot maps only the walls close to it, which keeps far noise out of the map. |
| `minimum_travel_distance`, `minimum_travel_heading`, `minimum_time_interval` | 0.05 m, 0.01 rad, 0.5 s | A new scan is processed when **any** of these is reached, so at least every 0.5 s even when standing still. This gives frequent map updates for the explorer. |
| `minimum_angle_penalty` | 5.0 (stock 0.9) | The floor of the heading penalty multiplier in scan matching: `max(penalty, floor)`. The penalty is at most 1, so with a floor of 5.0 the multiplier is always 5.0. Scan matching therefore does **not** penalise heading differences from the odometry, and `angle_variance_penalty` (0.8) has no effect. The position penalty stays active. The intent was the opposite: a strong angle penalty, to keep walls straight in a maze with only 90° corners. A stronger penalty needs a floor below 1 and a smaller variance, so this value does not do what was intended. |
| `correlation_search_space_smear_deviation` | 0.08 | Smoothing of the scan-matching search space. |

The map frame is anchored at the robot's start pose, which is why task 1 converts maze coordinates into the SLAM frame.

## 5. Task 1: known maze

1. **Download.** `task1_maze_requester` calls the maze service `/get_ros_maze` and writes the maze to `rosfile.maze`. The maze is a grid of cells; each cell is a 4-bit wall mask (left 8, right 4, top 2, bottom 1), plus start cell, goal cell and start orientation.
2. **Graph.** `calculate_adj_matrix` turns the grid into an adjacency matrix: a move to a neighbouring cell costs 1 when the wall between them is open. A diagonal move costs `√2` only when all four wall segments around the shared corner are open.
3. **Shortest path.** `networkx.dijkstra_path` finds the cheapest route from start to goal. Dijkstra is the right algorithm because the edges have two different weights (1 and `√2`). Breadth-first search would only be correct for uniform weights.
4. **Poses.** `MazeSolverSlam.path_nodes_to_xy` converts each cell on the path to its **cell centre**, then expresses it relative to the start cell. Because SLAM starts with the robot at the map origin, the coordinates are rotated into the SLAM frame according to the start orientation (`bottom`, `left`, `right`, `top`). The heading at each pose points to the next one.
5. **Execution.** `task1_goal_sequencer` sends the poses to Nav2 one at a time. After a 2 s hold-off on each goal, it sends the next one as soon as `distance_remaining` is between 0.05 and 0.4 m. A single cell (0.254 m) or diagonal (0.359 m) is already within 0.4 m, so in practice the hold-off sets the pace and the robot follows a rolling goal instead of stopping in each cell. A preempted goal returns status 6 (aborted), and the node logs this as an error while the sequence continues.

Only the solver runs offline. The sequencer runs online on the laptop during the drive, and Nav2 plans a global path and controls the robot to the current cell centre. So there are two Dijkstra searches: one on the maze graph, and one inside NavFn on the costmap.

The conversion assumes the robot is aligned with the maze at the start, because the SLAM frame takes its orientation from the start pose.

## 6. Tasks 2 and 3: exploration in an unknown maze

The explorer keeps a **visit counter per 0.254 m cell** on a virtual grid that starts at the robot's start cell, and works in two modes.

**Mode 1: cell walk.**
1. From the current cell, it proposes the four neighbouring cells, ordered relative to the last direction of travel: straight, left, right, back.
2. For each, `give_test_width` samples a strip of the SLAM map along the path (9 parallel lanes, ±4 cm at 1 cm resolution). A neighbour is accepted when the share of blocked lanes stays below 30 % (`is_test_width_free`). Unknown lanes do not count as blocked.
3. Among the accepted neighbours it picks the one with the **lowest visit count**; ties go to the earliest in the order above, which gives a left-hand bias.
4. A timer re-checks the next planned goal every 2 s while the robot drives, and replans if new walls have appeared in the map.
5. If no neighbour is free, or the best one has already been visited, the explorer switches to mode 2 and stays there.

As in task 1, the cell walk sends the next goal early: after a 3 s hold-off, when the distance is between 0.15 and 0.4 m.

**Mode 2: frontier search.** Goals here run to completion; there is no early advance.
1. `find_frontiers`: free cells (0) with at least one unknown neighbour (−1) in the occupancy grid.
2. `cluster_frontiers_cc`: 8-connected components via `scipy.ndimage.label`. Clusters under 20 cells are discarded and each remaining cluster gets one centroid.
3. `filter_frontiers_by_occupancy`: drop centroids that fall on an occupied cell (`RADIUS = 0`, so only the centroid's own cell is checked).
4. `choose_frontier`: the nearest centroid (straight-line distance) that is farther than the goal tolerance and not within 0.1 m of an already visited frontier.
5. Send it to Nav2. When no centroid is left, the node publishes `finished` right away, saves the robot track and sends one last goal to its previous goal position.

**Why a hybrid.** The cell walk came first. Its goals sit on an absolute grid that assumes the SLAM map is perfectly aligned with the maze, and in practice small rotations of the SLAM map made those goals drift away from the real cells. Frontier search was added to deal with this: its goals come from the SLAM map itself, so they do not depend on that alignment. The cell walk stayed for the start of a run, because early on the map is too small for frontier search to work well. This grew out of what happened on the robot and is not a clean-sheet design. It does not guarantee full coverage (see section 9).

**Output of task 2.** The task requires a plot that shows which walls exist and the driven path. `task2_map_reconstruction` listens for `finished` and rebuilds the maze cells from the SLAM map with Otsu thresholding, Canny edges, a minimum-area rectangle and a perspective warp, then draws the track.

## 7. Task 3: object search

Same explorer, with a camera pipeline in the background on the robot: `usb_cam` → image pipeline → AprilTag detections → `task3_object_detector` (distance and bearing from the tag's pixel size and the camera intrinsics, TF into the map frame, LAB colour classification with white balance). On the first detection the explorer cancels its goal, waits 10 s while detections accumulate, returns to the start and stops.

## 8. What happens when something goes wrong

| Situation | Response |
|---|---|
| Planning or following fails | The behaviour tree clears the global costmap and retries, up to 6 times. Spin, wait and backup recoveries are commented out. |
| Goal outside the current map | Task 1 and the explorer check this before sending and retry every 2 s. |
| Goal rejected (Nav2 still starting up) | Task 1 and the explorer retry the same goal every 2 s. |
| No motion for 5 s (after the robot has first moved) | Task 1 skips to the next goal; the explorer picks a new one. Standing still while waiting for the map to grow counts too. |
| No motion for 20 s (explorer) | The run ends and `finished` is published, even if frontiers remain. |
| `NO_VALID_PATH` (error code 208, explorer) | The node starts the two no-motion timers and picks a new goal straight away. |
| Something moves into the path | There is no reactive safety: the obstacle layer, the collision monitor and `use_collision_detection` are all off. New obstacles only appear through the SLAM map, published every 2 s. |

## 9. Known limitations of the algorithms

- Frontier search does not guarantee full coverage. Clusters under 20 cells and centroids on walls are dropped, the 0.1 m visited list can exclude remaining centroids, and the 20 s no-motion watchdog can end the run early.
- A cluster's centroid need not be reachable or free: for a curved frontier it can fall away from the free cells that form the cluster.
- Frontier selection uses straight-line distance, not path length, so it can pick a frontier that is close but behind a wall.
- `find_frontiers` is a pure Python double loop over the map. At 0.01 m resolution this is slow compared with a vectorised numpy version.
- Frontier mode is one-way, and the visit counter lives on a virtual grid anchored at the start pose, which assumes SLAM drift stays smaller than a cell.
- Task 1 and cell-walk goals assume the SLAM map is aligned with the maze, and a rotation error grows with distance from the start: 3° is about 16 cm at 3 m, more than half a cell.
- The maze number for task 1 is hard-coded.
