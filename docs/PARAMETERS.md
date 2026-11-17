# Tuned parameters compared with stock

The maze cells are 0.254 m wide. The stock tuning is made for larger robots in rooms, so we changed many values. The used values are in [`config/`](../ros2_ws/src/searchforce/config/). The stock values come from the Jazzy files that ship with the packages:

- `nav2_bringup/params/nav2_params.yaml`
- `slam_toolbox/config/mapper_params_online_async.yaml`

The stock Nav2 file uses the MPPI controller, so the Regulated Pure Pursuit parameters have no stock value. The notes are my reading of the configuration, not recorded project intent. [ARCHITECTURE.md](ARCHITECTURE.md) explains what the most important parameters do.

## Controller: Regulated Pure Pursuit

| Parameter | Used | Stock | Note |
|---|---|---|---|
| `desired_linear_vel` | 0.1 m/s | None | Slow driving next to the walls. |
| `lookahead_dist` | 0.15 m | None | Short lookahead for paths of cell size. |
| `min_lookahead_dist` | 0.15 m | None | Not used, because velocity scaling is off. |
| `max_lookahead_dist` | 0.3 m | None | Not used, because velocity scaling is off. |
| `use_velocity_scaled_lookahead_dist` | false | None | Fixed lookahead. |
| `use_collision_detection` | false | None | The costmap handles obstacles. |
| `allow_reversing` | true | None | Driving backwards is allowed. |
| `rotate_to_heading_angular_vel` | 1.3 rad/s | None | Gentler turns. |
| `max_angular_accel` | 1.3 rad/s² | None | Gentler turns. |
| `curvature_lookahead_dist` | 0.15 m | None | Scaled to the small maze. |
| `approach_velocity_scaling_dist` | 0.2 m | None | Scaled to the small maze. |
| `xy_goal_tolerance` | 0.15 m | 0.25 m | Accuracy at the cell center. |
| `yaw_goal_tolerance` | 3.4 rad | 0.25 rad | Ignores the final heading. The next goal sets the heading. |
| `required_movement_radius` | 0.05 m | 0.5 m | Progress checker for slow motion. |
| `transform_tolerance` | 10 s | 0.1 s | Accepts TF data up to 10 s late. |
| `controller_frequency` | 20 Hz | 20 Hz | Unchanged. |

## Costmaps and planner

| Parameter | Used | Stock |
|---|---|---|
| Local costmap layers | Static and inflation. The obstacle layer is disabled. The costmap is built from the SLAM map. | Voxel and inflation. |
| Global costmap layers | Static and inflation. The obstacle layer is disabled. | Static, obstacle, and inflation. |
| `inflation_radius` | 0.3 m | 0.7 m |
| `cost_scaling_factor` | 30 | 3.0 |
| Local resolution | 0.01 m | 0.05 m |
| Global resolution | 0.02 m configured. The static layer resizes the global costmap to the 0.01 m of the map. | 0.05 m |
| Local costmap size | 1 × 1 m, rolling window | 3 × 3 m |
| Footprint | A square of about 2 cm. The inflation gives the wall clearance. | `robot_radius: 0.22` |
| Planner | NavFn with `allow_unknown: true` and `tolerance: 0.05` | `allow_unknown: true` and `tolerance: 0.5` |
| Behavior tree | Custom. Replans at 1 Hz, clears the global costmap as recovery, and tries 6 times. | The stock replanning tree. |

## slam_toolbox

| Parameter | Used | Stock |
|---|---|---|
| `resolution` | 0.01 m | 0.05 m |
| `max_laser_range` | 1.3 m | 20.0 m |
| `minimum_travel_distance` | 0.05 m | 0.5 m |
| `minimum_travel_heading` | 0.01 rad | 0.5 rad |
| `correlation_search_space_smear_deviation` | 0.08 | 0.1 |
| `minimum_angle_penalty` | 5.0. The value is a floor on the multiplier, so the heading penalty is off. | 0.9 |
| `distance_variance_penalty` | 0.5 | 0.5 |
| `angle_variance_penalty` | 0.8 | 1.0 |

The short laser range of 1.3 m maps only the walls around the robot and keeps distant noise out of the map. The small travel thresholds update the map every few centimeters, so the explorer and the cell walk work on a current map.
