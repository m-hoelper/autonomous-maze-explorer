#!/usr/bin/env python3
import os
from searchforce.functions.paths import CACHE_DIR

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import tf_transformations
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from collections import Counter
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy import ndimage
from sensor_msgs.msg import Imu

class FrontierExplorer(Node):

    def __init__(self):

        super().__init__('frontier_explorer')

        # Subscribe auf Map
        self._map = None  # /global_costmap/costmap
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        self.searchforce_status = None
        self.finished = False
        self.found = False
        self.sending = False
        self._status_pub = self.create_publisher(
            String, '/searchforce_status', 10)
        self._status_sub = self.create_subscription(
            String, '/searchforce_status', self.status_callback, 10)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Warte auf Action-Server
        self.get_logger().info('Waiting for NavigateToPose Action-Server...')
        while not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('NavigateToPose Action-Server not available, waiting again...')

        self.current_index = 0

        self.distance_threshold = 0.4
        self.xy_goal_tolerance = 0.15
        self.check_distance_time = 3.0

        self.choose_frontier_threshold = 0.1
        
        self.check_distance = False
        self._current_goal_handle = None
        self.timer_distance = None

        self.all_goals_list = [(0.0, 0.0, 'pos_x')]
        self.cell_size = 0.254     # in meters
        self.direction = 'pos_x'
        self.occ_threshold = 1

        self.test_width_size = 0.04
        self.blocked_threshold = 0.3

        self.explore = False
        self.wait_for_explorer = False
        self.visited_frontiers = []
        self.MIN_SAMPLES = 20
        self.RADIUS = 0

        self.n = 120
        self.count_nodes_matrix = np.zeros((self.n, self.n))
        self.origin_node = 59
        self.count_nodes_matrix[self.origin_node][self.origin_node] = 1

        self.position_threshold = 0.05
        self.turning_threshold = 0.5
        self.robot_positions = [(0.0,0.0)]
        self.robot_yaws = [0.0]
        self.current_x = 0.0
        self.current_y = 0.0
        self.position_checker_timer_send = None
        self.position_checker_timer_stop = None

        self.cumulative_yaw = 0.0  
        self.prev_time = None       

        self.sub = self.create_subscription(
            Imu,
            '/imu',   # passe ggf. an dein Topic an
            self.imu_callback,
            10
        )
    
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10  # QoS-Depth
        )

        self.position_timer = self.create_timer(0.1, self.position_on_timer)

        self._check_object_found_timer = self.create_timer(0.5, self._check_object_found)

        # Map-Check starten
        self.get_logger().info('Warte auf erste /map-Nachricht…')
        self._map_check_timer = self.create_timer(0.5, self._check_map_ready)
        
        self._check_path_is_legit_timer = self.create_timer(2.0, self._check_path_is_legit)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def imu_callback(self, msg: Imu):
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_time is None:
            self.prev_time = now
            return

        dt = now - self.prev_time
        self.prev_time = now

        omega_z = msg.angular_velocity.z

        self.cumulative_yaw += omega_z * dt

    def _check_map_ready(self):
        if self._map is not None:
            self.get_logger().info('Map empfangen. Starte Zielversand…')
            self._map_check_timer.cancel()
            self._map_check_timer = None
            if not self.explore:
                x, y = self.change_direction_to_non_critic()
                self.send_next_goal(x,y)
            else:
                x, y = self.run_explorer()
                self.send_next_goal(x,y)

    def helper_send_goal(self):
        if self.finished or self.found or self.wait_for_explorer or self.sending:
                return
        self.get_logger().warn(f"no progress skipping goal ...")

        if not self.explore:
            x, y = self.change_direction_to_non_critic()
            self.send_next_goal(x,y)
        else:
            x, y = self.run_explorer()
            self.send_next_goal(x,y)
            self.get_logger().warn("sending next goal ...")

    def map_callback(self, msg: OccupancyGrid):
        self._map = msg

    def status_callback(self, msg: String):
        self.searchforce_status = msg.data

    def _check_path_is_legit(self):
        if self.finished or self.found or self.wait_for_explorer or self.explore or self.sending:
                return
        if self.current_index > (len(self.all_goals_list)-3):
            if self.current_index > (len(self.all_goals_list)-2) or not self.check_distance:
                return
        last_x = self.all_goals_list[self.current_index][0]
        last_y = self.all_goals_list[self.current_index][1]
        last_direction = self.all_goals_list[self.current_index][2]

        x = self.all_goals_list[self.current_index+1][0]
        y = self.all_goals_list[self.current_index+1][1]
        direction = self.all_goals_list[self.current_index+1][2]
        
        free_list = self.give_test_width(x, y, last_x, last_y, direction)
        if not self.is_test_width_free(free_list):
            self.get_logger().error(f"{x}, {y}, was wrong -> correcting path")
            minus_one_node_list = self.all_goals_list[(self.current_index+1):]
            self.minus_to_node_matrix(minus_one_node_list)
            self.all_goals_list[(self.current_index+1):] = []
            self.direction = last_direction
            self.sending = True
            if not self.explore:
                x, y = self.change_direction_to_non_critic()
                self.send_next_goal(x,y)
            else:
                x, y = self.run_explorer()
                self.send_next_goal(x,y)
            return
        self.current_index += 1
        
    def publish_finished(self):
        status_msg = String()
        status_msg.data = 'finished'
        self._status_pub.publish(status_msg)

    def publish_finish_detection(self):
        status_msg = String()
        status_msg.data = 'finish_detection'
        self._status_pub.publish(status_msg)

    def stop_exploration(self):
        self.finished = True
        if self.position_checker_timer_send is not None:
            self.position_checker_timer_send.cancel()
            self.position_checker_timer_send = None
        if self.position_checker_timer_stop is not None:
            self.position_checker_timer_stop.cancel()
            self.position_checker_timer_stop = None
        if self.position_timer is not None:
            self.position_timer.cancel()
            self.position_timer = None
        self.finished_timer = self.create_timer(1.0, self.publish_finished)
        self.get_logger().info(f"Exploration finished! Stopping ...")

        self.get_logger().info('Speichere zurückgelegten Weg ...')
        self.robot_positions = np.array(self.robot_positions)
        np.save(os.path.join(CACHE_DIR, 'robot_positions.npy'), self.robot_positions)

        self.robot_yaws = np.array(self.robot_yaws)
        np.save(os.path.join(CACHE_DIR, 'robot_yaws.npy'), self.robot_yaws)
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()

    def _check_object_found(self):
        if self.searchforce_status == 'found':
            self.found = True
            self._check_object_found_timer.cancel()
            self._check_object_found_timer = None
            if self._current_goal_handle is not None:
                self._current_goal_handle.cancel_goal_async()

            def _retry(timer):
                timer.cancel()
                timer = None
                self.finish_detection_timer = self.create_timer(1.0, self.publish_finish_detection)
                x, y = 0.0, 0.0
                self.send_next_goal(x,y)

            timer = self.create_timer(10.0, lambda: _retry(timer))

    def is_pose_in_grid(self, goal_x, goal_y):
        if self._map is None:
            return False
        info_map = self._map.info

        mx = int((goal_x - info_map.origin.position.x) / info_map.resolution)
        my = int((goal_y - info_map.origin.position.y) / info_map.resolution)
        
        if mx < 0 or my < 0 or mx >= info_map.width or my >= info_map.height:
            return False
        else:
            return True

    def give_test_width(self, goal_x, goal_y, last_x, last_y, direction):
        if self._map is None:
            return [2] 
        info_map = self._map.info
        data_map = self._map.data

        goal_mx = int((goal_x - info_map.origin.position.x) / info_map.resolution)
        goal_my = int((goal_y - info_map.origin.position.y) / info_map.resolution)

        if goal_mx < 0 or goal_my < 0 or goal_mx >= info_map.width or goal_my >= info_map.height:
            self.get_logger().info(
            f"Goal Pose Blocked mit out of map"
            )
            return [2] 
        
        last_mx = int((last_x - info_map.origin.position.x) / info_map.resolution)
        last_my = int((last_y - info_map.origin.position.y) / info_map.resolution)

        dist_mx = abs(goal_mx - last_mx)
        dist_my = abs(goal_my - last_my)

        test_width = int(self.test_width_size / info_map.resolution)

        is_free_list = []

        if direction == 'pos_x' :
            list_idx = 0
            for i in range(-test_width,test_width+1):
                # mx/my is free -> 0
                is_free_list.append(0)
                for j in range (dist_mx):
                    test_mx = last_mx + j
                    test_my = last_my + i

                    if test_mx < 0 or test_my < 0 or test_mx >= info_map.width or test_my >= info_map.height:
                        #mx/my is out of map -> 2
                        is_free_list[list_idx] = 2

                        break
                    idx = test_my * info_map.width + test_mx
                    val = data_map[idx]
                    if val == -1 or val == 254:
                        #mx/my is unknown -> -1
                        is_free_list[list_idx] = -1

                        break
                    if (val >= 0) and (val >= self.occ_threshold):
                        #mx/my is blocked -> 1
                        is_free_list[list_idx] = 1

                        break
                list_idx = list_idx + 1

        if direction == 'neg_x' :
            list_idx = 0
            for i in range(-test_width,test_width+1):
                # mx/my is free -> 0
                is_free_list.append(0)
                for j in range (dist_mx):
                    test_mx = last_mx - j
                    test_my = last_my + i

                    if test_mx < 0 or test_my < 0 or test_mx >= info_map.width or test_my >= info_map.height:
                        #mx/my is out of map -> 2
                        is_free_list[list_idx] = 2

                        break
                    idx = test_my * info_map.width + test_mx
                    val = data_map[idx]
                    if val == -1 or val == 254:
                        #mx/my is unknown -> -1
                        is_free_list[list_idx] = -1

                        break
                    if (val >= 0) and (val >= self.occ_threshold):
                        #mx/my is blocked -> 1
                        is_free_list[list_idx] = 1

                        break
                list_idx = list_idx + 1
            
        if direction == 'pos_y' :
            list_idx = 0
            for i in range(-test_width,test_width+1):
                # mx/my is free -> 0
                is_free_list.append(0)
                for j in range (dist_my):
                    test_mx = last_mx + i
                    test_my = last_my + j

                    if test_mx < 0 or test_my < 0 or test_mx >= info_map.width or test_my >= info_map.height:
                        #mx/my is out of map -> 2
                        is_free_list[list_idx] = 2

                        break
                    idx = test_my * info_map.width + test_mx
                    val = data_map[idx]
                    if val == -1 or val == 254:
                        #mx/my is unknown -> -1
                        is_free_list[list_idx] = -1

                        break
                    if (val >= 0) and (val >= self.occ_threshold):
                        #mx/my is blocked -> 1
                        is_free_list[list_idx] = 1

                        break
                list_idx = list_idx + 1
            
        if direction == 'neg_y' :
            list_idx = 0
            for i in range(-test_width,test_width+1):
                # mx/my is free -> 0
                is_free_list.append(0)
                for j in range (dist_my):
                    test_mx = last_mx + i
                    test_my = last_my - j

                    if test_mx < 0 or test_my < 0 or test_mx >= info_map.width or test_my >= info_map.height:
                        #mx/my is out of map -> 2
                        is_free_list[list_idx] = 2

                        break
                    idx = test_my * info_map.width + test_mx
                    val = data_map[idx]
                    if val == -1 or val == 254:
                        #mx/my is unknown -> -1
                        is_free_list[list_idx] = -1

                        break
                    if (val >= 0) and (val >= self.occ_threshold):
                        #mx/my is blocked -> 1
                        is_free_list[list_idx] = 1

                        break
                list_idx = list_idx + 1
        return is_free_list

    def is_test_width_free(self, is_free_list):
        result = Counter(is_free_list)
        if (len(is_free_list)-result[2]) == 0:
            return False
        if (result[1])/(len(is_free_list)-result[2]) > self.blocked_threshold:
            return False
        else:
            return True

    def send_next_goal(self, x, y):
        self.check_distance = False

        if self.timer_distance is not None:
            self.timer_distance.cancel()
            self.timer_distance = None

        # One‐Shot‐Timer
        def _retry(timer):
            timer.cancel()
            timer = None
            x, y = self.change_direction_to_non_critic()
            self.send_next_goal(x,y)

        # Grid‐Prüfung
        if not self.is_pose_in_grid(x, y):
            self.get_logger().info(
                f"Ziel (x={x:.2f}, y={y:.2f}) außerhalb der Map – warte 2.0 s"
            )
            timer = self.create_timer(2.0, lambda: _retry(timer))
            return
        
        yaw = 0.0
        self.all_goals_list.append((x, y, self.direction))
        self.add_to_node_matrix(x,y)

        # Pose erstellen
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        # NavigateToPose.Goal erstellen
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        self.get_logger().info(
            f'(x={x:.2f}, y={y:.2f}) in frame="{nav_goal.pose.header.frame_id}"'
        )

        # Goal asynchron senden
        send_goal_future = self._action_client.send_goal_async(
            nav_goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        def _timer_distance(timer):
            if self.timer_distance is not None:
                self.timer_distance.cancel()
                self.timer_distance = None
                self.check_distance = True

        self.timer_distance = self.create_timer(self.check_distance_time, lambda: _timer_distance(self.timer_distance))

    def goal_response_callback(self, future):
        goal_handle = future.result()  # type: rclpy.action.ClientGoalHandle[NavigateToPose]
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Goal was rejected by the Action Server, retrying in 2 s.'
            )
            if self.finished:
                return
            # Nav2 rejects goals while it is still starting up.
            x, y, _ = self.all_goals_list.pop()
            self.minus_to_node_matrix([(x, y)])

            def _retry(timer):
                timer.cancel()
                self.send_next_goal(x, y)

            timer = self.create_timer(2.0, lambda: _retry(timer))
            return

        self.get_logger().info(f'Goal accepted. Waiting for result...')
        self.sending = False
        self._current_goal_handle = goal_handle

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        if not self.check_distance:
            return
        if self.explore:
            return
        if self._current_goal_handle is None:
            return
        if self._current_goal_handle.status == None:
            return
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining
        if distance_remaining <= self.distance_threshold and distance_remaining >= self.xy_goal_tolerance and self._current_goal_handle.status == GoalStatus.STATUS_EXECUTING:
            if self.finished or self.found or self.wait_for_explorer or self.sending:
                return
            self.sending = True
            self.get_logger().info(
                f'Abstand {distance_remaining:.2f} ≤ {self.distance_threshold:.2f} m – nächstes Ziel wird gesendet.'
            )
            if not self.explore:
                x, y = self.change_direction_to_non_critic()
                self.send_next_goal(x,y)
            else:
                x, y = self.run_explorer()
                self.send_next_goal(x,y)

    def get_result_callback(self, future):
        result = future.result().result      # type: NavigateToPose.Result
        status = future.result().status      # numerischer Status‐Code

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal succeeded.')
            if self.found:
                self.stop_exploration()
        else:
            ec = result.error_code
            self.get_logger().error(
                f'Goal failed with status code {status}:{ec}. '
                f'Stopping the sequence.'
            )
            if ec == 208:
                if self.position_checker_timer_send is not None:
                    self.position_checker_timer_send.cancel()
                    self.position_checker_timer_send = None
                if self.position_checker_timer_stop is not None:
                    self.position_checker_timer_stop.cancel()
                    self.position_checker_timer_stop = None
                    
                self.position_checker_timer_send = self.create_timer(5.0, self.helper_send_goal)
                self.position_checker_timer_stop = self.create_timer(20.0, self.stop_exploration)

                self.helper_send_goal()
            return
        if self.finished or self.found or self.wait_for_explorer or self.sending:
                return
        self.sending = True
        if not self.explore:
            x, y = self.change_direction_to_non_critic()
            self.send_next_goal(x,y)
        else:
            x, y = self.run_explorer()
            self.send_next_goal(x,y)

    def run_explorer(self):
        self.wait_for_explorer = True
        self.explore = True
        self.get_logger().warn("run explorer ...")

        last_x = self.all_goals_list[len(self.all_goals_list)-1][0]
        last_y = self.all_goals_list[len(self.all_goals_list)-1][1]

        # Detect frontiers
        occ_grid = np.array(self._map.data).reshape(
            (self._map.info.height, self._map.info.width))
        occ_grid_filtered = occ_grid
        
        #     occ_grid, white_penalty=self.WHITE_PENALTY, angle_step=self.ANGLE_STEP)

        # Detect frontiers
        frontiers = self.find_frontiers(occ_grid_filtered)

        if not frontiers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            self.stop_exploration()
            return last_x,last_y
        
        centers = self.cluster_frontiers_cc(frontiers,occ_grid_filtered)

        if not centers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            self.stop_exploration()
            return last_x,last_y

        filtered_centers = self.filter_frontiers_by_occupancy(occ_grid,centers,radius=self.RADIUS)

        if not filtered_centers:
            self.get_logger().info("No frontiers found. Exploration complete!")
            self.stop_exploration()
            return last_x,last_y

        # Choose the closest frontier
        chosen_frontier = self.choose_frontier(filtered_centers)

        if not chosen_frontier:
            self.get_logger().warning("No frontiers to explore")
            self.stop_exploration()
            return last_x,last_y

        # Convert the chosen frontier to world coordinates
        x = chosen_frontier[0]
        y = chosen_frontier[1]

        self.wait_for_explorer = False
        return x, y

    def filter_frontiers_by_occupancy(self, occ_grid, frontiers, radius=5):

        filtered = []
        rows, cols = occ_grid.shape

        for r_f, c_f in frontiers:

            r = int(round(r_f))
            c = int(round(c_f))
            # sicherstellen, dass wir im Bild bleiben
            if not (0 <= r < rows and 0 <= c < cols):
                continue
            # Berechne Fensterränder, clamp an Bildränder
            r0 = max(0, r - radius)
            r1 = min(rows, r + radius + 1)
            c0 = max(0, c - radius)
            c1 = min(cols, c + radius + 1)

            # Schau nur in diesem Fenster nach einem 100er-Wert
            window = occ_grid[r0:r1, c0:c1]
            if not (window > 0).any():
                filtered.append((r, c))

        print(f"Filtered {len(filtered)} frontiers (radius={radius}) from {len(frontiers)} original")
        return filtered

    def find_frontiers(self, occ_grid):
        frontiers = []
        rows, cols = occ_grid.shape

        # Iterate through each cell in the map
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if occ_grid[r, c] == 0:  # Free cell
                    # Check if any neighbors are unknown
                    neighbors = occ_grid[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def cluster_frontiers_cc(self, frontiers, occ_grid):
        if not frontiers:
            return []

        # Erstelle binäre Karte der Frontiers
        mask = np.zeros_like(occ_grid, dtype=bool)
        for r, c in frontiers:
            mask[r, c] = True

        # Label zusammenhängender Regionen (8-Connectivity)
        structure = np.ones((3, 3), dtype=bool)
        labels, num_labels = ndimage.label(mask, structure=structure)

        centers = []
        # Berechne Zentroid jedes Labels
        for lab in range(1, num_labels + 1):
            coords = np.argwhere(labels == lab)
            if len(coords) < self.MIN_SAMPLES:
                continue
            # Mittelwert über Zeilen/Spalten-Koordinaten
            r_mean, c_mean = coords.mean(axis=0)
            centers.append((r_mean, c_mean))

        self.get_logger().info(f"{len(centers)} Cluster gefunden via CC, je >= {self.MIN_SAMPLES} Frontiers")
        return centers

    def choose_frontier(self, frontiers):
        robot_x, robot_y = self.current_x, self.current_y
        min_distance = float('inf')
        chosen_frontier = None

        for frontier in frontiers:
            
            frontier_x = frontier[1] * self._map.info.resolution + self._map.info.origin.position.x
            frontier_y = frontier[0] * self._map.info.resolution + self._map.info.origin.position.y

            #     continue

            too_close = any(
            math.hypot(frontier_x - vx, frontier_y - vy) <= self.choose_frontier_threshold
            for vx, vy in self.visited_frontiers
            )
            if too_close:
                continue
            
            distance = np.sqrt((robot_x - frontier_x)**2 + (robot_y - frontier_y)**2)
            if distance < min_distance and distance > self.xy_goal_tolerance:
                min_distance = distance
                chosen_frontier = (frontier_x,frontier_y)

        if chosen_frontier:
            self.visited_frontiers.append(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
        else:
            self.get_logger().warning("No valid frontier found")

        return chosen_frontier

    def change_direction_to_non_critic(self):
        last_x = self.all_goals_list[len(self.all_goals_list)-1][0]
        last_y = self.all_goals_list[len(self.all_goals_list)-1][1]
        if self.direction == 'pos_x':
            possible_goals = []
            possible_counter_nodes = []
            possible_directions = []

            x, y, direction= self.go_pos_x(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('pos_x')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            x, y, direction = self.go_pos_y(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('pos_y')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            x, y, direction = self.go_neg_y(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('neg_y')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")
                
            x, y, direction = self.go_neg_x(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('neg_x')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            if not possible_goals:
                self.explore = True
                last_x = self.all_goals_list[len(self.all_goals_list)-1][0]
                last_y = self.all_goals_list[len(self.all_goals_list)-1][1]
                self.direction = 'pos_x'
                return last_x,last_y

            for p in range(len(possible_goals)):
                x = possible_goals[p][0]
                y = possible_goals[p][1]

                i = int(x/self.cell_size) + self.origin_node
                j = int(y/self.cell_size) + self.origin_node

                possible_counter_nodes.append(self.count_nodes_matrix[i][j])
            
            min_count = min(possible_counter_nodes)
            min_n = possible_counter_nodes.index(min_count)

            if min_count > 0:
                self.explore = True

            self.direction = possible_directions[min_n]
            x = possible_goals[min_n][0]
            y = possible_goals[min_n][1]

            return x, y

        if self.direction == 'neg_x':
            possible_goals = []
            possible_counter_nodes = []
            possible_directions = []

            x, y, direction = self.go_neg_x(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('neg_x')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            x, y, direction = self.go_neg_y(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('neg_y')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            x, y, direction = self.go_pos_y(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('pos_y')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")
                
            x, y, direction = self.go_pos_x(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('pos_x')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            if not possible_goals:
                self.explore = True
                last_x = self.all_goals_list[len(self.all_goals_list)-1][0]
                last_y = self.all_goals_list[len(self.all_goals_list)-1][1]
                self.direction = 'neg_x'
                return last_x,last_y

            for p in range(len(possible_goals)):
                x = possible_goals[p][0]
                y = possible_goals[p][1]

                i = int(x/self.cell_size) + self.origin_node
                j = int(y/self.cell_size) + self.origin_node

                possible_counter_nodes.append(self.count_nodes_matrix[i][j])
            
            min_count = min(possible_counter_nodes)
            min_n = possible_counter_nodes.index(min_count)

            if min_count > 0:
                self.explore = True

            self.direction = possible_directions[min_n]
            x = possible_goals[min_n][0]
            y = possible_goals[min_n][1]

            return x, y
        
        if self.direction == 'pos_y':
            possible_goals = []
            possible_counter_nodes = []
            possible_directions = []

            x, y, direction = self.go_pos_y(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('pos_y')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            x, y, direction = self.go_neg_x(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('neg_x')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            x, y, direction = self.go_pos_x(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('pos_x')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")
                
            x, y, direction = self.go_neg_y(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('neg_y')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            if not possible_goals:
                self.explore = True
                last_x = self.all_goals_list[len(self.all_goals_list)-1][0]
                last_y = self.all_goals_list[len(self.all_goals_list)-1][1]
                self.direction = 'pos_y'
                return last_x,last_y

            for p in range(len(possible_goals)):
                x = possible_goals[p][0]
                y = possible_goals[p][1]

                i = int(x/self.cell_size) + self.origin_node
                j = int(y/self.cell_size) + self.origin_node

                possible_counter_nodes.append(self.count_nodes_matrix[i][j])
            
            min_count = min(possible_counter_nodes)
            min_n = possible_counter_nodes.index(min_count)

            if min_count > 0:
                self.explore = True

            self.direction = possible_directions[min_n]
            x = possible_goals[min_n][0]
            y = possible_goals[min_n][1]

            return x, y
        
        if self.direction == 'neg_y':
            possible_goals = []
            possible_counter_nodes = []
            possible_directions = []

            x, y, direction = self.go_neg_y(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('neg_y')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            x, y, direction = self.go_pos_x(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('pos_x')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            x, y, direction = self.go_neg_x(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('neg_x')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")
                
            x, y, direction = self.go_pos_y(last_x, last_y)
            free_list = self.give_test_width(x, y, last_x, last_y, direction)
            if self.is_test_width_free(free_list):
                possible_goals.append((x,y))
                possible_directions.append('pos_y')
            else:
                self.get_logger().info(f"Hindernis bei ({x:.2f}, {y:.2f}), skip.")

            if not possible_goals:
                self.explore = True
                last_x = self.all_goals_list[len(self.all_goals_list)-1][0]
                last_y = self.all_goals_list[len(self.all_goals_list)-1][1]
                self.direction = 'neg_y'
                return last_x,last_y

            for p in range(len(possible_goals)):
                x = possible_goals[p][0]
                y = possible_goals[p][1]

                i = int(x/self.cell_size) + self.origin_node
                j = int(y/self.cell_size) + self.origin_node

                possible_counter_nodes.append(self.count_nodes_matrix[i][j])
            
            min_count = min(possible_counter_nodes)
            min_n = possible_counter_nodes.index(min_count)

            if min_count > 0:
                self.explore = True

            self.direction = possible_directions[min_n]
            x = possible_goals[min_n][0]
            y = possible_goals[min_n][1]

            return x, y

    def add_to_node_matrix(self, x, y):
        # x->i
        i = int(x/self.cell_size) + self.origin_node
        # y->j
        j = int(y/self.cell_size) + self.origin_node

        self.count_nodes_matrix[i][j] += 1

    def minus_to_node_matrix(self, minus_one_node_list):
        for r in range(len(minus_one_node_list)):
            x = minus_one_node_list[r][0]
            y = minus_one_node_list[r][1]
            # x->i
            i = int(x/self.cell_size) + self.origin_node
            # y->j
            j = int(y/self.cell_size) + self.origin_node

            self.count_nodes_matrix[i][j] -= 1

    def go_pos_x(self, last_x, last_y):
        goal_x = last_x + self.cell_size
        goal_y = last_y
        direction = 'pos_x'

        return goal_x,goal_y,direction
    
    def go_neg_x(self, last_x, last_y):
        goal_x = last_x - self.cell_size
        goal_y = last_y
        direction = 'neg_x'

        return goal_x,goal_y,direction
    
    def go_pos_y(self, last_x, last_y):
        goal_x = last_x
        goal_y = last_y + self.cell_size
        direction = 'pos_y'
        return goal_x,goal_y,direction
    
    def go_neg_y(self, last_x, last_y):
        goal_x = last_x
        goal_y = last_y - self.cell_size
        direction = 'neg_y'

        return goal_x,goal_y,direction
    def position_on_timer(self):
        if self.finished:
            return
        distance = self.distance(self.robot_positions[len(self.robot_positions)-1],(self.current_x,self.current_y))
        turning = abs(self.cumulative_yaw - self.robot_yaws[len(self.robot_yaws)-1])

        if distance >= self.position_threshold or turning >= self.turning_threshold:
            if isinstance(self.robot_positions, list):
                if self.position_checker_timer_send is not None:
                    self.position_checker_timer_send.cancel()
                    self.position_checker_timer_send = None
                if self.position_checker_timer_stop is not None:
                    self.position_checker_timer_stop.cancel()
                    self.position_checker_timer_stop = None

                self.robot_positions.append((self.current_x,self.current_y))
                self.robot_yaws.append(self.cumulative_yaw)

                self.position_checker_timer_send = self.create_timer(5.0, self.helper_send_goal)
                self.position_checker_timer_stop = self.create_timer(20.0, self.stop_exploration)

    def distance(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def main(args=None):
    rclpy.init(args=args)

    try:
        node = FrontierExplorer()
    except KeyboardInterrupt:
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt erhalten – speichere Matrizen…')
        node.robot_positions = np.array(node.robot_positions)
        np.save(os.path.join(CACHE_DIR, 'robot_positions.npy'), node.robot_positions)
        node.get_logger().info('Speichern abgeschlossen.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
