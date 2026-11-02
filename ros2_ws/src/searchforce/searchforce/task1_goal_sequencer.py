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
from searchforce.maze_solver import MazeSolverSlam
from nav_msgs.msg import OccupancyGrid
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped

class GoalSequencer(Node):

    def __init__(self, tuple_list, frame_id='map'):

        super().__init__('goal_sequencer')

        # Subscribe auf Map
        self._map = None  # wird durch map_callback gesetzt
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Warte auf Action-Server
        self.get_logger().info('Waiting for NavigateToPose Action-Server...')
        while not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('NavigateToPose Action-Server not available, waiting again...')

        self.goal_list = self._build_pose_list_from_tuples(tuple_list, frame_id)
        self.current_index = 0
        
        self.distance_threshold = 0.4
        self.xy_goal_tolerance = 0.05
        self.check_distance_time = 2.0

        self.check_distance = False
        self._current_goal_handle = None
        self.timer_distance = None

        # Checke Goal-List
        if not self.goal_list:
            self.get_logger().warn('Given tuple list is empty. Node will not send any goal.')
            return

        # Map-Check starten
        self.get_logger().info('Warte auf erste /map-Nachricht…')
        self._map_check_timer = self.create_timer(0.5, self._check_map_ready)

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
            self.send_next_goal()

    def map_callback(self, msg: OccupancyGrid):
        self._map = msg

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

    def _build_pose_list_from_tuples(self, tuple_list, frame_id):
        pose_list = []
        for (x, y, yaw) in tuple_list:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self.get_clock().now().to_msg()

            # Position setzen
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            # Quaternion aus yaw erzeugen
            q = tf_transformations.quaternion_from_euler(0.0, 0.0, float(yaw))
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            pose_list.append(pose)
        return pose_list

    def send_next_goal(self):
        self.check_distance = False

        if self.timer_distance is not None:
            self.timer_distance.cancel()
            self.timer_distance = None

        # 1. Prüfen, ob wir bereits das Ende der Liste erreicht haben
        if self.current_index >= len(self.goal_list):
            if self.position_checker_timer_send is not None:
                self.position_checker_timer_send.cancel()
                self.position_checker_timer_send = None
            if self.position_checker_timer_stop is not None:
                self.position_checker_timer_stop.cancel()
                self.position_checker_timer_stop = None
            if self.position_timer is not None:
                self.position_timer.cancel()
                self.position_timer = None
            self.get_logger().info('All goals have been processed. Node will now idle.')
            return

        # 2. NavigateToPose.Goal erstellen und mit dem aktuellen PoseStamped befüllen
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.goal_list[self.current_index]

        x = nav_goal.pose.pose.position.x
        y = nav_goal.pose.pose.position.y

        self.get_logger().info(
            f'Sending goal {self.current_index + 1}/{len(self.goal_list)}: '
            f'(x={x:.2f}, y={y:.2f}) in frame="{nav_goal.pose.header.frame_id}"'
        )
        # One‐Shot‐Timer
        def _retry(timer):
            timer.cancel()
            timer = None
            self.send_next_goal()

        # Grid‐Prüfung
        if not self.is_pose_in_grid(x, y):
            self.get_logger().warn(
                f"Ziel {self.current_index+1} (x={x:.2f}, y={y:.2f}) außerhalb der Map – warte 2 s"
            )
            timer = self.create_timer(2.0, lambda: _retry(timer))
            return

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
                f'Goal {self.current_index + 1} was rejected by the Action Server, retrying in 2 s.'
            )
            # Nav2 rejects goals while it is still starting up.
            def _retry(timer):
                timer.cancel()
                self.send_next_goal()

            timer = self.create_timer(2.0, lambda: _retry(timer))
            return

        self.get_logger().info(f'Goal {self.current_index + 1} accepted. Waiting for result...')
        self._current_goal_handle = goal_handle

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        if not self.check_distance:
            return
        if self._current_goal_handle is None:
            return
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining
        if self._current_goal_handle.status == None:
            return
        if distance_remaining <= self.distance_threshold and distance_remaining >= self.xy_goal_tolerance and self._current_goal_handle.status == GoalStatus.STATUS_EXECUTING:
            self.get_logger().info(
                f'Abstand {distance_remaining:.2f} ≤ {self.distance_threshold:.2f} m – nächstes Ziel wird gesendet.'
            )
            self.current_index += 1
            self.send_next_goal()

    def get_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal {self.current_index + 1} succeeded.')
        else:
            self.get_logger().error(
                f'Goal {self.current_index + 1} failed with status code {status}. '
                f'Stopping the sequence.'
            )
            return

        self.current_index += 1
        self.send_next_goal()

    def position_on_timer(self):
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

    def distance(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def helper_send_goal(self):
        self.get_logger().warn(f"no progress skipping goal ...")
        self.current_index += 1
        self.send_next_goal()

def main(args=None):

    rclpy.init(args=args)

    filename = os.path.join(CACHE_DIR, 'rosfile.maze')

    with open(filename, 'r') as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]

    if len(lines) < 5:
        raise ValueError("File too short to contain valid maze information.")

    start_idx = int(lines[2])
    end_idx = int(lines[3])
    start_orientation = int(lines[4])
    
    # start_orientation = 4
    # start_idx = 1
    # end_idx = 98
    id_to_string = {
    1: "bottom",
    8: "left",
    4: "right",
    2: "top"
    }

    solver = MazeSolverSlam(filename=filename, init_node=start_idx, goal_node=end_idx, start_orientation=id_to_string[start_orientation])
    print(solver.path_nodes_dijkstra)
    path_xy_with_w = solver.path_nodes_to_xy(solver.path_nodes_dijkstra)
    print(path_xy_with_w)

    try:
        node = GoalSequencer(path_xy_with_w, frame_id='map')
    except KeyboardInterrupt:
        return
    rclpy.spin(node)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
