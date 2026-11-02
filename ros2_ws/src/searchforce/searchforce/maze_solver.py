import math
from searchforce.functions.L_to_adj_Matrix import calculate_adj_matrix
from searchforce.functions.rosfile_to_L import load_maze_from_file
from searchforce.functions.nodes import add_one_to_path, indices_from_node
import networkx as nx

class MazeSolverSlam:
    def __init__(self, filename, init_node, goal_node, start_orientation):
        self.L = load_maze_from_file(filename)
        self.adj_matrix, self.n, self.m = calculate_adj_matrix(self.L)
        self.cell_size = 0.254     # in meters
        self.init_node = init_node
        self.goal_node = goal_node
        self.G = nx.from_numpy_array(self.adj_matrix)
        self.path_nodes_dijkstra = add_one_to_path(nx.dijkstra_path(self.G, self.init_node - 1, self.goal_node - 1))
        self.start_orientation = start_orientation

    def path_nodes_to_xy(self, path_nodes):
        """Convert path nodes to SLAM-frame poses (x, y, yaw in rad). Yaw points to the next pose; the last pose keeps the previous yaw."""
        path_xy_with_w = []
        coords = []
        slam_coords = []

        for i in range(len(path_nodes)):
            goal_i, goal_j = indices_from_node(path_nodes[i], self.n)
            goal_i = self.n - 1 - goal_i
            x = goal_j * self.cell_size + 0.5 * self.cell_size
            y = goal_i * self.cell_size + 0.5 * self.cell_size
            coords.append((x, y))

            if self.start_orientation == "bottom":
                slam_x = coords[0][1] - y
                slam_y = x - coords[0][0]
        
            if self.start_orientation == "left":
                slam_x = coords[0][0] - x
                slam_y = coords[0][1] - y

            if self.start_orientation == "right":
                slam_x = x - coords[0][0]
                slam_y = y - coords[0][1]

            if self.start_orientation == "top":
                slam_x = y - coords[0][1]
                slam_y = coords[0][0] - x
            slam_coords.append((slam_x,slam_y))

        for i in range(len(slam_coords)):
            x, y = slam_coords[i]
            if i < len(slam_coords) - 1:
                x_next, y_next = slam_coords[i + 1]
                dx = x_next - x
                dy = y_next - y
                w = math.atan2(dy, dx)
            else:
                w = path_xy_with_w[i-1][2]
            path_xy_with_w.append((x, y, w))

        return path_xy_with_w
