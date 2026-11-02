from searchforce.functions.nodes import node_from_indices
import numpy as np
def calculate_adj_matrix(L):
    # calculate Adjacency matrix
    L = np.array(L)
    n,m = L.shape

    L_bin = [[ bin(L[row][col])[2:].zfill(4) for col in range(m)] for row in range(n)]
    L_bool = [[ [bit == '1' for bit in L_bin[row][col]] for col in range(m)] for row in range(n)]

    adj_matrix = np.zeros((n*m, n*m))
    node_matrix = np.zeros((n, m))

    for i in range(n):
        for j in range(m):
            node = node_from_indices(i, j, n)
            node_matrix[i][j] = node
            walls = L_bool[i][j]
            # No bottom wall
            if walls[3] == False:
                next_node = node_from_indices(i+1, j, n)
                adj_matrix[node-1][next_node-1] = 1
            # No top wall
            if walls[2] == False:
                next_node = node_from_indices(i-1, j, n)
                adj_matrix[node-1][next_node-1] = 1
            # No right wall
            if walls[1] == False:
                next_node = node_from_indices(i, j+1, n)
                adj_matrix[node-1][next_node-1] = 1
            # No left wall
            if walls[0] == False:
                next_node = node_from_indices(i, j-1, n)
                adj_matrix[node-1][next_node-1] = 1

            # Bottom-right corner (i+1, j+1)
            if i + 1 < n and j + 1 < m:
                walls_diag = L_bool[i+1][j+1]
                if (walls[1] == False and walls[3] == False and    # No right and bottom wall in current cell
                    walls_diag[0] == False and walls_diag[2] == False):  # No left and top wall in bottom-right cell
                    next_node =  node_from_indices(i+1, j+1, n)
                    adj_matrix[node - 1][next_node - 1] = np.sqrt(2)

            # Top-right corner (i-1, j+1)
            if i - 1 >= 0 and j + 1 < m:
                walls_diag = L_bool[i-1][j+1]
                if (walls[1] == False and walls[2] == False and    # No right and top wall in current cell
                    walls_diag[0] == False and walls_diag[3] == False):  # No left and bottom wall in top-right cell
                    next_node =  node_from_indices(i-1, j+1, n)
                    adj_matrix[node - 1][next_node - 1] = np.sqrt(2)

            # Bottom-left corner (i+1, j-1)
            if i + 1 < n and j - 1 >= 0:
                walls_diag = L_bool[i+1][j-1]
                if (walls[0] == False and walls[3] == False and    # No left and bottom wall in current cell
                    walls_diag[1] == False and walls_diag[2] == False):  # No right and top wall in bottom-left cell
                    next_node =  node_from_indices(i+1, j-1, n)
                    adj_matrix[node - 1][next_node - 1] = np.sqrt(2)

            # Top-left corner (i-1, j-1)
            if i - 1 >= 0 and j - 1 >= 0:
                walls_diag = L_bool[i-1][j-1]
                if (walls[0] == False and walls[2] == False and    # No left and top wall in current cell
                    walls_diag[1] == False and walls_diag[3] == False):  # No right and bottom wall in top-left cell
                    next_node =  node_from_indices(i-1, j-1, n)
                    adj_matrix[node - 1][next_node - 1] = np.sqrt(2)
    return adj_matrix, n, m