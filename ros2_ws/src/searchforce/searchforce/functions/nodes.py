# node with start in 1 for indices with start in [0][0]
def node_from_indices(i, j, n):
    node = n*j + i + 1
    return node
# indices with start in [0][0] for node with start in 1 
def indices_from_node(node, n):
    i = ((node - 1) % n)
    j = ((node - 1) // n)
    return i, j
def add_one_to_path(path):
    return [p + 1 for p in path]