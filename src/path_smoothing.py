from src.planner_rtt_star import Node
import numpy as np
import random
from src.q_validity_check import check_edge_validity
from src.sim_interface import SimInterface

def path_smoothness_metric(path):
    """Returns mean turning angle (radians) across the path.
    Smaller = smoother (straighter). 0 = perfectly straight."""
    if len(path) < 3:
        return 0.0  # no interior points to measure

    angles = []
    for i in range(1, len(path) - 1):
        v1 = path[i] - path[i-1]
        v2 = path[i+1] - path[i]
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        angles.append(np.arccos(np.clip(cos_angle, -1, 1)))

    return np.mean(angles)

def node_to_q_path(node_path:list[Node]):
    """ Convert path in list of Nodes to list of q"""

    q_path = [node.q for node in node_path]

    return q_path



def smooth_path(sim:SimInterface, path:list[np.ndarray], max_iter= 100):

    for _ in range(max_iter):

        if len(path) < 3:
            break  # nothing left to shortcut

        indices = list(range(len(path)))
        select_index = random.sample(indices, 2)

        i, j = min(select_index), max(select_index)

        if j <= i + 1: # adjacent check: nothing to remove
            continue  

        # else- no adjacent
        if check_edge_validity(sim, path[i], path[j]): # direct path valid
            path = path[0:i+1] + path[j:]

    return path

    






        

    


