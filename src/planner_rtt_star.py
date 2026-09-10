from sim_interface import SimInterface
from utils.config import config
import numpy as np
from dataclasses import dataclass
from q_validity_check import check_edge_validity, is_valid
from scripts.pid_tuning import generate_target





"""
Step 1: Define c-space C, step_size, goal_tol, q_goal, radius r, tree (list)
Step 2: Define a node dataclass with name, parent, q, g
Step 3: Set q_start and add to tree.
Step 4: Sample q_rand from C
Step 5: Get nearest node q_near
Step 6: Get q_new : Move towards q_new from q_near by 1 step
Step 7: Get nodes within radius r of q_new -> nearby_nodes
Step 8: Check g(q_new|parent=nearby_node) for each of the nodes above. Select argmin g(q_new|parent) as potential parent
Step 9: If validity of edge between parent and q_new,
        Add q_new to tree. Set parent as parent of q_new.
        If not valid, check next lowest argmin g(q_new|parent). 
        If none are valid, discard.
Step 10: Check g(node = nearby_node|parent = q_new)  for each nearby_nodes (except parent of q new). 
        If less than current g, 
        If edge between nearby_node and q_new is valid,
        change parent to q_new for the nearby_node.
Step 11: Repeat till |q_new - q_gol| < goal_tol or till max_iter is reached.



"""

@dataclass
class Node:
    id:int
    q:np.ndarray
    parent: "Node"
    g:float




def compute_edge_cost(q1:np.ndarray,q2:np.ndarray):
    return np.linalg.vector_norm(q2-q1)

def g_cost(q, parent_node):

    edge_cost = compute_edge_cost(parent_node.q, q)
    g = parent_node.g + edge_cost
    return g


def get_c_space_bounds(sim):

    joint_limits_low = sim.model.jnt_range[:, 0]
    joint_limits_high = sim.model.jnt_range[:, 1]

    return joint_limits_low,joint_limits_high


def sample_random_q(sim, q_goal=None, goal_bias=0.2):
 
    if q_goal is not None and np.random.rand() < goal_bias:
        return q_goal.copy()
    low, high = get_c_space_bounds(sim)
    return np.random.uniform(low, high)



def find_nearest_node(tree:list[Node], q_rand):
    nearest_node:Node = None
    min_dist = float('inf')
    for node in tree:
        dist = compute_edge_cost(node.q, q_rand)
        if dist < min_dist:
            min_dist = dist
            nearest_node = node
    return nearest_node

def get_nearby_nodes(tree:list[Node], q_new, radius):
    nearby_nodes = []
    for node in tree:
        if compute_edge_cost(node.q, q_new) < radius:
            nearby_nodes.append(node)

    return nearby_nodes


def find_parent(sim,q_new, nearby_nodes:list[Node]):
    parent = None
    min_g = float('inf')
    for node in nearby_nodes:
        g = g_cost(q_new, node)
        if g < min_g and check_edge_validity(sim, node.q, q_new):
            parent = node
            min_g = g
    return parent, min_g


def rewire_parent(sim, nearby_nodes, node_new):

    for nearby_node in nearby_nodes:

        if node_new.parent.id != nearby_node.id:
            g = g_cost(nearby_node.q, node_new)
            if g < nearby_node.g:
                if check_edge_validity(sim, node_new.q, nearby_node.q):
                    nearby_node.g = g
                    nearby_node.parent = node_new

    
def extract_path(end_node:Node):
    # Extract path

    path = []
    node =  end_node

    while node is not None:
        path.append(node)
        node = node.parent

    path.reverse()
    return path


def rtt_star(sim:SimInterface, q_start:np.ndarray, q_goal:np.ndarray,
             max_iter:int, step_size:float, radius:float, goal_tol:float):


    result = {}
    tree = []
    node_start = Node(id=0, q=q_start, parent = None, g = 0)
    tree.append(node_start)



    for i in range(max_iter):

        # Step 2: Sample random q in c-space 
        q_rand = sample_random_q(sim, q_goal)

        # Step 3: Find nearest node
        nearest_node = find_nearest_node(tree,q_rand)

        # Step 4: Compute q_new

        delta = q_rand - nearest_node.q
        delta_norm = np.linalg.vector_norm(delta)

        if delta_norm < step_size:
            q_new = q_rand.copy()
        else:
            dir =  delta / delta_norm
            q_new = nearest_node.q + step_size * dir




        # Step 4: Get nearby nodes to q_new
        nearby_nodes = get_nearby_nodes(tree, q_new, radius)


        # Step 5: Select Parent node 
        parent, g = find_parent(sim, q_new, nearby_nodes)


        # Step 6: Discard if q_new is invalid (i.e parent=None)

        if parent == None:
            continue

        else:
            node_new = Node(len(tree),q_new,parent,g)
            tree.append(node_new)

            # Step 7: Rewire parents
            rewire_parent(sim, nearby_nodes,node_new)


            # Check for tolerance
            if np.linalg.vector_norm(q_goal - q_new ) <= goal_tol:
                result["success"] = True
                result["end_node"] = node_new
                result["iter_conv"] = i
                result["tree"] = tree
                return result

    # Did not find path with the max_iter
    result["success"] = False
    result["end_node"] = None
    result["iter_conv"] = None
    result["tree"] = tree
    return result







if __name__ == '__main__':

    np.random.seed(42)

    sim = SimInterface(config['SCENE_EASY_PATH'])
    #sim.run_viewer()


    max_iter = 200

    step_size = 0.15 # in rad
    radius = step_size * 1.5
    goal_tol = 0.1 # in rad

    q_start = np.array(sim.theta_home)

    _, target_thetas = generate_target(sim, 1, 0.7, 0.8, seed=1)
    q_goal = np.array(target_thetas[0])

    
    # q_goal = q_start.copy()
    # q_goal[1] += 0.1   # nudge joint2 by 0.1 rad (< step_size of 0.15)

    # q_goal invalid
    # q_start = np.array([0, 0, 0, -1.0, 0, 1.57079, -0.7853])
    # q_goal  = np.array([0, 0, 0, -2.6, 0, 1.57079, -0.7853])
    print("q_start valid:", is_valid(sim, q_start))
    print("q_goal valid:", is_valid(sim, q_goal))
    print("direct edge valid:", check_edge_validity(sim, q_start, q_goal))

    
    result = rtt_star(sim, q_start, q_goal, max_iter, step_size, radius, goal_tol)


    print(result['success'])

    path = extract_path(result['end_node'])
    
    print(len(result["tree"]))
    print(len(path))
    #print(path)


    





    

    

    








    











    



    








