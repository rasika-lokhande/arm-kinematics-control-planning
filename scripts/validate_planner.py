from scripts.pid_tuning import generate_target
from src.planner_rtt_star import rtt_star, extract_path, get_c_space_bounds, sample_random_q
from src.q_validity_check import is_valid, check_edge_validity
from src.sim_interface import SimInterface
from utils.config import config
import numpy as np
import pandas as pd

def get_random_valid_q(sim, max_attempts=100):
    low, high = get_c_space_bounds(sim)
    for i in range(max_attempts):
        q = np.random.uniform(low, high)
        if is_valid(sim,q):
            return q
    raise RuntimeError(f"No valid q found in {max_attempts} attempts")



def generate_blocked_pair(sim, max_attempts=1000):
    """Sample q_start, q_goal such that both are valid but the direct edge is blocked."""
    for attempt in range(max_attempts):

        q_start = get_random_valid_q(sim)
        q_goal = get_random_valid_q(sim)
        if not check_edge_validity(sim, q_start, q_goal):
            return q_start, q_goal
    raise RuntimeError(f"No blocked pair found in {max_attempts} attempts")

def validate(sim:SimInterface, ntrials, 
             step_size=0.15, goal_tol = 0.1, radius = 0.225, max_iter=1000):

    #radius = step_size * 1.5


    successes = 0
    results = []

    for i in range(ntrials):
        np.random.seed(i)

        q_start, q_goal = generate_blocked_pair(sim)
        sim.set_joint_angles(q_start)
        result = rtt_star(sim, q_start, q_goal, max_iter, step_size, radius, goal_tol)

        if result['success']:
            path, path_cost = extract_path(result['end_node'])
            result['path_length'] = len(path)
            result['path_cost'] = path_cost

        if result["success"] == True:
            successes +=1

        start_goal_dist = np.linalg.norm(q_goal - q_start)
        result['start_goal_distance'] = start_goal_dist

        result.pop('tree', None)
        result.pop('end_node', None)
        result['q_start'] = q_start
        result['q_goal'] = q_goal

        results.append(result)
        print(i)


    df=pd.DataFrame(results)
    df.to_csv(config['PLANNER_VALIDATION_CSV_PATH'])
    print(f"\nSuccess rate: {successes}/{ntrials}")




if __name__ == '__main__':

    sim = SimInterface(config['SCENE_EASY_PATH'])
    validate(sim, 100)