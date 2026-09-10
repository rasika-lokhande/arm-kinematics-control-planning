from sim_interface import SimInterface
from utils.config import config
import numpy as np




def check_collision(sim:SimInterface, q:list):

    sim.set_joint_angles(q)
    num_contacts = sim.data.ncon

    if num_contacts == 0:
        is_colliding = False
    else:
        is_colliding = True

    # for i in range(sim.data.ncon):
    #     contact = sim.data.contact[i]
    #     body1_id = sim.model.geom_bodyid[contact.geom1]
    #     body2_id = sim.model.geom_bodyid[contact.geom2]
    #     body1_name = sim.model.body(body1_id).name
    #     body2_name = sim.model.body(body2_id).name
    #     print(f"contact {i}: {body1_name} <-> {body2_name}")
    return is_colliding


def check_joint_limits(sim: SimInterface, q: np.ndarray):

    joint_limits_low = sim.model.jnt_range[:, 0]
    joint_limits_high = sim.model.jnt_range[:, 1]

    if np.any(q < joint_limits_low) or np.any(q > joint_limits_high):
        return False
    else:
        return True



def lerp_vectors(q1, q2, n):
    """Linear interpolation between two vectors for n number of points"""
    t = np.linspace(0,1,n, endpoint=True).reshape(-1, 1) # shape = (n, 1)
    return q1 + t * (q2-q1)




def is_valid(sim,q):

    if (check_joint_limits(sim, q) == True) and (check_collision(sim,q) == False):
        return True
    else:
        return False


def check_edge_validity(sim, q1:np.ndarray, q2:np.ndarray, nlerp:int=10):

    lerp_points = lerp_vectors(q1,q2,nlerp)
    validity = True

    for q in lerp_points:

        if is_valid(sim,q):
            continue
        else:
            validity = False
            return validity

    return validity




if __name__ == '__main__':

    print(lerp_vectors(np.array([1,1]), np.array([2,2]), 3))

    # sim = SimInterface(config['SCENE_EASY_PATH'])
   

    # q_colliding = [0, 0, 0, -2.3, 0, 1.57079, -0.7853] # debugging

    # ans = check_collision(sim,q_colliding)
    # print(ans)


    # sim.run_viewer()

