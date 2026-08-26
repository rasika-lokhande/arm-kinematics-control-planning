# src/sim_interface.py
import mujoco
import numpy as np

class SimInterface:
    def __init__(self, model_path:str, ee_site_name:str):

        # Load model
        self.model = mujoco.MjModel.from_xml_path(model_path)

        # Load variables and states
        self.data = mujoco.MjData(self.model)

        # Load site
        self.ee_site_id = self.model.site(ee_site_name).id

    def step(self):
        mujoco.mj_step(self.model, self.data)

    def get_joint_angles(self):
        return self.data.qpos.copy()  

    def set_joint_angles(self, angles):
        self.data.qpos[:] = angles
        mujoco.mj_forward(self.model, self.data)

    def command_joint_torque(self, torques):
        self.data.ctrl[:] = torques

    def get_ee_pose(self):
        pos = self.data.site_xpos[self.ee_site_id].copy()
        rot = self.data.site_xmat[self.ee_site_id].copy().reshape(3, 3)
        return pos, rot