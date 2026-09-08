
import mujoco
from utils.config import config

model = mujoco.MjModel.from_xml_path(config['MODEL_PATH'])

print("=== Joints ===")
joint_names = [f"joint{i}" for i in range(1, 8)]

for name in joint_names:
    jid = model.joint(name).id
    bid = model.jnt_bodyid[jid]        # which body this joint belongs to
    print(name,
          "body_pos:", model.body_pos[bid],
          "body_quat:", model.body_quat[bid],   # (w, x, y, z)
          "jnt_pos (anchor in body frame):", model.jnt_pos[jid],
          "jnt_axis (in body frame):", model.jnt_axis[jid], 
          sep="\n")
    print("-----")


for i in range(model.njnt):
    joint_name = model.joint(i).name
    print(f"  {i}: {joint_name}")

    
print("\n=== Sites ===")
for i in range(model.nsite):
    print(f"  {i}: {model.site(i).name}")

print("\n=== Bodies ===")
for i in range(model.nbody):
    print(f"  {i}: {model.body(i).name}")

print(f"\nnq (position dims): {model.nq}")
print(f"nv (velocity dims): {model.nv}")