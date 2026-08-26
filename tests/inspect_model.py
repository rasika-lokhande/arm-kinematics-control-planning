
import mujoco

model = mujoco.MjModel.from_xml_path("models/franka_emika_panda/panda_nohand.xml")

print("=== Joints ===")
for i in range(model.njnt):
    print(f"  {i}: {model.joint(i).name}")

print("\n=== Sites ===")
for i in range(model.nsite):
    print(f"  {i}: {model.site(i).name}")

print("\n=== Bodies ===")
for i in range(model.nbody):
    print(f"  {i}: {model.body(i).name}")

print(f"\nnq (position dims): {model.nq}")
print(f"nv (velocity dims): {model.nv}")