import pybullet as p
import pybullet_data
import os

# Start PyBullet in GUI mode
p.connect(p.GUI)

path_src = os.path.abspath(__file__)
path_src = os.path.dirname(path_src)
path_src = os.path.join(path_src, "../robot_hand", "robot_pybullet.urdf")

# (Optional) Set PyBullet’s search path for built-in URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load your URDF file
robot_id = p.loadURDF(path_src, basePosition=[0, 0, 0], useFixedBase=True)

# Run simulation
while True:
    p.stepSimulation()
