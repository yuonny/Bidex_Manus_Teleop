import pybullet as p
import pybullet_data

# Start PyBullet in GUI mode
p.connect(p.GUI)

# (Optional) Set PyBullet’s search path for built-in URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load your URDF file
robot_id = p.loadURDF("ros2/telekinesis/robot_hand/robot.urdf", basePosition=[0, 0, 0], useFixedBase=True)

# Run simulation
while True:
    p.stepSimulation()
