import pybullet as p
import pybullet_data
import os

'''
# Start PyBullet in GUI mode
p.connect(p.GUI)

path_src = os.path.abspath(__file__)
path_src = os.path.dirname(path_src)
path_src = os.path.join(path_src, "../robot_hand", "robot_pybullet.urdf")

# (Optional) Set PyBullet’s search path for built-in URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load your URDF file
robot_id = p.loadURDF(path_src, basePosition=[0, 0, 0], useFixedBase=True)
balls = create_target_vis()

cameraDistance = 2  # Increase to zoom out, decrease to zoom in
cameraYaw = 45      # Horizontal rotation (in degrees)
cameraPitch = -30   # Vertical rotation (in degrees)
cameraTargetPosition = [0, 0, 0] 

def update_camera():
    p.resetDebugVisualizerCamera(
        cameraDistance=cameraDistance,
        cameraYaw=cameraYaw,
        cameraPitch=cameraPitch,
        cameraTargetPosition=cameraTargetPosition
    )

# Run simulation
while True:
    
    keys = p.getKeyboardEvents()
        
        # Zoom in (press 'z')
    if ord('z') in keys:
            cameraDistance = max(0.001, cameraDistance - 0.001)  # Decrease distance to zoom in
        
        # Zoom out (press 'x')
    if ord('x') in keys:
            cameraDistance += 0.001  # Increase distance to zoom out
        
        # Update the camera
    update_camera()
        
        # Step the simulation
    p.stepSimulation()
'''
import os
import pybullet as p

class PybIk:
    def __init__(self):
        # Start PyBullet
        p.connect(p.GUI)
        
        # Load right leap hand      
        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        path_src = os.path.join(path_src, "../robot_hand", "robot_pybullet.urdf")

        self.glove_to_leap_mapping_scale = 1.6  # Map our own
        #last portion of the robot that moves which in our case is the DIPs 
        self.leapEndEffectorIndex = [3, 7, 10, 14, 18]
        
        # Load the URDF file
        self.robotId = p.loadURDF(
            path_src,
            [-0.05, -0.03, -0.25],
            p.getQuaternionFromEuler([0, 1.57, 1.57]),
            useFixedBase=True
        )
        
        self.numJoints = p.getNumJoints(self.robotId)
        p.setGravity(0, 0, 0)
        useRealTimeSimulation = 0
        p.setRealTimeSimulation(useRealTimeSimulation)
        
        self.create_target_vis()

    # so this part creates 5 balls representing the last moving part of the hand which is the DIP
    # this creates, does not map the balls 
    def create_target_vis(self):
        # Load balls
        small_ball_radius = 0.01
        small_ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=small_ball_radius)
        ball_radius = 0.01
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        baseMass = 0.001
        basePosition = [0.25, 0.25, 0]
        
        self.ballMbt = []
        for i in range(0, 5):
            self.ballMbt.append(p.createMultiBody(baseMass=baseMass, baseCollisionShapeIndex=ball_shape, basePosition=basePosition))  # for base and finger tip joints    
            no_collision_group = 0
            no_collision_mask = 0
            p.setCollisionFilterGroupMask(self.ballMbt[i], -1, no_collision_group, no_collision_mask)
        p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[1, 0, 0, 1]) 
        p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[0, 1, 0, 1]) 
        p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 0, 1, 1])  
        p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[1, 1, 1, 1])
        p.changeVisualShape(self.ballMbt[4], -1, rgbaColor=[1, 0, 1, 1])

    def update_camera(self, cameraDistance):
        p.resetDebugVisualizerCamera(
            cameraDistance=cameraDistance,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )
    # here is where they map the balls to the fingers end DIP
    def update_target_vis(self):
        for i, joint_idx in enumerate(self.leapEndEffectorIndex): # for every joint that is considered the end 
            link_state = p.getLinkState(self.robotId, joint_idx) #this gets the specific joint information 
            position = link_state[0]  # Get the world position of the fingertip joint (in the link_state)
            p.resetBasePositionAndOrientation(self.ballMbt[i], position, [0, 0, 0, 1])  # sets the position of each of the balls 
        
if __name__ == "__main__":
    pyb_ik = PybIk()
    
    # Initialize camera distance
    cameraDistance = 2
    
    while True:
        keys = p.getKeyboardEvents()
        
        # Zoom in (press 'z')
        if ord('z') in keys:
            cameraDistance = max(0.001, cameraDistance - 0.001)  # Decrease distance to zoom in
        
        # Zoom out (press 'x')
        if ord('x') in keys:
            cameraDistance += 0.001  # Increase distance to zoom out
        
        # Update the camera
        pyb_ik.update_camera(cameraDistance)
        pyb_ik.update_target_vis() 
        # Step the simulation
        p.stepSimulation()