import pybullet as p
import pybullet_data
import time

# Connect to physics server with GUI
p.connect(p.GUI)

# initialize gravity
p.setGravity(0, 0, -9.8)

# Load plane and robot
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # for default URDFs
p.loadURDF("plane.urdf")
robot = p.loadURDF("diff_drive.urdf", [0,0,0.1])

# Run simulation
for i in range(1000000):
    p.stepSimulation()
    time.sleep(1/240)  # Slow it down to real-time

# Keep window open
input("Press Enter to exit...")

# Disconnect when done
p.disconnect()
