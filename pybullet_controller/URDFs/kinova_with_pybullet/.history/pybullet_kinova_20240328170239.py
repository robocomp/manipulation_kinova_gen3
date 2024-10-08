#!/bin/python3

import pybullet as p
import pybullet_data
import time
import numpy as  np

# Start PyBullet in GUI mode
physicsClient = p.connect(p.GUI)

# Set the path to PyBullet data
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setGravity(0, 0, -9.81)

# Load your object
# Replace 'path/to/your/object.urdf' with the path to your URDF file
flags = p.URDF_USE_INERTIA_FROM_FILE

robot_id = p.loadURDF("/home/alfith/Escritorio/kinova_with_pybullet/gen3_robotiq_2f_140.urdf", basePosition=[0, 0, 0], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)


# Simulation parameters
simulationTime = 5  # in seconds
timeStep = 1./240.  # time step for the simulation

# Define a steady forward speed
forwardSpeed = 0.05  # Adjust this value as needed

#info = p.getNumJoints(robot_id)
#print(info)
#for j in [2,6]:
#    info = p.getJointInfo(robot_id, j)
#    print(info)
targetVelocities = [0.5, 0.5, 0.5, 0.5]
jointIndices = [3, 5, 7, 9]
# Run the simulation indefinitely
while True:
    #p.setJointMotorControlArray(bodyUniqueId=robot_id, jointIndices=jointIndices,controlMode=p.VELOCITY_CONTROL,targetVelocities=targetVelocities)
    #p.stepSimulation()
    time.sleep(timeStep)