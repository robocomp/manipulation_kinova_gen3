import time

import numpy as np
from numpy.ma.core import angle
from sympy import preorder_traversal
import toolbox_controller

import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)
flags = p.URDF_USE_INERTIA_FROM_FILE
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35,
                                         cameraTargetPosition=[0, 0, 0.5])

plane = p.loadURDF("./URDFs/plane/plane.urdf")

robot_urdf = "./URDFs/kinova_with_pybullet/gen3_robotiq_2f_85-mod.urdf"
robot_launch_pos = [-0.3, 0.3, 0.0]
robot_launch_orien = p.getQuaternionFromEuler([0, 0, 0])

robot_id = p.loadURDF(robot_urdf, robot_launch_pos, robot_launch_orien,
                                       flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
home_angles = [0, -0.34, np.pi, -2.54, 0, -0.87, np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

for i in range(len(home_angles)):
    p.resetJointState(bodyUniqueId=robot_id, jointIndex=i + 1,
                      targetValue=home_angles[i], targetVelocity=0)

toolbox_controller = toolbox_controller.ToolboxController()

target_position = [0.5, 0.0, 0.2]

time.sleep(5)

arrived = False

while not arrived:
    joints_angle = []
    for i in range(7):
        joints_angle.append(p.getJointState(robot_id, i + 1)[0])

    target_velocities, arrived  = toolbox_controller.get_joint_velocities(target_position, joints_angle)

    for i in range(len(target_velocities)):
        p.setJointMotorControl2(robot_id, i + 1, p.VELOCITY_CONTROL,
                                targetVelocity=target_velocities[i])

while True:
    print("Arrived")
    for i in range(7):
        angle = p.getJointState(robot_id, i + 1)[0]
        p.setJointMotorControl2(robot_id, i + 1, p.POSITION_CONTROL,
                                targetPosition=angle)
    time.sleep(1)