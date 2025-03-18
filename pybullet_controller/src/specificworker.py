#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2024 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
import math

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
# spatialgeometry is a utility package for dealing with geometric objects
import spatialgeometry as sg
# typing utilities
from typing import Tuple

from orca.braille import setupKeyRanges
from pyAAMED import pyAAMED
import YoloDetector

from pyquaternion import Quaternion

import numpy
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import pybullet as p
import pybullet_data
import time
import numpy as np
from Kinova import KinovaGen3
import calibrator
import os
import cv2
import threading
import yaml
import collections
from pybullet_utils import urdfEditor as ed
from pybullet_utils import bullet_client as bc
import matplotlib.pyplot as plt


# from matplotlib.animation import FuncAnimation

# Set the LC_NUMERIC environment variable
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'
# RMEMEMBER to SET: export LC_NUMERIC="en_US.UTF-8" in your terminal

console = Console(highlight=False)


class JointData:
    def __init__(self):
        self.kinovaAngles = []
        self.pybulletAngles = []

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50
        self.rgb = []
        self.timestamp = int(time.time()*1000)
        if startup_check:
            self.startup_check()
        else:

            # Start PyBullet in GUI mode
            self.physicsClient = p.connect(p.GUI)
            # Set the path to PyBullet data
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

            p.setGravity(0, 0, -9.81)

            # Set the real time simulation
            p.setRealTimeSimulation(1)

            flags = p.URDF_USE_INERTIA_FROM_FILE
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35,
                                         cameraTargetPosition=[0, 0, 0.5])

            # Load floor in the simulation
            self.plane = p.loadURDF("./URDFs/plane/plane.urdf")

            # Load a cube to place the cups on
            self.cube = p.loadURDF("./URDFs/cube_and_square/cube_box.urdf", basePosition=[0.455, 0.0, 0.24],
                                   baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
            # self.cube = p.loadURDF("./URDFs/cube_and_square/cube_box_2.urdf", basePosition=[0.6, 0.0, 0.4],
            #                        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)

            self.cube_center = [0.455, 0.0, 0.82]

            # Load the arm in the simulation
            self.robot_urdf = "./URDFs/kinova_with_pybullet/gen3_robotiq_2f_85-mod.urdf"
            self.robot_launch_pos_pedro = [0, -0.01, 1.05]
            self.robot_launch_orien_pedro = p.getQuaternionFromEuler([np.pi/2, 0, 0])
            self.end_effector_link_index = 12
            self.robot_Pedro = p.loadURDF(self.robot_urdf, self.robot_launch_pos_pedro, self.robot_launch_orien_pedro,
                                          flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

            self.robot_launch_pos_pablo = [0, 0.01, 1.05]
            self.robot_launch_orien_pablo = p.getQuaternionFromEuler([-np.pi/2, 0, 0])
            self.robot_Pablo = p.loadURDF(self.robot_urdf, self.robot_launch_pos_pablo, self.robot_launch_orien_pablo,
                                          flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

            # Angles for the home position of the robot
            self.home_angles = [0, -0.34, np.pi, -2.54, 0, -0.87, np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            # Angles for the rest position of the robot

            self.rest_angles_pedro = [1.13, 4.71 - 2*np.pi, np.pi / 2, 3.83 - 2*np.pi, 0, 5.41 - 2*np.pi, np.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.rest_angles_pablo = [2.00, np.pi / 2, np.pi / 2 , 3.83 - 2*np.pi, 0, 5.41 - 2*np.pi, np.pi / 2, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


            # Angles for the observation position of the robot
            self.observation_angles_pedro = [1.5184, 4.8695 - 2*np.pi, 1.4486, 4.7124 - 2*np.pi, 6.1785 - 2*np.pi, 4.8695 - 2*np.pi, np.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.observation_angles_pablo = [1.6232, 1.4137, 1.693, 4.7124 - 2*np.pi, 0.1047, 4.8695 - 2*np.pi, np.pi / 2, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


            # Set the initial joint angles of the pybullet arm
            for i in range(7):
                p.resetJointState(bodyUniqueId=self.robot_Pedro, jointIndex=i + 1,
                                  targetValue=self.rest_angles_pedro[i], targetVelocity=0)
                p.resetJointState(bodyUniqueId=self.robot_Pablo, jointIndex=i + 1,
                                  targetValue=self.rest_angles_pablo[i], targetVelocity=0)

            # Load a cylinder to place on the table
            self.cylinder_pablo = p.loadURDF("./URDFs/cylinder/cylinder.urdf", [0.455, 0.1, 0.55], p.getQuaternionFromEuler([0, 0, 0]))
            # self.cylinder_pablo = p.loadURDF("./URDFs/cylinder/cylinder.urdf", [0.455, 0.1, 0.90], p.getQuaternionFromEuler([0, 0, 0]))

            # self.cylinder_pedro = p.loadURDF("./URDFs/cylinder/cylinder.urdf", [0.455, -0.1, 0.5], p.getQuaternionFromEuler([0, 0, 0]))
            self.arms_base = p.loadURDF("./URDFs/arms_base/arms_base.urdf", [0, 0.0, 0.0], p.getQuaternionFromEuler([0, 0, 0]))

            self.pos_obj = [0.455, 0.1, 0.225]

            # Thread to read the real arm angles from kinova_controller
            self.threadKinovaAngles = threading.Thread(target=self.readDataFromProxy)
            self.threadKinovaAngles.start()

            # Queues to store the images from the real arm camera
            self.colorKinova = collections.deque(maxlen=5)
            self.depthKinova = collections.deque(maxlen=5)
            self.colorKinova2 = collections.deque(maxlen=5)
            self.depthKinova2 = collections.deque(maxlen=5)

            # Wait for half a second
            time.sleep(0.5)

            # Change the mass of the links of the robot to do the simulation more stable
            for i in range(7):
                if i != 0:
                    val = p.getDynamicsInfo(self.robot_Pedro, i)[2]
                    nuevo_val = np.array(val) * 1000
                    p.changeDynamics(self.robot_Pedro, i, localInertiaDiagonal=nuevo_val)
                else:
                    val = p.getDynamicsInfo(self.robot_Pedro, i)[2]
                    nuevo_val = np.array(val) * 1000
                    p.changeDynamics(self.robot_Pedro, i, localInertiaDiagonal=nuevo_val)


            # This variables is to store the position of the end effector of the robot
            self.target_angles = self.home_angles
            self.target_position = p.getLinkState(self.robot_Pedro, self.end_effector_link_index)[0]
            self.target_orientation = p.getLinkState(self.robot_Pedro, self.end_effector_link_index)[1]
            self.target_velocities = [0.0] * 7

            # This variable is to store the joint selected in the joystick mode
            self.joy_selected_joint = 0

            # This variable is to store the mode of the robot
            self.move_mode = -101#4

            # This variable is to store the state of the real arm
            # self.ext_joints = self.kinovaarm_proxy.getJointsState() ############################################################################# DESCOMENTAR
            # self.ext_gripper = self.kinovaarm_proxy.getGripperState()

            # Timer to do the control loop
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

            # This variables are needed for move real arm
            self.joint_speeds = []
            for i in range(7):
                self.joint_speeds.append(0)

            self.speeds = ifaces.RoboCompKinovaArm.TJointSpeeds()
            self.speeds.jointSpeeds = self.joint_speeds

            self.angles = ifaces.RoboCompKinovaArm.TJointAngles()
            self.angles.jointAngles = []

            # This variables are needed for the gains of real arm speeds
            self.gains = np.ones(7).tolist()

            self.posesTimes = np.array([int(time.time()*1000)])
            self.poses = []
            joints = []
            for i in range(7):
                actual_angle = (i, p.getJointState(self.robot_Pedro, i + 1)[0])
                joints.append(actual_angle)
                self.poses.append(joints)

            # This variable was to test the calibration of the camera
            self.calibrator = calibrator.Calibrator()

            # Timer to read the camera of the real arm
            self.cameraKinovaTimer = QtCore.QTimer(self)
            self.cameraKinovaTimer.timeout.connect(self.readKinovaCamera)
            # self.cameraKinovaTimer.start(self.Period) ############################################################################################################### DESCOMENTAR

            # Timers to update the gains and show the real arm angles
            self.showKinovaStateTimer = QtCore.QTimer(self)
            self.showKinovaStateTimer.timeout.connect(self.showKinovaAngles)
            # self.showKinovaStateTimer.start(1000)

            self.gainsTimer = QtCore.QTimer(self)
            self.gainsTimer.timeout.connect(self.updateGains)

            # Initialize the AAMED algorithm for the cup position correction
            self.aamed = pyAAMED(722//2, 1282//2)
            self.aamed.setParameters(3.1415926 / 3, 3.4, 0.77)

            # self.contactPointTimer = QtCore.QTimer(self)
            # self.contactPointTimer.timeout.connect(self.detectContactPoints)
            # self.contactPointTimer.start(100)

            self.drawGraphTimer = QtCore.QTimer(self)
            self.drawGraphTimer.timeout.connect(self.drawGraph)
            # self.drawGraphTimer.start(1000)

            self.pybullet_offset = [-0.3, 0.0, 0.6]
            self.last_error = 0

            num_joints = p.getNumJoints(self.robot_Pedro)

            # Iterar sobre todos los enlaces y obtener su información
            for joint_index in range(num_joints):
                joint_info = p.getJointInfo(self.robot_Pedro, joint_index)
                link_name = joint_info[12].decode("utf-8")  # Nombre del enlace
                parent_link_index = joint_info[16]  # Índice del enlace padre
                link_state = p.getLinkState(self.robot_Pedro, joint_index)

                # print(f"Link {joint_index}:")
                # print(f"  Name: {link_name}")
                # print(f"  Parent Link Index: {parent_link_index}")
                # print(f"  Link State: {link_state}")

            plt.ion()
            self.fig, self.ax = plt.subplots(1, 1)
            plt.show()

            # self.jointsErrorMap = {} ############################################################################################################ DESCOMENTAR
            # for i in range(7):
            #     jointError = [
            #         self.ext_joints.joints[i].angle - math.degrees(p.getJointState(self.robot_id, i + 1)[0]) % 360]
            #     item = {i: jointError}
            #     self.jointsErrorMap.update(item)

            self.left_force_series = [0]
            self.right_force_series = [0]
            self.graphTimes = [0]

            self.ax.set_ylim(0, 10)

            plt.tight_layout()
            plt.draw()

            self.hide()

            print("SpecificWorker started", time.time()*1000 - self.timestamp)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):

        match self.move_mode:
            case -99:
                for i in range(len(self.observation_angles_pablo)):
                    p.setJointMotorControl2(self.robot_Pedro, i + 1, p.POSITION_CONTROL,
                                            self.observation_angles_pedro[i], maxVelocity=np.deg2rad(25))
                    p.setJointMotorControl2(self.robot_Pablo, i + 1, p.POSITION_CONTROL,
                                            self.observation_angles_pablo[i], maxVelocity=np.deg2rad(25))

                self.moveRealKinovaWithAngles(self.observation_angles_pedro[:7], self.robot_Pedro)
                self.moveRealKinovaWithAngles(self.observation_angles_pablo[:7], self.robot_Pablo)
                self.move_mode = -100
            case -100:
                self.movePybulletArmWithAngles(self.getPabloJointsAngle(), self.robot_Pablo)
                self.movePybulletArmWithVelocities(0.0 * np.ones(7), self.robot_Pablo)
                self.read_camera_fixed(self.robot_Pablo)
                self.read_camera_fixed(self.robot_Pedro)

            case -101:
                speed = np.zeros(7).tolist()
                speed[6] = 0.1

                self.movePybulletArmWithVelocities(speed, self.robot_Pedro)
                self.moveRealKinovaWithSpeeds(self.robot_Pedro)
                pass

            #Move joints
            case 0:
                jointsState = []
                for i in range(len(self.target_angles)):
                    p.setJointMotorControl2(self.robot_Pedro, i + 1, p.POSITION_CONTROL,
                                            self.target_angles[i], maxVelocity=np.deg2rad(25))
                    if i < 7:
                        angle_speed = (p.getJointState(self.robot_Pedro, i + 1)[0],
                                       p.getJointState(self.robot_Pedro, i + 1)[1])
                        print("Joint ", i, "angle: ", angle_speed)
                        jointsState.append(angle_speed)

                self.posesTimes = np.append(self.posesTimes, int(time.time()*1000))
                self.poses.append(jointsState)

                # update the gains every 1000 ms to avoid oscillations
                if self.timestamp+500 < int(time.time()*1000):
                    self.timestamp = int(time.time()*1000)
                    self.updateGains()

            #Cartesian movement
            case 1:
                joint_positions = p.calculateInverseKinematics(self.robot_Pedro, self.end_effector_link_index,
                                                               self.target_position, self.target_orientation)
                #print(joint_positions)
                joints = np.zeros(22).tolist()
                for i in range(len(joint_positions)):
                    joints[i] = joint_positions[i]
                    self.target_angles[i] = joint_positions[i]

                jointsState = []
                for i in range(len(joints)):
                    p.setJointMotorControl2(self.robot_Pedro, i + 1, p.POSITION_CONTROL, self.target_angles[i],
                                            maxVelocity=np.deg2rad(25))
                    if i < 7:
                        angle_speed = (p.getJointState(self.robot_Pedro, i + 1)[0],
                                       p.getJointState(self.robot_Pedro, i + 1)[1])
                        jointsState.append(angle_speed)

                self.posesTimes = np.append(self.posesTimes, int(time.time()*1000))
                self.poses.append(jointsState)

                # update the gains every 1000 ms to avoid oscillations
                if self.timestamp+500 < int(time.time()*1000):
                    self.timestamp = int(time.time()*1000)
                    self.updateGains()

            case 2:
                joints = np.zeros(22).tolist()
                for i in range(len(joint_positions)):
                    joints[i] = joint_positions[i]

                for i in range(len(joints)):
                    p.setJointMotorControl2(self.robot_Pedro, i + 1, p.POSITION_CONTROL, joints[i], maxVelocity=np.deg2rad(25))


            case 3:
                try:
                    actual_joints = np.rad2deg(self.target_angles[:7])

                    unfixed_target_angles, joints_velocity, joints_torque = ([0.0] * len(self.ext_joints.joints),
                                                                             [0.0] * len(self.ext_joints.joints),
                                                                             [0.0] * len(self.ext_joints.joints))
                    for i in range(len(self.ext_joints.joints)):
                        unfixed_target_angles[i] = self.ext_joints.joints[i].angle
                        joints_velocity[i] = self.ext_joints.joints[i].velocity
                        joints_torque[i] = self.ext_joints.joints[i].force

                        if abs(actual_joints[i] % 360) >= 300 and unfixed_target_angles[i] <= 60:
                            print("Actual_angle", (actual_joints[i] % 360), "Target angle: ", unfixed_target_angles[i])
                            self.n_rotations[i] += 1
                        if abs(actual_joints[i] % 360) <= 60 and unfixed_target_angles[i] >= 300:
                            print("Actual_angle", (actual_joints[i] % 360), "Target angle: ", unfixed_target_angles[i])
                            self.n_rotations[i] -= 1

                    self.target_angles[:7] = np.deg2rad(unfixed_target_angles)

                    for i in range(7):
                        self.target_angles[i] = self.target_angles[i] + self.n_rotations[i] * 2*np.pi

                    self.target_angles[8:] = [0.0] * len(self.target_angles[8:])

                    self.target_angles[13] = self.ext_gripper.distance
                    self.target_angles[15] = - self.ext_gripper.distance
                    self.target_angles[17] = self.ext_gripper.distance - 0.1

                    self.target_angles[18] = self.ext_gripper.distance
                    self.target_angles[20] = - self.ext_gripper.distance
                    self.target_angles[22] = self.ext_gripper.distance - 0.1

                    self.target_velocities = joints_velocity

                    for i in range(8, len(self.target_angles)):
                        p.setJointMotorControl2(self.robot_Pedro, i + 1, p.POSITION_CONTROL, targetPosition=self.target_angles[i]) #Move the arm with phisics


                except Ice.Exception as e:
                    print(e)

            case 4:    #Move to observation angles
                print("Moving to observation angles", int(time.time()*1000) - self.timestamp)
                self.changePybulletGripper(0.0, self.robot_Pedro)
                self.changePybulletGripper(0.0, self.robot_Pablo)
                self.kinovaarm_proxy.openGripper()  # Open the pablo gripper
                self.kinovaarm1_proxy.openGripper() # Open the pedro gripper

                self.moveRealKinovaWithAngles(self.observation_angles_pablo[:7], self.robot_Pablo)
                self.moveRealKinovaWithAngles(self.observation_angles_pedro[:7], self.robot_Pedro)

                for i in range(len(self.observation_angles_pedro)):
                    p.setJointMotorControl2(self.robot_Pedro, i + 1, p.POSITION_CONTROL,
                                            targetPosition=self.observation_angles_pedro[i], maxVelocity=np.deg2rad(25))
                    p.setJointMotorControl2(self.robot_Pablo, i + 1, p.POSITION_CONTROL,
                                            targetPosition=self.observation_angles_pablo[i], maxVelocity=np.deg2rad(25))

                angles_pedro, angles_pablo = [], []
                for i in range(7):
                    angles_pedro.append(p.getJointState(self.robot_Pedro, i + 1)[0])
                    angles_pablo.append(p.getJointState(self.robot_Pablo, i + 1)[0])

                error_pedro = np.sum(np.abs(np.array(angles_pedro) - np.array(self.observation_angles_pedro[:7])))
                error_pablo = np.sum(np.abs(np.array(angles_pablo) - np.array(self.observation_angles_pablo[:7])))

                pybullet_image_pedro, image_time = self.read_camera_fixed(self.robot_Pedro)
                pybullet_image_pablo, image_time = self.read_camera_fixed(self.robot_Pablo)

                if error_pedro < 0.05 and error_pablo < 0.05:
                    print("Observation angles reached", int(time.time()*1000) - self.timestamp)
                    self.move_mode = 5
                    # self.move_mode = -100
            case 5:    #Detect real cup and correct pybullet cup position
                error, kinova_target = self.correctTargetPosition()

                if error > 5 or kinova_target is False:
                    print("Correcting cup position", int(time.time()*1000) - self.timestamp)

                else:
                    print("Calibration finished", int(time.time()*1000 - self.timestamp))
                    self.move_mode = 6
                    # self.move_mode = -100

            case 6:    #Initialize toolbox
                self.timer.stop()

                target_position = list(p.getBasePositionAndOrientation(self.cylinder_pablo)[0])
                target_position[2] = target_position[2] + 0.17

                print("Resetting the initial state of toolbox", time.time()*1000 - self.timestamp)
                # self.initialize_toolbox(target_position)
                self.roboticstoolboxcontroller_proxy.setStatePablo(self.getPabloJointsAngle())
                self.roboticstoolboxcontroller_proxy.setStatePedro(self.getPedroJointsAngle())
                print("toolbox initialized", time.time()*1000 - self.timestamp)
                # self.showKinovaStateTimer.start(1000)
                # self.gainsTimer.start(1000)

                print("Moving to fixed cup")
                self.move_mode = 7
                # self.move_mode = -100
                print("Changing actual move mode to 7")
                self.timer.start(self.Period)

            case 7:    #Move to cup with camera feedback
                try:
                    # print("#====================================#", time.time()*1000 - self.timestamp)
                    error, kinova_target = self.correctTargetPosition()
                    if error != -1:
                        self.last_error = error

                    # print("Getting cup position", time.time()*1000 - self.timestamp)
                    target_position = list(p.getBasePositionAndOrientation(self.cylinder_pablo)[0])
                    # target_position[0] = target_position[0] - self.pybullet_offset[0]
                    # target_position[2] = target_position[2] - self.pybullet_offset[2] + 0.27
                    target_position[2] = target_position[2] + 0.37#0.42


                    # print("Toolbox request", time.time() * 1000 - self.timestamp)
                    ### Calculate the velocities with the toolbox for Pablo Arm
                    new_state_pablo = self.roboticstoolboxcontroller_proxy.calculateVelocitiesPablo(self.getPabloJointsAngle(), target_position)
                    # print("Toolbox response", time.time() * 1000 - self.timestamp)

                    self.movePybulletArmWithVelocities(new_state_pablo.jointVelocities, self.robot_Pablo)
                    # print("Pybullet Pablo arm moved", time.time() * 1000 - self.timestamp)

                    self.moveRealKinovaWithSpeeds(self.robot_Pablo)
                    # print("Kinova Pablo arm moved", time.time() * 1000 - self.timestamp)

                    # self.posesTimes = np.append(self.posesTimes, int(time.time() * 1000))
                    #
                    # jointsState = []

                    # for i in range(7):
                    #     state = p.getJointState(self.robot_Pedro, i + 1)
                    #     angle_speed = (state[0], state[1])
                    #     jointsState.append(angle_speed)
                    #
                    #     self.jointsErrorMap[i].append(abs(self.ext_joints.joints[i].angle - math.degrees(state[0]) % 360))
                    #
                    # self.graphTimes.append(int(time.time()*1000 - self.timestamp))

                    # self.poses.append(jointsState)

                    # print("Pybullet poses saved to update gains", time.time()*1000 - self.timestamp)
                    # print("//====================================//")

                    # if self.last_error > 80 and kinova_target is False:
                    #     print("Adjusting pose for target adjustment")
                    #     self.target_position = list(p.getLinkState(self.robot_Pablo, self.end_effector_link_index)[0])
                    #     self.target_position[2] = self.target_position[2] + 0.2
                    #     self.move_mode = 8
                    #     print("Changing actual move mode to 8")

                    if new_state_pablo.arrived:
                        print("Working without visual feedback", " Last error: ", self.last_error)
                        # if self.arrived:
                        #     self.arrived = False
                        self.move_mode = 9
                        # self.move_mode = -100
                        print("Changing actual move mode to 9")
                except Ice.Exception as e:
                    print(e)

            case 8:    #Move to a top position to adjust the cup position
                try:
                    self.arrived = False

                    self.correctTargetPosition()
                    self.toolbox_compute(self.target_position)
                    #TODO: Change the move method
                    self.movePybulletWithToolbox()
                    self.moveKinovaWithSpeeds()

                    self.posesTimes = np.append(self.posesTimes, int(time.time() * 1000))

                    jointsState = []
                    for i in range(7):
                        state = p.getJointState(self.robot_Pedro, i + 1)
                        angle_speed = (state[0], state[1])
                        jointsState.append(angle_speed)
                        self.jointsErrorMap[i].append(abs(self.ext_joints.joints[i].angle - math.degrees(state[0]) % 360))

                    self.graphTimes.append(int(time.time() * 1000 - self.timestamp))

                    self.poses.append(jointsState)

                    print("Arrived: ", self.arrived)

                    if self.arrived == True:
                        self.loopCounter = 0
                        self.target_velocities = [0.0] * 7
                        #TODO: Change the move method
                        self.movePybulletWithToolbox()
                        self.moveKinovaWithSpeeds()
                        self.last_error = 0
                        self.move_mode = 7
                        print("Changing actual move mode to 7")

                except Ice.Exception as e:
                    print(e)

            case 9:    #Move to cup without camera feedback and close the gripper
                try:
                    target_position = list(p.getBasePositionAndOrientation(self.cylinder_pablo)[0])
                    target_position[0] = target_position[0] - 0.005
                    target_position[1] = target_position[1] + 0.01
                    target_position[2] = target_position[2] + 0.22#0.17

                    # print("Toolbox request", time.time() * 1000 - self.timestamp)
                    new_state_pablo = self.roboticstoolboxcontroller_proxy.calculateVelocitiesPablo(
                        self.getPabloJointsAngle(), target_position)

                    # print("<=============================>")
                    # for i in range(7):
                    #     print(np.rad2deg(p.getJointState(self.robot_Pablo, i + 1)[0]), self.ext_joints.joints[i].angle)

                    if not new_state_pablo.arrived:

                        self.movePybulletArmWithVelocities(new_state_pablo.jointVelocities, self.robot_Pablo)
                        self.moveRealKinovaWithSpeeds(self.robot_Pablo)

                        imgPybullet, imageTime = self.read_camera_fixed(self.robot_Pablo)

                    # self.posesTimes = np.append(self.posesTimes, int(time.time() * 1000))
                    #
                    # jointsState = []
                    # for i in range(7):
                    #     state = p.getJointState(self.robot_Pedro, i + 1)
                    #     angle_speed = (state[0], state[1])
                    #     jointsState.append(angle_speed)
                    #     self.jointsErrorMap[i].append(abs(self.ext_joints.joints[i].angle - math.degrees(state[0]) % 360))
                    #
                    # self.graphTimes.append(int(time.time() * 1000 - self.timestamp))
                    #
                    # self.poses.append(jointsState)

                    # error = p.getLinkState(self.robot_Pablo, self.end_effector_link_index)[0] - target_position

                    # print("Error: ", error, p.getLinkState(self.robot_Pablo, self.end_effector_link_index)[0], target_position)

                    if new_state_pablo.arrived:
                        # self.move_mode = -100

                        print("Arrived")
                        self.timer.stop()
                        self.target_velocities = [0.0] * 7
                        self.movePybulletArmWithVelocities(self.target_velocities, self.robot_Pablo)
                        self.movePybulletArmWithAngles(self.getPabloJointsAngle(), self.robot_Pablo)
                        self.moveRealKinovaWithSpeeds(self.robot_Pablo)
                        cup_grapped = self.kinovaarm_proxy.closeGripper()
                        time.sleep(0.5)
                        print("Cup grapped:", cup_grapped)
                        if cup_grapped:
                            self.changePybulletGripper(self.ext_gripper.distance, self.robot_Pablo)
                            time.sleep(1)
                            self.move_mode = 10
                            # self.move_mode = -100
                            print("Changing actual move mode to 10")
                            self.target_position = list(p.getBasePositionAndOrientation(self.cylinder_pablo)[0])
                            self.target_position[0] = self.target_position[0]
                            self.target_position[2] = self.target_position[2] + 0.42
                            self.timer.start(self.Period)
                        else:
                            self.kinovaarm_proxy.openGripper()
                            self.target_position = list(p.getLinkState(self.robot_Pedro, self.end_effector_link_index)[0])
                            self.target_position[0] = self.target_position[0]
                            self.target_position[2] = self.target_position[2] + 0.60
                            self.move_mode = 7#8
                            print("Error at grappling, please replace the cup. Changing actual move mode to 8")
                            self.timer.start(self.Period)

                except Ice.Exception as e:
                    print(e)

            case 10:    #Move to a top position before moving to the center of the table
                try:
                    new_state_pablo = self.roboticstoolboxcontroller_proxy.calculateVelocitiesPablo(
                        self.getPabloJointsAngle(), self.target_position)

                    self.movePybulletArmWithVelocities(new_state_pablo.jointVelocities, self.robot_Pablo)
                    self.moveRealKinovaWithSpeeds(self.robot_Pablo)

                    imgPybullet, imageTime = self.read_camera_fixed(self.robot_Pablo)

                    # self.posesTimes = np.append(self.posesTimes, int(time.time() * 1000))
                    #
                    # jointsState = []
                    # for i in range(7):
                    #     state = p.getJointState(self.robot_Pedro, i + 1)
                    #     angle_speed = (state[0], state[1])
                    #     jointsState.append(angle_speed)
                    #     self.jointsErrorMap[i].append(abs(self.ext_joints.joints[i].angle - math.degrees(state[0]) % 360))
                    #
                    # self.graphTimes.append(int(time.time() * 1000 - self.timestamp))
                    #
                    # self.poses.append(jointsState)
                    for i in range(7):
                        print(p.getJointState(self.robot_Pablo, i + 1)[0], self.ext_joints.joints[i].angle)

                    if new_state_pablo.arrived:
                        # print("Arrived")
                        self.target_velocities = [0.0] * 7
                        self.movePybulletArmWithVelocities(self.target_velocities, self.robot_Pablo)
                        self.movePybulletArmWithAngles(self.getPabloJointsAngle(), self.robot_Pablo)
                        self.moveRealKinovaWithSpeeds(self.robot_Pablo)
                        self.move_mode = 11
                        # self.move_mode = -100
                        print("Changing actual move mode to 11")

                except Ice.Exception as e:
                    print(e)

            case 11:    #Move to the center of the table and open the gripper
                try:
                    new_state_pablo = self.roboticstoolboxcontroller_proxy.calculateVelocitiesPablo(
                        self.getPabloJointsAngle(), self.cube_center)

                    self.movePybulletArmWithVelocities(new_state_pablo.jointVelocities, self.robot_Pablo)
                    self.moveRealKinovaWithSpeeds(self.robot_Pablo)

                    imgPybullet, imageTime = self.read_camera_fixed(self.robot_Pablo)

                    # self.posesTimes = np.append(self.posesTimes, int(time.time() * 1000))
                    #
                    # jointsState = []
                    # for i in range(7):
                    #     state = p.getJointState(self.robot_Pedro, i + 1)
                    #     angle_speed = (state[0], state[1])
                    #     jointsState.append(angle_speed)
                    #     self.jointsErrorMap[i].append(abs(self.ext_joints.joints[i].angle - math.degrees(state[0]) % 360))
                    #
                    # self.graphTimes.append(int(time.time() * 1000 - self.timestamp))
                    #
                    # self.poses.append(jointsState)

                    if new_state_pablo.arrived:
                        self.timer.stop()
                        self.target_velocities = [0.0] * 7
                        self.movePybulletArmWithVelocities(self.target_velocities, self.robot_Pablo)
                        self.movePybulletArmWithAngles(self.getPabloJointsAngle(), self.robot_Pablo)
                        self.moveRealKinovaWithSpeeds(self.robot_Pablo)
                        self.kinovaarm_proxy.openGripper()
                        time.sleep(0.5)
                        self.changePybulletGripper(self.ext_gripper.distance, self.robot_Pablo)
                        # self.move_mode = 12
                        self.move_mode = -100
                        print("Changing actual move mode to 12")
                        self.timer.start(self.Period)

                except Ice.Exception as e:
                    print(e)

            case 12:    #Move to the observation angles and repeat the process(modes 7-12)
                print("Reseting pose")
                self.timer.stop()
                self.moveRealKinovaWithAngles(self.observation_angles_pablo[:7], self.robot_Pablo)
                for i in range(7):
                    p.setJointMotorControl2(self.robot_Pablo, i + 1, p.POSITION_CONTROL,
                                            targetPosition=self.observation_angles_pablo[i],
                                            maxVelocity=np.deg2rad(25))

                angles = []
                for i in range(7):
                    angles.append(p.getJointState(self.robot_Pablo, i + 1)[0])

                error = np.sum(np.abs(np.array(angles) - np.array(self.observation_angles_pablo[:7])))

                time.sleep(0.1)
                self.timer.start(self.Period)

                if error < 0.05:
                    p.resetBasePositionAndOrientation(self.cylinder_pablo, [0.074, 0.2, 0.70],
                                                      p.getQuaternionFromEuler([0, 0, 0]))
                    self.move_mode = 7
                    # self.move_mode = -100
                    print("Changing actual move mode to 7")

            case 99:
                self.timer.stop()
                cv2.imshow("Kinova camera", self.colorKinova[0][0])
                self.read_camera_fixed()
                self.calibrator.square_test(self.robot_Pedro, self.colorKinova[0][0].copy())
                cv2.waitKey(3)


    # =============== Methods ==================

    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.Point3D from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.Point3D()
        print(f"Testing RoboCompCameraRGBDSimple.TPoints from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TPoints()
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        print(f"Testing RoboCompCameraRGBDSimple.Point3D from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.Point3D()
        print(f"Testing RoboCompCameraRGBDSimple.TPoints from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TPoints()
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TGripper from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TGripper()
        print(f"Testing RoboCompKinovaArm.TJoint from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoint()
        print(f"Testing RoboCompKinovaArm.TJoints from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoints()
        print(f"Testing RoboCompKinovaArm.TJointSpeeds from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJointSpeeds()
        print(f"Testing RoboCompKinovaArm.TJointAngles from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJointAngles()
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TGripper from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TGripper()
        print(f"Testing RoboCompKinovaArm.TJoint from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoint()
        print(f"Testing RoboCompKinovaArm.TJoints from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoints()
        print(f"Testing RoboCompKinovaArm.TJointSpeeds from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJointSpeeds()
        print(f"Testing RoboCompKinovaArm.TJointAngles from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJointAngles()
        print(f"Testing RoboCompRoboticsToolboxController.JointStates from ifaces.RoboCompRoboticsToolboxController")
        test = ifaces.RoboCompRoboticsToolboxController.JointStates()
        print(f"Testing RoboCompJoystickAdapter.AxisParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.AxisParams()
        print(f"Testing RoboCompJoystickAdapter.ButtonParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.ButtonParams()
        print(f"Testing RoboCompJoystickAdapter.TData from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.TData()
        QTimer.singleShot(200, QApplication.instance().quit)

    def changePybulletGripper(self, distance, robot_id=None):
        """
        changePybulletGripper changes the distance of the gripper in pybullet simulation

        :param distance: distance to change the gripper
        :param robot_id: robot id to change the gripper
        """
        self.target_angles[13] = distance
        self.target_angles[15] = - distance
        self.target_angles[17] = distance - 0.1

        self.target_angles[18] = distance
        self.target_angles[20] = - distance
        self.target_angles[22] = distance - 0.1

        for i in range(8, len(self.target_angles)):
            p.setJointMotorControl2(robot_id, i + 1, p.POSITION_CONTROL,
                                    targetPosition=self.target_angles[i])

    def detectContactPoints(self):
        print("adding contact points", int(time.time()*1000) - self.timestamp)
        self.graphTimes.append(int(time.time()*1000)-self.timestamp)
        # Get contact points for the left fingertip
        contact_points_left = []
        contact_points_left.extend(p.getContactPoints(bodyA=self.robot_Pedro, linkIndexA=17))
        # self.left_force_series.append(len(contact_points_left))

        force_acum = 0
        for contact in contact_points_left:  # Contact force is at index 9 of the contact point tuple
            contact_force_left = contact[9]
            force_acum += contact_force_left
            # print(f"Contact force on left fingertip: {contact_force_left}")
        self.left_force_series.append(force_acum)

        # Get contact points for the right fingertip
        contact_points_right = []
        contact_points_right.extend(p.getContactPoints(bodyA=self.robot_Pedro, linkIndexA=22))
        # self.right_force_series.append(len(contact_points_right))

        force_acum = 0
        for contact in contact_points_right:
            # Contact force is at index 9 of the contact point tuple
            contact_force_right = contact[9]
            force_acum += contact_force_right
            # print(f"Contact force on right fingertip: {contact_force_right}")
        self.right_force_series.append(force_acum)

        # plt.cla()
        # plt.plot(self.graphTimes, self.left_force_series, label="Left fingertip force")
        # plt.plot(self.graphTimes, self.right_force_series, label="Right fingertip force")
        #
        # plt.tight_layout()
        # plt.draw()
        # plt.pause(0.001)

    def drawGraph(self):

        plt.cla()
        # print("Drawing graph", int(time.time()*1000) - self.timestamp)

        self.ax.set_ylim(0, 10)

        self.ax.plot(self.graphTimes, self.jointsErrorMap.get(0), label="joint 1 error")
        self.ax.plot(self.graphTimes, self.jointsErrorMap.get(1), label="joint 2 error")
        self.ax.plot(self.graphTimes, self.jointsErrorMap.get(2), label="joint 3 error")
        self.ax.plot(self.graphTimes, self.jointsErrorMap.get(3), label="joint 4 error")
        self.ax.plot(self.graphTimes, self.jointsErrorMap.get(4), label="joint 5 error")
        self.ax.plot(self.graphTimes, self.jointsErrorMap.get(5), label="joint 6 error")
        self.ax.plot(self.graphTimes, self.jointsErrorMap.get(6), label="joint 7 error")

        # self.axs[0, 0].plot(self.graphTimes, self.jointsErrorMap.get(0), label="joint 1 error")
        # self.axs[1, 0].plot(self.graphTimes, self.jointsErrorMap.get(1), label="joint 2 error")
        # self.axs[2, 0].plot(self.graphTimes, self.jointsErrorMap.get(2), label="joint 3 error")
        # self.axs[3, 0].plot(self.graphTimes, self.jointsErrorMap.get(3), label="joint 4 error")
        # self.axs[0, 1].plot(self.graphTimes, self.jointsErrorMap.get(4), label="joint 5 error")
        # self.axs[1, 1].plot(self.graphTimes, self.jointsErrorMap.get(5), label="joint 6 error")
        # self.axs[2, 1].plot(self.graphTimes, self.jointsErrorMap.get(6), label="joint 7 error")

        # self.axs[3, 1].plot(self.graphTimes, self.right_force_series, label="Right fingertip force")
        # self.axs[3, 1].plot(self.graphTimes, self.left_force_series, label="Left fingertip force")

        plt.tight_layout()
        plt.draw()

        # print("Graph drawn", int(time.time()*1000) - self.timestamp)
        # plt.pause(0.001)

    def correctTargetPosition(self):
        """
        correctTargetPosition corrects the position of the target in pybullet simulation using the keypoints detected

        :return: error between the keypoints detected in pybullet and kinova camera,
        and a boolean indicating if the target was detected in kinova camera
        """
        # print("################## Correcting target position ##################", int(time.time()*1000) - self.timestamp)
        pybulletImage, imageTime = self.read_camera_fixed(self.robot_Pablo)
        # print("Pybullet image generated", int(time.time()*1000) - self.timestamp)
        pybulletImage = cv2.resize(pybulletImage, (1280//2, 720//2))
        imgGPybullet = cv2.cvtColor(pybulletImage, cv2.COLOR_BGR2GRAY)
        # print("Pybullet image resized", int(time.time()*1000) - self.timestamp)
        resPybullet = self.aamed.run_AAMED(imgGPybullet)
        # print("Pybullet keypoints detected", int(time.time()*1000) - self.timestamp)
        if isinstance(resPybullet, list):
            resPybullet = np.array(resPybullet)

        diff = 5000
        index = 0
        for i in range(len(self.colorKinova)):
            if abs(imageTime - self.colorKinova[i][1]) < diff:
                diff = abs(self.colorKinova[i][1] - imageTime)
                index = i

        # print("time difference between two images:", diff)

        imgGKinova = cv2.cvtColor(self.colorKinova[index][0], cv2.COLOR_BGR2GRAY)
        imgGKinova = cv2.resize(imgGKinova, (1280//2, 720//2))
        resKinova = np.array(self.aamed.run_AAMED(imgGKinova))
        if isinstance(resKinova, list):
            resKinova = np.array(resKinova)

        if resKinova.size == 0 or resPybullet.size == 0:
            print("No keypoints detected")
            return -1, False

        error = np.abs(resKinova[0][1] - resPybullet[0][1] + resKinova[0][0] - resPybullet[0][0])

        # print("Error: ", error, "Kinova obj: ", resKinova, "Pybullet obj: ", resPybullet)

        if resKinova[0][5] > 0.85 and resPybullet[0][5] > 0.85:
            position = list(p.getBasePositionAndOrientation(self.cylinder_pablo)[0])
            position[0] = position[0] - 0.0002 * (resKinova[0][0] - resPybullet[0][0])
            position[1] = position[1] - 0.0002 * (resKinova[0][1] - resPybullet[0][1])
            p.resetBasePositionAndOrientation(self.cylinder_pablo, tuple(position), p.getQuaternionFromEuler([0, 0, 0]))

        # print("################## Target position corrected ##################", int(time.time()*1000) - self.timestamp)
        if len(resKinova) > 0:
            kinovaTargetDetected = True
        else:
            kinovaTargetDetected = False

        return error, kinovaTargetDetected


    def getPabloJointsAngle(self):
        """
        getPabloJointsAngle gets the angles of the joints of the Pablo robot in pybullet simulation

        :return: angles of the joints of the Pablo robot in pybullet simulation
        """
        joints = []
        for i in range(7):
            joints.append(p.getJointState(self.robot_Pablo, i + 1)[0])
        return joints

    def getPedroJointsAngle(self):
        """
        getPedroJointsAngle gets the angles of the joints of the Pedro robot in pybullet simulation

        :return: angles of the joints of the Pedro robot in pybullet simulation
        """
        joints = []
        for i in range(7):
            joints.append(p.getJointState(self.robot_Pedro, i + 1)[0])
        return joints

    def cvK2BulletP(self, K, w, h, near, far):
        """
        cvKtoPulletP converst the K interinsic matrix as calibrated using Opencv
        and ROS to the projection matrix used in openGL and Pybullet.

        :param K:  OpenCV 3x3 camera intrinsic matrix
        :param w:  Image width
        :param h:  Image height
        :near:     The nearest objects to be included in the render
        :far:      The furthest objects to be included in the render
        :return:   4x4 projection matrix as used in openGL and pybullet
        """
        f_x = K[0, 0]
        f_y = K[1, 1]
        c_x = K[0, 2]
        c_y = K[1, 2]
        A = (near + far) / (near - far)
        B = 2 * near * far / (near - far)

        projection_matrix = [
            [2 / w * f_x, 0, (w - 2 * c_x) / w, 0],
            [0, 2 / h * f_y, (2 * c_y - h) / h, 0],
            [0, 0, A, B],
            [0, 0, -1, 0]]
        # The transpose is needed for respecting the array structure of the OpenGL
        return np.array(projection_matrix).T.reshape(16).tolist()

    def cvPose2BulletView(self, q, t):
        """
        cvPose2BulletView gets orientation and position as used
        in ROS-TF and opencv and coverts it to the view matrix used
        in openGL and pyBullet.

        :param q: ROS orientation expressed as quaternion [qx, qy, qz, qw]
        :param t: ROS postion expressed as [tx, ty, tz]
        :return:  4x4 view matrix as used in pybullet and openGL

        """
        q = Quaternion([q[3], q[0], q[1], q[2]])
        R = q.rotation_matrix

        T = np.vstack([np.hstack([R, np.array(t).reshape(3, 1)]),
                       np.array([0, 0, 0, 1])])
        # Convert opencv convention to python convention
        # By a 180 degrees rotation along X
        Tc = np.array([[1, 0, 0, 0],
                       [0, -1, 0, 0],
                       [0, 0, -1, 0],
                       [0, 0, 0, 1]]).reshape(4, 4)

        # pybullet pse is the inverse of the pose from the ROS-TF
        T = Tc @ np.linalg.inv(T)
        # The transpose is needed for respecting the array structure of the OpenGL
        viewMatrix = T.T.reshape(16)
        return viewMatrix
    def read_camera_fixed(self, robot):
        """
        read_camera_fixed gets the image from the camera of the robot in the simulation

        :param robot: robot id in the simulation
        :return: image from the camera of the robot in the simulation and the time when the image was taken
        """
        while True:
            # print("Getting the pose", time.time()*1000-self.timestamp)
            com_p, com_o, _, _, _, _ = p.getLinkState(robot, 9)
            # print("Pose obtained", time.time()*1000-self.timestamp)
            # Define camera intrinsic parameters
            width = 1280  # image width
            height =  720  # image height
            f_in_pixels = 1298 #optimized
            near = 0.01  # near clipping plane
            far = 100  # far clipping plane

            # Optical center in pixel coordinates
            optical_center_x_pixels = 646.23 #620  # optimized
            optical_center_y_pixels = 267.62 #238  # optimized

            fov = 2 * np.degrees(np.arctan(width / (2 * f_in_pixels)))

            k = np.array([[f_in_pixels, 0, optical_center_x_pixels],
                          [0, f_in_pixels, optical_center_y_pixels],
                          [0, 0, 1]])

            projection_matrix = self.cvK2BulletP(k, width, height, near, far)

            # print("Projection matrix obtained", time.time() * 1000 - self.timestamp)
            # print("fixed proyection matrix", projection_matrix)

            # Define camera extrinsic parameters
            camera_translation = np.array([0.0, 0.0, 0.0])

            camera_rotation_matrix = np.array([
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]
            ])

            camera_translation += com_p
            com_o_matrix = p.getMatrixFromQuaternion(com_o)
            camera_rotation_matrix += np.array(com_o_matrix).reshape(3, 3)

            view_matrix = self.cvPose2BulletView(com_o, camera_translation)

            # print("View matrix obtained", time.time() * 1000 - self.timestamp)
            # print("fixed view matrix", np.matrix(view_matrix))
            # print("//////////////////////////////////////////////////////////////////")

            # Get the camera image
            img = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL, flags=p.ER_NO_SEGMENTATION_MASK)

            # print("Camera image obtained", time.time() * 1000 - self.timestamp)
            rgb = img[2]
            # rgb = cv2.resize(rgb, (1280, 720))
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            # # sum in one frame rgb and self.colorKinova[0][0]
            # cv2.imshow("Pybullet", rgb)

            if robot == self.robot_Pablo:
                sum = cv2.addWeighted(rgb, 0.5, self.colorKinova[0][0], 0.5, 0)
                cv2.imshow("Pybullet Pablo", sum)
                # print("Showing Pybullet Pablo image", time.time() * 1000 - self.timestamp)
                cv2.waitKey(1)
            elif robot == self.robot_Pedro:
                sum = cv2.addWeighted(rgb, 0.5, self.colorKinova2[0][0], 0.5, 0)
                cv2.imshow("Pybullet Pedro", sum)
                # print("Showing Pybullet Pedro image", time.time() * 1000 - self.timestamp)
                cv2.waitKey(1)
            else:
                assert "Robot not found"

            # print("Returning the image", time.time() * 1000 - self.timestamp)

            # print("Triying to put in queue", time.time()*1000 - self.timestamp)

            # self.pybulletImageQueue.put_nowait([rgb, time.time()*1000])

            # print("Pybullet image put in queue", time.time()*1000 - self.timestamp)

            # time.sleep(0.05)
            return rgb, time.time()*1000

    def readKinovaCamera(self):
        """
        readKinovaCamera gets the images from the camera of the Kinovas robots from CameraKinova.
        You can access the image from the camera using self.colorKinova and self.colorKinova2 queues

        :return: True if the images was obtained successfully, False otherwise
        """
        try:
            # both = self.camerargbdsimple_proxy.getImage("CameraRGBDViewer")                      # The depth flow slow down the queue then at the moment we are only using color flow
            # depthImage = (np.frombuffer(both.depth.depth, dtype=np.int16)
            #                     .reshape(both.depth.height, both.depth.width))
            # depthImage = cv2.normalize(src=depthImage, dst=None, alpha=0, beta=255,
            #                                  norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            # self.depthKinova.append([depthImage, both.depth.alivetime])
            # kinovaImage = (np.frombuffer(both.image.image, np.uint8)
            #                     .reshape(both.image.height, both.image.width, both.image.depth))

            image = self.camerargbdsimple_proxy.getImage("CameraRGBDViewer")
            kinovaImage = (np.frombuffer(image.image, np.uint8)
                                .reshape(image.height, image.width, image.depth))
            self.colorKinova.append([kinovaImage, image.alivetime])
            # cv2.imshow("Kinova camera", kinovaImage)

            # both = self.camerargbdsimple1_proxy.getImage("CameraRGBDViewer")
            # depthImage = (np.frombuffer(both.depth.depth, dtype=np.int16)
            #               .reshape(both.depth.height, both.depth.width))
            # depthImage = cv2.normalize(src=depthImage, dst=None, alpha=0, beta=255,
            #                            norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            # self.depthKinova2.append([depthImage, both.depth.alivetime])
            # kinovaImage = (np.frombuffer(both.image.image, np.uint8)
            #                .reshape(both.image.height, both.image.width, both.image.depth))

            image = self.camerargbdsimple1_proxy.getImage("CameraRGBDViewer")
            kinovaImage = (np.frombuffer(image.image, np.uint8)
                           .reshape(image.height, image.width, image.depth))
            self.colorKinova2.append([kinovaImage, image.alivetime])

            # cv2.imshow("Kinova camera 2", kinovaImage)
            # cv2.waitKey(3)

        except Ice.Exception as e:
            print(e)
        return True

    def showKinovaAngles(self, robot):
        """
        showKinovaAngles prints the angles of the joints of the Kinova robot

        :param robot: robot id in the simulation
        """
        print("//--------------------------------------------------------------------------------------------------//")
        ext_angles = []
        ext_torques = []
        diff_from_pybullet = []
        if robot == self.robot_Pablo:
            for i in range(7):
                ext_angles.append(self.ext_joints.joints[i].angle)
                ext_torques.append(self.ext_joints.joints[i].force)
                diff_from_pybullet.append((math.degrees(p.getJointState(self.robot_Pablo, i + 1)[0]) % 360) - self.ext_joints.joints[i].angle)

        elif robot == self.robot_Pedro:
            for i in range(7):
                ext_angles.append(self.ext_joints_2.joints[i].angle)
                ext_torques.append(self.ext_joints_2.joints[i].force)
                diff_from_pybullet.append((math.degrees(p.getJointState(self.robot_Pedro, i + 1)[0]) % 360) - self.ext_joints_2.joints[i].angle)

        else :
            assert "Robot not found"

        print("Kinova angles", ext_angles)
        print("Kinova torqe", ext_torques)
        print("Diff from pybullet", diff_from_pybullet)
        print("Gripper distance", self.ext_gripper.distance)

    def movePybulletWithExternalVel(self, robot):
        """
        movePybulletWithExternalVel moves the robot in pybullet simulation using the velocities of the joints from the real robot
        """
        if robot == self.robot_Pablo:
            for i in range(len(self.ext_joints.joints)):
                self.target_velocities[i] = self.ext_joints.joints[i].velocity

            self.target_velocities = numpy.deg2rad(self.target_velocities) * 1.2

            for i in range(len(self.target_velocities)):
                p.setJointMotorControl2(self.robot_Pedro, i + 1, p.VELOCITY_CONTROL,
                                        targetVelocity=self.target_velocities[i])

        elif robot == self.robot_Pedro:
            for i in range(len(self.ext_joints_2.joints)):
                self.target_velocities[i] = self.ext_joints_2.joints[i].velocity

            self.target_velocities = numpy.deg2rad(self.target_velocities) * 1.2

            for i in range(len(self.target_velocities)):
                p.setJointMotorControl2(self.robot_Pedro, i + 1, p.VELOCITY_CONTROL,
                                        targetVelocity=self.target_velocities[i])

        else:
            assert "Robot not found"

    def movePybulletWithToolbox(self):
        """
        movePybulletWithToolbox moves the robot in pybullet simulation using the velocities calculated with the toolbox
        """
        # print("Pybullet move with toolbox init", time.time()*1000 - self.timestamp)
        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_Pedro, i + 1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])
        # print("Pybullet move with toolbox end", time.time()*1000 - self.timestamp)

    def movePybulletArmWithAngles(self, angles, robot):
        """
        movePybulletArmWithAngles moves the Pablo robot in pybullet simulation using the angles of the joints

        :param angles: angles of the joints of the robot
        :param robot: robot in the simulation that will be moved
        """
        if robot == self.robot_Pedro:
            for i in range(7):
                p.setJointMotorControl2(self.robot_Pedro, i + 1, p.POSITION_CONTROL,
                                        targetPosition=angles[i],
                                        maxVelocity=np.deg2rad(25))

        if robot == self.robot_Pablo:
            for i in range(7):
                p.setJointMotorControl2(self.robot_Pablo, i + 1, p.POSITION_CONTROL,
                                        targetPosition=angles[i],
                                        maxVelocity=np.deg2rad(25))

        assert "Robot not found"

    def movePybulletArmWithVelocities(self, velocities, robot):
        """
        movePybulletArmWithVelocities moves the robot in pybullet simulation using the velocities of the joints

        :param velocities: velocities of the joints of the robot
        :param robot: robot in the simulation that will be moved
        """
        if robot == self.robot_Pablo:
            for i in range(7):
                p.setJointMotorControl2(self.robot_Pablo, i + 1, p.VELOCITY_CONTROL,
                                        targetVelocity=velocities[i])
        if robot == self.robot_Pedro:
            for i in range(7):
                p.setJointMotorControl2(self.robot_Pedro, i + 1, p.VELOCITY_CONTROL,
                                        targetVelocity=velocities[i])

        assert "Robot not found"

    def readDataFromProxy(self):
        """
        readDataFromProxy reads the data from the proxies of the kinova robots from the RoboCompKinovaArm interface
        """
        while True:
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()
            self.ext_gripper.distance = self.ext_gripper.distance * 0.8
            self.ext_joints_2 = self.kinovaarm1_proxy.getJointsState()
            self.ext_gripper_2 = self.kinovaarm1_proxy.getGripperState()
            self.ext_gripper_2.distance = self.ext_gripper_2.distance * 0.8

            time.sleep(0.05)

    def moveRealKinovaWithAngles(self, angles, robot):
        """
        moveRealKinovaWithAngles moves the Pablo robot in the simulation using the angles of the joints

        :param angles: angles of the joints of the robot
        :param robot: real robot that will be moved
        """
        if robot == self.robot_Pablo:
            array = np.round(np.rad2deg(angles) % 360)
            self.angles.jointAngles = array.tolist()
            self.kinovaarm_proxy.moveJointsWithAngle(self.angles)

        elif robot == self.robot_Pedro:
            array = np.round(np.rad2deg(angles) % 360)
            self.angles.jointAngles = array.tolist()
            self.kinovaarm1_proxy.moveJointsWithAngle(self.angles)

        else:
            assert "Robot not found"

    def moveRealKinovaWithSpeeds(self, robot):
        """
        moveRealKinovaWithSpeeds moves the Pablo robot in the simulation using the speeds of the joints

        :param robot: real robot that will be moved
        """
        self.joint_speeds = []

        for i in range(7):
            angle = p.getJointState(robot, i + 1)[0]
            if robot == self.robot_Pablo:
                error = (np.deg2rad(self.ext_joints.joints[i].angle)
                         - angle + math.pi) % (2 * math.pi) - math.pi
            elif robot == self.robot_Pedro:
                error = (np.deg2rad(self.ext_joints_2.joints[i].angle)
                         - angle + math.pi) % (2 * math.pi) - math.pi
            else:
                assert "Robot not found"

            speed = np.rad2deg(p.getJointState(robot, i + 1)[1]) * self.gains[i] - np.rad2deg(error) * 0.3

            self.joint_speeds.append(speed)

        self.speeds.jointSpeeds = self.joint_speeds

        if robot == self.robot_Pablo:
            self.kinovaarm_proxy.moveJointsWithSpeed(self.speeds)
        elif robot == self.robot_Pedro:
            self.kinovaarm1_proxy.moveJointsWithSpeed(self.speeds)
        else:
            assert "Robot not found"

    def updateGains(self):
        self.posesTimes = self.posesTimes - self.ext_joints.timestamp
        best_timestamp = np.abs(self.posesTimes).argmin()

        # print("Best timestamp: ", best_timestamp, self.posesTimes[best_timestamp],
        #       self.ext_joints.timestamp)

        joints_state = self.poses[best_timestamp]
        for i in range(7):
            angle = joints_state[i][0]
            speed = joints_state[i][1]
            error = (np.deg2rad(self.ext_joints.joints[i].angle)
                     - angle + math.pi) % (2 * math.pi) - math.pi
            if abs(speed) > 0.01:
                self.gains[i] += error * 0.2
            # print("Gains: joint ", i, self.gains[i], "Kinova angle: ",
            #       np.deg2rad(self.ext_joints.joints[i].angle),
            #       "Pybullet angle: ", self.target_angles[i], "Error:", error)

        self.posesTimes = np.array([int(time.time() * 1000)])
        self.poses = joints_state

  
    # =============== Methods for SubscribesTo ================
    # =========================================================
    #
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    #
    def JoystickAdapter_sendData(self, data):
        match self.move_mode:
            case 0:
                for axis in data.axes:
                    match axis.name:
                        case "mode":
                            if axis.value > 0.5:
                                self.joy_selected_joint = (self.joy_selected_joint + 1) % 7
                            elif axis.value < -0.5:
                                self.joy_selected_joint = (self.joy_selected_joint - 1) % 7
                        case "X_axis":
                            pass
                        case "Y_axis":
                            self.target_angles[self.joy_selected_joint] += axis.value * 0.05
                        case "Z_axis":
                            pass
                        case "gripper":
                            pass
                for button in data.buttons:
                    match button.name:
                        case "move_mode":
                            print("Mode: ", self.move_mode)
                            self.move_mode = (self.move_mode + button.step) % 5
                        case "home":
                            pass
            case 1:
                for axis in data.axes:
                    match axis.name:
                        case "mode":
                            pass
                        case "X_axis":
                            target_position = list(self.target_position)
                            target_position[1] += axis.value * 0.01
                            self.target_position = tuple(target_position)
                        case "Y_axis":
                            target_position = list(self.target_position)
                            target_position[0] += - axis.value * 0.01
                            self.target_position = tuple(target_position)
                        case "Z_axis":
                            target_position = list(self.target_position)
                            target_position[2] -= axis.value * 0.01
                            self.target_position = tuple(target_position)
                        case "gripper":
                            pass
                for button in data.buttons:
                    match button.name:
                        case "move_mode":
                            print("Mode: ", self.move_mode)
                            self.move_mode = (self.move_mode + button.step) % 5
                        case "home":
                            pass
            case 2:
                for axis in data.axes:
                    match axis.name:
                        case "mode":
                            pass
                        case "X_axis":
                            target_orientation = list(p.getEulerFromQuaternion(self.target_orientation))
                            target_orientation[0] += axis.value * 0.1
                            self.target_orientation = p.getQuaternionFromEuler(tuple(target_orientation))
                        case "Y_axis":
                            target_orientation = list(p.getEulerFromQuaternion(self.target_orientation))
                            target_orientation[1] += axis.value * 0.1
                            self.target_orientation = p.getQuaternionFromEuler(tuple(target_orientation))
                        case "Z_axis":
                            target_orientation = list(p.getEulerFromQuaternion(self.target_orientation))
                            target_orientation[2] += axis.value * 0.1
                            self.target_orientation = p.getQuaternionFromEuler(tuple(target_orientation))
                        case "gripper":
                            pass
                for button in data.buttons:
                    match button.name:
                        case "move_mode":
                            print("Mode: ", self.move_mode)
                            self.move_mode = (self.move_mode + button.step) % 5
                        case "home":
                            pass
            case 3:
                for button in data.buttons:
                    match button.name:
                        case "move_mode":
                            print("Mode: ", self.move_mode)
                            self.move_mode = (self.move_mode + button.step) % 5
                        case "home":
                            pass

            case 4:
                for axis in data.axes:
                    match axis.name:
                        case "mode":
                            pass
                        case "X_axis":
                            target_position = list(self.robot2_target_pos)
                            target_position[0] += axis.value * 0.05
                            self.robot2_target_pos = tuple(target_position)
                        case "Y_axis":
                            target_position = list(self.robot2_target_pos)
                            target_position[1] += axis.value * 0.05
                            self.robot2_target_pos = tuple(target_position)
                        case "Z_axis":
                            target_position = list(self.robot2_target_pos)
                            target_position[2] -= axis.value * 0.05
                            self.robot2_target_pos = tuple(target_position)
                        case "gripper":
                            pass
                for button in data.buttons:
                    match button.name:
                        case "move_mode":
                            print("Mode: ", self.move_mode)
                            self.move_mode = (self.move_mode + button.step) % 5
                        case "home":
                            pass

    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # self.camerargbdsimple_proxy.getAll(...)
    # self.camerargbdsimple_proxy.getDepth(...)
    # self.camerargbdsimple_proxy.getImage(...)
    # self.camerargbdsimple_proxy.getPoints(...)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.Point3D
    # RoboCompCameraRGBDSimple.TPoints
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # self.camerargbdsimple1_proxy.getAll(...)
    # self.camerargbdsimple1_proxy.getDepth(...)
    # self.camerargbdsimple1_proxy.getImage(...)
    # self.camerargbdsimple1_proxy.getPoints(...)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.Point3D
    # RoboCompCameraRGBDSimple.TPoints
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # self.kinovaarm_proxy.closeGripper(...)
    # self.kinovaarm_proxy.getCenterOfTool(...)
    # self.kinovaarm_proxy.getGripperState(...)
    # self.kinovaarm_proxy.getJointsState(...)
    # self.kinovaarm_proxy.moveJointsWithAngle(...)
    # self.kinovaarm_proxy.moveJointsWithSpeed(...)
    # self.kinovaarm_proxy.openGripper(...)
    # self.kinovaarm_proxy.setCenterOfTool(...)

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # RoboCompKinovaArm.TPose
    # RoboCompKinovaArm.TGripper
    # RoboCompKinovaArm.TJoint
    # RoboCompKinovaArm.TJoints
    # RoboCompKinovaArm.TJointSpeeds
    # RoboCompKinovaArm.TJointAngles

    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # self.kinovaarm1_proxy.closeGripper(...)
    # self.kinovaarm1_proxy.getCenterOfTool(...)
    # self.kinovaarm1_proxy.getGripperState(...)
    # self.kinovaarm1_proxy.getJointsState(...)
    # self.kinovaarm1_proxy.moveJointsWithAngle(...)
    # self.kinovaarm1_proxy.moveJointsWithSpeed(...)
    # self.kinovaarm1_proxy.openGripper(...)
    # self.kinovaarm1_proxy.setCenterOfTool(...)

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # RoboCompKinovaArm.TPose
    # RoboCompKinovaArm.TGripper
    # RoboCompKinovaArm.TJoint
    # RoboCompKinovaArm.TJoints
    # RoboCompKinovaArm.TJointSpeeds
    # RoboCompKinovaArm.TJointAngles

    ######################
    # From the RoboCompRoboticsToolboxController you can call this methods:
    # self.roboticstoolboxcontroller_proxy.calculateVelocitiesPablo(...)
    # self.roboticstoolboxcontroller_proxy.calculateVelocitiesPedro(...)
    # self.roboticstoolboxcontroller_proxy.setStatePablo(...)
    # self.roboticstoolboxcontroller_proxy.setStatePedro(...)

    ######################
    # From the RoboCompRoboticsToolboxController you can use this types:
    # RoboCompRoboticsToolboxController.JointStates

    ######################
    # From the RoboCompJoystickAdapter you can use this types:
    # RoboCompJoystickAdapter.AxisParams
    # RoboCompJoystickAdapter.ButtonParams
    # RoboCompJoystickAdapter.TData

