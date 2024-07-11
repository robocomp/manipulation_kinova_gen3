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

from pyAAMED import pyAAMED
import YoloDetector

from pyquaternion import Quaternion

import numpy
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
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

import queue

# Set the LC_NUMERIC environment variable
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'
# RMEMEMBER to SET: export LC_NUMERIC="en_US.UTF-8" in your terminal

console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    """
    Controls a robot's movements based on sensor data, with specific functions for
    each mode (e.g., home, move_mode, etc.). It updates the robot's position and
    orientation based on axis values and button presses, and switches between
    different modes based on the received data.

    Attributes:
        Period (int): 5 by default, indicating the maximum number of times the
            worker can move its end effector within a given time period before it
            needs to pause and recharge.
        rgb (3element): Used to represent the current position of the robot's end
            effector in RGB color space, where each component represents the
            position of the end effector along the X, Y, and Z axes, respectively.
        timestamp (int): 0 by default, indicating that the worker has just started
            processing the job.
        startup_check (int): 4 by default, indicating that the worker will check
            for startup-related data every 4 seconds.
        physicsClient (instance): Used to access the physics client for the robot's
            simulation, which allows for interaction with the robot's physical environment.
        plane (str): Used to store the plane that the robot is currently working
            on.
        table_id (str): A unique identifier for each worker, used to differentiate
            between workers in the table.
        robot_urdf (obb): A dictionary that contains the URDF (Unified Robot
            Description Format) file of the robot used for manipulation.
        robot_launch_pos (tuple): Defined as the position of the robot at the time
            of launch, used for storing the initial position of the robot in the
            simulation.
        robot_launch_orien (numpyndarray): Used to store the orientation of the
            robot in a specific configuration.
        end_effector_link_index (int): 0-based, indicating the index of the end
            effector link in the robot's linkage structure.
        robot_id (int): 4 in this case, indicating that the worker is the fourth
            robot in the system.
        home_angles (list): Predefined angles values for each axis (X, Y, Z, and
            gripper) to return the robot to its home position when a specific
            button is pressed.
        observation_angles (ndarray): 4D, representing the angle values for each
            axis (X, Y, Z, and gripper) as a numpy array.
        table_center (OptionalTuple): Used to store the center position of the
            table in the robot's workspace, which is used for motion planning and
            task execution.
        cylinderId (int): 1 by default, representing the first cylinder of a robot's
            body. It is used to identify which part of the robot's body the worker
            is attached to.
        threadKinovaAngles (ndarray): 0-dimensional, indicating that it has a
            single value representing the angles for the Kinova robot.
        readDataFromProxy (lambda): Used to read data from a proxy server. It takes
            in the robot's state as input and returns the data read from the proxy
            server.
        colorKinova (str): Used to represent the color of the Kinova robot.
        depthKinova (int): 0 by default, indicating that the worker is a Kinova
            robot with depth sensors.
        target_angles (list): Used to store the target angles for each axis (X,
            Y, Z) and gripper. It stores the target position of the end effector
            in angle space.
        target_position (list): Used to store the target position of the robot's
            end effector for each axis (X, Y, Z, and gripper). The values in the
            list are used to calculate the new position of the end effector based
            on user input.
        target_orientation (3D): Used to store the desired orientation of the
            robot's end effector relative to its initial position, after the move
            command has been executed.
        target_velocities (list): A reference to the target velocities for each
            axis of the robot's end effector, which are used to calculate the
            desired position of the robot.
        joy_selected_joint (str): 1-based index of the joint that the user has
            selected from the joystick, indicating which joint to move or manipulate.
        move_mode (int): 0-4, representing the move mode (home, absolute, or
            relative movement) of the robot.
        ext_joints (list): A list of joints that are external to the robot,
            indicating that they can be controlled by the worker programmatically,
            such as the gripper.
        kinovaarm_proxy (instance): A connection to the Kinova arm's API, which
            allows the worker to control the arm's movements and receive feedback
            on its position and orientation.
        ext_gripper (Optionalfloat): Used to represent the extension of the gripper
            in the robot's end effector coordinate system.
        timer (int): 0 by default, indicating that the worker has not yet completed
            its task. It increments every time the worker takes a break, and it
            resets to 0 when the worker is active again.
        compute (Callable): Defined as a function that computes the next target
            position, orientation, or gripper position based on the robot's current
            state and the received data packet.
        joint_speeds (list): 3x1 array, where each element is a 2-element tuple
            of floating-point numbers representing the desired speed and torque
            for each joint in the robot.
        speeds (int): 0-2, representing the speed values for the X, Y, and Z axes,
            respectively, to move the robot's end effector to its target position.
        angles (list): 0-dimensional, representing a sequence of angles to be
            applied to the robot's joints during movement.
        gains (float): Used to control the speed of the robot's movement, with
            higher values resulting in faster movement and lower values resulting
            in slower movement.
        posesTimes (list): 3D position and orientation of the worker at a specific
            time step, represented as numpy arrays with shape (3,) for position
            and (4,) for orientation.
        poses (list): 3D position and orientation vectors representing the worker's
            current pose in space.
        calibrator (instance): A method that takes no arguments, which when called,
            updates the worker's position and orientation based on the data received
            from the robot.
        cameraKinovaTimer (float): 0 by default, which represents the time it takes
            for the robot to move from its current position to the target position
            specified in the `target_position` attribute.
        readKinovaCamera (methods): Not a valid attribute of this class.
        showKinovaStateTimer (int): 0 by default, which means that it will display
            the current state of the Kinova robot every 0.05 seconds.
        showKinovaAngles (Optionalfloat): Used to display the angles of the Kinova
            robot's joints.
        gainsTimer (float): 0 by default, representing the timer for gripper gain
            control.
        updateGains (lambda): Used to update the gains for the robot's motors based
            on the current mode and position of the robot.
        aamed (instance): A list of 5 values representing the different work modes
            of the robot (move, home, undock, dock, and gripper).

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes the SpecificWorker class, setting up connections to the Kinova
        robot and defining the movement mode, target angles, and gains. It also
        starts timers for reading camera images, updating joint speeds, and computing
        torque commands.

        Args:
            proxy_map (int): Used to specify the mapping between the real joints
                of the Kinova arm and the virtual joints of the robot's end effector.
                It defines the correspondence between the actual joint angles of
                the Kinova arm and the desired joint angles of the robot.
            startup_check (int): 0 by default, which means it will check if the
                robot is properly initialized during startup.

        """
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

            # Load a table to place the arm on
            self.table_id = p.loadURDF("./URDFs/table/table.urdf", basePosition=[0, 0, 0],
                                       baseOrientation=p.getQuaternionFromEuler([0, 0, 1.57]), flags=flags)

            # Load the arm in the simulation
            self.robot_urdf = "./URDFs/kinova_with_pybullet/gen3_robotiq_2f_85-mod.urdf"
            self.robot_launch_pos = [-0.3, 0.0, 0.64]
            self.robot_launch_orien = p.getQuaternionFromEuler([0, 0, 0])
            self.end_effector_link_index = 12
            self.robot_id = p.loadURDF(self.robot_urdf, self.robot_launch_pos, self.robot_launch_orien,
                                       flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

            # Angles for the home position of the robot
            self.home_angles = [0, -0.34, np.pi, -2.54, 0, -0.87, np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            # Angles for the observation position of the robot
            self.observation_angles = [0, 0, np.pi, -0.96, 0, -2.1, np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            # Coordenates for the center of the table
            self.table_center = [0.374, 0.0, 0.26]

            # Set the initial joint angles of the pybullet arm
            for i in range(7):
                p.resetJointState(bodyUniqueId=self.robot_id, jointIndex=i+1,
                                  targetValue=self.home_angles[i], targetVelocity=0)

            # Load a square to place on the table for calibration
            # self.square = p.loadURDF("./URDFs/cube_and_square/cube_small_square.urdf", basePosition=[0.074, 0.0, 0.64], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
            # texture_path = "./URDFs/cube_and_square/square_texture.png"
            # textureIdSquare = p.loadTexture(texture_path)
            # p.changeVisualShape(self.square, -1, textureUniqueId=textureIdSquare)

            # # Load a cube to place on the table for calibration
            # self.cube = p.loadURDF("./URDFs/cube_and_square/cube_small.urdf", basePosition=[0.074, 0.0, 0.64], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
            # texture_path = "./URDFs/cube_and_texture/cube_texture.png"
            # textureId = p.loadTexture(texture_path)
            # p.changeVisualShape(self.cube, -1, textureUniqueId=textureId)

            # Load a cylinder to place on the table
            self.cylinderId = p.loadURDF("./URDFs/cylinder/cylinder.urdf", [0.074, 0.2, 0.70], p.getQuaternionFromEuler([0, 0, 0]))

            # Thread to read the real arm angles from kinova_controller
            self.threadKinovaAngles = threading.Thread(target=self.readDataFromProxy)
            self.threadKinovaAngles.start()

            # Queues to store the images from the real arm camera
            self.colorKinova = collections.deque(maxlen=5)
            self.depthKinova = collections.deque(maxlen=5)

            # Wait for half a second
            time.sleep(0.5)

            # Change the mass of the links of the robot to do the simulation more stable
            for i in range(7):
                if i != 0:
                    val = p.getDynamicsInfo(self.robot_id, i)[2]
                    nuevo_val = np.array(val) * 1000
                    p.changeDynamics(self.robot_id, i, localInertiaDiagonal=nuevo_val)
                else:
                    val = p.getDynamicsInfo(self.robot_id, i)[2]
                    nuevo_val = np.array(val) * 1000
                    p.changeDynamics(self.robot_id, i, localInertiaDiagonal=nuevo_val)

                print("Kinova", i, p.getDynamicsInfo(self.robot_id, i))


            # This variables is to store the position of the end effector of the robot
            self.target_angles = self.home_angles
            self.target_position = p.getLinkState(self.robot_id, self.end_effector_link_index)[0]
            self.target_orientation = p.getLinkState(self.robot_id, self.end_effector_link_index)[1]
            self.target_velocities = [0.0] * 7

            # This variable is to store the joint selected in the joystick mode
            self.joy_selected_joint = 0

            # This variable is to store the mode of the robot
            self.move_mode = 4

            # This variable is to store the state of the real arm
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()

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
                actual_angle = (i, p.getJointState(self.robot_id, i + 1)[0])
                joints.append(actual_angle)
                self.poses.append(joints)

            # This variable was to test the calibration of the camera
            self.calibrator = calibrator.Calibrator()

            # Timer to read the camera of the real arm
            self.cameraKinovaTimer = QtCore.QTimer(self)
            self.cameraKinovaTimer.timeout.connect(self.readKinovaCamera)
            self.cameraKinovaTimer.start(self.Period)

            # Timers to update the gains and show the real arm angles
            self.showKinovaStateTimer = QtCore.QTimer(self)
            self.showKinovaStateTimer.timeout.connect(self.showKinovaAngles)

            self.gainsTimer = QtCore.QTimer(self)
            self.gainsTimer.timeout.connect(self.updateGains)

            # Initialize the AAMED algorithm for the cup position correction
            self.aamed = pyAAMED(722//2, 1282//2)
            self.aamed.setParameters(3.1415926 / 3, 3.4, 0.77)

            print("SpecificWorker started", time.time()*1000 - self.timestamp)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        """
        Sets parameters for an instance of the `SpecificWorker` class, which
        inherits from `GenericWorker`. It returns `True` indicating successful
        parameter setting.

        Args:
            params (object): Used to set parameters for an instance of a class.

        Returns:
            Boolean: 1 if the method executed successfully and 0 otherwise.

        """
        return True


    @QtCore.Slot()
    def compute(self):

        """
        Computes the optimal joint angles for a specific task, such as picking up
        an object from a table and placing it on a surface. It uses the toolbox
        to compute the angles and moves the robot's joints using PyBullet and
        Kinova arm proxy.

        """
        match self.move_mode:
            #Move joints
            case 0:
                self.target_angles[7:] = [0.0] * len(self.target_angles[7:])

                jointsState = []
                for i in range(len(self.target_angles)):
                    p.setJointMotorControl2(self.robot_id, i+1, p.POSITION_CONTROL,
                                            self.target_angles[i], maxVelocity=np.deg2rad(25))
                    if i < 7:
                        angle_speed = (p.getJointState(self.robot_id, i + 1)[0],
                                       p.getJointState(self.robot_id, i + 1)[1])
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
                joint_positions = p.calculateInverseKinematics(self.robot_id, self.end_effector_link_index,
                                                               self.target_position, self.target_orientation)
                #print(joint_positions)
                joints = np.zeros(22).tolist()
                for i in range(len(joint_positions)):
                    joints[i] = joint_positions[i]
                    self.target_angles[i] = joint_positions[i]

                jointsState = []
                for i in range(len(joints)):
                    p.setJointMotorControl2(self.robot_id, i+1, p.POSITION_CONTROL, self.target_angles[i],
                                            maxVelocity=np.deg2rad(25))
                    if i < 7:
                        angle_speed = (p.getJointState(self.robot_id, i + 1)[0],
                                       p.getJointState(self.robot_id, i + 1)[1])
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
                    p.setJointMotorControl2(self.robot_id, i+1, p.POSITION_CONTROL, joints[i], maxVelocity=np.deg2rad(25))


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
                        p.setJointMotorControl2(self.robot_id, i+1, p.POSITION_CONTROL, targetPosition=self.target_angles[i]) #Move the arm with phisics


                except Ice.Exception as e:
                    print(e)

            case 4:    #Move to observation angles
                print("Moving to observation angles", int(time.time()*1000) - self.timestamp)
                self.moveKinovaWithAngles(self.observation_angles[:7])
                for i in range(7):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL,
                                            targetPosition=self.observation_angles[i], maxVelocity=np.deg2rad(25))

                self.target_angles[13] = self.ext_gripper.distance
                self.target_angles[15] = - self.ext_gripper.distance
                self.target_angles[17] = self.ext_gripper.distance - 0.1

                self.target_angles[18] = self.ext_gripper.distance
                self.target_angles[20] = - self.ext_gripper.distance
                self.target_angles[22] = self.ext_gripper.distance - 0.1

                for i in range(8, len(self.target_angles)):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL,
                                            targetPosition=self.target_angles[i])

                angles = []
                for i in range(7):
                    angles.append(p.getJointState(self.robot_id, i + 1)[0])

                error = np.sum(np.abs(np.array(angles) - np.array(self.observation_angles[:7])))

                pybulletImage, imageTime = self.read_camera_fixed()

                if error < 0.05:
                    print("Observation angles reached", int(time.time()*1000) - self.timestamp)
                    self.move_mode = 5
            case 5:
                # self.calibrator.calibrate3(self.robot_id, self.colorKinova)
                # self.calibrator.cube_test(self.robot_id, self.colorKinova.copy())
                # self.calibrator.square_test(self.robot_id, self.colorKinova.copy())

                # yolodetector = YoloDetector.YoloDetector()
                # results = yolodetector.detect(self.colorKinova)
                # yolodetector.plot(results)
                # print(self.threadPybulletImage.is_alive())

                if self.correctTargetPosition() > 5:
                    print("Correcting cup position", int(time.time()*1000) - self.timestamp)

                else:
                    print("Calibration finished", int(time.time()*1000 - self.timestamp))
                    self.move_mode = 6

            case 6:
                # self.showKinovaAngles()
                self.timer.stop()
                # self.calibrator.get_kinova_images(self.robot_id, self.kinovaarm_proxy, self.camerargbdsimple_proxy)
                # self.calibrator.calibrate4(self.robot_id)
                # self.move_mode = -1

                target_position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
                target_position[0] = target_position[0] + 0.3
                target_position[2] = target_position[2] - 0.45

                print("initilizing toolbox", time.time()*1000 - self.timestamp)
                self.initialize_toolbox(target_position)
                print("toolbox initialized", time.time()*1000 - self.timestamp)
                self.showKinovaStateTimer.start(1000)
                self.gainsTimer.start(1000)

                print("Moving to fixed cup")
                self.move_mode = 7
                self.timer.start(self.Period)
                self.loopCounter = 0

            case 7:
                try:
                    if self.correctTargetPosition() == -1:
                        self.loopCounter += 1
                    # print("Correct cup position end", time.time()*1000 - self.timestamp)

                    target_position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
                    target_position[0] = target_position[0] + 0.3
                    target_position[2] = target_position[2] - 0.45

                    self.toolbox_compute(target_position)
                    # print("Toolbox compute end", time.time()*1000 - self.timestamp)
                    self.movePybulletWithToolbox()
                    # print("Move pybullet with toolbox end", time.time()*1000 - self.timestamp)
                    self.moveKinovaWithSpeeds()
                    # print("Move kinova with speeds end", time.time()*1000 - self.timestamp)

                    self.posesTimes = np.append(self.posesTimes, int(time.time() * 1000))

                    jointsState = []
                    for i in range(7):
                        state = p.getJointState(self.robot_id, i + 1)
                        angle_speed = (state[0], state[1])
                        jointsState.append(angle_speed)

                    self.poses.append(jointsState)

                    # print("Pybullet poses saved to update gains", time.time()*1000 - self.timestamp)
                    # print("//====================================//")

                    gripper_position = p.getLinkState(self.robot_id, self.end_effector_link_index)[0]
                    target_position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])

                    if gripper_position[2] - target_position[2] < 0.1 or self.loopCounter > 10:
                        print("Working without visual feedback")
                        self.move_mode = 8
                except Ice.Exception as e:
                    print(e)

            case 8:
                try:
                    if not self.arrived:
                        target_position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
                        target_position[0] = target_position[0] + 0.3
                        target_position[2] = target_position[2] - 0.45
                        self.toolbox_compute(target_position)
                        self.movePybulletWithToolbox()

                    self.moveKinovaWithSpeeds()

                    imgPybullet, imageTime = self.read_camera_fixed()

                    self.posesTimes = np.append(self.posesTimes, int(time.time() * 1000))

                    jointsState = []
                    for i in range(7):
                        state = p.getJointState(self.robot_id, i + 1)
                        angle_speed = (state[0], state[1])
                        jointsState.append(angle_speed)

                    self.poses.append(jointsState)

                    if self.arrived == True:
                        # print("Arrived")
                        self.target_velocities = [0.0] * 7
                        self.movePybulletWithToolbox()
                        self.moveKinovaWithSpeeds()
                        self.kinovaarm_proxy.closeGripper(0.26)
                        self.move_mode = 9

                except Ice.Exception as e:
                    print(e)

            case 9:
                try:
                    self.arrived = False
                    target_position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
                    target_position[0] = target_position[0] + 0.3
                    target_position[2] = target_position[2] - 0.25

                    self.toolbox_compute(target_position)
                    self.movePybulletWithToolbox()
                    self.moveKinovaWithSpeeds()

                    imgPybullet, imageTime = self.read_camera_fixed()

                    self.posesTimes = np.append(self.posesTimes, int(time.time() * 1000))

                    jointsState = []
                    for i in range(7):
                        state = p.getJointState(self.robot_id, i + 1)
                        angle_speed = (state[0], state[1])
                        jointsState.append(angle_speed)

                    self.poses.append(jointsState)

                    if self.arrived == True:
                        # print("Arrived")
                        self.target_velocities = [0.0] * 7
                        self.movePybulletWithToolbox()
                        self.moveKinovaWithSpeeds()
                        self.move_mode = 10

                except Ice.Exception as e:
                    print(e)

            case 10:
                try:
                    self.arrived = False

                    self.toolbox_compute(self.table_center)
                    self.movePybulletWithToolbox()
                    self.moveKinovaWithSpeeds()

                    imgPybullet, imageTime = self.read_camera_fixed()

                    self.posesTimes = np.append(self.posesTimes, int(time.time() * 1000))

                    jointsState = []
                    for i in range(7):
                        state = p.getJointState(self.robot_id, i + 1)
                        angle_speed = (state[0], state[1])
                        jointsState.append(angle_speed)

                    self.poses.append(jointsState)

                    if self.arrived == True:
                        # print("Arrived")
                        self.target_velocities = [0.0] * 7
                        self.movePybulletWithToolbox()
                        self.moveKinovaWithSpeeds()
                        self.kinovaarm_proxy.closeGripper(0.0)
                        self.move_mode = 11

                except Ice.Exception as e:
                    print(e)




    # =============== Methods ==================

    def startup_check(self):
        """
        Tests various parts of the RoboCompKinovaArm and its associated adapters,
        including TPose, TGripper, TJoint, TJoints, AxisParams, ButtonParams, and
        TData.

        """
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TGripper from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TGripper()
        print(f"Testing RoboCompKinovaArm.TJoint from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoint()
        print(f"Testing RoboCompKinovaArm.TJoints from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoints()
        print(f"Testing RoboCompJoystickAdapter.AxisParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.AxisParams()
        print(f"Testing RoboCompJoystickAdapter.ButtonParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.ButtonParams()
        print(f"Testing RoboCompJoystickAdapter.TData from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.TData()
        QTimer.singleShot(200, QApplication.instance().quit)

    def correctTargetPosition(self):
        # print("//--------------------//")
        # print("Get pybullet image", time.time()*1000 - self.timestamp)
        """
        Compares the positions of keypoints detected by a camera and those predicted
        by a pose estimation algorithm, and adjusts the position of a cylinder to
        minimize the error between the two.

        Returns:
            float: The distance between the predicted position of the target and
            the actual position detected by the kinova.

        """
        pybulletImage, imageTime = self.read_camera_fixed()
        # print("Pybullet image obtained", time.time()*1000 - self.timestamp)
        pybulletImage = cv2.resize(pybulletImage, (1280//2, 720//2))
        imgGPybullet = cv2.cvtColor(pybulletImage, cv2.COLOR_BGR2GRAY)
        # print("processing first image", time.time()*1000 - self.timestamp)
        resPybullet = self.aamed.run_AAMED(imgGPybullet)
        if isinstance(resPybullet, list):
            resPybullet = np.array(resPybullet)
        # aamed.drawAAMED(imgGPybullet)
        # if len(resPybullet) > 0:
        #     cv2.circle(imgGPybullet, (round(resPybullet[0][1]), round(resPybullet[0][0])), 8, (0, 0, 255), -1)
        #     cv2.imshow("test pybullet", imgGPybullet)

        # print("select second image", time.time()*1000 - self.timestamp)

        diff = 5000
        index = 0
        for i in range(len(self.colorKinova)):
            # print("Index: ", i)
            # print("Kinova timestamps :", self.colorKinova[i][1])
            # print("Pybullet timestamp:", imageTime)
            if abs(imageTime - self.colorKinova[i][1]) < diff:
                diff = abs(self.colorKinova[i][1] - imageTime)
                index = i

        # print("Diff: ", diff)

        # print("processing second image", time.time()*1000 - self.timestamp)

        imgGKinova = cv2.cvtColor(self.colorKinova[index][0], cv2.COLOR_BGR2GRAY)
        imgGKinova = cv2.resize(imgGKinova, (1280//2, 720//2))
        resKinova = np.array(self.aamed.run_AAMED(imgGKinova))
        if isinstance(resKinova, list):
            resKinova = np.array(resKinova)
        # aamed.drawAAMED(imgGKinova)
        # if len(resKinova) > 0:
        #     cv2.circle(imgGKinova, (round(resKinova[0][1]), round(resKinova[0][0])), 8, (0, 0, 255), -1)
        #     cv2.imshow("test kinova", imgGKinova)

        # print("second image processed", time.time()*1000 - self.timestamp)

        if resKinova.size == 0 or resPybullet.size == 0:
            print("No keypoints detected")
            return -1

        error = np.abs(resKinova[0][1] - resPybullet[0][1] + resKinova[0][0] - resPybullet[0][0])

        # print("x: ", resKinova[0][1] - resPybullet[0][1], " y: ", resKinova[0][0] - resPybullet[0][0])

        position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
        position[0] = position[0] - 0.0002 * (resKinova[0][0] - resPybullet[0][0])
        position[1] = position[1] - 0.0002 * (resKinova[0][1] - resPybullet[0][1])
        p.resetBasePositionAndOrientation(self.cylinderId, tuple(position), p.getQuaternionFromEuler([0, 0, 0]))

        # print("//--------------------//")

        # print("Finish time", time.time()*1000 - self.timestamp)
        return error

    def initialize_toolbox(self, target_position):
        ## Launch the simulator Swift
        """
        Initializes toolbox components, including an environment, kinova robot,
        and goal axes. It sets up the robot's position, orientation, and gripper
        angles, and adds them to the environment.

        Args:
            target_position (3D): Used to set the position of the end effector
                (gripper) at the beginning of the simulation.

        """
        self.env = swift.Swift()
        self.env.launch(realtime=True)
        # env = rtb.backends.PyPlot.PyPlot()
        # env.launch(realtime=True)

        # Create a KionovaGen3 robot object
        self.kinova = rtb.models.KinovaGen3()
        print(self.kinova.grippers)
        # Set joint angles to ready configuration
        # observation_angles = self.observation_angles[:7]
        # observation_angles[5] = observation_angles[5] + 2*np.pi
        # print(observation_angles)
        # self.kinova.q = observation_angles
        self.kinova.q = self.kinova.qr

        # Add the robot to the simulator
        self.env.add(self.kinova)
        # kinova = rtb.models.Panda()

        # axes
        self.goal_axes = sg.Axes(0.1)
        self.ee_axes = sg.Axes(0.1)

        # Add the axes to the environment
        self.env.add(self.ee_axes)
        self.env.add(self.goal_axes)
        self.ee_axes.T = self.kinova.fkine(self.kinova.q)

        # Number of joint in the Kinova which we are controlling
        self.n = 7

        # cup_position = list(p.getBasePositionAndOrientation(self.pybullet_cup)[0])
        # cup_position[0] = cup_position[0] + 0.3  # Added the robot base offset in pybullet

        # target_position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
        # target_position[0] = target_position[0] + 0.3  # Added the robot base offset in pybullet
        # target_position[2] = target_position[2] - 0.45  # Added the robot base offset in pybullet and the height of the cup
        # objects
        # self.cup = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(cup_position[0], cup_position[1], 0), color=(0, 0, 1))
        # self.cup = sg.Cylinder(0.036, 0.168, pose=sm.SE3.Trans(target_position[0], target_position[1], 0), color=(0, 1, 0))

        # self.cup = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(0.4, 0.4, 0), color=(0, 0, 1))
        # self.env.add(self.cup)

        # Set the desired end-effector pose
        self.rot = self.kinova.fkine(self.kinova.q).R
        self.rot = sm.SO3.OA([1, 0, 0], [0, 0, -1])
        # self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.26])
        # self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.60])
        self.Tep = sm.SE3.Rt(self.rot, [target_position[0], target_position[1], 0.60])
        self.Tep = sm.SE3.Rt(self.rot, [target_position[0], target_position[1], target_position[2]])
        # self.Tep = sm.SE3.Rt(self.rot, [0.4, 0.4, 0.13])  # green = x-axis, red = y-axis, blue = z-axis
        self.goal_axes.T = self.Tep

        self.arrived = False
        self.dt = 0.05

        self.env.step(0)
        time.sleep(5)

    def toolbox_compute(self, target_position):
        # The current pose of the kinova's end-effector
        """
        Performs computations for a specific robot worker, including calculating
        joint velocities and angles based on desired end-effector positions,
        computing the end-effector's position and orientation, and updating the
        robot's state.

        Args:
            target_position (3D): Given as a list containing the position of the
                end effector in meters.

        """
        self.Te = self.kinova.fkine(self.kinova.q)

        # cup_position = list(p.getBasePositionAndOrientation(self.pybullet_cup)[0])
        # cup_position[0] = cup_position[0] + 0.3  # Added the robot base offset in pybullet

        # target_position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
        # target_position[0] = target_position[0] + 0.3
        # target_position[2] = target_position[2] - 0.45

        # self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.60])  # green = x-axis, red = y-axis, blue = z-axis
        self.Tep = sm.SE3.Rt(self.rot, [target_position[0], target_position[1], target_position[2]])  # green = x-axis, red = y-axis, blue = z-axis
        # Transform from the end-effector to desired pose
        self.eTep = self.Te.inv() * self.Tep

        # Spatial error
        self.e = np.sum(np.abs(np.r_[self.eTep.t, self.eTep.rpy() * np.pi / 180]))

        # Calulate the required end-effector spatial velocity for the robot
        # to approach the goal. Gain is set to 1.0
        self.v, self.arrived = rtb.p_servo(self.Te, self.Tep, 1.0, threshold=0.01)

        # Gain term (lambda) for control minimisation
        self.Y = 0.01

        # Quadratic component of objective function
        self.Q = np.eye(self.n + 6)

        # Joint velocity component of Q
        self.Q[:self.n, :self.n] *= self.Y

        # Slack component of Q
        self.Q[self.n:, self.n:] = (1 / self.e) * np.eye(6)

        # The equality contraints
        self.Aeq = np.c_[self.kinova.jacobe(self.kinova.q), np.eye(6)]
        beq = self.v.reshape((6,))

        # The inequality constraints for joint limit avoidance
        self.Ain = np.zeros((self.n + 6, self.n + 6))
        self.bin = np.zeros(self.n + 6)

        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        self.ps = 0.05

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        self.pi = 0.9

        # Form the joint limit velocity damper
        self.Ain[:self.n, :self.n], self.bin[:self.n] = self.kinova.joint_velocity_damper(self.ps, self.pi, self.n)

        # Linear component of objective function: the manipulability Jacobian
        c = np.r_[-self.kinova.jacobm().reshape((self.n,)), np.zeros(6)]

        # The lower and upper bounds on the joint velocity and slack variable
        lim = np.deg2rad(10)
        self.qdlim = [lim, lim, lim, lim, lim, lim, lim]  # inventadas
        lb = -np.r_[self.qdlim, 10 * np.ones(6)]
        ub = np.r_[self.qdlim, 10 * np.ones(6)]

        # Solve for the joint velocities dq
        qd = qp.solve_qp(self.Q, c, self.Ain, self.bin, self.Aeq, beq, lb=lb, ub=ub, solver='piqp')

        # Apply the joint velocities to the kinova
        self.kinova.qd[:self.n] = qd[:self.n]
        self.target_velocities = qd[:self.n]

        joints_angle = []
        for i in range(7):
            joints_angle.append(p.getJointState(self.robot_id, i + 1)[0])

        error = np.sum(np.rad2deg(np.abs(np.array(joints_angle) - np.array(self.kinova.q))))
        # print("Error joints", np.rad2deg(self.kinova.q-joints_angle), "Error: ", error)
        if error > 1:
            self.kinova.q = joints_angle

        # print("/////////////////////////////////////////////////////////////////////////////////////////////////////")

        # Update the ee axes
        self.ee_axes.T = self.Te

        # Step the simulator by 50 ms
        self.env.step(self.dt)

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
    def read_camera_fixed(self):
        """
        Receives camera data from a PyBullet simulator and processes it to generate
        an RGB image that is displayed in a window using OpenCV.

        Returns:
            2D: A color image captured by a camera connected to PyBullet, along
            with the time stamp in milliseconds since the start of the program.

        """
        while True:
            # print("Getting the pose", time.time()*1000-self.timestamp)
            com_p, com_o, _, _, _, _ = p.getLinkState(self.robot_id, 9)
            # print("Pose obtained", time.time()*1000-self.timestamp)
            # Define camera intrinsic parameters
            width = 1280  # image width
            height =  720  # image height
            f_in_pixels = 1298 #1298
            near = 0.01  # near clipping plane
            far = 100  # far clipping plane

            # Optical center in pixel coordinates
            optical_center_x_pixels = 646.23 #620  # example x-coordinate in pixels
            optical_center_y_pixels = 267.62 #238  # example y-coordinate in pixels

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

            ## sum in one frame rgb and self.colorKinova[0][0]
            sum = cv2.addWeighted(rgb, 0.5, self.colorKinova[0][0], 0.5, 0)
            cv2.imshow("Pybullet", sum)
            # print("Showing Pybullet image", time.time() * 1000 - self.timestamp)
            cv2.waitKey(3)

            # print("Returning the image", time.time() * 1000 - self.timestamp)

            # print("Triying to put in queue", time.time()*1000 - self.timestamp)

            # self.pybulletImageQueue.put_nowait([rgb, time.time()*1000])

            # print("Pybullet image put in queue", time.time()*1000 - self.timestamp)

            # time.sleep(0.05)
            return rgb, time.time()*1000

    def readKinovaCamera(self):
        """
        Retrieves depth and color images from a Kinova camera, normalizes the depth
        image, and appends it to a list along with the timestamp of when the image
        was captured.

        Returns:
            OptionalTuplenumpyndarray: A tuple containing two elements, the first
            being an array of depth image data and the second being an array of
            image data with corresponding alive times.

        """
        try:
            both = self.camerargbdsimple_proxy.getAll("CameraRGBDViewer")
            depthImage = (np.frombuffer(both.depth.depth, dtype=np.int16)
                                .reshape(both.depth.height, both.depth.width))
            depthImage = cv2.normalize(src=depthImage, dst=None, alpha=0, beta=255,
                                             norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            self.depthKinova.append([depthImage, both.depth.alivetime])
            kinovaImage = (np.frombuffer(both.image.image, np.uint8)
                                .reshape(both.image.height, both.image.width, both.image.depth))
            self.colorKinova.append([kinovaImage, both.image.alivetime])

        except Ice.Exception as e:
            print(e)
        return True

    def showKinovaAngles(self):
        """
        Calculates and prints the angles of the Kinova robot's joints, the difference
        between the current angle and the corresponding PyBullet angle, and the
        gripper distance.

        """
        print("//--------------------------------------------------------------------------------------------------//")
        ext_angles = []
        diff_from_pybullet = []
        for i in range(7):
            ext_angles.append(self.ext_joints.joints[i].angle)
            diff_from_pybullet.append((math.degrees(p.getJointState(self.robot_id, i + 1)[0]) % 360) - self.ext_joints.joints[i].angle)
        print("Kinova angles", ext_angles)
        print("Diff from pybullet", diff_from_pybullet)
        print("Gripper distance", self.ext_gripper.distance)

    def movePybulletWithExternalVel(self):
        """
        Updates the target velocities of joints based on external velocity input,
        converts them to radians, and then sets the motor control targets for each
        joint using PyBullet's `setJointMotorControl2` method.

        """
        for i in range(len(self.ext_joints.joints)):
            self.target_velocities[i] = self.ext_joints.joints[i].velocity

        self.target_velocities = numpy.deg2rad(self.target_velocities) * 1.2

        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_id, i+1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])

    def movePybulletWithToolbox(self):
        # print("Pybullet move with toolbox init", time.time()*1000 - self.timestamp)
        """
        Controls the velocity of a robot using PyBullet, setting joint motor control
        parameters to reach target velocities for each joint.

        """
        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_id, i+1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])
        # print("Pybullet move with toolbox end", time.time()*1000 - self.timestamp)

    def readDataFromProxy(self):
        """
        Reads data from a proxy server and updates the values of class instance
        variables `ext_joints` and `ext_gripper`. It also scales down the value
        of the gripper's distance by a factor of 0.8 before sleeping for 0.05 seconds.

        """
        while True:
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()
            self.ext_gripper.distance = self.ext_gripper.distance * 0.8
            #print("ext_joints", self.ext_joints.joints)

            time.sleep(0.05)

    def moveKinovaWithAngles(self, angles):
        """
        Moves the Kinova arm based on angles provided as input. The angles are
        converted to degrees and then passed to the `moveJointsWithAngle` method
        of the `kinovaarm_proxy` object.

        Args:
            angles (ndarray): 2D, representing the angles to move the Kinova arm
                in degrees.

        """
        array = np.round(np.rad2deg(angles) % 360)
        self.angles.jointAngles = array.tolist()
        self.kinovaarm_proxy.moveJointsWithAngle(self.angles)

    def moveKinovaWithSpeeds(self):
        # print("Kinova move with speeds init", time.time()*1000 - self.timestamp)
        """
        Computes and sends joint speeds to the Kinova arm proxy for movement based
        on gains and error values calculated from joint angles.

        """
        self.joint_speeds = []

        for i in range(7):
            angle = p.getJointState(self.robot_id, i + 1)[0]
            error = (np.deg2rad(self.ext_joints.joints[i].angle)
                     - angle + math.pi) % (2 * math.pi) - math.pi

            speed = np.rad2deg(p.getJointState(self.robot_id, i + 1)[1]) * self.gains[i] - np.rad2deg(error) * 0.3

            # print("e", error)
            self.joint_speeds.append(speed)

        self.speeds.jointSpeeds = self.joint_speeds

        # print("Kinova move with speeds proxy action start", time.time()*1000 - self.timestamp)
        self.kinovaarm_proxy.moveJointsWithSpeed(self.speeds)
        # print("Kinova move with speeds end", time.time()*1000 - self.timestamp)

    def updateGains(self):
        """
        Updates the gains for the robot's joints based on the current position and
        speed, and calculates the new timestamp for the next sampling period.

        """
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
        """
        Receives and processes joystick input data for a specific robot, updating
        its position and orientation based on the input from the joystick.

        Args:
            data (dict): Passed the raw data from the joystick, which contains the
                axis values and button states.

        """
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
