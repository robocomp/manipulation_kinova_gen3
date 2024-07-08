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

# Set the LC_NUMERIC environment variable
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'
# RMEMEMBER to SET: export LC_NUMERIC="en_US.UTF-8" in your terminal

console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    """
    Acts as a bridge between a kinova arm and an external joystick, allowing the
    user to control the robot's movements and orientations through the joystick
    inputs. It translates the joystick data into movement commands for the kinova
    arm and also handles buttons presses for moving the robot to specific positions
    or orientations.

    Attributes:
        Period (instance): 50 milliseconds by default, indicating how often the
            worker will check for new tasks to execute.
        rgb (8element): Used to store the red, green, and blue values of the color
            of the robot's body.
        startup_check (lambda): Used to check if the robot arm should move to a
            specific position or orientation during startup. The lambda function
            takes in the joint angles and returns True if the arm should move,
            False otherwise.
        physicsClient (instance): Used to get physics client for controlling the
            robot movement
        plane (ndarray): 4D, representing the position and orientation of the
            robot's end effector in a 4D space.
        table_id (int): Used to store a unique identifier for each table in the
            worker's workspace. It is used to differentiate between tables in the
            worker's attention.
        robot_urdf (str): A file path to a Robot URDF file which contains information
            about the robot's geometry, joints, and limits.
        robot_launch_pos (list): Used to store the position of the robot at the
            time of launching the application, which is used for calculating the
            target position and orientation of the robot.
        robot_launch_orien (list): Defined as a list of Euler angles, representing
            the orientation of the robot at the time of launch.
        end_effector_link_index (int): Used to specify the index of the end effector
            link in the robot's kinematic chain.
        home_angles (ndarray): 7-dimensional, representing the angles of the robot's
            joints when it is at its home position.
        observation_angles (ndarray): 7-dimensional, representing the joint angles
            observed by the worker.
        observation_angles_2 (ndarray): 7-dimensional, representing the angles of
            the robot's joints as observed by the worker.
        observation_angles_3 (ndarray): 3D, representing the joint angles of the
            robot in radians as observed by the Kinova arm.
        observation_angles_cube (ndarray): 3-dimensional, representing the angles
            of the cube robot's joints as observations from a simulation or
            real-world data.
        robot_id (int): Used to identify a specific robot in a fleet of robots,
            allowing for more efficient control
            of the robots' movements and actions.
        pybullet_cup (attribute): Used to specify the name of a pybullet environment
            that the worker uses for its simulations.
        square (instance): 4 by default, which means that the worker can move to
            any position within a square shape with sides of length equal to the
            maximum distance it can move in each direction.
        hilo_lectura (instance): A function that sends data from a joystick to a
            HIL (Hardware-in-the-Loop) system.
        readDataFromProxy (method): Responsible for reading data from a proxy
            server, which is used to control a robot's movements based on user input.
        target_angles (7element): Used to store the target angles for each joint
            of the robot, which are calculated based on the input joystick data.
        target_position (list): Used to store the target position of the robot's
            end effector for each axis (X, Y, Z, gripper). The values are stored
            as floats in a list format.
        target_orientation (3x3): Used to store the desired orientation of the
            robot's end effector as a quaternion. It is updated based on the user
            input and is used to compute the robot's new position and orientation
            using the kinematic chain.
        target_velocities (ndarray): 7-dimensional, where each element represents
            the target velocity for one joint in radians per second. It is used
            to store the desired joint velocities for movement planning and control.
        joy_selected_joint (int): 0-based indexing of a joint in the robot's end
            effector, indicating which joint to control based on the joystick input.
        move_mode (int): 0 by default, which controls the move mode (joint movement
            or end-effector movement) based on the input joystick data. It can
            take on values of 0, 1, 2, 3, or 4 to switch between different move modes.
        n_rotations (int): 10 by default, indicating the number of rotational
            degrees to move the end effector for each joint.
        ext_joints (dict): Used to store the joint angles of a kinova arm for which
            the worker is responsible, as well as various gains and timestamps
            related to the joints.
        kinovaarm_proxy (ROBOT_PROXY): Used to interact with the Kinova robot arm
            through a Python interface, allowing the worker to move the robot's
            joints and perform other actions.
        ext_gripper (instance): 7-dimensional array that stores the gripper angles
            for the kinova arm, which are used to control the movement of the gripper.
        posesTimes (ndarray): Used to store the timestamp of each pose of the
            robot, used for calculating joint gains through the `updateGains()` method.
        poses (ndarray): 3D array containing the joint angles and positions of the
            robot for a specific timestamp.
        timestamp (int): Used to store the current timestamp in milliseconds since
            the robot's startup.
        timer (int): 0 by default, it counts the time spent in the move_mode method
            since it was last reset.
        compute (instance): Used to compute the robot's position, orientation, and
            joint angles based on the received data from the joystick adapter.
        timer2 (float): Used to represent the time it takes for the worker to
            perform its task after the move mode has been selected.
        movePybulletWithExternalVel (method): Responsible for moving the robot arm
            based on external velocities provided by the user.
        joint_speeds (ndarray): Used to store the joint speeds for movement with
            angles.
        gains (7element): Used to store the gains for each joint in the kinova
            arm. The gains are calculated based on the error between the desired
            angle and the actual angle, and they are updated in real-time during
            the movement.
        speeds (ndarray): Used to store the joint speeds of the kinova robot in
            radians per second. It's updated by calling the `updateGains()` method
            based on the joystick data received.
        angles (ndarray): Used to store the joint angles of the robot in radians,
            which can be modified using the `moveKinovaWithAngles()` method.
        timer3 (int): 3 by default, indicating that the worker will run for 3
            iterations or steps of its move mode loop.
        moveKinovaWithSpeeds (method): Responsible for moving the Kinova arm based
            on the joint speeds provided by the `updateGains` method. It takes in
            the `speeds` attribute, which contains the joint speeds, and moves the
            arm using the `kinovaarm_proxy` object.
        timer4 (float): 4 seconds by default, which is the time it takes for the
            worker to move the robotic arm to a specific position based on the
            input data from the joystick.
        movePybulletWithToolbox (instance): Used to move the pybullt arm with the
            tool box.
        colorKinova (attribute): Used to specify the color of the Kinova arm. It
            allows the user to customize the appearance of the robot.
        depthKinova (instance): Used to store information about a kinova arm's
            joint angles, velocities, and accelerations.
        calibrator (instance): Used to store the calibration data for the robotic
            arm, including the joint angles and speeds at
            different points in time.
        timer5 (float): Used to keep track of the time spent executing the
            `updateGains` method. It is used to update the gains of the robot's
            joints based on the error between the desired and actual angles.
        readKinovaCamera (method): Responsible for reading the camera data from
            the Kinova robot's onboard camera, which is then used to determine the
            position and orientation of the robot.
        timer6 (float): 6 seconds, which is the time interval between updates of
            the robot's joint angles and positions in the simulation.
        correctCupPosition (numpyarray): Used to store the correct position of the
            cup during the robot's movement, which is calculated based on the angle
            of each joint.
        timer7 (float): Used to keep track of the elapsed time since the start of
            the worker's move_mode period.
        showKinovaAngles (instance): Used to display the angles of the Kinova arm
            in degrees.
        gainsTimer (int): Used to keep track of the elapsed time since the last
            joint angle update, which is used to calculate the gains for each joint.
        updateGains (instance): Used to update the gains for each joint based on
            the error between the desired angle and the actual angle.
        aamed (instance): Used to store the absolute angle values of the joints
            of the robot in radians.
        flag (int): 4-bit field that stores a boolean flag indicating whether the
            worker is in move mode or not. It can take on values of 0, 1, 2, or 3
            to represent different modes of movement (e.g., move, home, etc.).

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes the SpecificWorker class by setting up timers, connecting
        signal handlers, and defining various variables needed for the robot's
        control and state estimation. It also sets the desired motion mode and
        home angles for the robot.

        Args:
            proxy_map (dict): Used to map the pybullet joints indices to the Kinova
                arm's joint names. It allows the worker to communicate with the
                kinova arm using the pybullet library, while still using the kinova
                arm's native joint names and angles.
            startup_check (int): 0 by default, which means that the worker will
                check for kinova specific code at startup. If set to 1, it skips
                the check and moves straight to the main loop.

        """
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50
        self.rgb = []
        if startup_check:
            self.startup_check()
        else:

            # Start PyBullet in GUI mode
            self.physicsClient = p.connect(p.GUI)
            # Set the path to PyBullet data
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.setGravity(0, 0, -9.81)
            #p.setGravity(0, 0, 0)

            p.setRealTimeSimulation(1)
            # p.setTimeStep(1 / 240)

            flags = p.URDF_USE_INERTIA_FROM_FILE
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35,
                                         cameraTargetPosition=[0, 0, 0.5])

            # load floor
            self.plane = p.loadURDF("plane.urdf")

            # Load a table to place the arm on
            self.table_id = p.loadURDF("/home/robolab/software/bullet3/data/table/table.urdf", basePosition=[0, 0, 0],
                                       baseOrientation=p.getQuaternionFromEuler([0, 0, 1.57]), flags=flags)

            # Load Kinova arm
            #self.robot = KinovaGen3()

            #////////////////////////////////////////////////////

            # Carga manual del brazo kinova

            # self.robot_urdf = "/home/robolab/robocomp_ws/src/robocomp/components/manipulation_kinova_gen3/pybullet_controller/kinova/kinova_with_pybullet/gen3_robotiq_2f_85.urdf"
            self.robot_urdf = "/home/robolab/robocomp_ws/src/robocomp/components/manipulation_kinova_gen3/pybullet_controller/kinova/kinova_with_pybullet/gen3_robotiq_2f_85-mod.urdf"
            self.robot_launch_pos = [-0.3, 0.0, 0.64]
            self.robot_launch_orien = p.getQuaternionFromEuler([0, 0, 0])
            self.end_effector_link_index = 12
            self.home_angles = [0, -0.34, np.pi, -2.54, 0, -0.87, np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.observation_angles = [0, 0, np.pi, -0.96, 0, -2.1, np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.observation_angles_2 = [0.87, 0.17, 4.01, -1.74, -6.80, -1.92, 3.22, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.observation_angles_3 = [0.26, 0.17, 3.52, -1.13, -6.406, -1.78, 2.18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.observation_angles_cube = [2.1753, 0.6980, 2.6924, -1.3950, -6.6862, -1.8915, 3.9945, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.robot_id = p.loadURDF(self.robot_urdf, self.robot_launch_pos, self.robot_launch_orien,
                                       flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

            for i in range(7):
                p.resetJointState(bodyUniqueId=self.robot_id, jointIndex=i+1,
                                  targetValue=self.home_angles[i], targetVelocity=0)


            for i in range(7):
                p.setJointMotorControl2(self.robot_id, i+1, p.POSITION_CONTROL, targetPosition=self.home_angles[i])

            #////////////////////////////////////////////////////

            # Load a cup to place on the table
            self.pybullet_cup = p.loadURDF("/home/robolab/software/bullet3/data/dinnerware/cup/cup_small.urdf", basePosition=[0.074, 0.20, 0.64], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)

            self.square = p.loadURDF("/home/robolab/software/bullet3/data/cube_small_square.urdf", basePosition=[0.074, 0.0, 0.64], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
            texture_path = "/home/robolab/Escritorio/textura_cubo.png"
            textureIdSquare = p.loadTexture(texture_path)
            p.changeVisualShape(self.square, -1, textureUniqueId=textureIdSquare)

            # # Load a cube to place on the table
            # self.cube = p.loadURDF("/home/robolab/software/bullet3/data/cube_small.urdf", basePosition=[0.074, 0.0, 0.64], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
            #
            # # Cargar la textura
            # texture_path = "/home/robolab/Escritorio/textura_cubo_v2.png"
            # textureId = p.loadTexture(texture_path)
            #
            # # Aplicar la textura al cubo
            # # Cambiar el visual shape del cubo
            # p.changeVisualShape(self.cube, -1, textureUniqueId=textureId)

            # Crear una restricción fija entre los dos modelos
            fixed_constraint = p.createConstraint(self.table_id, -1, self.robot_id, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0.3, 0.64], [0, 0, 0], childFrameOrientation=p.getQuaternionFromEuler([0, 0, 1.57]))

            self.hilo_lectura = threading.Thread(target=self.readDataFromProxy)
            self.hilo_lectura.start()

            # wait for half a second
            time.sleep(0.5)

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

            self.target_angles = self.home_angles
            self.target_position = p.getLinkState(self.robot_id, self.end_effector_link_index)[0]
            self.target_orientation = p.getLinkState(self.robot_id, self.end_effector_link_index)[1]
            self.target_velocities = [0.0] * 7
            self.joy_selected_joint = 0
            self.move_mode = 5
            self.n_rotations = np.zeros(7).tolist()
            self.n_rotations = [0, -1, 0, -1, -1, -1, 0]
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()

            self.posesTimes = np.array([int(time.time()*1000)])
            self.poses = []
            joints = []
            for i in range(7):
                actual_angle = (i, p.getJointState(self.robot_id, i + 1)[0])
                joints.append(actual_angle)
                self.poses.append(joints)

            self.timestamp = int(time.time()*1000)

            # self.initialize_toolbox()

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

            self.timer2 = QtCore.QTimer(self)
            self.timer2.timeout.connect(self.movePybulletWithExternalVel)
            #self.timer2.start(50)

            self.joint_speeds = []
            for i in range(7):
                self.joint_speeds.append(0)

            self.gains = np.ones(7).tolist()

            self.speeds = ifaces.RoboCompKinovaArm.TJointSpeeds()
            self.speeds.jointSpeeds = self.joint_speeds

            self.angles = ifaces.RoboCompKinovaArm.TJointAngles()

            self.angles.jointAngles = []

            self.timer3 = QtCore.QTimer(self)
            self.timer3.timeout.connect(self.moveKinovaWithSpeeds)
            # self.timer3.start(self.Period)

            self.timer4 = QtCore.QTimer(self)
            self.timer4.timeout.connect(self.movePybulletWithToolbox)
            # self.timer4.start(50)

            self.colorKinova = collections.deque(maxlen=5)
            self.depthKinova = collections.deque(maxlen=5)
            self.calibrator = calibrator.Calibrator()

            self.timer5 = QtCore.QTimer(self)
            self.timer5.timeout.connect(self.readKinovaCamera)
            self.timer5.start(self.Period)

            self.timer6 = QtCore.QTimer(self)
            self.timer6.timeout.connect(self.correctCupPosition)
            # self.timer6.start(500)

            self.timer7 = QtCore.QTimer(self)
            self.timer7.timeout.connect(self.showKinovaAngles)

            self.gainsTimer = QtCore.QTimer(self)
            self.gainsTimer.timeout.connect(self.updateGains)

            # Initialize the AAMED algorithm for the cup position correction
            self.aamed = pyAAMED(722//2, 1282//2)
            self.aamed.setParameters(3.1415926 / 3, 3.4, 0.77)

            print("SpecificWorker started")

            self.flag = True


    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        """
        Sets the parameters of an object of the `SpecificWorker` class, returning
        `True`.

        Args:
            params (object): Passed to set parameters of the class.

        Returns:
            True: Returned when the parameters are successfully set.

        """
        return True


    @QtCore.Slot()
    def compute(self):

        """
        Performs computations for the SpecificWorker, including correcting cup
        position and moving to observation angles using PyBullet. It also reads
        camera images and detects objects using YOLO.

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

                    #print("Joints: ", self.target_angles)
                    #print(self.n_rotations)

                    for i in range(8, len(self.target_angles)):
                        p.setJointMotorControl2(self.robot_id, i+1, p.POSITION_CONTROL, targetPosition=self.target_angles[i]) #Move the arm with phisics


                except Ice.Exception as e:
                    print(e)

            case 4:
                try:
                    # print("Toolbox compute init", time.time()*1000 - self.timestamp)
                    self.toolbox_compute()
                    self.correctCupPosition()
                    self.movePybulletWithToolbox()
                    self.moveKinovaWithSpeeds()

                    # print("Toolbox compute end", time.time()*1000 - self.timestamp)

                    self.posesTimes = np.append(self.posesTimes, int(time.time()*1000))

                    jointsState = []
                    for i in range(7):
                        state = p.getJointState(self.robot_id, i + 1)
                        angle_speed = (state[0], state[1])
                        jointsState.append(angle_speed)

                    self.poses.append(jointsState)

                    # pybulletImage, imageTime = self.read_camera_fixed()
                    # cv2.imshow("Pybullet", pybulletImage)
                    # cv2.waitKey(1)
                    # if self.arrived == True:
                    #     print("Arrived")
                    #     self.timer4.stop()
                    #     self.timer3.stop()
                    #     self.timer6.stop()
                    #     self.target_velocities = [0.0] * 7
                    #     self.move_mode = 8
                except Ice.Exception as e:
                    print(e)

            case 5:    #Move to observation angles
                print("Moving to observation angles", int(time.time()*1000) - self.timestamp)
                self.moveKinovaWithAngles(self.observation_angles[:7])
                for i in range(7):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL, targetPosition=self.observation_angles[i], maxVelocity=np.deg2rad(25))

                self.target_angles[13] = self.ext_gripper.distance
                self.target_angles[15] = - self.ext_gripper.distance
                self.target_angles[17] = self.ext_gripper.distance - 0.1

                self.target_angles[18] = self.ext_gripper.distance
                self.target_angles[20] = - self.ext_gripper.distance
                self.target_angles[22] = self.ext_gripper.distance - 0.1

                for i in range(8, len(self.target_angles)):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL,
                                            targetPosition=self.target_angles[i])  # Move the arm with phisics

                angles = []
                for i in range(7):
                    angles.append(p.getJointState(self.robot_id, i + 1)[0])

                error = np.sum(np.abs(np.array(angles) - np.array(self.observation_angles[:7])))

                pybulletImage, imageTime = self.read_camera_fixed()

                if error < 0.05:
                    print("Observation angles reached", int(time.time()*1000) - self.timestamp)
                    self.move_mode = 6
            case 6:
                # self.calibrator.calibrate3(self.robot_id, self.colorKinova)
                # self.calibrator.cube_test(self.robot_id, self.colorKinova.copy())
                # self.calibrator.square_test(self.robot_id, self.colorKinova.copy())

                # yolodetector = YoloDetector.YoloDetector()
                # results = yolodetector.detect(self.colorKinova)
                # yolodetector.plot(results)

                if self.correctCupPosition() > 5:
                    print("Correcting cup position")

                else:
                    print("Calibration finished")
                    self.move_mode = 7

            case 7:
                # self.showKinovaAngles()
                self.timer.stop()
                # self.calibrator.get_kinova_images(self.robot_id, self.kinovaarm_proxy, self.camerargbdsimple_proxy)
                # self.calibrator.calibrate4(self.robot_id)
                # self.move_mode = -1

                # self.moveKinovaWithAngles(self.home_angles[:7])
                #
                # for i in range(7):
                #     p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL, targetPosition=self.home_angles[i])

                # print("initilizing toolbox", time.time()*1000 - self.timestamp)
                self.initialize_toolbox()
                # print("toolbox initialized", time.time()*1000 - self.timestamp)
                # self.timer4.start(self.Period)
                # self.timer3.start(self.Period)
                # self.timer6.start(self.Period)
                self.timer7.start(500)
                self.gainsTimer.start(1000)

                print("Moving to fixed cup")
                self.move_mode = 4
                self.timer.start(self.Period)

            case 8:
                if self.flag == True:
                    cv2.imwrite("pybullet_image.png", self.colorKinova[0][0])
                    self.flag = False

                self.moveKinovaWithAngles(self.home_angles[:7])
                for i in range(7):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL, targetPosition=self.home_angles[i])


    # =============== Methods ==================

    def startup_check(self):
        """
        Tests various components and methods of the `ifaces` module, including
        `RoboCompKinovaArm`, `TGripper`, `TJoint`, `TJoints`, `RoboCompJoystickAdapter`,
        and `AxisParams`, `ButtonParams`, and `TData`.

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

    def correctCupPosition(self):
        # print("Init time", time.time()*1000 - self.timestamp)
        # aamed = pyAAMED(1080, 1940)
        # aamed.setParameters(3.1415926 / 3, 3.4, 0.77)

        # print("Get pybullet image", time.time()*1000 - self.timestamp)
        """
        Calculates the error between the positions of a cup and a Kinect RGB camera,
        and adjusts the position of the cup to minimize the error.

        Returns:
            float: Ether a positive error value indicating a mismatch between the
            estimated and actual positions of the cup or -1 if no keypoints were
            detected.

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

        position = list(p.getBasePositionAndOrientation(self.pybullet_cup)[0])
        position[0] = position[0] - 0.0005 * (resKinova[0][0] - resPybullet[0][0])
        position[1] = position[1] - 0.0005 * (resKinova[0][1] - resPybullet[0][1])
        p.resetBasePositionAndOrientation(self.pybullet_cup, tuple(position), p.getQuaternionFromEuler([0, 0, 0]))

        # print("Finish time", time.time()*1000 - self.timestamp)
        return error

    def initialize_toolbox(self):
        ## Launch the simulator Swift
        """
        Sets up the toolbox for a specific worker by initializing various objects,
        adding them to the environment, and defining the desired end-effector pose.

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

        cup_position = list(p.getBasePositionAndOrientation(self.pybullet_cup)[0])
        cup_position[0] = cup_position[0] + 0.3  # Added the robot base offset in pybullet

        # objects
        self.cup = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(cup_position[0], cup_position[1], 0), color=(0, 0, 1))
        # self.cup = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(0.4, 0.4, 0), color=(0, 0, 1))
        self.env.add(self.cup)

        # Set the desired end-effector pose
        self.rot = self.kinova.fkine(self.kinova.q).R
        self.rot = sm.SO3.OA([1, 0, 0], [0, 0, -1])
        self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.23])
        # self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.60])
        # self.Tep = sm.SE3.Rt(self.rot, [0.4, 0.4, 0.13])  # green = x-axis, red = y-axis, blue = z-axis
        self.goal_axes.T = self.Tep

        self.arrived = False
        self.dt = 0.05

        self.env.step(0)
        time.sleep(5)

    def toolbox_compute(self):
        # The current pose of the kinova's end-effector
        """
        Performs various computations related to the robot's end effector, including
        forward kinematics, Jacobian matrices, and joint velocity limiting. It
        also updates the robot's position and orientation based on the computed
        end effector pose.

        """
        self.Te = self.kinova.fkine(self.kinova.q)

        cup_position = list(p.getBasePositionAndOrientation(self.pybullet_cup)[0])
        cup_position[0] = cup_position[0] + 0.3  # Added the robot base offset in pybullet

        self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.26])  # green = x-axis, red = y-axis, blue = z-axis
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
        lim = np.deg2rad(20)
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
        # print("Getting the pose", time.time()*1000-self.timestamp)
        """
        Computes a fixed perspective projection matrix and a view matrix for a
        camera, given the intrinsic parameters of the camera and the position and
        rotation of the camera in the world. It then retrieves an image from the
        camera using the view matrix and returns the image.

        Returns:
            2D: A grayscale image of the camera's field of view obtained by applying
            a fixed perspective projection to the raw camera data and then resizing
            it to the desired size.

        """
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
        img = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)

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

        return rgb, time.time()*1000

    def readKinovaCamera(self):
        """
        Retrieves depth and color images from Kinova cameras, normalizes them, and
        stores them in instance variables `depthKinova` and `colorKinova`.

        Returns:
            Boolean: `True` if the operation was successful, otherwise it returns
            `False`.

        """
        try:
            both = self.camerargbdsimple_proxy.getAll("CameraRGBDViewer")
            # self.colorKinova.append(both.image)
            # self.depthKinova.append(both.depth)
            # print(both.image.alivetime)

            depthImage = (np.frombuffer(both.depth.depth, dtype=np.int16)
                                .reshape(both.depth.height, both.depth.width))
            depthImage = cv2.normalize(src=depthImage, dst=None, alpha=0, beta=255,
                                             norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            self.depthKinova.append([depthImage, both.depth.alivetime])

            kinovaImage = (np.frombuffer(both.image.image, np.uint8)
                                .reshape(both.image.height, both.image.width, both.image.depth))

            self.colorKinova.append([kinovaImage, both.image.alivetime])

            # cv2.imshow("ColorKinova", self.colorKinova[0][0])
            # cv2.imshow("DepthKinova", self.depthKinova)
            # cv2.waitKey(5)
        except Ice.Exception as e:
            print(e)
        return True

    def showKinovaAngles(self):
        """
        Displays the angles of a Kinova robot's joints and compares them to the
        corresponding values from PyBullet.

        """
        print("///////////////////////////////////////////////////////////////////////////////////////////////")
        ext_angles = []
        diff_from_pybullet = []
        for i in range(7):
            ext_angles.append(self.ext_joints.joints[i].angle)
            diff_from_pybullet.append((math.degrees(p.getJointState(self.robot_id, i + 1)[0]) % 360) - self.ext_joints.joints[i].angle)
        print("Kinova angles", ext_angles)
        print("Diff from pybullet", diff_from_pybullet)
        print("///////////////////////////////////////////////////////////////////////////////////////////////")


        # angles = []
        # for i in range(7):
        #     angles.append(p.getJointState(self.robot_id, i+1)[0])
        # print(np.rad2deg(angles)%360)

    def movePybulletWithExternalVel(self):
        """
        Controls the velocity of the robot's joints using PyBullet, taking into
        account external velocities provided by an external joint controller.

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
        Controls the velocity of a robot using PyBullet. It iterates over the
        robot's joints and sets their velocities to the target velocities provided
        in the function arguments.

        """
        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_id, i+1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])
        # print("Pybullet move with toolbox end", time.time()*1000 - self.timestamp)

    def readDataFromProxy(self):
        """
        Reads joint state and gripper state from a proxy server, scales down gripper
        distance by 80%, and sleeps for 0.05 seconds before repeating the process.

        """
        while True:
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()
            self.ext_gripper.distance = self.ext_gripper.distance * 0.8
            #print("ext_joints", self.ext_joints.joints)

            time.sleep(0.05)

    def moveKinovaWithAngles(self, angles):
        """
        Sets joint angles for a Kinova arm based on supplied angles in degrees,
        then calls the `moveJointsWithAngle` method of the `kinovaarm_proxy` object
        to move the arm.

        Args:
            angles (ndarray): 360-degree radian angles to move the Kinova arm.

        """
        array = np.round(np.rad2deg(angles) % 360)
        self.angles.jointAngles = array.tolist()
        self.kinovaarm_proxy.moveJointsWithAngle(self.angles)

    def moveKinovaWithSpeeds(self):
        # print("Kinova move with speeds init", time.time()*1000 - self.timestamp)
        """
        Calculates and applies joint speeds for a Kinova arm based on angles and
        gains, and moves the joints with the calculated speeds using the
        `moveJointsWithSpeed` method of the `kinovaarm_proxy` object.

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
        Updates the gains of the robot's joints based on the difference between
        the current pose and the desired position, and also updates the timestamp
        of the pose.

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
        Receives input data from a joystick and updates the position and orientation
        of a robot based on the received data. It handles different modes, axes,
        and buttons in the input data.

        Args:
            data (dict): Passed to the function when sending data from a joystick.
                The dict contains axis and button values as keys and corresponding
                values, which are then processed by the function according to their
                names and modes.

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
