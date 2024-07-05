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
    Manages the communication between a PyBullet environment and a Kinova arm,
    translating joint angles and button presses into actions for the arm to perform.
    It also updates the robot's position and orientation based on user inputs.

    Attributes:
        Period (float): 0 by default, indicating that the worker will run its
            function at a fixed rate (in this case, once every 100 milliseconds).
        rgb (ndarray): 3D array representing the red, green, and blue values of a
            color image or video frame.
        startup_check (int): 1 by default. It represents whether the worker has
            finished starting up and is ready to work.
        physicsClient (instance): Used to store a reference to the physics client
            that is used for communication with the robot's physics engine.
        plane (8dimensional): Used to store the position and orientation of the
            end effector in a reference frame, specifically the world coordinate
            system.
        table_id (int): Used to store a unique identifier for each table that the
            worker can handle, allowing the worker to manage multiple tables simultaneously.
        robot_urdf (obbOBB): Used to represent the robot's end effector in 3D
            space, which is essential for controlling the robot's movements.
        robot_launch_pos (8element): Used to store the position of the robot's
            launch pad when it is launched from a specific location on the ground.
        robot_launch_orien (npndarray): Used to store the orientation of the robot
            at the time of launch.
        end_effector_link_index (8bit): Used to specify the index of the end
            effector link (e.g., gripper) of a specific robot arm in a kinematic
            chain. It helps identify the correct end effector for joint-based
            control of the robot arm.
        home_angles (ndarray): 7-dimensional, representing the desired joint angles
            for each joint in the robot's home position.
        observation_angles (7dimensional): Used to store the joint angles observed
            by the worker's end effector during execution of a task.
        observation_angles_2 (ndarray): Used to store the angles of the robot's
            joints observed from the simulation.
        observation_angles_3 (7element): Used to store the angles of the robot's
            joints in the Kinova Arm, which are observed by the worker.
        observation_angles_cube (3D): Used to store the angles of the cube observed
            by the worker, which will be used to calculate the joint speeds.
        robot_id (int): Used to identify the specific worker robot, with each robot
            having a unique ID.
        pybullet_cup (str): A reference to a PyBullet cup object, which is used
            to handle interactions with the robot's end effector.
        square (ndarray): 4-dimensional, representing the speed of each joint in
            the robot's arm as a square vector.
        hilo_lectura (list): Used to store the joint speeds read from the HIL
            (Hardware-in-the-Loop) interface.
        readDataFromProxy (async): Used to read data from a Kinova robot proxy,
            which is a simulation of the real-world kinematics of a robot. It takes
            in data from the proxy and updates the target positions, orientations,
            and gains of the robot based on user inputs from the joystick.
        target_angles (7element): Used to store the target angles for each joint
            of the robot.
        target_position (3D): Used to store the target position of the robot's end
            effector in Python Robotics library.
        target_orientation (3x3): Used to store the desired orientation of the end
            effector (gripper) of the robot in quaternion form. It is used to
            calculate the orientation of the end effector based on the joystick inputs.
        target_velocities (ndarray): 7-dimensional, representing the target
            velocities for each joint of the robot in radians per second.
        joy_selected_joint (7element): Initialized to a random joint and updated
            based on the joystick inputs, representing the currently selected joint
            for movement.
        move_mode (int): 0, 1, 2, or 3, representing four different movement modes
            for the robot arm, including home positioning, joint angle control,
            gripper control, and mode switching.
        n_rotations (ndarray): Used to store the rotation angles of the end effector
            for each joint in a specific order.
        ext_joints (7element): Used to store the joint angles of a Kinova arm,
            which are used to compute the gains for the robot's motors.
        kinovaarm_proxy (pybulletPyBullet): Used to simulate the movement of a
            Kinova arm using the PyBullet library. It allows the worker to send
            movement commands to the Kinova arm and receive its current position
            and orientation.
        ext_gripper (8element): A gripper state, which represents the position and
            orientation of the robot's end effector (gripper) in 3D space.
        posesTimes (ndarray): 1D array storing the time stamps of the robot's poses
            in milliseconds since the start of the simulation.
        poses (7element): Used to store the current joint positions of the robot
            in radians.
        timestamp (int): Used to store the current timestamp of the worker, which
            is used to control the movement of the robot.
        timer (int): Used to keep track of the time spent working on a task,
            allowing for the calculation of the worker's speed.
        compute (instance): Used to compute the target position, orientation, and
            speed for each joint based on the user input data from a joystick or
            keyboard and mouse.
        timer2 (int): 2 seconds, indicating the time interval between executions
            of the `updateGains` method.
        movePybulletWithExternalVel (instance): Used to move a robot arm with an
            external velocity signal. It calls the `moveJointsWithSpeed` method
            of the `KinovaArm` proxy object with the external velocity signal as
            input.
        joint_speeds (ndarray): 7-element vector containing joint speeds (radians
            per second) for a Kinova arm, which are used to calculate the joint
            angles based on the input joystick data.
        gains (7element): Used to store the joint gains for each joint in the
            robot, which are calculated based on the user's input and the error
            between the desired and actual joint angles.
        speeds (npndarray): Used to store the joint speeds as a list of values in
            radians per second.
        angles (ndarray): 7-dimensional, where each element represents the angle
            of a joint in radians. It stores the current angles of the robot's
            joints based on the input from the joystick adapter.
        timer3 (int): 3, indicating that the worker is running on a machine with
            3 CPU cores.
        moveKinovaWithSpeeds (instance): Used to move the robot's joints with
            specific speeds. It takes in a list of joint speeds and moves the
            joints using the PyBullet API.
        timer4 (int): 4, indicating that this worker has a timer with a duration
            of 4 seconds.
        movePybulletWithToolbox (instance): Used to move a robot arm with joint
            speeds calculated by the toolbox and sent to the PyBullet environment.
            It handles the communication between the toolbox and PyBullet to execute
            the moves.
        colorKinova (str): Used to represent the color of the Kinova robot's end
            effector. It determines the color of the
            end effector in the simulation, which can be useful for visualization
            purposes or when
            communicating with other agents.
        depthKinova (ndarray): 1x7, representing the joint angles of a Kinova arm
            as a depth image.
        calibrator (instance): Used to store the calibration parameters for the
            robot's joints, such as gains and timestamps, which are calculated
            during the calibration process.
        timer5 (int): 5 seconds, which is the time interval between data updates
            from the joystick adapter.
        readKinovaCamera (method): Used to read the camera data from the Kinova
            robot's camera sensor. It takes no arguments and returns a list of
            tuples containing the camera image and its timestamp.
        timer6 (float): 6 seconds, which is the time interval between data updates
            from the joystick adapter.
        correctCupPosition (ndarray): 1-dimensional, indicating the correct position
            of the cup to be reached by the robot arm.
        timer7 (int): 7, indicating that the worker has 7 joints or degrees of freedom.
        showKinovaAngles (ndarray): 0 or 1, indicating whether to show the kinova
            angles or not when the move mode changes.
        aamed (attribute): Used to hold the joint speeds in radians per second for
            each joint of the robot. It is a list of 7 elements, where each element
            represents the speed of one of the joints of the robot.
        flag (int): 4 by default, which means that the worker can perform 4 different
            actions based on the input data.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Of SpecificWorker initializes various kinova arm variables, creates timers
        for movement and calibration, and sets up the PyBullet environment for the
        robot.

        Args:
            proxy_map (dict): Used to map Kinova arm joints names to PyBullet
                joints names. It allows the user to customize the mapping between
                the two frameworks.
            startup_check (int): Used to check if the startup sequence has been
                run before, avoiding unnecessary recomputation of the joints angles.

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
            self.home_angles = [0, -0.34, np.pi, -2.54, -6.28, -0.87, np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.observation_angles = [0, 0, np.pi, -0.96, -6.28, -2.1, np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
        Sets parameters for an instance of the `GenericWorker` class it belongs
        to, returning `True`.

        Args:
            params (object): Passed to set parameters.

        Returns:
            Boolean: True when successful and otherwise an error occurs.

        """
        return True


    @QtCore.Slot()
    def compute(self):

        """
        Calculates the angles for the robot to reach a specific position and
        orientation, based on the observation angles obtained from the camera feed.
        It also performs the necessary joint movements to achieve these angles and
        updates the target positions and velocities accordingly.

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
                    # print("Toolbox compute end", time.time()*1000 - self.timestamp)

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
                self.timer4.start(self.Period)
                # self.timer3.start(self.Period)
                # self.timer6.start(self.Period)
                self.timer7.start(500)

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
        Performs various tests on different components of the RoboCompKinovaArm
        and its adapters, including TPose, TGripper, TJoint, TJoints, AxisParams,
        ButtonParams, and TData.

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
        Calculates the position of a cup based on the error between the detected
        keypoints and the expected position, and updates the base position and
        orientation of the cup using PyBullet's API.

        Returns:
            float: The difference between the position of the cup and the keypoints
            detected by AAMED.

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
        Initializes the toolbox of a SpecificWorker, setting up the kinematic
        chain, adding objects to the environment, and defining the desired
        end-effector pose for task execution.

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
        # self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.23])
        self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.60])
        # self.Tep = sm.SE3.Rt(self.rot, [0.4, 0.4, 0.13])  # green = x-axis, red = y-axis, blue = z-axis
        self.goal_axes.T = self.Tep

        self.arrived = False
        self.dt = 0.05

        self.env.step(0)
        time.sleep(5)

    def toolbox_compute(self):
        # The current pose of the kinova's end-effector
        """
        Performs computations for a specific worker robot in a simulated environment.
        It updates the worker's position, orientation, and velocity based on its
        kinematics and dynamics, and computes joint angles to approach a target
        position and orientation.

        """
        self.Te = self.kinova.fkine(self.kinova.q)

        cup_position = list(p.getBasePositionAndOrientation(self.pybullet_cup)[0])
        cup_position[0] = cup_position[0] + 0.3  # Added the robot base offset in pybullet

        self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.60])  # green = x-axis, red = y-axis, blue = z-axis
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
            if i == 4:
                joints_angle[i] = joints_angle[i]
            if i == 5:
                joints_angle[i] = joints_angle[i]

        error = np.sum(np.rad2deg(np.abs(np.array(joints_angle) - np.array(np.rad2deg(self.kinova.q)))))
        print("Error joints", np.rad2deg(self.kinova.q-joints_angle), "Error: ", error)
        if error > 7:
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
        Generates a camera image using PyBullet, given its intrinsic parameters
        and the current position and rotation of the camera. It then displays the
        resulting image in an OpenGL window.

        Returns:
            3D: 4 dimensional numpy array that represents a RGB image of the camera
            frame.

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
        cv2.waitKey(5)

        # print("Returning the image", time.time() * 1000 - self.timestamp)

        return rgb, time.time()*1000

    def readKinovaCamera(self):
        """
        Retrieves depth and color images from Kinova cameras through a proxy,
        normalizes the depth image, and appends it to a list along with the
        corresponding alive time.

        Returns:
            OptionalTupleNParray: 2-element tuple containing a 3D numpy array of
            depth image and a list of alive time for each pixel, as well as a
            2-dimensional numpy array of color image and a list of alive time for
            each pixel.

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
        Displays kinova angles and differences between joint states from PyBullet
        and the SpecificWorker's `ext_joints` attribute.

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
        Sets the target velocities for joints defined by an external module, then
        converts those velocities to radians and multiplies them by a factor of
        1.2 before passing them to PyBullet's `setJointMotorControl2` method to
        control the robot's movement.

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
        Within the `SpecificWorker` class controls the velocities of joints
        associated with a robot identified by `robot_id` using PyBullet's
        `setJointMotorControl2` method, based on target velocities provided in an
        array.

        """
        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_id, i+1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])
        # print("Pybullet move with toolbox end", time.time()*1000 - self.timestamp)

    def readDataFromProxy(self):
        """
        Retrieves joint and gripper state data from a Kinova arm proxy, scales the
        gripper distance by 80%, and sleeps for 0.05 seconds before repeating the
        process.

        """
        while True:
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()
            self.ext_gripper.distance = self.ext_gripper.distance * 0.8
            #print("ext_joints", self.ext_joints.joints)

            time.sleep(0.05)

    def moveKinovaWithAngles(self, angles):
        """
        Converts angles to degrees and rounds them to an array, then passes the
        array to the `kinovaarm_proxy.moveJointsWithAngle` method to move the
        robot's joints based on the provided angles.

        Args:
            angles (ndarray): 2D array representing joint angles in radians.

        """
        array = np.round(np.rad2deg(angles) % 360)
        self.angles.jointAngles = array.tolist()
        self.kinovaarm_proxy.moveJointsWithAngle(self.angles)

    def moveKinovaWithSpeeds(self):
        # print("Kinova move with speeds init", time.time()*1000 - self.timestamp)
        """
        Moves the joints of a Kinova arm using predefined speeds, retrieved from
        the robot's state.

        """
        self.joint_speeds = []
        for i in range(7):
            speed = np.rad2deg(p.getJointState(self.robot_id, i + 1)[1]) # * self.gains[i]
            self.joint_speeds.append(speed)

        self.speeds.jointSpeeds = self.joint_speeds

        # print("Kinova move with speeds proxy action start", time.time()*1000 - self.timestamp)
        self.kinovaarm_proxy.moveJointsWithSpeed(self.speeds)
        # print("Kinova move with speeds end", time.time()*1000 - self.timestamp)

    def updateGains(self):
        """
        Updates gains for joints based on error between desired and actual angles,
        calculated using PyBullet and Kinova angles. It also updates the current
        timestamp and the poses array.

        """
        self.posesTimes = self.posesTimes - self.ext_joints.timestamp
        best_timestamp = np.abs(self.posesTimes).argmin()

        print("Best timestamp: ", best_timestamp, self.posesTimes[best_timestamp],
              self.ext_joints.timestamp)

        joints_state = self.poses[best_timestamp]
        for i in range(7):
            angle = joints_state[i][0]
            speed = joints_state[i][1]
            error = (np.deg2rad(self.ext_joints.joints[i].angle)
                     - angle + math.pi) % (2 * math.pi) - math.pi
            if abs(speed) > 0.01:
                self.gains[i] += error * 0.1
            print("Gains: joint ", i, self.gains[i], "Kinova angle: ",
                  np.deg2rad(self.ext_joints.joints[i].angle),
                  "Pybullet angle: ", self.target_angles[i], "Error:", error)
        # now draw the gains as timeseries in matplotlib
        self.posesTimes = np.array([int(time.time() * 1000)])
        self.poses = joints_state

        print("/////////////////////")
  
    # =============== Methods for SubscribesTo ================
    # =========================================================
    #
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    #
    def JoystickAdapter_sendData(self, data):
        """
        Receives data from a joystick and updates the position and orientation of
        a robot based on the received data. It also handles button presses and
        changes the move mode of the robot accordingly.

        Args:
            data (dict): Passed as an argument to the function, containing the
                data from the joystick input, including axis values and button states.

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
