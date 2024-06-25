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
    Manages a robot's movements based on data from sensors and buttons. It updates
    the robot's position and orientation using quaternion math and moves to the
    target position when the mode is "move". It also handles home commands for
    both the main robot and a secondary robot.

    Attributes:
        Period (int): 3 by default, which indicates that the worker will run for
            3 consecutive cycles before stopping and resetting its state.
        rgb (3element): Used to store the RGB values for the robot's LED lights,
            which can be set or retrieved through the `set_rgb` method.
        startup_check (int): 4 by default, which represents the startup check code
            for the robot's axis movement and gripper movement.
        physicsClient (instance): Used to interact with a physics engine client,
            allowing for simulated physics-based interactions between objects.
        plane (str): A plane name, indicating which plane the worker operates on
            (e.g., "X-Y", "YZ", etc.).
        table_id (int): 4.
        robot_urdf (obb): 3D robot model's URDF (Universal Robot Description Format)
            file, which defines the geometry and kinematics of the robot.
        robot_launch_pos (list): A tuple of the robot's launch position (X, Y, Z
            coordinates).
        robot_launch_orien (Tuple): 4-element list representing the quaternion
            orientation of the robot's end effector in world coordinates for
            launching the gripper.
        end_effector_link_index (int): 0-based indexing for the end effector links
            of a robot, indicating the index of the link in the robot's chain.
        home_angles (Tuple): Used to store the angles of the robot's joints when
            it is at its home position. It is used to compare with the angles read
            from the robot's encoders during movement to determine if the robot
            has reached its home position.
        observation_angles (numpyndarray): 3D angles representing the orientation
            of the worker's end effector, which are observed by the robot during
            its motion.
        observation_angles_2 (3dimensional): Used to store the current angle
            observation of the second robot arm in degrees. It is used to compute
            the desired position and orientation of the end effector based on the
            observation angles.
        observation_angles_3 (3D): A list of 3 values representing the angles for
            the robot's gripper, X, Y, Z axes.
        observation_angles_cube (3D): Used to store the observation angles of the
            robot for each axis (X, Y, Z) in a cube format.
        robot_id (int): 4 in this example, indicating that the robot being controlled
            is the fourth robot in a sequence of robots to be controlled by the worker.
        cup (instance): 0 by default, which means that the worker does not have a
            cup.
        square (int): 4 by default, indicating that the worker can move in 4
            directions (up, down, left, and right).
        hilo_lectura (float): 0.01, which represents the incremental change in
            position or orientation of the robot's end effector when a button is
            pressed.
        readDataFromProxy (method): Used to read data from a proxy server. It takes
            no arguments and returns a dictionary containing the data read from
            the proxy server.
        hilo_camara (str): 14 characters long. It seems to be a camera ID or name,
            possibly for a specific robot or device.
        readCamera (instance): Not described in the code snippet provided. Therefore,
            I cannot provide a concise answer to this question based on the given
            code.
        target_angles (ndarray): 4D, representing the target angles for each axis
            (X, Y, Z, and gripper) as a list of floats in degrees.
        target_position (list): Used to store the target position of the robot's
            end effector for each axis (X, Y, Z, and gripper). It contains the
            desired position of the end effector in a specific coordinate system.
        target_orientation (3D): Used to store the target orientation of the robot
            after a move command has been executed. It is updated with the new
            orientation value based on the `axis` attribute of the `data` list
            received as input.
        target_velocities (list): A list of tuples containing the target velocity
            for each axis (X, Y, Z) and the gripper. The targets are the desired
            velocities of the robot in the specified axes and gripper movement.
        joy_selected_joint (int): 0-based indexing of the selected joint to control
            the robot's movement based on the Joy input device.
        move_mode (int): 0-based indexing for different move modes (e.g., home,
            move, or reset). It stores the current move mode selected by the user.
        n_rotations (int): 3, indicating that the worker has 3 rotational degrees
            of freedom (axes).
        ext_joints (list): A list of joints that are external to the robot, such
            as those on a gripper or end effector.
        kinovaarm_proxy (object): Used to store a proxy for interacting with the
            KINOVA robot's API. It is set to the result of calling the `KinovaArmProxy()`
            function, which creates a Python interface for the KINOVA robot's API.
        ext_gripper (str): 2D position of the gripper in the external reference
            frame, which is used to control the movement of the gripper.
        posesTimes (ndarray): 3D array containing the robot's pose (position and
            orientation) at different time steps, with each element representing
            a specific moment in time.
        poses (list): 3D positions of the robot's end effector over time, used for
            positioning and orienting the robot's end effector to perform specific
            tasks.
        timestamp (int): Used to keep track of the worker's current state,
            representing the number of milliseconds that have passed since the
            worker was created.
        inicializar_toolbox (method): Responsible for initializing the `ToolBox`
            module of the PyRobot library, which provides tools to work with the
            robot's end effector and other features.
        timer (int): 0 by default, it represents the time (in seconds) after which
            the worker will stop working if no new task is received.
        compute (lambda): Defined as a function that takes a `data` argument, which
            is a `dict` object containing data from the Robot Operating System
            (ROS) message, and returns a value based on the data. The exact output
            of the `compute` attribute depends on the implementation of the class,
            but it is generally used to compute some value or perform some action
            based on the received data.
        timer2 (float): 2 seconds by default, which represents the time it takes
            for the worker to perform its task after the move mode has been triggered.
        movePybulletWithExternalVel (methods): Responsible for moving the robot
            based on external velocity commands.
        joint_speeds (ndarray): 4D, representing the joint speeds as (joint name,
            axis, position, speed) tuples for each joint in the robot. It is used
            to specify the desired joint positions and speeds during movement.
        gains (float): Representing the gains (or sensitivity) of each axis of the
            robot's end effector, which determines how much the robot moves when
            a button is pressed.
        speeds (list): 0-dimensional, representing a list of target positions for
            each axis of the robot, which are used to control the robot's movement.
        angles (list): Used to store the angles of the robot's axes in radians.
        timer3 (float): 3 seconds by default, representing the time it takes for
            the worker to perform its actions.
        moveKinovaWithSpeeds (float): 3-dimensional array that contains the target
            positions for the Kinova robot's end effectors, along with their speeds,
            in the x, y, and z axes.
        timer4 (int): 4, indicating that this worker has a timer for move mode 4.
        movePybulletWithToolbox (Callable): A method that moves the robot based
            on data from the PyBullet API and the Toolbox API, updating the target
            position and orientation of the robot.
        colorKinova (str): 4 characters long, representing the color of the Kinova
            gripper.
        depthKinova (int): 0-based indexing of the depth sensor of a Kinova robot
            arm, indicating the position of the end effector in the robot's workspace.
        calibrator (instance): Used to store a reference to the calibration data
            for the specific worker. It is used to read the calibration data from
            the file and to update the target position and orientation based on
            the calibration data.
        timer5 (int): 5 by default, representing the time (in seconds) for which
            the worker will stay in move mode before returning to home position.
        readKinovaCamera (Callable): Used to read data from a Kinova camera. It
            takes no arguments and returns a tuple containing the camera's data.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Sets up the necessary components for the SpecificWorker to operate, including
        a Kinova arm proxy, a timer for controlling the movement of the robot, and
        various variables for storing information about the robot's position,
        orientation, velocities, and more.

        Args:
            proxy_map (dict): Used to define a mapping between the robot's joint
                angles and the corresponding joint speeds for each joint. This
                mapping is used to compute the joint speeds that will move the
                robot to a desired position and orientation.
            startup_check (int): Used to check if the kinova arm is properly
                connected and calibrated before starting the worker thread.

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

            p.setRealTimeSimulation(10)

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
            # self.home_angles = [0, 5.93, 3.14, 3.73, -6.28, 5.41, 1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            #                     0.0,
            #                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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
            self.cup = p.loadURDF("/home/robolab/software/bullet3/data/dinnerware/cup/cup_small.urdf", basePosition=[0.074, -0.20, 0.64], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)

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

            self.hilo_camara = threading.Thread(target=self.readCamera)
            # self.hilo_camara.start()

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

            for i in range(7):
                # if i % 2 == 0:
                #     p.changeDynamics(self.robot_id, i, localInertiaDiagonal=[val, val, val/2])
                # else:
                #     p.changeDynamics(self.robot_id, i, localInertiaDiagonal=[val, val/4, val])
                #
                # print("Kinova",i,p.getDynamicsInfo(self.robot_id, i))
                # print("Guille",i,p.getDynamicsInfo(self.robot2, i))
                pass

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

            self.inicializar_toolbox()

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
            #self.timer3.start(self.Period)

            self.timer4 = QtCore.QTimer(self)
            self.timer4.timeout.connect(self.movePybulletWithToolbox)
            #self.timer4.start(50)

            self.colorKinova = []
            self.depthKinova = []
            self.calibrator = calibrator.Calibrator()

            self.timer5 = QtCore.QTimer(self)
            self.timer5.timeout.connect(self.readKinovaCamera)
            self.timer5.start(self.Period)

            print("SpecificWorker started")


    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        """
        Sets parameters for an instance of `SpecificWorker`. It returns `True`
        upon successful completion, indicating that the parameter setting process
        was successful.

        Args:
            params (object): Used to set parameters for the function.

        Returns:
            Boolean: True if successful, otherwise it raises an exception.

        """
        return True


    @QtCore.Slot()
    def compute(self):

        # print("Errors:",
        #       [round(x - y, 2) for x, y in
        #        zip(self.robot.get_actual_control_joints_velocity(), self.target_velocities[:7])], self.target_velocities)

        # print("Errors:",
        #       [round(x - y, 2) for x, y in
        #        zip(self.robot.get_actual_control_joints_angle(), self.target_angles[:7])],
        #       self.target_angles[:7])

        # self.robot.move_joints_control_vel( joint_param_value=self.target_velocities,
        #                                     desired_force_per_one_list=[1],
        #                                     desired_vel_per_one_list=[1],
        #                                     wait=True,
        #                                     counter_max=10 ** 2,
        #                                     error_threshold=0.005)

        """
        Performs tasks related to observing angles, moving Kinova with angles, and
        calibrating the robot's end effector. It also handles observation angles
        and square test for Yolo detector.

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
                    self.toolbox_compute()
                except Ice.Exception as e:
                    print(e)

            case 5:    #Move to observation angles
                print("Moving to observation angles")
                self.moveKinovaWithAngles(self.observation_angles_cube[:7])
                for i in range(7):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL, targetPosition=self.observation_angles_cube[i])

                self.target_angles[13] = self.ext_gripper.distance
                self.target_angles[15] = - self.ext_gripper.distance
                self.target_angles[17] = self.ext_gripper.distance - 0.1

                self.target_angles[18] = self.ext_gripper.distance
                self.target_angles[20] = - self.ext_gripper.distance
                self.target_angles[22] = self.ext_gripper.distance - 0.1

                for i in range(8, len(self.target_angles)):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL,
                                            targetPosition=self.target_angles[i])  # Move the arm with phisics

                if self.timestamp+10000 < int(time.time()*1000):
                    self.move_mode = 6
            case 6:
                self.timer.stop()
                # self.calibrator.calibrate(self.colorKinova, self.robot_id)

                # self.calibrator.calibrate3(self.robot_id, self.colorKinova)
                # self.calibrator.cube_test(self.robot_id, self.colorKinova.copy())
                self.calibrator.square_test(self.robot_id, self.colorKinova.copy())

                # self.calibrator.prueba(self.robot_id)
                # corners = self.calibrator.detect_green_corners(self.colorKinova)
                # print(corners)
                # for corner in corners:
                #     cv2.circle(self.colorKinova, corner, 8, (255, 0, 0), -1)
                #
                # cv2.imshow("Color", self.colorKinova)
                # self.calibrator.calibrate2(self.colorKinova, self.robot_id)

                yolodetector = YoloDetector.YoloDetector()
                results = yolodetector.detect(self.colorKinova)
                yolodetector.plot(results)

                aamed = pyAAMED(1080, 1940)
                aamed.setParameters(3.1415926 / 3, 3.4, 0.77)
                imgG = cv2.cvtColor(self.colorKinova, cv2.COLOR_BGR2GRAY)
                res = aamed.run_AAMED(imgG)
                print(res)
                aamed.drawAAMED(imgG)
                print("Observing")
                self.move_mode = -1
                self.timer.start(self.Period)

            case 7:
                # self.showKinovaAngles()
                self.timer.stop()
                # self.calibrator.get_kinova_images(self.robot_id, self.kinovaarm_proxy, self.camerargbdsimple_proxy)
                self.calibrator.calibrate4(self.robot_id)
                self.move_mode = -1

                self.moveKinovaWithAngles(self.home_angles[:7])
                self.timer.start(self.Period)
                pass

        #p.stepSimulation()
        # pass

    # =============== Methods ==================

    def startup_check(self):
        """
        Tests various components and interfaces of a RoboCompKinovaArm, including
        TPose, TGripper, TJoint, TJoints, AxisParams, ButtonParams, and TData. It
        also sets a timer to quit the application after 200 milliseconds.

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

    def inicializar_toolbox(self):
        ## Launch the simulator Swift
        """
        Initializes the worker's toolbox by creating an environment, adding grippers
        and objects, and setting up axes and goal points. It also defines the
        worker's arrival point and time step size.

        """
        self.env = swift.Swift()
        self.env.launch(realtime=True)
        # env = rtb.backends.PyPlot.PyPlot()
        # env.launch(realtime=True)

        # Create a KionovaGen3 robot object
        self.kinova = rtb.models.KinovaGen3()
        print(self.kinova.grippers)
        # Set joint angles to ready configuration
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

        # objects
        self.cup = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(0.4, 0.4, 0), color=(0, 0, 1))
        self.env.add(self.cup)

        # Set the desired end-effector pose
        self.rot = self.kinova.fkine(self.kinova.q).R
        self.rot = sm.SO3.OA([-1, 0, 0], [0, 0, -1])
        self.Tep = sm.SE3.Rt(self.rot, [0.4, 0.4, 0.13])  # green = x-axis, red = y-axis, blue = z-axis
        self.goal_axes.T = self.Tep

        self.arrived = False
        self.dt = 0.05

        self.env.step(0)
        time.sleep(5)

    def toolbox_compute(self):
        # The current pose of the kinova's end-effector
        """
        Computes and updates the joint states of a robot based on its target
        velocities, while also handling the limits of the joints' angles.

        """
        self.Te = self.kinova.fkine(self.kinova.q)

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
                joints_angle[i] = joints_angle[i] + np.pi
            if i == 5:
                joints_angle[i] = joints_angle[i] + np.pi/2

        print("Error joints", self.kinova.q-joints_angle)
        print("/////////////////////////////////////////////////////////////////////////////////////////////////////")

        # Update the ee axes
        self.ee_axes.T = self.Te

        # Step the simulator by 50 ms
        self.env.step(self.dt)

    def readCamera(self):
        """
        Computes and updates the view matrix, projection matrix, and camera vector
        based on the robot's position and orientation. It then captures an image
        using the Segmentation Mask Object and displays it in a window.

        """
        while True:
            fov, aspect, nearplane, farplane = 60, 1.78, 0.01, 100
            projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
            com_p, com_o, _, _, _, _ = p.getLinkState(self.robot_id, 9)
            rot_matrix = p.getMatrixFromQuaternion(com_o)
            rot_matrix = np.array(rot_matrix).reshape(3, 3)
            # Initial vectors
            init_camera_vector = (0, 0, 1)  # z-axis
            init_up_vector = (0, 1, 0)  # y-axis
            # Rotated vectors
            camera_vector = rot_matrix.dot(init_camera_vector)
            up_vector = rot_matrix.dot(init_up_vector)
            view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
            img = p.getCameraImage(1280, 720, view_matrix, projection_matrix, renderer=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
            self.rgb = img[2]
            self.rgb = cv2.rotate(self.rgb, cv2.ROTATE_180)   # Get the RGB image
            #print(self.rgb.shape)
            self.rgb = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR)
            cv2.imshow('img', self.rgb)
            time.sleep(0.5)

    def readKinovaCamera(self):
        """
        Retrieves and processes RGBD data from a Kinova camera, including depth
        normalization and display in two separate images using OpenCV.

        Returns:
            True: 1-dimensional numpy array representing a boolean value indicating
            whether the operation was successful or not.

        """
        try:
            both = self.camerargbdsimple_proxy.getAll("CameraRGBDViewer")
            self.colorKinova = both.image
            self.depthKinova = both.depth

            self.depthKinova = (np.frombuffer(self.depthKinova.depth, dtype=np.int16)
                                .reshape(self.depthKinova.height, self.depthKinova.width))

            self.depthKinova = cv2.normalize(src=self.depthKinova, dst=None, alpha=0, beta=255,
                                             norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            self.colorKinova = (np.frombuffer(self.colorKinova.image, np.uint8)
                                .reshape(self.colorKinova.height, self.colorKinova.width, self.colorKinova.depth))

            cv2.imshow("ColorKinova", self.colorKinova)
            cv2.imshow("DepthKinova", self.depthKinova)
            cv2.waitKey(self.Period)
        except Ice.Exception as e:
            print(e)
        return True

    def showKinovaAngles(self):
        """
        Calculates and prints the radian values of the joints of a Kinova robot.

        """
        ext_angles = []
        for i in range(7):
            ext_angles.append(self.ext_joints.joints[i].angle)
        print(np.deg2rad(ext_angles))

        # angles = []
        # for i in range(7):
        #     angles.append(p.getJointState(self.robot_id, i+1)[0])
        # print(np.rad2deg(angles)%360)

    def get_kinova_instrinsic(self):
        """
        Computes camera calibration for a Kinova manipulator using a Chessboard
        pattern. It captures images, detects chessboard corners, and computes the
        intrinsic parameters of the camera using the Extended Kalman Filter (EKF)
        algorithm. The resulting camera matrix and distortion coefficients are
        saved to a file.

        Returns:
            bool: 1 if the camera calibration was successful and 0 otherwise.

        """
        try:
            # Parameters
            # TODO : Read from file
            n_row = 5
            n_col = 7
            n_min_img = 10  # img needed for calibration
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # termination criteria
            corner_accuracy = (11, 11)
            result_file = "./calibration.yaml"

            # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(n_row-1,n_col-1,0)
            objp = np.zeros((n_row * n_col, 3), np.float32)
            objp[:, :2] = np.mgrid[0:n_row, 0:n_col].T.reshape(-1, 2)

            # Intialize camera and window
            # camera = cv2.VideoCapture(0)  # Supposed to be the only camera
            # if not camera.isOpened():
            #     print("Camera not found!")
            #     quit()
            # width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            # height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))

            width = self.colorKinova.shape[1]
            height = self.colorKinova.shape[0]
            cv2.namedWindow("Calibration")

            # Usage
            def usage():
                """
                Provides a brief message and four options for user interaction:
                taking a picture, computing calibration, resetting the program,
                or quitting.

                """
                print("Press on displayed window : \n")
                print("[space]     : take picture")
                print("[c]         : compute calibration")
                print("[r]         : reset program")
                print("[ESC]    : quit")

            usage()
            initialization = True

            while True:
                if initialization:
                    print("Initialize data structures ..")
                    objpoints = []  # 3d point in real world space
                    imgpoints = []  # 2d points in image plane.
                    n_img = 0
                    initialization = False
                    tot_error = 0

                # Read from camera and display on windows
                img = self.colorKinova
                cv2.imshow("Calibration", img)

                # Wait for instruction
                k = cv2.waitKey(50)

                # SPACE pressed to take picture
                if k % 256 == 32:
                    print("Adding image for calibration...")
                    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                    # Find the chess board corners
                    ret, corners = cv2.findChessboardCorners(imgGray, (n_row, n_col), None)

                    # If found, add object points, image points (after refining them)
                    if not ret:
                        print("Cannot found Chessboard corners!")

                    else:
                        print("Chessboard corners successfully found.")
                        objpoints.append(objp)
                        n_img += 1
                        corners2 = cv2.cornerSubPix(imgGray, corners, corner_accuracy, (-1, -1), criteria)
                        imgpoints.append(corners2)

                        # Draw and display the corners
                        imgAugmnt = cv2.drawChessboardCorners(img, (n_row, n_col), corners2, ret)
                        cv2.imshow('Calibration', imgAugmnt)
                        cv2.waitKey(500)

                        # "c" pressed to compute calibration
                elif k % 256 == 99:
                    if n_img <= n_min_img:
                        print("Only ", n_img, " captured, ", " at least ", n_min_img, " images are needed")

                    else:
                        print("Computing calibration ...")
                        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (width, height), None,
                                                                           None)

                        if not ret:
                            print("Cannot compute calibration!")

                        else:
                            print("Camera calibration successfully computed")
                            # Compute reprojection errors
                            for i in range(len(objpoints)):
                                imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                                tot_error += error
                            print("Camera matrix: ", mtx)
                            print("Distortion coeffs: ", dist)
                            print("Total error: ", tot_error)
                            print("Mean error: ", np.mean(error))

                            # Saving calibration matrix
                            print("Saving camera matrix .. in ", result_file)
                            data = {"camera_matrix": mtx.tolist(), "dist_coeff": dist.tolist()}
                            with open(result_file, "w") as f:
                                yaml.dump(data, f, default_flow_style=False)

                # ESC pressed to quit
                elif k % 256 == 27:
                    print("Escape hit, closing...")
                    cv2.destroyAllWindows()
                    break
                # "r" pressed to reset
                elif k % 256 == 114:
                    print("Reset program...")
                    initialization = True

        except Ice.Exception as e:
            print(e)
        return True

    def movePybulletWithExternalVel(self):
        """
        Calculates and sets the target velocities for joints based on external
        joint velocities, then applies velocity control to move the robot's end
        effector along a predefined path.

        """
        for i in range(len(self.ext_joints.joints)):
            self.target_velocities[i] = self.ext_joints.joints[i].velocity

        self.target_velocities = numpy.deg2rad(self.target_velocities) * 1.2

        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_id, i+1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])

    def movePybulletWithToolbox(self):
        """
        Sets joint motor control velocity for a robot with toolbox.

        """
        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_id, i+1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])

    def readDataFromProxy(self):
        """
        Continuously retrieves joints state and gripper state data from an external
        proxy, scales the gripper distance by 0.8, and sleeps for 0.05 seconds
        between updates.

        """
        while True:
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()
            self.ext_gripper.distance = self.ext_gripper.distance * 0.8
            #print("ext_joints", self.ext_joints.joints)

            time.sleep(0.05)

    def moveKinovaWithAngles(self, angles):
        """
        Moves the kinova arm based on angles specified by the user, rounding the
        angle values to the nearest degree and passing them to the `moveJointsWithAngle`
        method of the kinova arm proxy.

        Args:
            angles (ndarray): Rounded to the nearest degree (in radians) between
                0 and 360 before being assigned to a list.

        """
        array = np.round(np.rad2deg(angles) % 360)
        self.angles.jointAngles = array.tolist()
        self.kinovaarm_proxy.moveJointsWithAngle(self.angles)

    def moveKinovaWithSpeeds(self):
        """
        Generates joint speeds for a Kinova arm based on joint angles and gains,
        and sends the speeds to the Kinova arm controller for movement.

        """
        self.joint_speeds = []
        for i in range(7):
            speed = np.rad2deg(p.getJointState(self.robot_id, i + 1)[1]) * self.gains[i]
            self.joint_speeds.append(speed)

        self.speeds.jointSpeeds = self.joint_speeds
        #print(self.gains)
        self.kinovaarm_proxy.moveJointsWithSpeed(self.speeds)

    def updateGains(self):
        """
        Updates the gains of the robot's joints based on the difference between
        the actual positions and target angles, and prints relevant information
        for error correction.

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
        Manages data from a joystick and sends it to the robot's control system,
        updating the robot's position and orientation based on the received data.

        Args:
            data (dict): Passed as an instance of PySerial's serial.Serial(). It
                contains the joystick data, including axes and buttons.

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

















 #self.set_joints_positions([x for x in range(22)], np.zeros(22).tolist())
        #self.print_joints_positions()

        # Get the state of the link, including its orientation
        #end_effector_link_state = p.getLinkState(self.robot_id, self.end_effector_link_index)
        #link_world_position = end_effector_link_state[0]  # World position of the link's frame
        #link_world_orientation = end_effector_link_state[1]  # World orientation of the link's frame (quaternion)

        # Calculate the inverse kinematics solution (joint positions)
        #target_position = link_world_position + np.array([0.0, -0.1, -0.1])  # Target position 10 cm above the current position
        #target_position[0] = 0.3

        #target_position = [0, -0.3, 1]
        #joint_positions = p.calculateInverseKinematics(self.robot_id,
        #                                               self.end_effector_link_index,
        #                                               target_position)
        # for joint_index, joint_position in enumerate(joint_positions):
        #     p.setJointMotorControl2(self.robot_id, joint_index, p.POSITION_CONTROL, targetPosition=joint_position,
        #                             force=1000)
        #
        # self.print_joints_positions([x for x in range(7)])
        #time.sleep(0.01)