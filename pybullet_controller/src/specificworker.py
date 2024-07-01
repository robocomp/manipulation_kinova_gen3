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
    Manages a robot's movements based on user inputs, controlling the robot's
    position and orientation using quaternions. It also implements move modes and
    homes for different axes.

    Attributes:
        Period (int): 4 by default, indicating that the worker will move in 4 modes
            (up, down, left, right) before returning to its home position.
        rgb (ndarray): 3D, containing the red, green, and blue values of the robot's
            target position as a tuple of floats.
        startup_check (int): 4 by default, indicating that the worker will check
            for startup commands every 4 seconds.
        physicsClient (instance): A Python `physics.client` client object, which
            provides an interface to interact with the physical world, such as
            moving robots or simulating physics simulations.
        plane (str): Used to store the position of the robot's end effector relative
            to a reference plane, which can be set using the `set_plane()` method.
        table_id (int): Used to identify a specific worker within a group of
            workers, each assigned to a specific table in a work cell.
        robot_urdf (obb): 3D object representation definition file that defines
            the robot's geometry, joints, and constraints.
        robot_launch_pos (3element): Used to store the launch position of the
            robot's gripper.
        robot_launch_orien (str): A reference to the current orientation of the
            robot in Euler angles format.
        end_effector_link_index (int): 0-based indexing for the end effector link
            of a robot, indicating which link of the robot's end effector it belongs
            to.
        home_angles (Tuple): Defined as `(pitch, roll, yaw)` representing the home
            angles of the robot's end effector.
        observation_angles (8element): Used to store the angles observed during
            the robot's movement.
        observation_angles_2 (3D): Used to store the orientation angles of the
            robot's end effector as observed by the worker.
        observation_angles_3 (3element): Used to store the angles of observation
            for the robot arm's end effector.
        observation_angles_cube (3D): Used to store the angles of the cube in the
            observation space. It contains the angles of the cube's faces relative
            to the base frame.
        robot_id (int): 4 by default, representing the ID of the robot being controlled.
        pybullet_cup (instance): 4D vector representing the cup position in the
            robot's workspace.
        square (str): 4-characters long, representing a square shape for the
            worker's workspace.
        hilo_lectura (str): 20 characters long. It contains the name of a specific
            worker, such as "Hilo".
        readDataFromProxy (async): Used to read data from a robot's proxy server
            through its API, which contains information such as the robot's current
            position, orientation, and buttons states.
        target_angles (tuple): 3D angles (in radians) representing the target
            orientation of the robot's end effector for each axis (X, Y, Z, and gripper).
        target_position (list): A list of tuples containing the desired position
            of the robot's end effector for each axis (X, Y, Z) and gripper. It
            stores the target positions for each mode.
        target_orientation (3D): Used to store the target orientation of the robot's
            end effector, which is calculated based on the position and orientation
            of the robot's joints.
        target_velocities (list): Defined as `self.target_velocities = [(0, 0,
            0)]`. It stores the target velocities for each joint in the robot's
            end effector.
        joy_selected_joint (str): 0-based indexed, representing the selected joint
            in the robot's arm, as determined by the user input from the joystick.
        move_mode (int): 0-based index of a mode, which can be "home", "move", or
            "gripper". It indicates the current mode of the robot.
        n_rotations (int): 4, indicating that there are 4 possible states or modes
            for the robot (move, home, etc.).
        ext_joints (list): A list of joints that are extended outside of their
            typical range of motion, allowing for greater freedom of movement.
        kinovaarm_proxy (object): Used to represent the proxy for communication
            with the Kinova arm controller.
        ext_gripper (str): 16 characters long, representing the external gripper
            position in degrees. It is used to control the movement of the robot's
            gripper arm.
        posesTimes (8element): A list of tuples containing the pose (position and
            orientation) of the robot at different times, along with the time
            elapsed since the last call to `do_work()`.
        poses (list): A collection of dictionaries, where each dictionary represents
            a pose (position and orientation) of the robot's end effector.
        timestamp (int): Representing the timestamp of when the worker was created,
            which can be used to track the lifetime of the worker.
        initialize_toolbox (instance): Used to initialize the toolbox of the worker
            with a specific set of tools.
        timer (int): 0 by default, indicating that the worker has not been running
            for any significant amount of time.
        compute (instance): A method that takes no arguments and returns itself,
            which is used to compute new positions for the robot's end effector
            based on the received data.
        timer2 (float): 0 by default, which represents the duration of a single
            move command in seconds. It is used to control the speed of movement
            of the robot.
        movePybulletWithExternalVel (OptionalCallable): Used to move the robot
            using PyBullet's `external_velocity` function, which allows for more
            precise control over the robot's movement.
        joint_speeds (ndarray): 5-dimensional, representing the joint speeds for
            each axis in the robot's end effector frame.
        gains (float): Used to control the speed and positioning accuracy of the
            robot's joints during movement.
        speeds (list): Used to store the speed values for each axis of movement,
            with a maximum value of 10.
        angles (list): Used to store the angles (in degrees) for each axis of
            movement, home position, and gripper orientation.
        timer3 (float): 0 by default, which represents the time taken to complete
            the task in the third mode (mode 3) of the robot.
        moveKinovaWithSpeeds (callable): Used to move the Kinova robot with specified
            speeds for each axis, allowing for precise control over the robot's movements.
        timer4 (int): 4, indicating that the worker has a timer with a resolution
            of 1 second and a maximum value of 4.
        movePybulletWithToolbox (Callable): Used to move the robot using PyBullet
            and Toolbox. It takes no arguments and returns a tuple containing the
            new position and orientation of the robot's end effector.
        colorKinova (str): 32-bit floating point color values, used to represent
            the color of the robot's body.
        depthKinova (int): 0-based index of the worker's depth kinematic chain,
            used for calculating joint angles.
        calibrator (instance): A method for calibrating the robot's joint angles
            to match the desired positions and orientations of the end effector.
        timer5 (int): 5 by default, which represents the timer value for move mode
            5.
        readKinovaCamera (Callable): A method that reads the camera data from the
            Kinova robot's sensors.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Of the SpecificWorker class initializes the necessary kinematic and dynamic
        variables for the Kinova arm, including joint angles, velocities,
        accelerations, and time stamps. It also sets up timers for moving the
        pybullet end effector and the Kinova arm with external velocities and
        toolbox speeds, respectively.

        Args:
            proxy_map (dict): Used to map the PyBullet joint names to the Kinova
                arm joint names. It allows the user to define a mapping between
                the two joint systems, which can be useful when working with
                different robotic arms.
            startup_check (int): 0 by default, which means that the `RoboCompKinovaArm`
                class will initialize itself with the default values for the Kinova
                arm. If you want to customize the initialization process, you can
                pass a non-zero value as the `startup_check` parameter to override
                the default behavior.

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

            self.initialize_toolbox()

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
        Sets parameters for an instance of the `SpecificWorker` class, which
        inherits from `GenericWorker`. The function returns `True` without providing
        any additional information or exceptions.

        Args:
            params (object): Used to set parameters for an instance of a class.

        Returns:
            True: The default return value for a function with no specified return
            statement.

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
        Computes the desired joint angles for the robot to move the cup to the
        correct position based on the observation angles, and moves the robot using
        the `moveKinovaWithAngles` method.

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
                    # self.correctCupPosition()
                    if self.arrived == True:
                        print("Arrived")
                        self.timer4.stop()
                        self.timer3.stop()
                        self.move_mode = 8
                except Ice.Exception as e:
                    print(e)

            case 5:    #Move to observation angles
                print("Moving to observation angles")
                self.moveKinovaWithAngles(self.observation_angles[:7])
                for i in range(7):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL, targetPosition=self.observation_angles[i])

                self.target_angles[13] = self.ext_gripper.distance
                self.target_angles[15] = - self.ext_gripper.distance
                self.target_angles[17] = self.ext_gripper.distance - 0.1

                self.target_angles[18] = self.ext_gripper.distance
                self.target_angles[20] = - self.ext_gripper.distance
                self.target_angles[22] = self.ext_gripper.distance - 0.1

                for i in range(8, len(self.target_angles)):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL,
                                            targetPosition=self.target_angles[i])  # Move the arm with phisics

                pybulletImage = self.read_camera_fixed()
                cv2.imshow("Pybullet", pybulletImage)

                print("/////////////////////////////////////////////////////////////////////////////")

                # angles = []
                # for i in range(7):
                #     angles.append(p.getJointState(self.robot_id, i + 1)[0])
                #
                # print(angles, self.target_angles[:7])
                # error = np.sum(np.abs(np.array(angles)-self.target_angles[:7]))

                if self.timestamp+1000 < int(time.time()*1000):
                # if error < 0.1:
                    self.move_mode = 6
            case 6:
                # self.timer.stop()

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
                    # print(p.getBasePositionAndOrientation(self.pybullet_cup)[0])
                    # print("Suposed final position: 0.074, 0.20, 0.64")

                    self.move_mode = 7

                    # self.timer.start(self.Period)

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

                self.initialize_toolbox()
                self.timer4.start(self.Period)
                self.timer3.start(self.Period)

                print("Moving to fixed cup")
                self.move_mode = 4
                self.timer.start(self.Period)

            case 8:
                self.moveKinovaWithAngles(self.home_angles[:7])
                for i in range(7):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL, targetPosition=self.home_angles[i])

        #p.stepSimulation()
        # pass

    # =============== Methods ==================

    def startup_check(self):
        """
        Performs various checks related to the RoboCompKinovaArm and its components,
        including testing their properties and functionality, and scheduling the
        QApplication to quit after a 200ms delay.

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
        """
        Compares the positions and orientations of a kinova robot and a pybullet
        robot, using the AAMED algorithm to determine the difference between them.
        It then adjusts the position of the pybullet cup based on the error found.

        Returns:
            float: The magnitude of the difference between the predicted position
            of the cup and the actual position obtained from both kinova and
            pybullet images.

        """
        aamed = pyAAMED(1080, 1940)
        aamed.setParameters(3.1415926 / 3, 3.4, 0.77)
        imgGKinova = cv2.cvtColor(self.colorKinova, cv2.COLOR_BGR2GRAY)
        resKinova = aamed.run_AAMED(imgGKinova)
        # aamed.drawAAMED(imgGKinova)
        # if len(resKinova) > 0:
        #     cv2.circle(imgGKinova, (round(resKinova[0][1]), round(resKinova[0][0])), 8, (0, 0, 255), -1)
        #     cv2.imshow("test kinova", imgGKinova)

        pybulletImage = self.read_camera_fixed()
        imgGPybullet = cv2.cvtColor(pybulletImage, cv2.COLOR_BGR2GRAY)
        resPybullet = aamed.run_AAMED(imgGPybullet)
        # aamed.drawAAMED(imgGPybullet)
        # if len(resPybullet) > 0:
        #     cv2.circle(imgGPybullet, (round(resPybullet[0][1]), round(resPybullet[0][0])), 8, (0, 0, 255), -1)
        #     cv2.imshow("test pybullet", imgGPybullet)

        error = np.abs(resKinova[0][1] - resPybullet[0][1] + resKinova[0][0] - resPybullet[0][0])

        print("x: ", resKinova[0][1] - resPybullet[0][1], " y: ", resKinova[0][0] - resPybullet[0][0])

        position = list(p.getBasePositionAndOrientation(self.pybullet_cup)[0])
        position[0] = position[0] - 0.0005 * (resKinova[0][0] - resPybullet[0][0])
        position[1] = position[1] - 0.0005 * (resKinova[0][1] - resPybullet[0][1])
        p.resetBasePositionAndOrientation(self.pybullet_cup, tuple(position), p.getQuaternionFromEuler([0, 0, 0]))

        return error

    def initialize_toolbox(self):
        ## Launch the simulator Swift
        """
        Sets up a simulation environment, defines a kinematic worker and a cylindrical
        object, and defines the desired end-effector pose for the worker to reach.

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
        self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.15])
        # self.Tep = sm.SE3.Rt(self.rot, [0.4, 0.4, 0.13])  # green = x-axis, red = y-axis, blue = z-axis
        self.goal_axes.T = self.Tep

        self.arrived = False
        self.dt = 0.05

        self.env.step(0)
        time.sleep(5)

    def toolbox_compute(self):
        # The current pose of the kinova's end-effector
        """
        Computes the joint velocities for a robot based on its current state and
        desired end-effector pose, using a feedback control strategy. It updates
        the joint angles and computes an error signal to be used in the control loop.

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
                joints_angle[i] = joints_angle[i]
            if i == 5:
                joints_angle[i] = joints_angle[i]

        print("Error joints", self.kinova.q-joints_angle)
        print("/////////////////////////////////////////////////////////////////////////////////////////////////////")

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
        Defines camera intrinsic parameters, computes a projection matrix and a
        view matrix using the quaternion representation of the camera orientation,
        and then retrieves a camera image from the simulation environment.

        Returns:
            3D: A numpy array with shape (height, width, 4).

        """
        com_p, com_o, _, _, _, _ = p.getLinkState(self.robot_id, 9)
        # print("Pose obtained", time.time()*1000-self.timestamp)
        # Define camera intrinsic parameters
        width = 1280  # image width
        height = 720  # image height
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
        rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # print("Returning the image", time.time() * 1000 - self.timestamp)

        return rgb

    def readKinovaCamera(self):
        """
        Retrieves RGB and depth images from a Kinova camera, normalizes the depth
        image, and assigns it to instance variables `colorKinova` and `depthKinova`.
        It also catches and prints any exceptions raised by the Ice.Exception module.

        Returns:
            True: 1-dimensional numpy array containing the depth and color images
            captured by the Kinova camera.

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

            # cv2.imshow("ColorKinova", self.colorKinova)
            # cv2.imshow("DepthKinova", self.depthKinova)
            # cv2.waitKey(self.Period)
        except Ice.Exception as e:
            print(e)
        return True

    def showKinovaAngles(self):
        """
        Computes and prints the angles of the kinova joints in radians and degrees,
        using the `np.deg2rad()` and `np.rad2deg()` functions.

        """
        ext_angles = []
        for i in range(7):
            ext_angles.append(self.ext_joints.joints[i].angle)
        print(np.deg2rad(ext_angles))

        # angles = []
        # for i in range(7):
        #     angles.append(p.getJointState(self.robot_id, i+1)[0])
        # print(np.rad2deg(angles)%360)

    def movePybulletWithExternalVel(self):
        """
        Controls the velocity of joints in a robot using external velocities
        provided by an external library, PyBullet.

        """
        for i in range(len(self.ext_joints.joints)):
            self.target_velocities[i] = self.ext_joints.joints[i].velocity

        self.target_velocities = numpy.deg2rad(self.target_velocities) * 1.2

        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_id, i+1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])

    def movePybulletWithToolbox(self):
        """
        Sets joint motor control targets for a robot with index `robot_id` based
        on the target velocities provided in a list.

        """
        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_id, i+1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])

    def readDataFromProxy(self):
        """
        Reads data from a proxy object, specifically the joints state and gripper
        state, and modifies the gripper distance by multiplying it by 0.8. It also
        sleeps for 0.05 seconds before continuing to read data.

        """
        while True:
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()
            self.ext_gripper.distance = self.ext_gripper.distance * 0.8
            #print("ext_joints", self.ext_joints.joints)

            time.sleep(0.05)

    def moveKinovaWithAngles(self, angles):
        """
        Converts angles into degrees, rounds them to the nearest degree, and passes
        them to the `moveJointsWithAngle` method of the `kinovaarm_proxy` object.

        Args:
            angles (ndarray): 2D, containing angles in radians for each joint of
                the kinova arm.

        """
        array = np.round(np.rad2deg(angles) % 360)
        self.angles.jointAngles = array.tolist()
        self.kinovaarm_proxy.moveJointsWithAngle(self.angles)

    def moveKinovaWithSpeeds(self):
        """
        Computes joint speeds for a Kinova arm based on user-defined gains and
        moves the joints with those speeds using the `kinovaarm_proxy` object.

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
        Updates the gains of a robot's joints based on the current pose and
        timestamp, and prints the current gains and error values to console.

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
        a robot based on the axis values and button presses. It also sends the
        current move mode and target positions and orientations to the robot's controller.

        Args:
            data (dict): Passed with the joystick data from the input device, which
                contains axis values for different joints and buttons.

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
