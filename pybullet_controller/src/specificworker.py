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
    Manages the movement of a robotic arm based on the received data from a sensor
    and controls the arm's position, orientation, and gripper mode using Python's
    `robot` module. It also keeps track of the move mode and home positions.

    Attributes:
        Period (float|int): 4 seconds by default, indicating how often the worker
            will check for new commands from the robot.
        rgb (Tuple[float,float,float]): Used to represent the current orientation
            of the robot's end effector in terms of its position on the XY plane.
        timestamp (float|int): Used to store the current time at which the worker
            is running, allowing for efficient execution of tasks based on time intervals.
        startup_check (bool): Used to track whether the worker has been started
            or not. It's initial value is `False`, and it's updated to `True` after
            the worker has been started, and back to `False` when the worker is
            stopped or reset.
        physicsClient (Tuple[float,float,float]): Used to store the position and
            orientation of a robot's end effector in 3D space.
        plane (Tuple|List[float]): Used to store the target position of the robot's
            end effector in 3D space, which can be moved along the X, Y, or Z axes
            using the `move_mode` attribute.
        table_id (int|str): Used to identify a specific worker's position in a
            table of workers, allowing for efficient handling of multiple workers
            with different positions.
        robot_urdf (Tuple[float,float,float]): A reference to the robot's URDF
            (Unified Robot Description Format) definition. It contains the position
            and orientation of the robot in 3D space.
        robot_launch_pos (Tuple[float,float,float]): Defined as the position of
            the robot's launch point.
        robot_launch_orien (Tuple[float,float,float]): Used to store the orientation
            of the robot's end effector relative to its base.
        end_effector_link_index (int|str): 0-based index of the end effector link
            in the robot's kinematic chain, which represents the position of the
            gripper or other tool in the robot's workspace.
        robot_id (int|str): 4 digits long, representing the unique identifier of
            the robot in a specific worker's profile.
        home_angles (Tuple[float,float,float]): Defined as a set of angles for
            each axis (X, Y, Z) to home, which are default angles that the robot
            will move to when the "home" button is pressed.
        observation_angles (Tuple[float,float,float]): 3 angles representing the
            orientation of the robot's end effector relative to its base, used for
            calculating the target position and orientation of the end effector
            based on user inputs.
        table_center (Tuple[float,float,float]): Used to store the center coordinates
            of a table that the worker moves around.
        cylinderId (int|str): Used to identify a specific cylinder for picking and
            placing objects.
        threadKinovaAngles (Tuple[float,float,float]): Used to store the current
            angles of the robot's kinematic chain in degrees.
        readDataFromProxy (Tuple[int,List[str]]): Used to read data from a proxy
            server. It takes two arguments: the first is the protocol version,
            which can be either 1, 2, or 3, and the second is a list of strings
            representing the data from the proxy server. The attribute then parses
            the data based on the protocol version and returns the parsed data as
            a tuple of integers and a list of strigns.
        colorKinova (str|int): Set to a color value representing the robot's color,
            specifically "red", "blue", or "green".
        depthKinova (int|str): 4D depth values from Kinova robot's end effector
            to the workspace origin in meter units.
        target_angles (Tuple[float]): Used to store the target angles for each
            axis of movement (X, Y, Z) and gripper, which are obtained from the
            robot's end effector position data.
        target_position (List[float]): Used to store the target positions of the
            robot's end effector for the X, Y, and Z axes, as well as the gripper
            position.
        target_orientation (List[float]): Used to store the target orientation of
            the robot's end effector, which is represented as a quaternion in the
            code. The quaternion represents the orientation of the end effector
            relative to the world coordinate system.
        target_velocities (Tuple[float]): Defined as a list of target velocities
            for each axis (X, Y, Z) and the gripper. The values in the list represent
            the desired velocity of the robot at each position and orientation.
        joy_selected_joint (Tuple[float]): 0-based indexing for the selected joint
            (axis) from the Joy device, where each element corresponds to a different
            axis.
        move_mode (int|str): Used to store the current mode of the worker's movement,
            which can be one of five options: "home", "forward", "backward", "left",
            or "right".
        ext_joints (Tuple[float]): A list of joint angles for the robot's end
            effector, which are used to control the movement of the robot's arm.
        kinovaarm_proxy (Tuple[float,float,float]): A reference to the robot's end
            effector position in 3D space.
        ext_gripper (Union[float,int]): A variable for gripper position in case 4
            of the switch statement. It represents the movement of the gripper in
            the X, Y, or Z axis.
        timer (float|int): Used to store the elapsed time since the last call to
            `start_work`.
        compute (function|NoneType): Used to compute the new target position or
            orientation based on the received data packet from the robot. The
            function takes into account the current position, orientation, and
            other parameters such as move mode, home, and axis values, and updates
            the target position or orientation accordingly.
        joint_speeds (List[float]): Used to store the joint speeds for each axis,
            which are calculated based on the target position and orientation of
            the robot.
        speeds (float|int): A list of speeds for each axis (X, Y, Z) and the
            gripper, which represent the desired speed of the robot's joints and
            the gripper during movement.
        angles (List[float]): Used to store the angles of the robot's joints in radians.
        gains (float|List[float]): Used to store the gains for each axis, which
            are used to control the movement of the robot's end effector.
        posesTimes (Tuple[float,]): A list of pairs of position, orientation, and
            time stamps for each joint of the robot in a specific worker. It
            contains the current position and orientation of the robot's end
            effector at different time steps during the task execution.
        poses (Tuple[float,float,float,float]): A list of robot positions in a
            format compatible with the Robot Operating System (ROS) kinematic tree.
            It stores the robot's current position and orientation in a form that
            can be used to calculate the robot's end effector pose relative to its
            base link.
        calibrator (Union[int,str]): Used to store the current calibration mode
            of the worker, which can be either 0, 1, 2, 3, or 4.
        cameraKinovaTimer (float|int): 0 by default, representing the timer for
            the camera's kinematic movements.
        readKinovaCamera (Callable[[],Any]): Called when the Kinova camera is read.
            It takes no arguments and returns any data read from the camera.
        showKinovaStateTimer (int|bool): 0 by default, indicating that the worker
            does not show the Kinova state timer.
        showKinovaAngles (bool): 1 when the worker wants to display the angles of
            the robot's joints, and 0 otherwise.
        gainsTimer (int|float): 0 by default, which represents the time it takes
            for a worker to gain their target position after a move command has
            been executed.
        updateGains (Callable[Tuple[float],None]): Used to update the gains of the
            robot's joints based on the current position and orientation of the robot.
        contactPointTimer (float|int): Used to store the time taken for a specific
            worker to contact a target point, which can be used to calculate the
            move mode and home position.
        detectContactPoints (float|List[float]): Used to store the contact points
            detected by the worker's end effector during its motion.
        aamed (str|int): Used to store the name of the current move mode or home
            position in a robot. It can take on values such as "move_mode" or
            "home", indicating the different modes of the robot's movement or its
            default home position, respectively.
        pybullet_offset (Tuple[float,float,float]): Used to define the offset of
            the robot's end effector from its initial position in world coordinates.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes variables and timers to communicate with the Kinova robot arm,
        retrieve joint angles and velocities, and perform calibration and visualization
        tasks.

        Args:
            proxy_map (Dict[str, int]): Used to map the joint names of the Kinova
                arm to their corresponding indices in the PyBullet robot simulation.
                It allows to customize the mapping between the joints of the
                real-world Kinova arm and the virtual joints in PyBullet, which
                is useful when the real-world arm has a different number of joints
                or different joint names than the PyBullet robot.
            startup_check (bool): Used to check if the robot's end effector is
                within the desired workspace during startup. It is set to `True`
                by default, which will perform the check. If set to `False`, the
                check will be skipped, and the robot will proceed with its initialization.

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
            self.table_center = [0.374, 0.0, 0.28]

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

            self.contactPointTimer = QtCore.QTimer(self)
            self.contactPointTimer.timeout.connect(self.detectContactPoints)
            self.contactPointTimer.start(1000)

            # Initialize the AAMED algorithm for the cup position correction
            self.aamed = pyAAMED(722//2, 1282//2)
            self.aamed.setParameters(3.1415926 / 3, 3.4, 0.77)

            self.pybullet_offset = [-0.3, 0.0, 0.6]

            num_joints = p.getNumJoints(self.robot_id)

            # Iterar sobre todos los enlaces y obtener su información
            for joint_index in range(num_joints):
                joint_info = p.getJointInfo(self.robot_id, joint_index)
                link_name = joint_info[12].decode("utf-8")  # Nombre del enlace
                parent_link_index = joint_info[16]  # Índice del enlace padre
                link_state = p.getLinkState(self.robot_id, joint_index)

                print(f"Link {joint_index}:")
                print(f"  Name: {link_name}")
                print(f"  Parent Link Index: {parent_link_index}")
                print(f"  Link State: {link_state}")

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
        Sets parameters for further processing of data.

        Args:
            params (ParamType.any): Passed in to update the parameters of an object.

        Returns:
            bool: 1 if the parameters are set successfully, otherwise it returns
            a value of False.

        """
        return True


    @QtCore.Slot()
    def compute(self):

        """
        Computes the joint angles to reach a target position and orientation of
        the robot's end effector, taking into account the robot's dynamics and
        constraints. It also handles the communication with the PyBullet and Kinova
        APIs.

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

                self.changePybulletGripper(0.0)
                self.kinovaarm_proxy.closeGripper(0.0)

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
                target_position[2] = target_position[2] - 0.44

                print("initilizing toolbox", time.time()*1000 - self.timestamp)
                self.initialize_toolbox(target_position)
                print("toolbox initialized", time.time()*1000 - self.timestamp)
                # self.showKinovaStateTimer.start(1000)
                # self.gainsTimer.start(1000)

                print("Moving to fixed cup")
                self.move_mode = 7
                self.timer.start(self.Period)
                self.loopCounter = 0

            case 7:
                try:
                    error = self.correctTargetPosition()
                    if error != -1:
                        self.last_error = error
                    else:
                        self.loopCounter += 1

                    # print("Correct cup position end", time.time()*1000 - self.timestamp)

                    target_position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
                    target_position[0] = target_position[0] - self.pybullet_offset[0]
                    target_position[2] = target_position[2] - self.pybullet_offset[2] + 0.16

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

                    if self.last_error > 20 and (self.loopCounter > 20 or gripper_position[2] - target_position[2] < 0.25):
                        print("Adjusting pose for target adjustment")
                        self.target_position = list(p.getLinkState(self.robot_id, self.end_effector_link_index)[0])
                        self.target_position[0] = self.target_position[0] - self.pybullet_offset[0]
                        self.target_position[2] = self.target_position[2] - self.pybullet_offset[2] + 0.20
                        self.move_mode = 8

                    if gripper_position[2] - target_position[2] < 0.1 or self.loopCounter > 20:
                        print("Working without visual feedback", " Last error: ", self.last_error)
                        self.move_mode = 9
                except Ice.Exception as e:
                    print(e)

            case 8:
                try:
                    self.arrived = False

                    self.toolbox_compute(self.target_position)
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

                    print("Arrived: ", self.arrived)

                    if self.arrived == True:
                        self.loopCounter = 0
                        self.target_velocities = [0.0] * 7
                        self.movePybulletWithToolbox()
                        self.moveKinovaWithSpeeds()
                        self.last_error = 0
                        self.move_mode = 7

                except Ice.Exception as e:
                    print(e)

            case 9:
                try:
                    if not self.arrived:
                        target_position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
                        target_position[0] = target_position[0] - self.pybullet_offset[0]
                        target_position[2] = target_position[2] - self.pybullet_offset[2] + 0.16
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
                        self.changePybulletGripper(0.26)
                        self.kinovaarm_proxy.closeGripper(0.26)
                        self.move_mode = 10
                        self.target_position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
                        self.target_position[0] = self.target_position[0] - self.pybullet_offset[0]
                        self.target_position[2] = self.target_position[2] - self.pybullet_offset[2] + 0.25

                except Ice.Exception as e:
                    print(e)

            case 10:
                try:
                    self.arrived = False

                    self.toolbox_compute(self.target_position)
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
                        self.move_mode = 11

                except Ice.Exception as e:
                    print(e)

            case 11:
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
                        self.target_velocities = [0.0] * 7
                        self.movePybulletWithToolbox()
                        self.moveKinovaWithSpeeds()
                        self.changePybulletGripper(0.0)
                        self.kinovaarm_proxy.closeGripper(0.0)
                        self.move_mode = 12

                except Ice.Exception as e:
                    print(e)

            case 12:
                print("Reseting pose")
                self.timer.stop()
                self.moveKinovaWithAngles(self.observation_angles[:7])
                for i in range(7):
                    p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL,
                                            targetPosition=self.observation_angles[i],
                                            maxVelocity=np.deg2rad(25))

                angles = []
                for i in range(7):
                    angles.append(p.getJointState(self.robot_id, i + 1)[0])

                error = np.sum(np.abs(np.array(angles) - np.array(self.observation_angles[:7])))

                time.sleep(0.1)
                self.timer.start(self.Period)

                if error < 0.05:
                    self.move_mode = 7
                    self.loopCounter = 0





    # =============== Methods ==================

    def startup_check(self):
        """
        Tests various components and interfaces of the RoboCompKinovaArm library,
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

    def changePybulletGripper(self, distance):
        """
        Modifies the target angles of a robot's grippers to reach a desired distance,
        using PyBullet's `setJointMotorControl2` function.

        Args:
            distance (float): Used to modify the angles of the gripper's joints
                for proper gripping of an object.

        """
        self.target_angles[13] = distance
        self.target_angles[15] = - distance
        self.target_angles[17] = distance - 0.1

        self.target_angles[18] = distance
        self.target_angles[20] = - distance
        self.target_angles[22] = distance - 0.1

        for i in range(8, len(self.target_angles)):
            p.setJointMotorControl2(self.robot_id, i + 1, p.POSITION_CONTROL,
                                    targetPosition=self.target_angles[i])

    def detectContactPoints(self):
        # Get contact points for the left fingertip
        """
        Detects contact forces on the left and right fingertips based on data from
        the robot's end effector joint positions.

        """
        contact_points_left = []
        contact_points_left.extend(p.getContactPoints(bodyA=self.robot_id, linkIndexA=14))
        contact_points_left.extend(p.getContactPoints(bodyA=self.robot_id, linkIndexA=15))
        contact_points_left.extend(p.getContactPoints(bodyA=self.robot_id, linkIndexA=16))
        contact_points_left.extend(p.getContactPoints(bodyA=self.robot_id, linkIndexA=17))
        contact_points_left.extend(p.getContactPoints(bodyA=self.robot_id, linkIndexA=18))

        for contact in contact_points_left:  # Contact force is at index 9 of the contact point tuple
            contact_force_left = contact[9]
            print(f"Contact force on left fingertip: {contact_force_left}")
        # Get contact points for the right fingertip
        contact_points_right = []
        contact_points_right.extend(p.getContactPoints(bodyA=self.robot_id, linkIndexA=19))
        contact_points_right.extend(p.getContactPoints(bodyA=self.robot_id, linkIndexA=20))
        contact_points_right.extend(p.getContactPoints(bodyA=self.robot_id, linkIndexA=21))
        contact_points_right.extend(p.getContactPoints(bodyA=self.robot_id, linkIndexA=22))
        contact_points_right.extend(p.getContactPoints(bodyA=self.robot_id, linkIndexA=23))
        for contact in contact_points_right:
            # Contact force is at index 9 of the contact point tuple
            contact_force_right = contact[9]
            print(f"Contact force on right fingertip: {contact_force_right}")

    def correctTargetPosition(self):
        """
        Calculates the difference between the predicted and actual positions of a
        target based on keypoints detected using the AAMED algorithm, and adjusts
        the position of a cylinder to minimize the error.

        Returns:
            float: Ether the error between the estimated position and the actual
            position or -1 if no keypoints are detected.

        """
        pybulletImage, imageTime = self.read_camera_fixed()
        pybulletImage = cv2.resize(pybulletImage, (1280//2, 720//2))
        imgGPybullet = cv2.cvtColor(pybulletImage, cv2.COLOR_BGR2GRAY)
        resPybullet = self.aamed.run_AAMED(imgGPybullet)
        if isinstance(resPybullet, list):
            resPybullet = np.array(resPybullet)

        diff = 5000
        index = 0
        for i in range(len(self.colorKinova)):
            if abs(imageTime - self.colorKinova[i][1]) < diff:
                diff = abs(self.colorKinova[i][1] - imageTime)
                index = i

        imgGKinova = cv2.cvtColor(self.colorKinova[index][0], cv2.COLOR_BGR2GRAY)
        imgGKinova = cv2.resize(imgGKinova, (1280//2, 720//2))
        resKinova = np.array(self.aamed.run_AAMED(imgGKinova))
        if isinstance(resKinova, list):
            resKinova = np.array(resKinova)

        if resKinova.size == 0 or resPybullet.size == 0:
            print("No keypoints detected")
            return -1

        error = np.abs(resKinova[0][1] - resPybullet[0][1] + resKinova[0][0] - resPybullet[0][0])

        position = list(p.getBasePositionAndOrientation(self.cylinderId)[0])
        position[0] = position[0] - 0.0002 * (resKinova[0][0] - resPybullet[0][0])
        position[1] = position[1] - 0.0002 * (resKinova[0][1] - resPybullet[0][1])
        p.resetBasePositionAndOrientation(self.cylinderId, tuple(position), p.getQuaternionFromEuler([0, 0, 0]))

        return error

    def initialize_toolbox(self, target_position):
        ## Launch the simulator Swift
        """
        Initializes tools and adds them to the environment, sets up grippers and
        end effectors, defines axes for movement, and prepares for work.

        Args:
            target_position (Tuple[float, float, float]): Used to specify the
                position where the toolbox should arrive at the end of the simulation.

        """
        self.env = swift.Swift()
        self.env.launch(realtime=True)

        # Create a KionovaGen3 robot object
        self.kinova = rtb.models.KinovaGen3()
        print(self.kinova.grippers)

        # Set joint angles to ready configuration
        self.kinova.q = self.kinova.qr

        # Add the robot to the simulator
        self.env.add(self.kinova)

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
        # self.cup = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(cup_position[0], cup_position[1], 0), color=(0, 0, 1))
        # self.cup = sg.Cylinder(0.036, 0.168, pose=sm.SE3.Trans(target_position[0], target_position[1], 0), color=(0, 1, 0))

        # self.cup = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(0.4, 0.4, 0), color=(0, 0, 1))
        # self.env.add(self.cup)

        # Set the desired end-effector pose
        self.rot = self.kinova.fkine(self.kinova.q).R
        self.rot = sm.SO3.OA([1, 0, 0], [0, 0, -1])
        # self.Tep = sm.SE3.Rt(self.rot, [target_position[0], target_position[1], 0.60])
        self.Tep = sm.SE3.Rt(self.rot, [target_position[0], target_position[1], target_position[2]])  # green = x-axis, red = y-axis, blue = z-axis
        self.goal_axes.T = self.Tep

        self.arrived = False
        self.dt = 0.05

        self.env.step(0)
        time.sleep(1)

    def toolbox_compute(self, target_position):
        # The current pose of the kinova's end-effector
        """
        Computes the joint velocities for the robot to approach its goal position
        and orientation while avoiding joint limits. It uses the kinematic tree,
        Jacobian matrix, and velocity damping to compute the joint angles.

        Args:
            target_position (Tuple[float, float, float]): Used to specify the
                desired position of the end effector.

        """
        self.Te = self.kinova.fkine(self.kinova.q)

        # self.Tep = sm.SE3.Rt(self.rot, [cup_position[0], cup_position[1], 0.60])  # green = x-axis, red = y-axis, blue = z-axis
        self.Tep = sm.SE3.Rt(self.rot, [target_position[0], target_position[1], target_position[2]])  # green = x-axis, red = y-axis, blue = z-axis
        # Transform from the end-effector to desired pose
        self.eTep = self.Te.inv() * self.Tep

        # Spatial error
        self.e = np.sum(np.abs(np.r_[self.eTep.t, self.eTep.rpy() * np.pi / 180]))

        # Calulate the required end-effector spatial velocity for the robot
        # to approach the goal. Gain is set to 1.0
        self.v, self.arrived = rtb.p_servo(self.Te, self.Tep, 1.0, threshold=0.015)

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
        Reads camera data from a PyBullet environment and processes it to generate
        an RGB image, which is displayed in a window using OpenCV functions.

        Returns:
            Tuple[npndarray,float]: The RGB image of the camera and time stamp in
            milliseconds.

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
        Retrieves depth and color images from a Kinova camera and stores them in
        instance variables for later processing.

        Returns:
            bool: `True` when the operation is successful, and `False` otherwise.

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
        Computes and prints the angles of Kinova joints, the difference between
        the current angles and those provided by PyBullet, and the distance of the
        gripper.

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
        Calculates and sets the target velocities for joints external to the robot,
        based on the robot's internal velocity signals, and then sets those
        velocities using PyBullet's `setJointMotorControl2` function.

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
        Controls the velocity of joints of a robot using PyBullet's setJointMotorControl2
        function, based on target velocities provided in a list.

        """
        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_id, i+1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])
        # print("Pybullet move with toolbox end", time.time()*1000 - self.timestamp)

    def readDataFromProxy(self):
        """
        Retrieves joint state and gripper state data from a proxy server, scales
        the gripper distance by 0.8, and sleeps for 0.05 seconds before repeating
        the process.

        """
        while True:
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()
            self.ext_gripper.distance = self.ext_gripper.distance * 0.8
            #print("ext_joints", self.ext_joints.joints)

            time.sleep(0.05)

    def moveKinovaWithAngles(self, angles):
        """
        Takes an angle array as input and moves the Kinova arm based on the angles.

        Args:
            angles (float | List[float]): Representing the joint angles to move
                the kinova arm in radians, with values between 0 and 360 degrees.

        """
        array = np.round(np.rad2deg(angles) % 360)
        self.angles.jointAngles = array.tolist()
        self.kinovaarm_proxy.moveJointsWithAngle(self.angles)

    def moveKinovaWithSpeeds(self):
        # print("Kinova move with speeds init", time.time()*1000 - self.timestamp)
        """
        Computes and sends joint speeds to the Kinova arm proxy for movement based
        on current joint angles and gain values.

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
        Updates the gains for each joint based on the difference between the robot's
        current pose and its desired pose, with a maximum gain update of 0.2 radians
        per step.

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
        Receives input data from a joystick and updates the robot's position,
        orientation, and move mode based on the received data.

        Args:
            data (Tuple[Axis, Button]): Passed the raw joystick data from the
                user's input device.

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
