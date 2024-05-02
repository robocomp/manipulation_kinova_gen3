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
import os
import cv2
import threading
from pybullet_utils import urdfEditor as ed
from pybullet_utils import bullet_client as bc

# Set the LC_NUMERIC environment variable
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'
# RMEMEMBER to SET: export LC_NUMERIC="en_US.UTF-8" in your terminal

console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 5
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
            #p.setGravity(0, 0, 0) # Set gravity to 0 to avoid the arm falling down when the simulation starts (it is not realistic)
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
            self.home_angles = [0, -0.34, 3.14, -2.54, -6.28, -0.87, 1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.robot_id = p.loadURDF(self.robot_urdf, self.robot_launch_pos, self.robot_launch_orien,
                                       flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

            for i in range(7):
                p.resetJointState(bodyUniqueId=self.robot_id, jointIndex=i+1,
                                  targetValue=self.home_angles[i], targetVelocity=0)


            for i in range(7):
                p.setJointMotorControl2(self.robot_id, i+1, p.POSITION_CONTROL, targetPosition=self.home_angles[i])

            #////////////////////////////////////////////////////

            # Load Kinova arm
            # self.robot_id = p.loadURDF(
            #     "/home/robocomp/robocomp/components/manipulation_kinova_gen3/pybullet_controller/gen3_robotiq_2f_140.urdf",
            #     basePosition=[0, 0.3, 0.6], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
            self.end_effector_link_index = 12
            #
            # # get number of robot joints
            # info = p.getNumJoints(self.robot_id)
            # # for each joint, get the joint info
            # for j in range(info):
            #     info = p.getJointInfo(self.robot_id, j) # joint index
            #     pose = p.getJointState(self.robot_id, j)
            #     print("Joint ", j, "pos = ",  pose[0], info)

            self.cup = p.loadURDF("/home/robolab/software/bullet3/data/dinnerware/cup/cup_small.urdf", basePosition=[0, 0.1, 0.64], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)

            #//////////////////////////////////////////////////

            # Carga del brazo de guille para pruebas

            # self.robot2 = p.loadURDF("/home/robolab/Descargas/open_manipulator_p/open_manipulator_p.urdf", basePosition=[0, -0.3, 0.6], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
            #
            # self.n_joints_arm2 = p.getNumJoints(self.robot2)
            # self.joints_robot2 = np.zeros(self.n_joints_arm2).tolist()
            #
            # self.robot2_target_pos = p.getLinkState(self.robot2, 11)[0]
            # self.robot2_target_ori = p.getLinkState(self.robot2, 11)[1]
            #
            # for i in range(self.n_joints_arm2):
            #     self.joints_robot2[i] = p.getJointState(self.robot2, i)[0]

            #//////////////////////////////////////////////////

            # Crear una restricción fija entre los dos modelos
            fixed_constraint = p.createConstraint(self.table_id, -1, self.robot_id, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0.3, 0.64], [0, 0, 0], childFrameOrientation=p.getQuaternionFromEuler([0, 0, 1.57]))

            self.hilo_lectura = threading.Thread(target=self.readDataFromProxy)
            self.hilo_lectura.start()

            self.hilo_camara = threading.Thread(target=self.readCamera)
            #self.hilo_camara.start()

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
            #self.home_position = p.getLinkState(self.robot_id, self.end_effector_link_index)[0]
            self.target_position = p.getLinkState(self.robot_id, self.end_effector_link_index)[0]
            self.target_orientation = p.getLinkState(self.robot_id, self.end_effector_link_index)[1]
            self.target_velocities = [0.0] * 7
            self.joy_selected_joint = 0
            self.move_mode = 1
            self.n_rotations = np.zeros(7).tolist()
            self.n_rotations = [0, -1, 0, -1, -1, -1, 0]
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()
            self.timestamp = int(time.time()*1000)
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

            self.timer2 = QtCore.QTimer(self)
            self.timer2.timeout.connect(self.move_joint_fixed_vel)
            self.timer2.start(100)
            print("SpecificWorker started")

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

        match self.move_mode:
            #Move joints
            case 0:
                self.target_angles[7:] = [0.0] * len(self.target_angles[7:])
                for i in range(len(self.target_angles)):
                    p.resetJointState(bodyUniqueId=self.robot.robot_id,jointIndex=i,
                                      targetValue=self.robot.home_angles[i], targetVelocity=0)

            #Cartesian movement
            case 1:
                joint_positions = p.calculateInverseKinematics(self.robot_id, self.end_effector_link_index,
                                                               self.target_position, self.target_orientation)
                #print(joint_positions)
                joints = np.zeros(22).tolist()
                for i in range(len(joint_positions)):
                    joints[i] = joint_positions[i]

                joint_speeds = []

                for i in range(len(joints)):
                    p.setJointMotorControl2(self.robot_id, i+1, p.POSITION_CONTROL, joints[i])
                    if i < 7:
                        speed = math.degrees(p.getJointState(self.robot_id, i+1)[1])
                        joint_speeds.append(speed)

                print(joint_speeds)
                speeds = ifaces.RoboCompKinovaArm.TJointSpeeds()
                speeds.jointSpeeds = joint_speeds

                if abs(self.timestamp - (time.time() * 1000)) > 200:
                    self.timestamp = int(time.time() * 1000)
                    self.kinovaarm_proxy.moveJointsWithSpeed(speeds)

            case 2:
                joint_positions = p.calculateInverseKinematics(self.robot.robot_id, self.end_effector_link_index,
                                                               self.target_position, self.target_orientation)
                joints = np.zeros(22).tolist()
                for i in range(len(joint_positions)):
                    joints[i] = joint_positions[i]

                for i in range(len(joints)):
                    p.resetJointState(bodyUniqueId=self.robot.robot_id, jointIndex=i,
                                      targetValue=joints[i])

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
                    for i in range(len(self.target_velocities)):
                        p.setJointMotorControl2(bodyUniqueId=self.robot_id, jointIndex=i, controlMode=p.VELOCITY_CONTROL, targetPosition=self.target_velocities[i])

                except Ice.Exception as e:
                    print(e)


        p.stepSimulation()
        # pass

    # =============== Methods ==================

    def startup_check(self):
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

    def readCamera(self):
        while True:
            fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
            projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
            com_p, com_o, _, _, _, _ = p.getLinkState(self.robot.robot_id, 8)
            rot_matrix = p.getMatrixFromQuaternion(com_o)
            rot_matrix = np.array(rot_matrix).reshape(3, 3)
            # Initial vectors
            init_camera_vector = (0, 0, 1)  # z-axis
            init_up_vector = (0, 1, 0)  # y-axis
            # Rotated vectors
            camera_vector = rot_matrix.dot(init_camera_vector)
            up_vector = rot_matrix.dot(init_up_vector)
            view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
            img = p.getCameraImage(400, 400, view_matrix, projection_matrix)
            self.rgb = img[2]
            self.rgb = cv2.rotate(self.rgb, cv2.ROTATE_180)   # Get the RGB image
            cv2.imshow('img', self.rgb)
            time.sleep(0.05)

    def move_joint_fixed_vel(self):
        for i in range(len(self.ext_joints.joints)):
            self.target_velocities[i] = self.ext_joints.joints[i].velocity

        self.target_velocities = numpy.deg2rad(self.target_velocities) * 1.2

        for i in range(len(self.target_velocities)):
            p.setJointMotorControl2(self.robot_id, i+1, p.VELOCITY_CONTROL,
                                    targetVelocity=self.target_velocities[i])


    def readDataFromProxy(self):
        while True:
            self.ext_joints = self.kinovaarm_proxy.getJointsState()
            self.ext_gripper = self.kinovaarm_proxy.getGripperState()
            self.ext_gripper.distance = self.ext_gripper.distance * 0.8
            time.sleep(0.1)

  
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