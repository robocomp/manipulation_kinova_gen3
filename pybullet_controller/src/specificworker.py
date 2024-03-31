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

# Set the LC_NUMERIC environment variable
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'
# RMEMEMBER to SET: export LC_NUMERIC="en_US.UTF-8" in your terminal

console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 0
        if startup_check:
            self.startup_check()
        else:

            # Start PyBullet in GUI mode
            self.physicsClient = p.connect(p.GUI)
            # Set the path to PyBullet data
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.setGravity(0, 0, -9.81)
            flags = p.URDF_USE_INERTIA_FROM_FILE
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35,
                                         cameraTargetPosition=[0, 0, 0.5])

            # load floor
            self.plane = p.loadURDF("plane.urdf")

            # Load a table to place the arm on
            self.table_id = p.loadURDF("/home/robocomp/software/bullet3/data/table/table.urdf", basePosition=[0, 0, 0],
                                       baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)

            # Load Kinova arm
            self.robot = KinovaGen3()

            # Load Kinova arm
            # self.robot_id = p.loadURDF(
            #     "/home/robocomp/robocomp/components/manipulation_kinova_gen3/pybullet_controller/gen3_robotiq_2f_140.urdf",
            #     basePosition=[0, 0.3, 0.6], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
            # self.end_effector_link_index = 9
            #
            # # get number of robot joints
            # info = p.getNumJoints(self.robot_id)
            # # for each joint, get the joint info
            # for j in range(info):
            #     info = p.getJointInfo(self.robot_id, j) # joint index
            #     pose = p.getJointState(self.robot_id, j)
            #     print("Joint ", j, "pos = ",  pose[0], info)

            constraint_id = p.createConstraint(
                parentBodyUniqueId=self.table_id,
                parentLinkIndex=-1,  # Use -1 for the base of the object
                childBodyUniqueId=self.robot.robot_id,
                childLinkIndex=-1,  # Use -1 for the base of the object
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],  # Not relevant for fixed joints
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, -0.3, 0.6],  # This needs to match how the arm is offset when loaded
                parentFrameOrientation=[0, 0, 0, 1],
                childFrameOrientation=[0, 0, 0, 1]
            )

            # move arm home
            self.robot.set_home()
            # wait for half a second
            time.sleep(0.5)

            self.target_angles = self.robot.home_angles
            self.target_velocities = [0.0] * 7
            self.joy_selected_joint = 0

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
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

        print("Errors:",
              [round(x - y, 2) for x, y in
               zip(self.robot.get_actual_control_joints_angle(), self.target_angles[:7])],
              self.target_angles[:7])

        # self.robot.move_joints_control_vel( joint_param_value=self.target_velocities,
        #                                     desired_force_per_one_list=[1],
        #                                     desired_vel_per_one_list=[1],
        #                                     wait=True,
        #                                     counter_max=10 ** 2,
        #                                     error_threshold=0.005)

        (self.robot.move_joints(joint_param_value=self.target_angles,
                                         desired_force_per_one_list=[1],
                                         desired_vel_per_one_list=[1],
                                         wait=True,
                                         counter_max=10 ** 2,
                                         error_threshold=0.3))

        # print(self.target_angles[:7])
        # print([round(x, 2) for x in self.robot.get_actual_control_joints_angle()])
        # print("Errors:", [round(x-y, 2) for x,y in zip(self.robot.get_actual_control_joints_angle(), self.target_angles[:7])])

        # print("JOINT: ", self.joy_selected_joint)
        # print("-------------------")

        p.stepSimulation()

    # =============== Methods ==================

    def startup_check(self):
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TGripper from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TGripper()
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of closeGripper method from KinovaArm interface
    #
    def KinovaArm_closeGripper(self):

        #
        # write your CODE here
        #
        pass


    #
    # IMPLEMENTATION of getCenterOfTool method from KinovaArm interface
    #
    def KinovaArm_getCenterOfTool(self, referencedTo):
        ret = ifaces.RoboCompKinovaArm.TPose()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getGripperState method from KinovaArm interface
    #
    def KinovaArm_getGripperState(self):
        ret = ifaces.RoboCompKinovaArm.TGripper()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of openGripper method from KinovaArm interface
    #
    def KinovaArm_openGripper(self):

        #
        # write your CODE here
        #
        pass


    #
    # IMPLEMENTATION of setCenterOfTool method from KinovaArm interface
    #
    def KinovaArm_setCenterOfTool(self, pose, referencedTo):

        #
        # write your CODE here
        #
        pass

    # =============== Methods for SubscribesTo ================
    # =========================================================
    #
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    #
    def JoystickAdapter_sendData(self, data):
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
                    self.target_angles[self.joy_selected_joint] += axis.value
                case "Z_axis":
                    pass
                case "gripper":
                    pass
        print(self.joy_selected_joint, self.target_angles[:7])


















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