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
import numpy
import numpy as np
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from rich.console import Console
from genericworker import *
import interfaces as ifaces
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


import utilities
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

from test import KinovaGen3

class SpecificWorker(GenericWorker):
    """
    Manages a robotic arm and gripper, controlling their movements and joint angles
    through method calls. It also provides methods for getting joint states, gripper
    state, and pose information.

    Attributes:
        Period (int): 1 by default, which represents the time interval for the
            timer in milliseconds.
        startup_check (inline): Defined as a function that tests various aspects
            of the `RoboCompKinovaArm` interface, including its pose and gripper
            state.
        kinova (ifacesRoboCompKinovaArmTKinova): Used to interact with the Kinova
            Gen3 robot arm.
        flag (int): Initialized to False. It is used to indicate whether the startup
            check has been performed or not.
        timer (QTimer): Used to schedule a call to the `compute` method at regular
            intervals (period) using the `timeout` method.
        compute (QtCoreSlot): Called when the timer timeout occurs. It initializes
            variables, gets joints and gripper state from the Kinova arm, and then
            computes the final pose of the arm based on the joint angles and gripper
            distance.
        joints (ifacesRoboCompKinovaArmTJoints): Initialized with joint positions,
            velocities, and forces from the KinovaGen3 arm.
        gripper (ifacesRoboCompKinovaArmTGripper): A representation of the gripper
            state of the Kinova arm.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes an instance of SpecificWorker, setting up its internal state
        and starting a timer for periodic computation.

        Args:
            proxy_map (dict): Used to map proxy names to their corresponding kinova
                objects, allowing for easy interaction with multiple kinova devices
                through a single interface.
            startup_check (bool): Used to determine whether to run the `startup_check()`
                method during initialization.

        """
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 1
        if startup_check:
            self.startup_check()
        else:
            self.kinova = KinovaGen3()
            self.flag = False
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
            self.joints = []
            self.gripper = []

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        """
        Sets parameters for an instance of a `GenericWorker` subclass, `SpecificWorker`.
        It returns `True` upon successful parameter setting.

        Args:
            params (object): Used to set parameters for the function.

        Returns:
            OptionalTrue: A value that may or may not be present and can have any
            value.

        """
        return True


    @QtCore.Slot()
    def compute(self):
        """
        Computes and returns joints and gripper states for an arm robot based on
        sensor readings.

        Returns:
            ifacesRoboCompKinovaArmTJoints: A list of TJoint objects containing
            information about the robot's joints, followed by a TGripper object
            containing information about the gripper's state.

        """
        print("timer init", time.time()*1000)
        if self.flag:
            self.kinova.get_camera_info()
            self.flag = False

        ret = ifaces.RoboCompKinovaArm.TJoints()
        js = self.kinova.get_joints()
        joints = []
        for i in range(len(js["position"])):
            joint = ifaces.RoboCompKinovaArm.TJoint()
            joint.angle = js["position"][i]
            joint.velocity = js["velocity"][i]
            joint.force = js["torque"][i]
            joints.append(joint)
        ret.joints = joints
        ret.timestamp = int(time.time()*1000)
        self.joints = ret
        #print(self.kinova.get_state())
        #print("Joints: ", self.joints)

        ret = ifaces.RoboCompKinovaArm.TGripper()
        ret.distance = self.kinova.get_gripper_state()
        self.gripper = ret

        print("timer end", time.time()*1000)

        return True

    def startup_check(self):
        """
        Tests various aspects of the `ifaces.RoboCompKinovaArm` interface and quits
        the application after 200 milliseconds.

        """
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TGripper from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TGripper()
        QTimer.singleShot(200, QApplication.instance().quit)
        kinova_arm = KinovaGen3


    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of closeGripper method from KinovaArm interface
    #
    def KinovaArm_closeGripper(self):

        #
        # write your CODE here
        #
        """
        Is a method of the `SpecificWorker` class that closes the gripper of a
        Kinova arm at a speed determined by the `close_gripper_speed()` method of
        the kinova object.

        """
        self.kinova.close_gripper_speed()
        pass


    #
    # IMPLEMENTATION of getCenterOfTool method from KinovaArm interface
    #
    def KinovaArm_getCenterOfTool(self, referencedTo):
        """
        Retrieves the center of a tool relative to a Kinova Arm, using the arm's
        pose as reference.

        Args:
            referencedTo (ifacesRoboCompKinovaArmTPose): Used to specify the
                reference frame for the tool center coordinates.

        Returns:
            ifacesRoboCompKinovaArmTPose: A pose object representing the center
            of the tool referenced to the arm.

        """
        ret = ifaces.RoboCompKinovaArm.TPose()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of getGripperState method from KinovaArm interface
    #
    def KinovaArm_getGripperState(self):
        return self.gripper
    #
    # IMPLEMENTATION of openGripper method from KinovaArm interface
    #
    def KinovaArm_openGripper(self):
    
        #
        # write your CODE here
        #
        """
        Opens the gripper of a Kinova arm, a type of robotic arm, using the
        `kinova.open_gripper_speed()` method.

        """
        self.kinova.open_gripper_speed()
        pass

    #
    # IMPLEMENTATION of getJointsState method from KinovaArm interface
    #
    def KinovaArm_getJointsState(self):
        return self.joints

    #
    # IMPLEMENTATION of setCenterOfTool method from KinovaArm interface
    #
    def KinovaArm_setCenterOfTool(self, pose, referencedTo):
    
        #
        # write your CODE here
        #
        """
        Sets the center of a tool referenced to a pose in the KinovaArm framework.

        Args:
            pose (kinovaPose): Used to set the center of the tool relative to the
                arm's end effector.
            referencedTo (object): Used to reference the coordinate system that
                the tool should be set to.

        """
        pass

    #
    # IMPLEMENTATION of moveJointsWithSpeed method from KinovaArm interface
    #
    def KinovaArm_moveJointsWithSpeed(self, speeds):
        self.kinova.move_joints_with_speeds(speeds.jointSpeeds)

    #
    # IMPLEMENTATION of moveJointsWithAngle method from KinovaArm interface
    #
    def KinovaArm_moveJointsWithAngle(self, angles):
        """
        Moves the joints of a Kinova arm to specified angles using the
        `kinova.move_joints_to()` method.

        Args:
            angles (Angle): Used to set the joint angles for movement of the Kinova
                arm.

        """
        print(angles.jointAngles)
        self.kinova.move_joints_to(angles.jointAngles)
        pass
    # ===================================================================
    # ===================================================================


    def buclePrueba(self):
            """
            Retrieves and stores the current pose state of an object using the
            `kinova.get_pose()` method, which is a part of the `GenericWorker`
            class that the function inherits from.

            """
            state = self.kinova.get_pose()
            #self.kinova.cartesian_move_to(state[0],state[1],0.1,state[3],state[4],state[5])



    ######################
    # From the RoboCompKinovaArm you can use this types:
    # RoboCompKinovaArm.TPose
    # RoboCompKinovaArm.TGripper
    # RoboCompKinovaArm.TJoint
    # RoboCompKinovaArm.TJoints
    # RoboCompKinovaArm.TJointSpeeds

