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
    Manages a robotic arm's joints and gripper, providing methods for computing
    the arm's pose, getting the joints and gripper state, and moving the joints
    with speed or angle.

    Attributes:
        Period (int): 1 by default, indicating the interval between timer timeouts
            for computing joint positions.
        startup_check (unbound): Called when the worker is initialized. It performs
            some testing and checks to ensure that the robotic arm is working correctly.
        kinova (ifacesRoboCompKinovaArm): A reference to an instance of the
            KinovaGen3 class, which provides access to the RoboCompKinovaArm API.
        flag (bool): Used to indicate when the worker should perform its job.
        timer (QTimer): Used to schedule a callable object at a specific interval,
            in this case, the function `compute`.
        compute (QtCoreSlot): A method that runs periodically to retrieve joint
            information from the Kinova arm.
        joints (ifacesRoboCompKinovaArmTJoints): A list of objects representing
            the joint states of the Kinova arm.
        gripper (ifacesRoboCompKinovaArmTGripper): A representation of the gripper
            state of the RoboCompKinovaArm.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes an instance of the SpecificWorker class by setting up its
        properties and event handlers. It creates a KinovaGen3 object, sets a flag
        to False, and starts a timer to trigger computation every `Period` seconds.

        Args:
            proxy_map (dict): Used to map proxy objects to their corresponding
                worker objects.
            startup_check (bool): Used to enable or disable a check when the
                SpecificWorker initializes.

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
        Sets parameters for an instance of the `SpecificWorker` class, which
        inherits from `GenericWorker`. It returns `True` indicating successful
        parameter setting.

        Args:
            params (objectinstance): Passed to the method for modification or alteration.

        Returns:
            True: 100% true.

        """
        return True


    @QtCore.Slot()
    def compute(self):
        # print("timer init", time.time()*1000)
        """
        Computes and updates the positions, velocities, and forces of joints and
        gripper of a Robocomp Kinova arm based on sensor readings and returns True
        if successful.

        Returns:
            ifacesRoboCompKinovaArmTGripper: A struct consisting of distance and
            timestamp, and another value of type ifacesRoboCompKinovaArmTJoints
            which is also a struct consisting of joint angles, velocities, forces,
            and timestamp.

        """
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

        # print("timer end", time.time()*1000)

        return True

    def startup_check(self):
        """
        Tests the RoboCompKinovaArm TPose and TGripper classes, and then calls
        QTimer.singleShot(200, QApplication.instance().quit) to quit the application
        after a delay of 200 milliseconds.

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
        Closes the gripper of a robotic arm with the `kinova` module.

        """
        self.kinova.close_gripper_speed()
        pass


    #
    # IMPLEMENTATION of getCenterOfTool method from KinovaArm interface
    #
    def KinovaArm_getCenterOfTool(self, referencedTo):
        """
        Computes and returns the center of a tool referenced to a specific interface,
        using the `RoboCompKinovaArm` interface.

        Args:
            referencedTo (ifacesRoboCompKinovaArmTPose): Used to specify the
                reference frame for the tool center point calculation.

        Returns:
            ifacesRoboCompKinovaArmTPose: A Pose object representing the center
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
        Opens the gripper of a Kinova arm, a robotic arm used for material handling
        and manipulation tasks, using the `kinova.open_gripper_speed()` method.

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
        Sets the center of a tool referenced to a given pose within a specific
        worker class, inheriting from `GenericWorker`.

        Args:
            pose (kinovageometryPose): Used to specify the position and orientation
                of the tool relative to the arm.
            referencedTo (KinovaArm_ReferencePoint): Used to specify the reference
                point for the tool center coordinate calculation.

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
        Moves the joints of a robot arm to specified angles, using the
        `kinova.move_joints_to()` method.

        Args:
            angles (angles): A joint angle representation containing multiple
                angles for different joints of the robot arm to move to.

        """
        print(angles.jointAngles)
        self.kinova.move_joints_to(angles.jointAngles)
        pass
    # ===================================================================
    # ===================================================================


    def buclePrueba(self):
            """
            Retrieves the current pose of an object using the `kinova.get_pose()`
            method and stores it in a variable called `state`.

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

