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
    Manages a RoboCompKinovaArm, providing methods for getting and setting joints'
    state, gripper state, and pose. It also includes a startup check and a method
    to move joints with speed or angle.

    Attributes:
        Period (int): 1 by default, representing the period of time (in milliseconds)
            between successive invocations of the `compute()` method.
        startup_check (Python): Used for testing the RoboCompKinovaArm functionality.
        kinova (KinovaGen3): Used to interact with the RoboCompKinovaArm device.
        flag (bool): Used to indicate whether the worker has performed its computation
            or not, allowing the `compute()` method to be called only once.
        timer (QtCoreQTimer): Used to schedule a callable object (i.e., a slot)
            to be executed after a specified interval, typically for processing
            sensor data or other tasks that require periodic updates.
        compute (QtCoreSlot): Used to execute a function when a specific event
            occurs, such as a timeout. The function executed by the `compute`
            attribute is responsible for retrieving data from the Kinova arm and
            updating the worker's internal state.
        joints (ifacesRoboCompKinovaArmTJoints): A list of objects representing
            the joint states of the RoboCompKinovaArm.
        gripper (ifacesRoboCompKinovaArmTGripper): Used to store the current state
            of the gripper, such as its position and speed.
        speeds (ifacesRoboCompKinovaArmTJointSpeeds): Used to store the joint speed
            values for moving the robotic arm with speed.
        moveWithSpeed (bool): Used to control the movement of joints with speed.
            When set to True, it enables the movement of joints with a predefined
            speed, otherwise it disables it.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Sets up an instance of `SpecificWorker`, inheriting from `GenericWorker`.
        It initializes member variables, including a period for computing and a
        list of joints and gripper speeds.

        Args:
            proxy_map (dict): Used to pass a mapping of kinova interface names to
                their corresponding implementations.
            startup_check (bool): Used to check if the kinova arm is already started
                or not.

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
            self.speeds = ifaces.RoboCompKinovaArm.TJointSpeeds()
            self.speeds.jointSpeeds = [0, 0, 0, 0, 0, 0, 0]
            self.moveWithSpeed = False

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        """
        Sets parameters for an instance of `SpecificWorker` subclassing `GenericWorker`.
        It returns `True` indicating successful parameter setting.

        Args:
            params (objectinstance): Used to set parameters for an instance of a
                class.

        Returns:
            True: The default value returned by the method when no exception occurs.

        """
        return True


    @QtCore.Slot()
    def compute(self):
        # print("timer init", time.time()*1000)
        """
        Computes the joint positions, velocities, and forces for the RoboComp
        kinova arm based on input from sensors and other sources. It also retrieves
        gripper state information and moves the joints with specified speeds if necessary.

        Returns:
            ifacesRoboCompKinovaArmTJoints: An instance of a class that represents
            the joints information of the kinova arm, including the angles,
            velocities, and forces of each joint.

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

        if self.moveWithSpeed:
            if self.lastMoveOrder + 1000 < time.time() * 1000:
                self.speeds.jointSpeeds = [0, 0, 0, 0, 0, 0, 0]
                self.moveWithSpeed = False
            self.kinova.move_joints_with_speeds(self.speeds.jointSpeeds)

        # print("timer end", time.time()*1000)

        return True

    def startup_check(self):
        """
        Tests RoboCompKinovaArm.TPose and RoboCompKinovaArm.TGripper classes from
        the ifaces module and then quits the application after a 200 milliseconds
        delay.

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
    def KinovaArm_closeGripper(self, position):

        """
        Moves the gripper of a robotic arm to a specified position, using the
        `kinova.gripper_move_to()` method.

        Args:
            position (float): Used to specify the desired position of the gripper
                in terms of the range of motion of the robotic arm.

        """
        self.kinova.gripper_move_to(position)


    #
    # IMPLEMENTATION of getCenterOfTool method from KinovaArm interface
    #
    def KinovaArm_getCenterOfTool(self, referencedTo):
        """
        Retrieves the center of a tool referenced by `referencedTo`.

        Args:
            referencedTo (ifacesRoboCompKinovaArmTPose): Used to return the center
                of the tool referenced to a specific position or orientation.

        Returns:
            ifacesRoboCompKinovaArmTPose: A pose representation of the tool center
            position relative to the arm's end effector.

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
        Opens the gripper of a robotic arm controlled by the `kinova` library.

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
        Sets the center of a tool referenced to a given pose within a Kinova Arm.

        Args:
            pose (opencvcoreMat): Used to represent the tool's pose relative to
                the end effector.
            referencedTo (kinova_msgsmsgReferenceFrame): Used to specify the
                reference frame for the tool center point.

        """
        pass

    #
    # IMPLEMENTATION of moveJointsWithSpeed method from KinovaArm interface
    #
    def KinovaArm_moveJointsWithSpeed(self, speeds):
        """
        Sets the speeds of joint movements for an arm using a timer to determine
        the move order and speed.

        Args:
            speeds (numpyarray): 3D array of float values representing the desired
                joint speeds for the Kinova arm to move with.

        """
        self.speeds = speeds
        self.moveWithSpeed = True
        self.lastMoveOrder = time.time()*1000
        # self.kinova.move_joints_with_speeds(speeds.jointSpeeds)

    #
    # IMPLEMENTATION of moveJointsWithAngle method from KinovaArm interface
    #
    def KinovaArm_moveJointsWithAngle(self, angles):
        """
        Moves the joints of a Kinova arm to specific angles, using the
        `kinova.move_joints_to()` method.

        Args:
            angles (Angle): Passed as an instance of the Angle class, containing
                the desired joint angles in degrees for the Kinova arm to move.

        """
        print(angles.jointAngles)
        self.kinova.move_joints_to(angles.jointAngles)
        pass
    # ===================================================================
    # ===================================================================


    def buclePrueba(self):
            """
            Retrieves the current pose of an object using the `kinova` module and
            stores it in a variable called `state`.

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

