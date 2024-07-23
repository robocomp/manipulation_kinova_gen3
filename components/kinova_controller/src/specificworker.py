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
    Manages a kinova gen3 arm, controlling joint movement and gripper state. It
    provides methods for computing joint angles based on speeds, moving the arm
    with speed, getting joints and gripper state, opening and closing the gripper,
    and setting the center of tool.

    Attributes:
        Period (int|float): 1 by default, which sets the timer interval for the
            worker's computation method to run every 1 second.
        startup_check (bool): Used to test certain features of the `ifaces.RoboCompKinovaArm`
            module, such as getting the joints state and gripper state, at the
            beginning of the worker's execution.
        kinova (KinovaGen3|KinovaArm): Used to interact with the Kinova robotic arm.
        flag (bool): Used to indicate whether the worker has finished its work or
            not.
        timer (object): Used to schedule a callable object to be executed after a
            certain period of time.
        compute (bool): Defined as a slot function that handles the computation
            of the kinova arm state based on the current state of the joints,
            gripper, and other parameters. It also performs movement with speed
            and checks for startup errors.
        joints (ifacesRoboCompKinovaArmTJoints): Populated with joint angles from
            the Kinova Gen3 arm through the `compute()` method.
        gripper (ifacesRoboCompKinovaArmTGripper): Used to store the current state
            of the gripper, such as its distance from the reference position.
        speeds (List[float]): Used to store joint speed values for moving the
            robotic arm with speed.
        moveWithSpeed (bool): Used to control the movement of the robotic arm with
            speed. It is set to True when the arm should move with speed, and False
            otherwise.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes various objects and sets up timers for computing and moving
        the robotic arm based on the provided proxy map and startup check status.

        Args:
            proxy_map (Dict[str, Any]): Used to specify a mapping between the names
                of kinova joints and their corresponding Python objects.
            startup_check (bool): Set to false by default. It checks if the kinova
                device is connected and ready for use before starting the computation.

        """
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 1
        if startup_check:
            self.startup_check()
        else:
            self.kinova = KinovaGen3()
            self.flag = True
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
        Sets parameters for the worker.

        Args:
            params (Union[dict, List[str]]): Used to set parameters for the
                function's execution.

        Returns:
            bool: True if the parameters are successfully set, and False otherwise.

        """
        return True


    @QtCore.Slot()
    def compute(self):
        # print("timer init", time.time()*1000)
        """
        Computes and updates joint states and gripper distance based on Kinova
        Arm's API calls, and manages moveWithSpeed mode.

        Returns:
            bool: True if the operation was successful, and False otherwise.

        """
        if self.flag:
            # self.kinova.get_camera_info()
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

        # self.kinova.gripper_move_speed(-0.005)

        # print("timer end", time.time()*1000)

        return True

    def startup_check(self):
        """
        Checks and prints the functionality of RoboCompContactile, FingerTip,
        FingerTips, TPose, and TGripper classes from the ifaces module.

        """
        print(f"Testing RoboCompContactile.FingerTip from ifaces.RoboCompContactile")
        test = ifaces.RoboCompContactile.FingerTip()
        print(f"Testing RoboCompContactile.FingerTips from ifaces.RoboCompContactile")
        test = ifaces.RoboCompContactile.FingerTips()
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
        """
        Calculates and moves the gripper towards a target distance, based on tactile
        sensor values, until it reaches within 8mm of its final position, at which
        point it stops moving.

        """
        force = 0
        while force < 8 and self.gripper.distance < 0.9:
            self.kinova.gripper_move_speed(-0.005)
            tactileValues = self.contactile_proxy.getValues()
            leftValues = tactileValues.left
            righValues = tactileValues.right
            force = (abs(leftValues.x) + abs(leftValues.y) + abs(leftValues.z)
                     + abs(righValues.x) + abs(righValues.y) + abs(righValues.z))

        print("Gripper closed")
        self.kinova.gripper_move_speed(0)

    #
    # IMPLEMENTATION of getCenterOfTool method from KinovaArm interface
    #
    def KinovaArm_getCenterOfTool(self, referencedTo):
        """
        Calculates and returns the center of a tool referenced to the Kinova Arm's
        frame of reference.

        Args:
            referencedTo (RoboCompKinovaArm.TPose | str): Used to specify the
                reference frame for calculating the center of the tool.

        Returns:
            Pose: A position and orientation of a tool relative to the arm's base
            link.

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
        """
        Opens the gripper of a Kinova arm at a speed of 0.005 when the distance
        between the gripper and an object is greater than 0.01 meters, and then
        stops the movement of the gripper.

        """
        while self.gripper.distance > 0.01:
            self.kinova.gripper_move_speed(0.005)

        self.kinova.gripper_move_speed(0)

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
        Sets the center of a tool referenced to a specific pose in space, based
        on the arm's configuration and reference frame.

        Args:
            pose (Pose): Representing the arm's current pose relative to the world
                coordinates.
            referencedTo (ReferenceType | str): Used to specify whether the center
                of the tool should be set relative to the end effector or the base
                link of the robot.

        """
        pass

    #
    # IMPLEMENTATION of moveJointsWithSpeed method from KinovaArm interface
    #
    def KinovaArm_moveJointsWithSpeed(self, speeds):
        """
        Sets the speeds for joint movement based on the input parameter, and enables
        or disables the movement with speed feature depending on whether the
        `moveWithSpeed` attribute is set to `True` or `False`.

        Args:
            speeds (List[float]): Passed as an input to control the speed of joint
                movement.

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
        Moves the joints of a Kinova arm to a specified angle using the
        `kinova.move_joints_to()` function.

        Args:
            angles (angles.jointAngles | float): Passed as an instance of the
                JointAngles class or a list of joint angles in radians.

        """
        print(angles.jointAngles)
        self.kinova.move_joints_to(angles.jointAngles)
        pass
    # ===================================================================
    # ===================================================================


    def buclePrueba(self):
            """
            Retrieves the current pose of a kinova robot using the `get_pose()`
            method and stores it in the variable `state`.

            """
            state = self.kinova.get_pose()
            #self.kinova.cartesian_move_to(state[0],state[1],0.1,state[3],state[4],state[5])


    ######################
    # From the RoboCompContactile you can call this methods:
    # self.contactile_proxy.getValues(...)

    ######################
    # From the RoboCompContactile you can use this types:
    # RoboCompContactile.FingerTip
    # RoboCompContactile.FingerTips

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # RoboCompKinovaArm.TPose
    # RoboCompKinovaArm.TGripper
    # RoboCompKinovaArm.TJoint
    # RoboCompKinovaArm.TJoints
    # RoboCompKinovaArm.TJointSpeeds

