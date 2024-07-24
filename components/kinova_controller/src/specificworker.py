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
    Is a worker thread that interacts with a Kinova Gen3 robotic arm and its
    gripper, providing methods to control the arm's movement, get its joints and
    gripper states, open and close the gripper, and retrieve tactile data from
    contact sensors.

    Attributes:
        Period (int): 1 by default. It likely represents the period at which a
            certain action or computation is performed, possibly tied to a timer
            event with a duration of 1 second.
        startup_check (bool): Set to True when called. It calls a series of tests
            on various interfaces, including RoboCompContactile.FingerTip and RoboCompKinovaArm.TPose.
        kinova (KinovaGen3): Used to interact with a kinova arm, such as getting
            its joints' state, moving its gripper, or setting its joint angles.
        flag (bool): Initialized to True in the `__init__` method. It is used as
            a control variable in the `compute` method, toggling its value each
            time the timer event occurs.
        timer (QTimer): Initialized with a periodic timer that calls the `compute`
            method at regular intervals specified by the `Period` attribute.
        compute (QtCoreSlot|None): Called when a timer timeout occurs. It gets
            joints data from KinovaGen3, calculates joint speeds, updates timestamps,
            and prints contactile data.
        joints (ifacesRoboCompKinovaArmTJoints|None): Initialized as None. It
            stores a list of joints information, including position, velocity, and
            force, retrieved from the KinovaGen3 robot arm.
        gripper (ifacesRoboCompKinovaArmTGripper): Updated in the `compute` method
            with the current state of the gripper, retrieved by calling the
            `get_gripper_state` method of the KinovaGen3 object.
        speeds (ifacesRoboCompKinovaArmTJointSpeeds): 7-element list, representing
            the speed of each joint of the Kinova arm, initialized with all elements
            set to zero.
        moveWithSpeed (bool): Set to False by default. When it is set to True, the
            worker moves the joints with speeds specified in the `speeds.jointSpeeds`
            list.
        timestamp (int): Initialized to the current time (in milliseconds) when
            the worker object is created. It is used to measure the elapsed time
            between timer events.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes an instance with a proxy map and optional startup check. If
        the startup check is enabled, it performs some checks. Otherwise, it sets
        up a timer to run the compute function at regular intervals, initializes
        joints and gripper variables, and sets default speeds for the robot arm.

        Args:
            proxy_map (object): Passed to the parent class `SpecificWorker` during
                initialization. It sets up the proxy mapping for the worker's
                interfaces, allowing it to interact with other components in the
                system.
            startup_check (bool): Optional, with default value False. Its presence
                determines whether to perform startup checks or initialize the
                Kinova robot directly. If set to True, startup checks are performed;
                otherwise, initialization proceeds as usual.

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
            self.timestamp = time.time()*1000

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        """
        Sets parameters and returns `True`. If any error occurs during the process,
        it does not handle or report it explicitly.

        Args:
            params (Any): Intended to be used for setting or updating parameters
                in the class instance. The actual data type of `params` can vary
                depending on how it is used within the function.

        Returns:
            bool: `True`. This indicates that the function execution has been
            successful and all parameters have been set correctly.

        """
        return True


    @QtCore.Slot()
    def compute(self):
        # print("timer init", time.time()*1000)
        """
        Retrieves joint positions, velocities, and torques from a kinova arm, sends
        gripper state to a RoboCompKinovaArm object, moves joints with specified
        speeds, updates timestamp, and returns True.

        Returns:
            bool: `True`.

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
        print(time.time()*1000 - self.timestamp)
        print("Contactile data:", self.contactile_proxy.getValues())

        return True

    def startup_check(self):
        """
        Performs startup checks for several RoboComp interfaces, including
        RoboCompContactile and RoboCompKinovaArm. It instantiates test objects
        from these interfaces and prints corresponding messages to the console
        before quitting the application after a 200ms delay.

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
        Controls the gripper of a Kinova robot arm to close slowly and safely by
        monitoring tactile sensors until a target force is reached, then stops and
        prints a success message.

        """
        force = 0
        while force < 6 and self.gripper.distance < 0.9:
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
        Returns an object of type TPose, representing the center of the tool
        (end-effector) of a Kinova arm robot, referenced to a specific coordinate
        system or frame.

        Args:
            referencedTo (object): Used to specify the coordinate system from which
                the center of tool coordinates are calculated. It serves as a
                reference frame for calculating the center of tool position.

        Returns:
            RoboCompKinovaArmTPose: A data structure representing a pose (position
            and orientation) in space, specifically for the Kinova Arm robot.

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
        Controls the opening of the gripper on a Kinova arm. It continuously reduces
        the distance between the gripper's fingers until it reaches a threshold
        value of 0.01, then stops and sets the gripper speed to 0.

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
        Sets the center of tool for the Kinova arm according to the provided pose
        and referencedTo parameters, presumably used for robotic arm control or simulation.

        Args:
            pose (Pose): 6-dimensional array representing the position and orientation
                of the robot's tool center point.
            referencedTo (object): Used to specify the frame of reference for the
                pose provided.

        """
        pass

    #
    # IMPLEMENTATION of moveJointsWithSpeed method from KinovaArm interface
    #
    def KinovaArm_moveJointsWithSpeed(self, speeds):
        """
        Sets the speeds for moving joints of Kinova arm, marks that movement with
        speed is enabled, and records the time of last move order.

        Args:
            speeds (List[int]): Utilized to store the speeds at which each joint
                of Kinova Arm should move when executing the movement command.

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
        Moves a Kinova arm to a specified joint angle configuration. The angles
        are retrieved from an input object, printed for debugging purposes, and
        then sent to the kinova.move_joints_to function to control the arm's joints
        accordingly.

        Args:
            angles (object): Expected to contain information about joint angles
                for the Kinova arm movement.

        """
        print(angles.jointAngles)
        self.kinova.move_joints_to(angles.jointAngles)
        pass
    # ===================================================================
    # ===================================================================


    def buclePrueba(self):
            """
            Retrieves the current pose (position and orientation) of the kinova
            robot using the `get_pose()` method inherited from the GenericWorker
            class. It assigns this information to a variable named `state`.

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

