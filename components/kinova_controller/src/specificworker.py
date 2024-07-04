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
    def __init__(self, proxy_map, startup_check=False):
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
        return True


    @QtCore.Slot()
    def compute(self):
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
        self.kinova.close_gripper_speed()
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
        return self.gripper
    #
    # IMPLEMENTATION of openGripper method from KinovaArm interface
    #
    def KinovaArm_openGripper(self):
    
        #
        # write your CODE here
        #
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
        print(angles.jointAngles)
        self.kinova.move_joints_to(angles.jointAngles)
        pass
    # ===================================================================
    # ===================================================================


    def buclePrueba(self):
            state = self.kinova.get_pose()
            #self.kinova.cartesian_move_to(state[0],state[1],0.1,state[3],state[4],state[5])



    ######################
    # From the RoboCompKinovaArm you can use this types:
    # RoboCompKinovaArm.TPose
    # RoboCompKinovaArm.TGripper
    # RoboCompKinovaArm.TJoint
    # RoboCompKinovaArm.TJoints
    # RoboCompKinovaArm.TJointSpeeds

