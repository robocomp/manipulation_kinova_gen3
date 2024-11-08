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

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
import spatialgeometry as sg

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 50
        if startup_check:
            self.startup_check()
        else:
            # Environment setup
            self.env = swift.Swift()
            self.env.launch(realtime=True)
            self.n = 7
            self.cup1 = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(0.455, -0.1, 0.50), color=(0, 0, 1))
            self.cup2 = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(0.455, 0.1, 0.50), color=(0, 0, 1))
            self.cube = sg.Cuboid((0.334, 0.468, 0.45), pose=sm.SE3.Trans(0.455, 0.0, 0.225), color=(1, 0, 0))
            self.env.add(self.cup1)
            self.env.add(self.cup2)
            self.env.add(self.cube)
            self.collisions = []

            # Initialice the Pedro robot arm and the variables for his control
            self.kinova_pedro = rtb.models.KinovaGen3()
            self.kinova_pedro.q = [1.13, 4.71 - 2*np.pi, np.pi/2, 3.7 - 2*np.pi, 0, 5.41 - 2*np.pi, np.pi/2]
            T = sm.SE3.Rx(90, 'deg') * sm.SE3(0, 1.05, 0.03)
            self.kinova_pedro.base = T
            self.env.add(self.kinova_pedro)

            self.goal_axes_pedro = sg.Axes(0.1)
            self.ee_axes_pedro = sg.Axes(0.1)
            self.ee_axes_pedro.T = self.kinova_pedro.fkine(self.kinova_pedro.q)
            self.rot_pedro = self.kinova_pedro.fkine(self.kinova_pedro.q).R  # Rotation matrix of the end-effector of pedro
            self.Tep_pedro = sm.SE3.Rt(self.rot_pedro, [0.455, -0.1, 0.60])
            self.goal_axes_pedro.T = self.Tep_pedro
            self.env.add(self.ee_axes_pedro)
            self.env.add(self.goal_axes_pedro)

            # Initialice the Pablo robot arm and the variables for his control
            self.kinova_pablo = rtb.models.KinovaGen3()
            self.kinova_pablo.q = [2.00, np.pi/2, np.pi/2, 3.7 - 2*np.pi, 0, 5.41 - 2*np.pi, np.pi/2]
            T = sm.SE3.Rx(270, 'deg') * sm.SE3(0, -1.05, 0.03)
            self.kinova_pablo.base = T
            self.env.add(self.kinova_pablo)

            self.goal_axes_pablo = sg.Axes(0.1)
            self.ee_axes_pablo = sg.Axes(0.1)
            self.ee_axes_pablo.T = self.kinova_pablo.fkine(self.kinova_pablo.q)
            self.rot_pablo = self.kinova_pablo.fkine(self.kinova_pablo.q).R  # Rotation matrix of the end-effector of pedro
            self.Tep_pablo = sm.SE3.Rt(self.rot_pablo, [0.455, 0.1, 0.60])
            self.goal_axes_pablo.T = self.Tep_pablo
            self.env.add(self.ee_axes_pablo)
            self.env.add(self.goal_axes_pablo)

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

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
        self.env.step(0.05)
        return True

    def startup_check(self):
        print(f"Testing RoboCompRoboticsToolboxController.JointStates from ifaces.RoboCompRoboticsToolboxController")
        test = ifaces.RoboCompRoboticsToolboxController.JointStates()
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of calculateVelocities method from RoboticsToolboxController interface
    #
    def RoboticsToolboxController_calculateVelocitiesPablo(self, angles, targetPosition):
        ret = ifaces.RoboCompRoboticsToolboxController.JointStates()

        error = np.sum(np.rad2deg(np.abs(np.array(angles) - np.array(self.kinova_pablo.q))))
        if error > 1:
            self.kinova_pablo.q = np.array(angles)

        Te = self.kinova_pablo.fkine(self.kinova_pabloq)

        self.Tep_pablo = sm.SE3.Rt(self.rot_pablo, [targetPosition[0], targetPosition[1],
                                        targetPosition[2]])  # green = x-axis, red = y-axis, blue = z-axis
        self.goal_axes_pablo.T = self.Tep_pablo
        # Transform from the end-effector to desired pose
        eTep = Te.inv() * self.Tep_pablo

        e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))

        v, arrived = rtb.p_servo(Te, self.Tep_pablo, 1.0, threshold=0.01)

        # Gain term (lambda) for control minimisation
        Y = 0.01

        # Quadratic component of objective function
        Q = np.eye(self.n + 6)

        # Joint velocity component of Q
        Q[:self.n, :self.n] *= Y

        # Slack component of Q
        Q[self.n:, self.n:] = (1 / e) * np.eye(6)

        # The equality contraints
        Aeq = np.c_[self.kinova_pablo.jacobe(self.kinova_pablo.q), np.eye(6)]
        beq = v.reshape((6,))

        # The inequality constraints for joint limit avoidance
        Ain = np.zeros((self.n + 6, self.n + 6))
        bin = np.zeros(self.n + 6)

        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.05

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        # Form the joint limit velocity damper
        Ain[:self.n, :self.n], bin[:self.n] = self.kinova_pablo.joint_velocity_damper(ps, pi, self.n)

        # For each collision in the scene
        for collision in self.collisions:
            # # Form the velocity damper inequality contraint for each collision
            # # object on the robot to the collision in the scene
            c_Ain, c_bin = self.kinova_pablo.link_collision_damper(
                collision,
                self.kinova_pablo.q[:self.n],
                0.3,
                0.05,
                1.0,
                start=self.kinova_pablo.link_dict["half_arm_1_link"],
                end=self.kinova_pablo.link_dict["end_effector_link"],
            )

            # If there are any parts of the robot within the influence distance
            # to the collision in the scene
            if c_Ain is not None and c_bin is not None:
                c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 6))]

                # Stack the inequality constraints
                Ain = np.r_[Ain, c_Ain]
                bin = np.r_[bin, c_bin]

        # Linear component of objective function: the manipulability Jacobian
        c = np.r_[-self.kinova_pablo.jacobm().reshape((self.n,)), np.zeros(6)]

        # The lower and upper bounds on the joint velocity and slack variable
        qdlim = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]  # inventadas
        lb = -np.r_[qdlim, 10 * np.ones(6)]
        ub = np.r_[qdlim, 10 * np.ones(6)]

        # Solve for the joint velocities dq
        try:
            qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='piqp')
        except e:
            print(e)

        self.kinova_pablo.qd[:self.n] = qd[:self.n]

        ret.jointVelocities = self.kinova_pablo.qd.tolist()
        ret.arrived = arrived

        self.env.step(0.05)

        return ret

    #
    # IMPLEMENTATION of calculateVelocitiesPedro method from RoboticsToolboxController interface
    #
    def RoboticsToolboxController_calculateVelocitiesPedro(self, angles, targetPosition):
        ret = ifaces.RoboCompRoboticsToolboxController.JointStates()

        error = np.sum(np.rad2deg(np.abs(np.array(angles) - np.array(self.kinova_pedro.q))))
        if error > 1:
            self.kinova_pedro.q = np.array(angles)

        Te = self.kinova_pedro.fkine(self.kinova_pedro.q)

        self.Tep_pedro = sm.SE3.Rt(self.rot_pedro, [targetPosition[0], targetPosition[1],
                                        targetPosition[2]])  # green = x-axis, red = y-axis, blue = z-axis
        self.goal_axes_pedro.T = self.Tep_pedro
        # Transform from the end-effector to desired pose
        eTep = Te.inv() * self.Tep_pedro

        e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi / 180]))

        v, arrived = rtb.p_servo(Te, self.Tep_pedro, 1.0, threshold=0.01)

        # Gain term (lambda) for control minimisation
        Y = 0.01

        # Quadratic component of objective function
        Q = np.eye(self.n + 6)

        # Joint velocity component of Q
        Q[:self.n, :self.n] *= Y

        # Slack component of Q
        Q[self.n:, self.n:] = (1 / e) * np.eye(6)

        # The equality contraints
        Aeq = np.c_[self.kinova_pedro.jacobe(self.kinova_pedro.q), np.eye(6)]
        beq = v.reshape((6,))

        # The inequality constraints for joint limit avoidance
        Ain = np.zeros((self.n + 6, self.n + 6))
        bin = np.zeros(self.n + 6)

        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.05

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        # Form the joint limit velocity damper
        Ain[:self.n, :self.n], bin[:self.n] = self.kinova_pedro.joint_velocity_damper(ps, pi, self.n)

        # For each collision in the scene
        for collision in self.collisions:
            # # Form the velocity damper inequality contraint for each collision
            # # object on the robot to the collision in the scene
            c_Ain, c_bin = self.kinova_pedro.link_collision_damper(
                collision,
                self.kinova_pedro.q[:self.n],
                0.3,
                0.05,
                1.0,
                start=self.kinova_pedro.link_dict["half_arm_1_link"],
                end=self.kinova_pedro.link_dict["end_effector_link"],
            )

            # If there are any parts of the robot within the influence distance
            # to the collision in the scene
            if c_Ain is not None and c_bin is not None:
                c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], 6))]

                # Stack the inequality constraints
                Ain = np.r_[Ain, c_Ain]
                bin = np.r_[bin, c_bin]

        # Linear component of objective function: the manipulability Jacobian
        c = np.r_[-self.kinova_pedro.jacobm().reshape((self.n,)), np.zeros(6)]

        # The lower and upper bounds on the joint velocity and slack variable
        qdlim = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]  # inventadas
        lb = -np.r_[qdlim, 10 * np.ones(6)]
        ub = np.r_[qdlim, 10 * np.ones(6)]

        # Solve for the joint velocities dq
        try:
            qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='piqp')
        except e:
            print(e)

        self.kinova_pedro.qd[:self.n] = qd[:self.n]

        ret.jointVelocities = self.kinova_pedro.qd.tolist()
        ret.arrived = arrived

        self.env.step(0.05)

        return ret

    #
    # IMPLEMENTATION of setStatePablo method from RoboticsToolboxController interface
    #
    def RoboticsToolboxController_setStatePablo(self, angles):
        print("Setting state Pablo")
        self.kinova_pablo.q = np.array(angles)
        self.ee_axes_pablo.T = self.kinova_pablo.fkine(self.kinova_pablo.q)

    #
    # IMPLEMENTATION of setStatePedro method from RoboticsToolboxController interface
    #
    def RoboticsToolboxController_setStatePedro(self, angles):
        print("Setting state Pedro")
        self.kinova_pedro.q = np.array(angles)
        self.ee_axes_pedro.T = self.kinova_pedro.fkine(self.kinova_pedro.q)

    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompRoboticsToolboxController you can use this types:
    # RoboCompRoboticsToolboxController.JointStates


