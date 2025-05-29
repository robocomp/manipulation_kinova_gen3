#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by Jorge Calderon Gonzalez
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
import time

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from tensorflow.python.ops.gen_linalg_ops import self_adjoint_eig_v2_eager_fallback

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
        self.last_time = time.time() * 1000
        if startup_check:
            self.startup_check()
        else:
            # Environment setup
            self.env = swift.Swift()
            self.env.launch(realtime=True)
            self.env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])

            self.p3bot = rtb.models.P3Bot()

            T = sm.SE3(0, 0, 0.04)
            Rz = sm.SE3.Rz(np.pi / 2)
            self.p3bot.base = T * Rz

            # # Guardamos la rotación inicial deseada (-90° en Z)
            # rotacion_inicial = sm.SE3.Rz(-90, unit='deg')
            #
            # # Aplicamos esta rotación SOLO como offset visual/cinemático
            # self.p3bot.base = rotacion_inicial  # Se aplica una única vez

            #
            # # qd = self.p3bot.qd
            # # qd[44] = 0.5
            # # self.p3bot.qd = qd
            self.env.add(self.p3bot)

            #
            # # for link in self.p3bot.ee_links:
            # #     print(f"Link {link.name} has mass {link.m} and inertia {link.I}")

            self.goal_axes = sg.Axes(0.1)
            self.rot = sm.SO3.Rx(90, 'deg') * sm.SO3.Ry(180, 'deg') * sm.SO3.Rz(0, 'deg')
            self.Tep = sm.SE3.Rt(self.rot, [0, 2.2, 0.70])
            self.goal_axes.T = self.Tep
            self.env.add(self.goal_axes)

            self.loop_count = 0

            # self.n = 7
            # self.cup1 = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(0.455, -0.1, 0.50), color=(0, 0, 1))
            # self.cup2 = sg.Cylinder(0.05, 0.1, pose=sm.SE3.Trans(0.455, 0.1, 0.50), color=(0, 0, 1))
            # self.cube = sg.Cuboid((0.334, 0.468, 0.48), pose=sm.SE3.Trans(0.5, 1, 0.24), color=(1, 0, 0))
            # self.env.add(self.cup1)
            # self.env.add(self.cup2)
            # self.env.add(self.cube)
            # self.collisions = [self.cube]

            # # Initialice the Pedro robot arm and the variables for his control
            # self.kinova_pedro = rtb.models.KinovaGen3()
            # self.kinova_pedro.q = [1.13, 4.71 - 2*np.pi, np.pi/2, 3.7 - 2*np.pi, 0, 5.41 - 2*np.pi, np.pi/2]
            # T = sm.SE3.Rx(90, 'deg') * sm.SE3(0, 1.05, 0.03)
            # self.kinova_pedro.base = T
            # self.env.add(self.kinova_pedro)
            #
            # self.goal_axes_pedro = sg.Axes(0.1)
            # self.ee_axes_pedro = sg.Axes(0.1)
            # self.ee_axes_pedro.T = self.kinova_pedro.fkine(self.kinova_pedro.q)
            # self.rot_pedro = self.kinova_pedro.fkine(self.kinova_pedro.q).R  # Rotation matrix of the end-effector of pedro
            # self.Tep_pedro = sm.SE3.Rt(self.rot_pedro, [0.455, -0.1, 0.60])
            # self.goal_axes_pedro.T = self.Tep_pedro
            # self.env.add(self.ee_axes_pedro)
            # self.env.add(self.goal_axes_pedro)
            #
            # # Initialice the Pablo robot arm and the variables for his control
            # self.kinova_pablo = rtb.models.KinovaGen3()
            # self.kinova_pablo.q = [2.00, np.pi/2, np.pi/2, 3.7 - 2*np.pi, 0, 5.41 - 2*np.pi, np.pi/2]
            # T = sm.SE3.Rx(270, 'deg') * sm.SE3(0, -1.05, 0.03)
            # self.kinova_pablo.base = T
            # self.env.add(self.kinova_pablo)
            #
            # self.goal_axes_pablo = sg.Axes(0.1)
            # self.ee_axes_pablo = sg.Axes(0.1)
            # self.ee_axes_pablo.T = self.kinova_pablo.fkine(self.kinova_pablo.q)
            # self.rot_pablo = self.kinova_pablo.fkine(self.kinova_pablo.q).R  # Rotation matrix of the end-effector of pedro
            # self.Tep_pablo = sm.SE3.Rt(self.rot_pablo, [0.455, 0.1, 0.60])
            # self.goal_axes_pablo.T = self.Tep_pablo
            # self.env.add(self.ee_axes_pablo)
            # self.env.add(self.goal_axes_pablo)

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
        print("///////////////////////////////////////////////////////////////////////////////////////////")
        self.last_time = time.time() * 1000
        arrived, qd = self.step_robot(self.p3bot, self.Tep.A)
        self.p3bot.qd = qd
        self.env.step(0.05)

        self.omnirobot_proxy.setSpeedBase(0, qd[1]*1000, -qd[0])

        angular_vel = self.imu_proxy.getAngularVel()

        print(f"Webots angular velocity: {angular_vel}/ Angular velocity sended to omnirobot: {qd[0]}")

        base_new = self.p3bot.fkine(self.p3bot._q, end=self.p3bot.links[2])
        self.p3bot._T = base_new.A
        self.p3bot.q[:2] = 0

        if arrived:
            self.loop_count += 1
            if self.loop_count % 2 == 0:
                self.change_target([0, 2.2, 0.70], sm.SO3.Rx(90, 'deg') * sm.SO3.Ry(180, 'deg') * sm.SO3.Rz(0, 'deg'))
            # if self.loop_count % 4 == 1:
            #     self.change_target([2.5, 0.455, 1.0], sm.SO3.Rx(90, 'deg') * sm.SO3.Ry(90, 'deg') * sm.SO3.Rz(0, 'deg'))
            if self.loop_count % 2 == 1:
                self.change_target([0, -2.2, 0.70], sm.SO3.Rx(90, 'deg') * sm.SO3.Ry(0, 'deg') * sm.SO3.Rz(0, 'deg'))
            # if self.loop_count % 4 == 3:
            #     self.change_target([-2.0, 0.455, 1.30], sm.SO3.Rx(90, 'deg') * sm.SO3.Ry(-90, 'deg') * sm.SO3.Rz(0, 'deg'))

        print(f"Compute time required:{(time.time() * 1000) - self.last_time}")

        return True

    def startup_check(self):
        print(f"Testing RoboCompIMU.Acceleration from ifaces.RoboCompIMU")
        test = ifaces.RoboCompIMU.Acceleration()
        print(f"Testing RoboCompIMU.Gyroscope from ifaces.RoboCompIMU")
        test = ifaces.RoboCompIMU.Gyroscope()
        print(f"Testing RoboCompIMU.Magnetic from ifaces.RoboCompIMU")
        test = ifaces.RoboCompIMU.Magnetic()
        print(f"Testing RoboCompIMU.Orientation from ifaces.RoboCompIMU")
        test = ifaces.RoboCompIMU.Orientation()
        print(f"Testing RoboCompIMU.DataImu from ifaces.RoboCompIMU")
        test = ifaces.RoboCompIMU.DataImu()
        print(f"Testing RoboCompOmniRobot.TMechParams from ifaces.RoboCompOmniRobot")
        test = ifaces.RoboCompOmniRobot.TMechParams()
        print(f"Testing RoboCompRoboticsToolboxController.JointStates from ifaces.RoboCompRoboticsToolboxController")
        test = ifaces.RoboCompRoboticsToolboxController.JointStates()
        QTimer.singleShot(200, QApplication.instance().quit)

    def change_target(self, translate, rot):
        # Change the target position of the end-effector
        self.Tep = sm.SE3.Rt(rot, [translate[0], translate[1], translate[2]])
        self.goal_axes.T = self.Tep
        self.env.add(self.goal_axes)

    def step_robot(self, r: rtb.ERobot, Tep):

        wTe = r.fkine(r.q)

        eTep = np.linalg.inv(wTe) @ Tep

        # Spatial error
        et = np.sum(np.abs(eTep[:3, -1]))

        # print("Spatial error: ", et)

        # Gain term (lambda) for control minimisation
        Y = 0.01

        # Quadratic component of objective function
        Q = np.eye(r.n + 6)

        # Joint velocity component of Q
        Q[: r.n, : r.n] *= Y
        Q[:3, :3] *= 1.0 / et

        # Slack component of Q
        Q[r.n:, r.n:] = (1.0 / et) * np.eye(6)

        v, _ = rtb.p_servo(wTe, Tep, 1.5)

        v[3:] *= 1.3

        # The equality contraints
        Aeq = np.c_[r.jacobe(r.q), np.eye(6)]
        beq = v.reshape((6,))

        # The inequality constraints for joint limit avoidance
        Ain = np.zeros((r.n + 6, r.n + 6))
        bin = np.zeros(r.n + 6)

        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.1

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        # Form the joint limit velocity damper
        Ain[: r.n, : r.n], bin[: r.n] = r.joint_velocity_damper(ps, pi, r.n)

        # Linear component of objective function: the manipulability Jacobian
        c = np.concatenate(
            (np.zeros(2), -r.jacobm(start=r.links[3]).reshape((r.n - 2,)), np.zeros(6))
        )

        # Get base to face end-effector
        kε = 0.5
        bTe = r.fkine(r.q, include_base=False).A
        θε = math.atan2(bTe[1, -1], bTe[0, -1])
        ε = kε * θε
        c[0] = -ε

        # The lower and upper bounds on the joint velocity and slack variable
        lb = -np.r_[r.qdlim[: r.n], 10 * np.ones(6)]
        ub = np.r_[r.qdlim[: r.n], 10 * np.ones(6)]

        # Solve for the joint velocities dq
        qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver="piqp").copy()
        qd = qd[: r.n]
        # print(qd)

        if et > 0.5:
            qd *= 0.7 / et
        else:
            qd *= 1.4

        if et < 0.02:
            return True, qd
        else:
            return False, qd

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of calculateVelocities method from RoboticsToolboxController interface
    #
    def RoboticsToolboxController_calculateVelocitiesPablo(self, angles, targetPosition):
        ret = ifaces.RoboCompRoboticsToolboxController.JointStates()

        error = np.sum(np.rad2deg(np.abs(np.array(angles) - np.array(self.kinova_pablo.q))))
        if error > 2.8:
            self.kinova_pablo.q = np.array(angles)

        Te = self.kinova_pablo.fkine(self.kinova_pablo.q)

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
        lim = np.deg2rad(10)
        qdlim = [lim, lim, lim, lim, lim, lim, lim]  # inventadas
        lb = -np.r_[qdlim, 10 * np.ones(6)]
        ub = np.r_[qdlim, 10 * np.ones(6)]

        # Solve for the joint velocities dq
        try:
            qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='piqp')
        except e:
            print(e)

        self.kinova_pablo.qd[:self.n] = qd[:self.n]

        self.ee_axes_pablo.T = Te

        print(targetPosition - self.kinova_pablo.fkine(self.kinova_pablo.q).t)

        if arrived:
            self.kinova_pablo.qd = [0, 0, 0, 0, 0, 0, 0]
            ret.jointVelocities = [0, 0, 0, 0, 0, 0, 0]
        else:
            ret.jointVelocities = self.kinova_pablo.qd.tolist()

        ret.arrived = arrived

        return ret

    #
    # IMPLEMENTATION of calculateVelocitiesPedro method from RoboticsToolboxController interface
    #
    def RoboticsToolboxController_calculateVelocitiesPedro(self, angles, targetPosition=[0.455, -0.1, 0.60]):
        ret = ifaces.RoboCompRoboticsToolboxController.JointStates()

        error = np.sum(np.rad2deg(np.abs(np.array(angles) - np.array(self.kinova_pedro.q))))
        if error > 2.8:
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
        lim = np.deg2rad(10)
        qdlim = [lim, lim, lim, lim, lim, lim, lim]  # inventadas
        lb = -np.r_[qdlim, 10 * np.ones(6)]
        ub = np.r_[qdlim, 10 * np.ones(6)]

        # Solve for the joint velocities dq
        try:
            qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='piqp')
        except e:
            print(e)

        self.kinova_pedro.qd[:self.n] = qd[:self.n]

        self.ee_axes_pedro.T = Te

        if arrived:
            self.kinova_pedro.qd = [0, 0, 0, 0, 0, 0, 0]
            ret.jointVelocities = [0, 0, 0, 0, 0, 0, 0]
        else:
            ret.jointVelocities = self.kinova_pedro.qd.tolist()

        ret.arrived = arrived

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
    # From the RoboCompIMU you can call this methods:
    # self.imu_proxy.getAcceleration(...)
    # self.imu_proxy.getAngularVel(...)
    # self.imu_proxy.getDataImu(...)
    # self.imu_proxy.getMagneticFields(...)
    # self.imu_proxy.getOrientation(...)
    # self.imu_proxy.resetImu(...)

    ######################
    # From the RoboCompIMU you can use this types:
    # RoboCompIMU.Acceleration
    # RoboCompIMU.Gyroscope
    # RoboCompIMU.Magnetic
    # RoboCompIMU.Orientation
    # RoboCompIMU.DataImu

    ######################
    # From the RoboCompOmniRobot you can call this methods:
    # self.omnirobot_proxy.correctOdometer(...)
    # self.omnirobot_proxy.getBasePose(...)
    # self.omnirobot_proxy.getBaseState(...)
    # self.omnirobot_proxy.resetOdometer(...)
    # self.omnirobot_proxy.setOdometer(...)
    # self.omnirobot_proxy.setOdometerPose(...)
    # self.omnirobot_proxy.setSpeedBase(...)
    # self.omnirobot_proxy.stopBase(...)

    ######################
    # From the RoboCompOmniRobot you can use this types:
    # RoboCompOmniRobot.TMechParams

    ######################
    # From the RoboCompRoboticsToolboxController you can use this types:
    # RoboCompRoboticsToolboxController.JointStates


