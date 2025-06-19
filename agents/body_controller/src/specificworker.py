#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by YOUR NAME HERE
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
from itertools import count

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from rich.text import Text
from genericworker import *
import interfaces as ifaces
from time import time, sleep

sys.path.append('/opt/robocomp/lib')
sys.path.append('src')
console = Console(highlight=False)

from pydsr import *
import math

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
import spatialgeometry as sg
from roboticstoolbox import Robot

from kinova_gen3 import KinovaGen3

try:
    import setproctitle
    setproctitle.setproctitle(os.path.basename(os.getcwd()))
except:
    pass


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel
SCALE = 0.001
ROBOT_DSR = ("robot", 200)



class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = self.configData["Period"]["Compute"]
        self.useRTPose = self.configData["useRTPose"]
        self.automatic = self.configData["automatic"]
        self.simulated = self.configData["simulated"]
        assert self.simulated in [0, 1, 2], f"Simulated must be #0:swift, 1:webots, 2:real, dont {self.simulated}"
        print(self.simulated)
        self.pose = None

        self.cubes_positions = [sm.SE3.Trans(0.0, 0.0, 0.20), sm.SE3.Trans(-0.10, 0, 0.7), sm.SE3.Trans(-0.125, -0., 1)]

        self.collisions = [sg.Cuboid((0.46, 0.46, 0.40), pose=self.cubes_positions[0], color=(1, 0, 0)),
                           sg.Cuboid((0.20, 0.20, 0.750), pose=self.cubes_positions[1], color=(1, 0, 0)),
                           sg.Cuboid((0.10, 0.10, 0.750), pose=self.cubes_positions[2], color=(1, 0, 0))]

        self.home =  np.radians(np.array([50,-125,55,-130,-20,-65,85]))
        self.pick =  np.radians(np.array([90,-125,55,-130,-20, 65,85]))

        try:
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)

        # Kinova Gen 3 robot initialization
        
        if self.simulated==2:
            self.kinova_right_arm = KinovaGen3(configData["kinova_right_arm_ip"])
            #self.kinova_left_arm = KinovaGen3(configData["kinova_left_arm_ip"])
            # self.kinova_right_arm.list_posibles_actions()

        if startup_check:
            self.startup_check()
        else:

            self.env = swift.Swift()
            self.env.launch(realtime=True)
            self.env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])

            for colision in self.collisions:
                self.env.add(colision)

            # self.p3bot = Robot.URDF("/home/robolab/software/robotics-toolbox-python/rtb-data/rtbdata/xacro/p3bot_description/urdf/P3Bot_scaled.urdf")
            self.p3bot = rtb.models.P3Bot()
            self.p3bot.qdlim = np.array([ 1.5, 0.4, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])

            T = sm.SE3(0, 0, 0.04)
            Rz = sm.SE3.Rz(1.57)
            self.p3bot.base = T * Rz
            self.env.add(self.p3bot)

            self.set_joints(self.home)

            # # for link in self.p3bot.ee_links:
            # #     print(f"Link {link.name} has mass {link.m} and inertia {link.I}")
            self.goal_axes = sg.Axes(0.1)
            self.target = None
            if self.automatic:
                edge = self.g.get_edge(ROBOT_DSR[0], "Table1", "RT")
                self.target = "Table1"
                if edge is not None:
                    self.change_target( np.array(edge.attrs["rt_translation"].value), np.array(edge.attrs["rt_rotation_euler_xyz"].value))
                else:
                    self.change_target(np.array([0, 2.2, 0.70]), np.array([1.57, 3.1416, 0]))
            else:
                targets = self.g.get_edges_by_type("TARGET")
                print("Targets encontrados:", targets)
                for t in targets:
                    if t == ROBOT_DSR[1]:
                        pose = t.attrs["rt_translation"].value
                        rot = t.attrs["rt_rotation_euler_xyz"].value
                        self.change_target(rot=rot, translate=pose)
                        self.target = t.to
                        break

            self.loop_count = 0
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
            # print("Valores articulares actuales:", self.p3bot.link_dict)


    def __del__(self):
        """Destructor"""
    def set_joints(self, pose):
        """ Move the arm to a specified pose.
        :param pose: joins poses in degrees.
        """
        print(f"set{pose}")
        match self.simulated:
            case 0:
                self.p3bot.q[2:] = pose
            #TODO make to interface
            case 1:
                speeds = ifaces.TJointSpeeds()
                self.kinovaarm_proxy.moveJointsWithSpeed(speeds)
            case 2:
                try:
                    self.kinova_right_arm.move_joints_with_angles(np.degrees(pose)) 
                    pass
                except:
                    pass
        self.p3bot.q[2:] = self.get_joints()
        


    def get_joints(self):
        """ Move the arm to a specified pose. 7 JOINS
        """
        match self.simulated:
            case 0:
                return self.p3bot.q[2:]
            #TODO make to interface
            case 1:
               return self.kinovaarm_proxy.getJointsState()
            case 2:
                try:
                    angles = np.array(self.kinova_right_arm.get_joints_state()["angles"])
                    mask = angles>180
                    angles[mask] = angles[mask]-360
                    return np.radians(angles).tolist()
                    
                except:
                    pass



    def noisePose(self):
        current_base = self.p3bot.base  # Transformación actual (sm.SE3)
        # Ruido gaussiano en X, Y, Z (media=0, desviación estándar=0.01 metros)
        translation_noise = np.random.normal(0, 0.001, size=3)  # [Δx, Δy, Δz]
        # Crear una transformación de traslación con el ruido
        T_noise = sm.SE3(translation_noise)
        # Ruido en la rotación (pequeño ángulo en radianes, ej: σ=0.1 rad)
        rotation_noise_z = np.random.normal(0, 0.01)  # Ruido en rotación Z
        # Crear una transformación de rotación con el ruido
        R_noise = sm.SE3.Rz(rotation_noise_z)
        self.p3bot.base = current_base * R_noise * T_noise

    def update_collisions(self, pose:sm.SE3.Trans):
        for i in range(len(self.collisions)):
            self.collisions[i].T = pose * self.cubes_positions[i]

    @QtCore.Slot()
    def compute(self):

        #Update pose in swift
        if self.pose is not None:
            T = sm.SE3(self.pose[0:3])
            RPY = sm.SE3.RPY(self.pose[3:6])
            self.pose = None
            self.p3bot.base = T * RPY
        self.p3bot.q[2:] = self.get_joints()
        self.update_collisions(self.p3bot.base)

        #Go to target
        if self.target is not None:
            distance = np.linalg.norm(self.p3bot.base.t - self.Tep.t)
            print(distance)

            #finish, velocity
            arrived, qd = self.step_robot(self.p3bot, self.Tep.A)

            #Block arm to far targets
            if distance > 2.5:
                qd[2:] = [0]*7
            self.p3bot.qd = qd

            self.env.step(0.05)
            print(self.p3bot.qd)

            # print(f"\radv:{qd[1]*1000:.2f} | rot:{qd[0]:.2f} ejes {qd[2:]}", end="")
            #Move motors
            if qd is not None:
                try:
                    #self.omnirobot_proxy.setSpeedBase(0, qd[1]*1000, qd[0])
                    pass
                except Ice.ConnectionRefusedException:
                    pass
                if self.simulated >0: self.kinova_right_arm.move_joints_with_speeds(np.degrees(self.p3bot.qd[2:])) 

            base_new = self.p3bot.fkine(self.p3bot._q, end=self.p3bot.links[2])
            self.p3bot._T = base_new.A
            self.p3bot.q[:2] = 0
            
            if arrived:
                self.set_joints(self.home) 
                if self.automatic:
                    self.loop_count += 2

                    self.target =  f"Table{(self.loop_count % 4) +1}"

                    edge = self.g.get_edge(ROBOT_DSR[0], self.target, "RT")
                    if edge is not None:
                        self.change_target( np.array(edge.attrs["rt_translation"].value), np.array(edge.attrs["rt_rotation_euler_xyz"].value))
                else:
                    self.g.delete_edge(ROBOT_DSR[1], self.target, "TARGET")
        return True

    def change_target(self, translate:np.ndarray, rot:np.ndarray):
        print(f"Changed goal {translate}, {rot}")
        self.set_joints(self.pick)

        # Change the target position of the end-effector
        T = sm.SE3(translate*SCALE)
        RPY = sm.SE3.RPY(rot)

        self.Tep = T * RPY
        self.goal_axes.T = self.Tep
        self.env.add(self.goal_axes)

    def step_robot(self, r: rtb.ERobot, Tep, collisions=True):

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

        rot_boost = 1
        vel_decay = 1

        #################COLISIONS##################
        if collisions:
            for i, collision in enumerate(self.collisions):
                c_Ain, c_bin = self.p3bot.link_collision_damper(
                        collision,
                        self.p3bot.q,
                        di=0.1, # Distancia mínima más pequeña (ej: 0.1 metros)
                        ds=0.05, # Ganancia más alta (ej: 0.1)
                        xi=1, # Mayor peso en la optimización
                        start= self.p3bot.link_dict["right_arm_half_arm_1_link"],
                        end= self.p3bot.link_dict["right_arm_bracelet_link"]
                    )

                # If there are any parts of the robot within the influence distance
                # to the collision in the scene
                if c_Ain is not None and c_bin is not None:
                    c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], r.n + 6 - c_Ain.shape[1]))]
                    print(f"{i}, colision {c_Ain.shape}, {c_bin.shape}")
                    # if len(c_Ain) > 1 : vel_decay +=len(c_bin)*2

                    # Stack the inequality constraints
                    Ain = np.r_[Ain, c_Ain]
                    bin = np.r_[bin, c_bin]

        ############################

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
        qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver="piqp")
        if qd is not None:
            qd = qd.copy()
            qd = qd[: r.n]
            # print("antes", qd)
            # qd[0] = qd[0] * rot_boost
            # qd[2:] = qd[2:] / vel_decay
            # print("despues", qd)

            # if et > 0.5:
            #     qd *= 0.5
            # else:
            #     qd *= et if et > 0.25 else 1

            if et < 0.02:
                return True, qd
        else:
            console.print(Text("Optimización fallida.", "yellow"))
            return False, np.zeros(r.n)
        return False, qd

    def startup_check(self):
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TGripper from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TGripper()
        print(f"Testing RoboCompKinovaArm.TJoint from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoint()
        print(f"Testing RoboCompKinovaArm.TJoints from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoints()
        print(f"Testing RoboCompKinovaArm.TJointSpeeds from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJointSpeeds()
        print(f"Testing RoboCompKinovaArm.TJointAngles from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJointAngles()
        print(f"Testing RoboCompOmniRobot.TMechParams from ifaces.RoboCompOmniRobot")
        test = ifaces.RoboCompOmniRobot.TMechParams()
        print(f"Testing RoboCompJoystickAdapter.AxisParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.AxisParams()
        print(f"Testing RoboCompJoystickAdapter.ButtonParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.ButtonParams()
        print(f"Testing RoboCompJoystickAdapter.TData from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.TData()
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to newFullPose method from FullPoseEstimationPub interface
    #
    def FullPoseEstimationPub_newFullPose(self, pose):
        if not self.useRTPose:
            #Change to  ros coordinates
            self.pose = np.array([pose.x*SCALE, pose.y*SCALE, pose.z*SCALE, pose.rx, pose.ry, pose.rz+1.57])
            # print(f"\rNew pose X:{self.pose[0]:.2f} | Y:{self.pose[1]:.2f} | Z:{self.pose[2]:.2f} | Roll:{self.pose[3]:.2f} | Pitch:{self.pose[4]:.2f} | Yaw:{self.pose[5]:.2f}", end="")


    #
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    #
    def JoystickAdapter_sendData(self, data):

        #
        # write your CODE here
        #
        pass


    # ===================================================================
    # ===================================================================



    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # RoboCompKinovaArm.bool self.kinovaarm_proxy.closeGripper()
    # RoboCompKinovaArm.TPose self.kinovaarm_proxy.getCenterOfTool(ArmJoints referencedTo)
    # RoboCompKinovaArm.TGripper self.kinovaarm_proxy.getGripperState()
    # RoboCompKinovaArm.TJoints self.kinovaarm_proxy.getJointsState()
    # RoboCompKinovaArm.void self.kinovaarm_proxy.moveJointsWithAngle(TJointAngles angles)
    # RoboCompKinovaArm.void self.kinovaarm_proxy.moveJointsWithSpeed(TJointSpeeds speeds)
    # RoboCompKinovaArm.void self.kinovaarm_proxy.openGripper()
    # RoboCompKinovaArm.void self.kinovaarm_proxy.setCenterOfTool(TPose pose, ArmJoints referencedTo)

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # ifaces.RoboCompKinovaArm.TPose
    # ifaces.RoboCompKinovaArm.TGripper
    # ifaces.RoboCompKinovaArm.TJoint
    # ifaces.RoboCompKinovaArm.TJoints
    # ifaces.RoboCompKinovaArm.TJointSpeeds
    # ifaces.RoboCompKinovaArm.TJointAngles

    ######################
    # From the RoboCompOmniRobot you can call this methods:
    # RoboCompOmniRobot.void self.omnirobot_proxy.correctOdometer(int x, int z, float alpha)
    # RoboCompOmniRobot.void self.omnirobot_proxy.getBasePose(int x, int z, float alpha)
    # RoboCompOmniRobot.void self.omnirobot_proxy.getBaseState(RoboCompGenericBase.TBaseState state)
    # RoboCompOmniRobot.void self.omnirobot_proxy.resetOdometer()
    # RoboCompOmniRobot.void self.omnirobot_proxy.setOdometer(RoboCompGenericBase.TBaseState state)
    # RoboCompOmniRobot.void self.omnirobot_proxy.setOdometerPose(int x, int z, float alpha)
    # RoboCompOmniRobot.void self.omnirobot_proxy.setSpeedBase(float advx, float advz, float rot)
    # RoboCompOmniRobot.void self.omnirobot_proxy.stopBase()

    ######################
    # From the RoboCompOmniRobot you can use this types:
    # ifaces.RoboCompOmniRobot.TMechParams

    ######################
    # From the RoboCompJoystickAdapter you can use this types:
    # ifaces.RoboCompJoystickAdapter.AxisParams
    # ifaces.RoboCompJoystickAdapter.ButtonParams
    # ifaces.RoboCompJoystickAdapter.TData



    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        console.print(f"UPDATE EDGE: {fr} to {to}", type, style='green')
        if fr == 100 and to == ROBOT_DSR[1] and type == "RT":
            try:
                edge = self.g.get_edge(fr, to, type)
                if edge is not None:
                    pose = edge.attrs["rt_translation"].value
                    rot = edge.attrs["rt_rotation_euler_xyz"].value
                    self.pose = np.array([pose[0], pose[1], pose[2], rot[2]])
                    print(f"\rNew pose X:{self.pose[0]:.2f} | Y:{self.pose[1]:.2f} | Z:{self.pose[2]:.2f} | Roll:{self.pose[3]:.2f} | Pitch:{self.pose[4]:.2f} | Yaw:{self.pose[5]:.2f}", end="")
            except Exception as e:
                print(f"Error procesando edge: {e}")
        if fr == ROBOT_DSR[1] and type == "TARGET" and not self.automatic:
            edge = self.g.get_edge(fr, to, "RT")
            if edge is not None:
                pose = edge.attrs["rt_translation"].value
                rot = edge.attrs["rt_rotation_euler_xyz"].value
                self.change_target(rot=rot, translate=pose)
                self.target = to

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
        if fr == ROBOT_DSR[1] and to == self.target and type == "TARGET" and not self.automatic:
            self.target = None


