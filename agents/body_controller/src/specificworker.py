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
        self.directKinematic = self.configData["directKinematic"]
        assert self.simulated in [0, 1, 2], f"Simulated must be #0:swift, 1:webots, 2:real, dont {self.simulated}"
        print(self.simulated)
        self.pose = None


        #region Objects and poses
        self.cubes_positions = [sm.SE3.Trans(0.0, 0.0, 0.20), sm.SE3.Trans(-0.10, 0, 0.7), sm.SE3.Trans(-0.125, -0., 1)]
        self.collisions = [sg.Cuboid((0.46, 0.46, 0.40), pose=self.cubes_positions[0], color=(1, 0, 0)),
                           sg.Cuboid((0.20, 0.20, 0.750), pose=self.cubes_positions[1], color=(1, 0, 0)),
                           sg.Cuboid((0.10, 0.10, 0.750), pose=self.cubes_positions[2], color=(1, 0, 0))]

        self.home =  np.radians(np.array([[50,-125,55,-130,-20,-65, 85], [-50,-125,-55,-130,20,-65, 85]], dtype=np.float32))
        self.pick =  np.radians(np.array([[90,-125,80,-130,-20, 45, 85], [-90,-125,-80,-130, 20, 45, 85]], dtype=np.float32))


        #endregion

        #region DSR
        self.rt = rt_api(self.g)
        try:
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)
        #endregion

        # Kinova Gen 3 robot initialization
        self.kinova_arms = [None, None]
        if self.simulated==2:
            self.kinova_arms = [KinovaGen3(configData["kinova_right_arm_ip"]), KinovaGen3(configData["kinova_left_arm_ip"])]
        elif self.simulated==1:
            self.kinova_arms = [self.kinovaarm_proxy, self.kinovaarm1_proxy]

        if startup_check:
            self.startup_check()
        else:

            self.env = swift.Swift()
            self.env.launch(realtime=True)
            self.env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])

            for colision in self.collisions:
                self.env.add(colision)

            #region P3Bot
            # self.p3bot = Robot.URDF("/home/robolab/software/robotics-toolbox-python/rtb-data/rtbdata/xacro/p3bot_description/urdf/P3Bot_scaled.urdf")
            self.p3bot = rtb.models.P3Bot()
            self.p3bot.qdlim[:2] = [ 1.5, 0.4]

            T = sm.SE3(0, 0, 0.04)
            Rz = sm.SE3.Rz(1.57)
            self.p3bot.base = T * Rz
            self.env.add(self.p3bot)
            #endregion

            #region Tool Points
            self.toolPoints = []
            for i in range(2):
                self.toolPoints.append(self.p3bot.ets(end=self.p3bot.grippers[i]))
                frame = sg.Axes(0.1, pose=self.p3bot.grippers[i].tool)
                frame.attach_to(self.p3bot.grippers[i].links[0])
                self.env.add(frame)
            #endregion

            for arm in range(len(self.kinova_arms)): self.set_joints(arm, self.home[arm])
            
            self.gain = np.array([1, 1, 1, 1.6, 1.6, 1.6])

            # # for link in self.p3bot.ee_links:
            # #     print(f"Link {link.name} has mass {link.m} and inertia {link.I}")

            #region getGoal
            self.goal_axes = sg.Axes(0.1)
            self.target = None
            if self.automatic:
                table_node = self.g.get_node("Table1")
                if table_node is None:
                    print("Table node not found")
                    return
                target_edge = Edge(table_node.id, ROBOT_DSR[1], "TARGET", self.agent_id)
                self.g.insert_or_assign_edge(target_edge)
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
            self.loop_count = 1
            #endregion

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
            # print("Valores articulares actuales:", self.p3bot.link_dict)


    def __del__(self):
        """Destructor"""

    def set_velocity_joints(self, arm: int, velocity: list[float]) -> None:
        """Set the velocity of the robot arm joints.

        Args:
            arm (int): Index of the robot arm (0-based).
            velocity (list[float]): List of velocities (in rad/s) for each joint.

        Raises:
            AssertionError: If the arm index is out of bounds.
            Exception: If setting velocity fails (logged to console).
        """
        assert arm < len(self.kinova_arms), f"Robot has {len(self.kinova_arms)} arms, tried to access arm {arm + 1}"
        print(velocity)
        
        try:
            match self.simulated:
                case 0:  # Simulation mode 0
                    self.p3bot.qd[2 + arm * 7 : 9 + arm * 7] = velocity
                case 1:  # Real robot via proxy
                    speed = ifaces.RoboCompKinovaArm.TJointSpeeds(jointSpeeds=ifaces.RoboCompKinovaArm.Speeds(velocity))
                    self.kinova_arms[arm].moveJointsWithSpeed(speed)
                case 2:  # Real robot via API
                    self.kinova_arms[arm].move_joints_with_speeds(np.degrees(velocity))
        except Exception as e:
            console.print(Text(f"Failed to set joint velocities: {e}", "red"))
            console.print_exception()
        finally:
            self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)


    def set_joints(self, arm: int, pose: list[float]) -> None:
        """Move the robot arm to the specified joint angles.

        Args:
            arm (int): Index of the robot arm (0-based).
            pose (list[float]): Target joint angles (in radians).

        Raises:
            AssertionError: If the arm index is out of bounds.
            Exception: If setting joint angles fails (logged to console).
        """
        assert arm < len(self.kinova_arms), f"Robot has {len(self.kinova_arms)} arms, tried to access arm {arm + 1}"
        
        try:
            match self.simulated:
                case 0:  # Simulation mode 0
                    self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = pose
                    self.env.step(0)
                case 1:  # Real robot via proxy
                    angles = ifaces.RoboCompKinovaArm.TJointAngles(jointAngles=ifaces.RoboCompKinovaArm.Angles(pose))
                    self.kinova_arms[arm].moveJointsWithAngle(angles)
                case 2:  # Real robot via API
                    self.kinova_arms[arm].move_joints_with_angles(np.degrees(pose))
        except Exception as e:
            console.print(Text(f"Failed to set joint angles: {e}", "red"))
            console.print_exception()
        finally:
            self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)



    def get_joints(self, arm: int) -> list[float]:
        """Retrieve the current joint angles of the robot arm.

        Args:
            arm (int): Index of the robot arm (0-based).

        Returns:
            list[float]: Current joint angles (in radians).

        Raises:
            AssertionError: If the arm index is out of bounds.
            Exception: If fetching joint angles fails (logged to console).
        """
        assert arm < len(self.kinova_arms), f"Robot has {len(self.kinova_arms)} arms, tried to access arm {arm + 1}"
        
        try:
            match self.simulated:
                case 0:  # Simulation mode 0
                    return self.p3bot.q[2 + arm * 7 : 9 + arm * 7].tolist()
                case 1:  # Real robot via proxy
                    data = self.kinovaarm_proxy.getJointsState()
                    return [joint.angle for joint in data.joints]
                case 2:  # Real robot via API
                    angles = np.array(self.kinova_right_arm.get_joints_state()["angles"])
                    angles[angles > 180] -= 360  # Normalize angles >180° to [-180°, 180°]
                    return np.radians(angles).tolist()
        except Exception as e:
            console.print(Text(f"Failed to get joint angles: {e}", "red"))
            console.print_exception()
            return []



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
            # print("update", self.pose)
            T = sm.SE3(self.pose[0:3])
            RPY = sm.SE3.RPY(self.pose[3:6])
            self.pose = None
            self.p3bot.base = T * RPY
        for arm in range(len(self.kinova_arms)):self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)
        self.update_collisions(self.p3bot.base)

        #Go to target
        if self.target is not None:
            distance = np.linalg.norm(self.p3bot.base.t - self.Tep.t)

            if self.directKinematic:
                # Tep_matrix = self.Tep.A.copy()

                # # Intercambiar y negar x e y:
                # # Nueva x = -y_original
                # # Nueva y = -x_original
                # Tep_matrix[0, 3] = self.Tep.A[1, 3]  # Nueva posición x
                # Tep_matrix[1, 3] = self.Tep.A[0, 3]  # Nueva posición y

                # # Opcional: Si también quieres intercambiar/negar la orientación
                # Tep_matrix[:2, :2] = -self.Tep.A[[1,0], :2] 

                
                arrived, qd = self.direct_kinematic_robot(self.p3bot, self.toolPoints[(self.loop_count//2) %2], self.Tep.A)
            else:
                #finish, velocity
                arrived, qd = self.step_robot(self.p3bot, self.toolPoints[(self.loop_count//2) %2], self.Tep.A)

            #Block arm to far targets
            if distance > 2.5:
                qd[2:] = [0]*(len(qd)-2)


            print(f"\rDistance: {distance:0.2f}, velocity:", end="")
            for vel in qd: print(f" {vel:0.2f}", end="") 

            # print(f"\radv:{qd[1]*1000:.2f} | rot:{qd[0]:.2f} ejes {qd[2:]}", end="")
            #Move motors
            if qd is not None:
                if self.simulated==0:
                    self.p3bot.qd[:2] = qd[:2]
                else:
                    try:
                        #self.omnirobot_proxy.setSpeedBase(0, qd[1]*1000, qd[0])
                        pass
                    except Ice.ConnectionRefusedException:
                        console.print_exception()
                print(qd)
                for arm in range(len(self.kinova_arms)): self.set_velocity_joints(arm, qd[2 + arm * 7 : 9 + arm * 7])

            self.env.step(0.05)

            base_new = self.p3bot.fkine(self.p3bot._q, end=self.p3bot.links[2])
            self.p3bot._T = base_new.A
            self.p3bot.q[:2] = 0
            
            if arrived:
                self.g.delete_edge(ROBOT_DSR[1], self.target, "TARGET")
                for arm in range(len(self.kinova_arms)): self.set_joints(arm, self.home[arm])

                if self.automatic:
                    self.loop_count += 2
                    #link root-robot
                        
                    table_node = self.g.get_node(f"Table{(self.loop_count % 4) +1}")
                    if table_node is None:
                        print("Root node not found")
                        return
                    target_edge = Edge( table_node.id, ROBOT_DSR[1], "TARGET", self.agent_id)
                    self.g.insert_or_assign_edge(target_edge)

                    # edge = self.g.get_edge(ROBOT_DSR[0], self.target, "RT")
                    # if edge is not None:
                    #     self.change_target( np.array(edge.attrs["rt_translation"].value), np.array(edge.attrs["rt_rotation_euler_xyz"].value))
        return True

    def change_target(self, translate:np.ndarray, rot:np.ndarray):
        print(f"Changed goal {translate}, {rot}")
        for arm in range(len(self.kinova_arms)): self.set_joints(arm, self.pick[arm])


        # Change the target position of the end-effector
        T = sm.SE3(translate*SCALE)
        RPY = sm.SE3.RPY(rot)

        self.Tep = T * RPY
        self.goal_axes.T = self.Tep
        self.env.add(self.goal_axes)

    def direct_kinematic_robot(self, r: rtb.ERobot, toolPoint, Tep):

        v, arrived = rtb.p_servo(r.fkine(r.q, end=r.grippers[(self.loop_count//2) %2]), Tep, gain=self.gain, threshold=0.005)

        qd = r.qd.copy()
        qd[toolPoint.jindices] = np.clip(np.linalg.pinv(toolPoint.jacobe(r.q)) @ v, 
                                         -r.qdlim[toolPoint.jindices], 
                                         r.qdlim[toolPoint.jindices])
        
        return arrived, qd
        


    def step_robot(self, r: rtb.ERobot, toolPoint, Tep, collisions=True):
        print(toolPoint.n)

        wTe = r.fkine(r.q, end=r.grippers[(self.loop_count//2) %2])

        eTep = np.linalg.inv(wTe) @ Tep

        # Spatial error
        et = np.sum(np.abs(eTep[:3, -1]))

        # print("Spatial error: ", et)

        # Gain term (lambda) for control minimisation
        Y = 0.01

        # Quadratic component of objective function
        Q = np.eye(toolPoint.n + 6)

        # Joint velocity component of Q
        Q[: toolPoint.n, : toolPoint.n] *= Y
        Q[:3, :3] *= 1.0 / et

        # Slack component of Q
        Q[toolPoint.n:, toolPoint.n:] = (1.0 / et) * np.eye(6)

        v, _ = rtb.p_servo(wTe, Tep, 1.5)

        v[3:] *= 1.3

        # The equality contraints
        Aeq = np.c_[toolPoint.jacobe(r.q), np.eye(6)]
        beq = v.reshape((6,))

        # The inequality constraints for joint limit avoidance
        Ain = np.zeros((toolPoint.n + 6, toolPoint.n + 6))
        bin = np.zeros(toolPoint.n + 6)

        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.1

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        # Form the joint limit velocity damper
        Ain[: toolPoint.n, : toolPoint.n], bin[: toolPoint.n] = r.joint_velocity_damper(ps, pi, toolPoint.n)

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
                    c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], toolPoint.n + 6 - c_Ain.shape[1]))]
                    # print(f"{i}, colision {c_Ain.shape}, {c_bin.shape}")
                    # if len(c_Ain) > 1 : vel_decay +=len(c_bin)*2

                    # Stack the inequality constraints
                    Ain = np.r_[Ain, c_Ain]
                    bin = np.r_[bin, c_bin]

        ############################

        # Linear component of objective function: the manipulability Jacobian
        c = np.concatenate(
            (np.zeros(2), -r.jacobm(start=r.links[3]).reshape((toolPoint.n - 2,)), np.zeros(6))
        )

        # Get base to face end-effector
        kε = 0.5
        bTe = r.fkine(r.q, end=r.grippers[(self.loop_count//2) %2], include_base=False).A
        θε = math.atan2(bTe[1, -1], bTe[0, -1])
        ε = kε * θε
        c[0] = -ε

        # The lower and upper bounds on the joint velocity and slack variable
        lb = -np.r_[r.qdlim[: toolPoint.n], 10 * np.ones(6)]
        ub = np.r_[r.qdlim[: toolPoint.n], 10 * np.ones(6)]

        # Solve for the joint velocities dq
        qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver="piqp")
        if qd is not None:
            ret_qd = r.qd.copy()
            ret_qd[toolPoint.jindices] = qd[toolPoint.jindices].copy()
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
            return False, np.zeros(toolPoint.n)
        return False, qd

    def startup_check(self):
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TAxis from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TAxis()
        print(f"Testing RoboCompKinovaArm.TToolInfo from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TToolInfo()
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
    # RoboCompKinovaArm.TToolInfo self.kinovaarm_proxy.getToolInfo()
    # RoboCompKinovaArm.void self.kinovaarm_proxy.moveJointsWithAngle(TJointAngles angles)
    # RoboCompKinovaArm.void self.kinovaarm_proxy.moveJointsWithSpeed(TJointSpeeds speeds)
    # RoboCompKinovaArm.void self.kinovaarm_proxy.openGripper()
    # RoboCompKinovaArm.void self.kinovaarm_proxy.setCenterOfTool(TPose pose, ArmJoints referencedTo)

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # ifaces.RoboCompKinovaArm.TPose
    # ifaces.RoboCompKinovaArm.TAxis
    # ifaces.RoboCompKinovaArm.TToolInfo
    # ifaces.RoboCompKinovaArm.TGripper
    # ifaces.RoboCompKinovaArm.TJoint
    # ifaces.RoboCompKinovaArm.TJoints
    # ifaces.RoboCompKinovaArm.TJointSpeeds
    # ifaces.RoboCompKinovaArm.TJointAngles

    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # RoboCompKinovaArm.bool self.kinovaarm1_proxy.closeGripper()
    # RoboCompKinovaArm.TPose self.kinovaarm1_proxy.getCenterOfTool(ArmJoints referencedTo)
    # RoboCompKinovaArm.TGripper self.kinovaarm1_proxy.getGripperState()
    # RoboCompKinovaArm.TJoints self.kinovaarm1_proxy.getJointsState()
    # RoboCompKinovaArm.TToolInfo self.kinovaarm1_proxy.getToolInfo()
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.moveJointsWithAngle(TJointAngles angles)
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.moveJointsWithSpeed(TJointSpeeds speeds)
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.openGripper()
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.setCenterOfTool(TPose pose, ArmJoints referencedTo)

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # ifaces.RoboCompKinovaArm.TPose
    # ifaces.RoboCompKinovaArm.TAxis
    # ifaces.RoboCompKinovaArm.TToolInfo
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
        # console.print(f"UPDATE EDGE: {fr} to {to}", type, style='green')
        if self.useRTPose:
            room = self.g.get_node(fr)
            if room is not None:
                if room.name == "room_0" and to == ROBOT_DSR[1] and type == "RT":
                    try:
                        edge = self.rt.get_edge_RT(room, to)
                        if edge is not None:
                            index_new = np.argmax(edge.attrs["rt_timestamps"].value)*3
                            rot = edge.attrs["rt_rotation_euler_xyz"].value[index_new:index_new+3]
                            pose = edge.attrs["rt_translation"].value[index_new:index_new+3]
                            self.pose = np.array([pose[0]*SCALE, pose[1]*SCALE, pose[2]*SCALE, rot[0], rot[1], rot[2]+1.57])

                            # print(f"\rNew {index_new} pose X:{self.pose[0]:.2f} | Y:{self.pose[1]:.2f} | Z:{self.pose[2]:.2f} | Roll:{self.pose[3]:.2f} | Pitch:{self.pose[4]:.2f} | Yaw:{self.pose[5]:.2f}", end="")
                    except Exception as e:
                        print(f"Error procesando edge: {e}")
        if fr == ROBOT_DSR[1] and type == "TARGET":
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
        if fr == ROBOT_DSR[1] and to == self.target and type == "TARGET":
            self.target = None


