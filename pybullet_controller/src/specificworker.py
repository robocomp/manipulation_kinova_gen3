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
import pybullet as p
import pybullet_data
import time
import numpy as np
from Kinova import KinovaGen3
import os

# Set the LC_NUMERIC environment variable
os.environ['LC_NUMERIC'] = 'en_US.UTF-8'
# RMEMEMBER to SET: export LC_NUMERIC="en_US.UTF-8" in your terminal

console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        """
        of `SpecificWorker` sets up a Bullet Python environment and loads a kinova
        gen3 robot arm, floor, and table. It creates a constraint to fix the robot
        arm at home position and sets up timers for computing target joint angles
        and velocities.

        Args:
            proxy_map (object.): 2D coordinate map of a grid-based robot environment,
                which is used to simulate the robot's motion within the specified
                space.
                
                		- `proxy_map`: This is a dictionary-like object that contains
                various properties and attributes of the robot's proxy map.
                		- `startup_check`: This parameter is a boolean value indicating
                whether the startup check should be performed or not. It defaults
                to `False`.
                		- `physicsClient`: This is an instance of the `pybullet`
                `PhysicsClient` class, which represents the simulation environment
                for the robot. It is initialized with the given `proxy_map` and
                other properties.
                		- ` Period`: This is an integer value representing the time
                interval between successive computations in milliseconds.
                		- `table_id`: This is an integer value representing the unique
                ID of the table that the arm will be placed on.
                		- `robot`: This is an instance of the `KinovaGen3` class, which
                represents the robot and its end effector. It is loaded from a
                file using the `loadURDF()` function.
                		- `robot_id`: This is an integer value representing the unique
                ID of the robot.
                		- `end_effector_link_index`: This is an integer value representing
                the index of the end effector link in the robot's URDF file.
                		- `info`: This is a tuple containing information about the number
                of joints in the robot.
                		- `pose`: This is a 3D array representing the position and
                orientation of the robot's end effector in global coordinates.
                		- `joint_axis`: This is an integer value representing the axis
                of rotation for each joint in the robot.
                		- `parentFramePosition`, `childFramePosition`, `parentFrameOrientation`,
                and `childFrameOrientation`: These are 3D arrays representing the
                position and orientation of the parent frame and child frame in
                global coordinates, respectively.
                
            startup_check (bool): start check of the pybullet environment and runs
                it in GUI mode when set to True and skips it when set to False.

        """
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 0
        if startup_check:
            self.startup_check()
        else:

            # Start PyBullet in GUI mode
            self.physicsClient = p.connect(p.GUI)
            # Set the path to PyBullet data
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.setGravity(0, 0, -9.81)
            flags = p.URDF_USE_INERTIA_FROM_FILE
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=50, cameraPitch=-35,
                                         cameraTargetPosition=[0, 0, 0.5])

            # load floor
            self.plane = p.loadURDF("plane.urdf")

            # Load a table to place the arm on
            self.table_id = p.loadURDF("/home/robocomp/software/bullet3/data/table/table.urdf", basePosition=[0, 0, 0],
                                       baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)

            # Load Kinova arm
            self.robot = KinovaGen3()

            # Load Kinova arm
            # self.robot_id = p.loadURDF(
            #     "/home/robocomp/robocomp/components/manipulation_kinova_gen3/pybullet_controller/gen3_robotiq_2f_140.urdf",
            #     basePosition=[0, 0.3, 0.6], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), flags=flags)
            # self.end_effector_link_index = 9
            #
            # # get number of robot joints
            # info = p.getNumJoints(self.robot_id)
            # # for each joint, get the joint info
            # for j in range(info):
            #     info = p.getJointInfo(self.robot_id, j) # joint index
            #     pose = p.getJointState(self.robot_id, j)
            #     print("Joint ", j, "pos = ",  pose[0], info)

            constraint_id = p.createConstraint(
                parentBodyUniqueId=self.table_id,
                parentLinkIndex=-1,  # Use -1 for the base of the object
                childBodyUniqueId=self.robot.robot_id,
                childLinkIndex=-1,  # Use -1 for the base of the object
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],  # Not relevant for fixed joints
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, -0.3, 0.6],  # This needs to match how the arm is offset when loaded
                parentFrameOrientation=[0, 0, 0, 1],
                childFrameOrientation=[0, 0, 0, 1]
            )

            # move arm home
            self.robot.set_home()
            # wait for half a second
            time.sleep(0.5)

            self.target_angles = self.robot.home_angles
            self.target_velocities = [0.0] * 7
            self.joy_selected_joint = 0

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
            print("SpecificWorker started")

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        """
        sets the inner model of an instance by calling the `InnerModel` constructor
        with a parameter containing the path to the model file. If any error occurs
        during the construction, it handles the exceptions and prints an error
        message instead. The function returns `True`.

        Args:
            params (dict): configuration parameters for the code, which are used
                to initialize the inner model.

        Returns:
            `True` value.: `True`.
            
            		- `True`: This is the return value indicating successful configuration
            of parameters.
            		- `self.innermodel`: This attribute contains the `InnerModel` instance
            created from the provided `params["InnerModelPath"]`.
            

        """
        return True


    @QtCore.Slot()
    def compute(self):

        # print("Errors:",
        #       [round(x - y, 2) for x, y in
        #        zip(self.robot.get_actual_control_joints_velocity(), self.target_velocities[:7])], self.target_velocities)

        """
        computes and adjusts joint control angles based on target angles and error
        thresholds, and then updates the robot's actual control joint angles using
        the move_joints method.

        """
        print("Errors:",
              [round(x - y, 2) for x, y in
               zip(self.robot.get_actual_control_joints_angle(), self.target_angles[:7])],
              self.target_angles[:7])

        # self.robot.move_joints_control_vel( joint_param_value=self.target_velocities,
        #                                     desired_force_per_one_list=[1],
        #                                     desired_vel_per_one_list=[1],
        #                                     wait=True,
        #                                     counter_max=10 ** 2,
        #                                     error_threshold=0.005)

        (self.robot.move_joints(joint_param_value=self.target_angles,
                                         desired_force_per_one_list=[1],
                                         desired_vel_per_one_list=[1],
                                         wait=True,
                                         counter_max=10 ** 2,
                                         error_threshold=0.3))

        # print(self.target_angles[:7])
        # print([round(x, 2) for x in self.robot.get_actual_control_joints_angle()])
        # print("Errors:", [round(x-y, 2) for x,y in zip(self.robot.get_actual_control_joints_angle(), self.target_angles[:7])])

        # print("JOINT: ", self.joy_selected_joint)
        # print("-------------------")

        p.stepSimulation()

    # =============== Methods ==================

    def startup_check(self):
        """
        tests and initializes variables related to a RoboCompKinovaArm and its
        gripper. It prints the test results to the console and schedules an
        application quit after 200 milliseconds.

        """
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TGripper from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TGripper()
        QTimer.singleShot(200, QApplication.instance().quit)

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
        is used to close the gripper of a robotic arm, enabling it to interact
        with its environment by manipulating objects or performing tasks.

        """
        pass


    #
    # IMPLEMENTATION of getCenterOfTool method from KinovaArm interface
    #
    def KinovaArm_getCenterOfTool(self, referencedTo):
        """
        calculates the center of a tool referenced to a specific interface of a
        Kinova Arm.

        Args:
            referencedTo (`ifaces.RoboCompKinovaArm.TPose` object.): 3D position
                of the reference frame to which the arm's center should be aligned.
                
                		- `referencedTo`: A reference to an instance of `RoboCompKinovaArm`,
                representing the arm to which the center of the tool will be computed.
                
                	The function computes and returns the center of the tool referenced
                to the specified arm, following the input parameters.
                

        Returns:
            `TPose`.: a `RoboCompKinovaArm.TPose` object containing the center of
            the tool referenced to the arm.
            
            		- `TPose`: This is a `RoboCompKinovaArm.TPose` object, which represents
            the center of the tool in relation to the robot's end effector.
            		- `x`: The x-coordinate of the center of the tool.
            		- `y`: The y-coordinate of the center of the tool.
            		- `z`: The z-coordinate of the center of the tool.
            
            	These coordinates represent the position of the tool's center relative
            to the robot's base, which can be used for further processing or
            analysis in the application.
            

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
        """
        retrieves the current gripper state of a RoboComp Kinova Arm.

        Returns:
            instance of `ifaces.RoboCompKinovaArm.TGripper`.: a `TGripper` object
            containing information about the gripper state of the Kinova arm.
            
            	1/ `ret`: This is the `TGripper` object that represents the current
            state of the gripper. It has several attributes and methods related
            to the gripper's position, velocity, acceleration, force, and other
            aspects of its behavior.
            	2/ `RoboCompKinovaArm`: This is the class that `ret` belongs to. It
            provides a set of methods for interacting with the Kinova Arm, including
            `getGripperState`, which returns the current state of the gripper.
            	3/ `TGripper`: This is a class that represents the gripper's state
            as a set of values that describe its position, velocity, and other
            attributes. It is the output type of the `KinovaArm_getGripperState`
            function.
            

        """
        ret = ifaces.RoboCompKinovaArm.TGripper()
        #
        # write your CODE here
        #
        return ret
    #
    # IMPLEMENTATION of openGripper method from KinovaArm interface
    #
    def KinovaArm_openGripper(self):

        #
        # write your CODE here
        #
        """
        appears to open the gripper mechanism of an robotic arm.

        """
        pass


    #
    # IMPLEMENTATION of setCenterOfTool method from KinovaArm interface
    #
    def KinovaArm_setCenterOfTool(self, pose, referencedTo):

        #
        # write your CODE here
        #
        """
        sets the center of a tool referenced to a specific pose in space.

        Args:
            pose (3D vector.): 7-dimensional position of the tool relative to the
                end effector of the robot, which is referenced to a specific frame
                of reference.
                
                		- `pose`: A `Pose` object containing the desired position and
                orientation of the tool relative to the base link.
                
            referencedTo (`kinova.Arm.ReferenceFrame`.): 3D reference frame for
                the tool center point, which is used to transform the pose from
                the world coordinate system to the tool coordinate system.
                
                		- `referencedTo`: A reference to an instance of `KinovaTool` or
                its subclass, which contains information about a tool used in the
                arm's task.
                

        """
        pass

    # =============== Methods for SubscribesTo ================
    # =========================================================
    #
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    #
    def JoystickAdapter_sendData(self, data):
        """
        receives input data from a joystick and updates the position of a gripper
        based on the joystick's axis values.

        Args:
            data (`Axis`.): 3D orientation data of a joystick or game controller,
                which is used to update the joint angles of the robot's end effector
                based on the user inputs.
                
                		- `axes`: A list of axes in the joystick, each represented as a
                dictionary with name and value attributes.
                		+ `name`: The axis name (e.g., "mode", "X_axis", etc.).
                		+ `value`: The current value of the axis (either a float value
                or -1 if the axis is not present in the input data).
                		- `joy_selected_joint`: An integer variable representing the
                currently selected joint from 0 to 6.
                		- `target_angles`: A list of angle values for each of the 7
                joints, initialized to 0.
                

        """
        for axis in data.axes:
            match axis.name:
                case "mode":
                    if axis.value > 0.5:
                        self.joy_selected_joint = (self.joy_selected_joint + 1) % 7
                    elif axis.value < -0.5:
                        self.joy_selected_joint = (self.joy_selected_joint - 1) % 7
                case "X_axis":
                    pass
                case "Y_axis":
                    self.target_angles[self.joy_selected_joint] += axis.value
                case "Z_axis":
                    pass
                case "gripper":
                    pass
        print(self.joy_selected_joint, self.target_angles[:7])


















 #self.set_joints_positions([x for x in range(22)], np.zeros(22).tolist())
        #self.print_joints_positions()

        # Get the state of the link, including its orientation
        #end_effector_link_state = p.getLinkState(self.robot_id, self.end_effector_link_index)
        #link_world_position = end_effector_link_state[0]  # World position of the link's frame
        #link_world_orientation = end_effector_link_state[1]  # World orientation of the link's frame (quaternion)

        # Calculate the inverse kinematics solution (joint positions)
        #target_position = link_world_position + np.array([0.0, -0.1, -0.1])  # Target position 10 cm above the current position
        #target_position[0] = 0.3

        #target_position = [0, -0.3, 1]
        #joint_positions = p.calculateInverseKinematics(self.robot_id,
        #                                               self.end_effector_link_index,
        #                                               target_position)
        # for joint_index, joint_position in enumerate(joint_positions):
        #     p.setJointMotorControl2(self.robot_id, joint_index, p.POSITION_CONTROL, targetPosition=joint_position,
        #                             force=1000)
        #
        # self.print_joints_positions([x for x in range(7)])
        #time.sleep(0.01)