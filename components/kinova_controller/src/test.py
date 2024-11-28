#! /usr/bin/env python3

###
#
# Kinova Gen3 Control class
#
###

import sys
import os
import time
import threading

import numpy as np
from cv2 import accumulateSquare
from sympy.physics.units import action

import utilities
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2, DeviceConfig_pb2, Session_pb2, DeviceManager_pb2, VisionConfig_pb2

from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient

# Position of the protection zone (in meters)
PROTECTION_ZONE_POS =  [0.75, 0.0, 0.4]

# Size of the protection zone (in meters)
PROTECTION_ZONE_DIMENSIONS = [0.05, 0.3, 0.4]

# Theta values of the protection zone movement (in degrees)
PROTECTION_ZONE_MOVEMENT_THETAS = [90.0, 0.0, 90.0]

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

#
# Dictionary of all Sensor strings
#
all_sensor_strings = {
    VisionConfig_pb2.SENSOR_UNSPECIFIED : "Unspecified sensor",
    VisionConfig_pb2.SENSOR_COLOR       : "Color",
    VisionConfig_pb2.SENSOR_DEPTH       : "Depth"
}

#
# Dictionary of all Resolution strings
#
all_resolution_strings = {
    VisionConfig_pb2.RESOLUTION_UNSPECIFIED : "Unspecified resolution",
    VisionConfig_pb2.RESOLUTION_320x240     : "320x240",
    VisionConfig_pb2.RESOLUTION_424x240     : "424x240",
    VisionConfig_pb2.RESOLUTION_480x270     : "480x270",
    VisionConfig_pb2.RESOLUTION_640x480     : "640x480",
    VisionConfig_pb2.RESOLUTION_1280x720    : "1280x720",
    VisionConfig_pb2.RESOLUTION_1920x1080   : "1920x1080"
}

class KinovaGen3():
    def __init__(self, ip):

        username = "admin"
        psw = "admin"

        self.connection = utilities.DeviceConnection.createTcpConnectionExplicit(ip, username, psw)
        self.router = self.connection.connect()

        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)

    def __del__(self):
        """Destructor"""
        self.connection.disconect()

    # Create closure to set an event after an END or an ABORT
    def check_for_end_or_abort(self, e):
        """Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """

        def check(notification, e=e):
            print("EVENT : " + \
                  Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
                    or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()

        return check

    def list_posibles_actions(self):
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        for action in action_list.action_list:
            print(action.name)

    def move_to_selected_position(self, action_name):
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)

        # Move arm to ready position
        print("Moving the arm to a safe position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == action_name:
                action_handle = action.handle

        if action_handle == None:
            print("Can't reach safe position. Exiting")
            sys.exit(1)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)

        e.wait()

        self.base.Unsubscribe(notification_handle)

    def print_protection_zones(self):

        all_protection_zones = self.base.ReadAllProtectionZones()

        print("PROTECTION ZONES")
        for protection_zone in all_protection_zones.protection_zones:
            message = "Protection Zone : " + protection_zone.name + \
                      " Origin : [ " \
                      + str(protection_zone.shape.origin.x) + " " \
                      + str(protection_zone.shape.origin.y) + " " \
                      + str(protection_zone.shape.origin.z) \
                      + " ] Dimensions : [ "
            for dim in protection_zone.shape.dimensions:
                message += str(dim) + " "
            message += "]"
            print(message)

    def create_protection_zone(self):

        zone = Base_pb2.ProtectionZone()

        zone.name = "Example Protection Zone"
        zone.is_enabled = True
        shape = zone.shape
        shape.shape_type = Base_pb2.RECTANGULAR_PRISM

        point = shape.origin
        point.x = PROTECTION_ZONE_POS[0]
        point.y = PROTECTION_ZONE_POS[1]
        point.z = PROTECTION_ZONE_POS[2]
        shape.dimensions.append(PROTECTION_ZONE_DIMENSIONS[0])
        shape.dimensions.append(PROTECTION_ZONE_DIMENSIONS[1])
        shape.dimensions.append(PROTECTION_ZONE_DIMENSIONS[2])

        shape.orientation.row1.column1 = 1.0
        shape.orientation.row2.column2 = 1.0
        shape.orientation.row3.column3 = 1.0

        return self.base.CreateProtectionZone(zone)

    def move_in_front_of_protection_zone(self):

        print("Starting Cartesian action movement ...")
        action = Base_pb2.Action()
        action.name = "Example Cartesian action movement"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = PROTECTION_ZONE_POS[0] - 0.1  # (meters)
        cartesian_pose.y = PROTECTION_ZONE_POS[1]  # (meters)
        cartesian_pose.z = PROTECTION_ZONE_POS[2]  # (meters)
        cartesian_pose.theta_x = PROTECTION_ZONE_MOVEMENT_THETAS[0]  # (degrees)
        cartesian_pose.theta_y = PROTECTION_ZONE_MOVEMENT_THETAS[1]  # (degrees)
        cartesian_pose.theta_z = PROTECTION_ZONE_MOVEMENT_THETAS[2]  # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        e.wait()

        print("Cartesian movement completed")

        self.base.Unsubscribe(notification_handle)

    def example_forward_kinematics(self):
        # Current arm's joint angles (in home position)
        try:
            print("Getting Angles for every joint...")
            input_joint_angles = self.base.GetMeasuredJointAngles()
        except KServerException as ex:
            print("Unable to get joint angles")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            return False

        print("Joint ID : Joint Angle")
        for joint_angle in input_joint_angles.joint_angles:
            print(joint_angle.joint_identifier, " : ", joint_angle.value)
        print()

        # Computing Foward Kinematics (Angle -> cartesian convert) from arm's current joint angles
        try:
            print("Computing Foward Kinematics using joint angles...")
            pose = self.base.ComputeForwardKinematics(input_joint_angles)
        except KServerException as ex:
            print("Unable to compute forward kinematics")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            return False

        print("Pose calculated : ")
        print("Coordinate (x, y, z)  : ({}, {}, {})".format(pose.x, pose.y, pose.z))
        print("Theta (theta_x, theta_y, theta_z)  : ({}, {}, {})".format(pose.theta_x, pose.theta_y, pose.theta_z))
        print()
        return True

    def get_state(self):
        feedback = self.base_cyclic.RefreshFeedback()
        # for j in feedback.actuators:
        #     print (j.command_id, j.position)
        # print("------")
        return feedback.base

    def get_joints(self):
        feedback = self.base_cyclic.RefreshFeedback()
        jointsPosition = [j.position for j in feedback.actuators]
        jointsVelocity = [j.velocity for j in feedback.actuators]
        jointsTorque = [j.torque for j in feedback.actuators]
        joints = {"position": jointsPosition, "velocity": jointsVelocity, "torque": jointsTorque}
        return joints

    def get_gripper_state(self):
        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        return self.base.GetMeasuredGripperMovement(gripper_request).finger[0].value

    def get_pose(self):
        state = self.get_state()

        return [state.tool_pose_x, state.tool_pose_y, state.tool_pose_z,
                state.tool_pose_theta_x, state.tool_pose_theta_y, state.tool_pose_theta_z]

        # return {"x": state.tool_pose_x,
        #         "y": state.tool_pose_y,
        #         "z": state.tool_pose_z,

        #         "theta_x": state.tool_pose_theta_x,
        #         "theta_y": state.tool_pose_theta_y,
        #         "theta_z": state.tool_pose_theta_z
        #         }

    def stop_movement(self):
        self.cartesian_move_relative(0, 0, 0, 0, 0, 0)


    def gripper_move_to(self, target_position):
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        # print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.00
        finger.finger_identifier = 1
        finger.value = target_position
        self.base.SendGripperCommand(gripper_command)
        # while position < 1.0:
        #     finger.value = position
        #     print("Going to position {:0.2f}...".format(finger.value))
        #     self.base.SendGripperCommand(gripper_command)
        #     position += 0.1
        #     time.sleep(1)
        return True

    def gripper_move_speed(self, speed):
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with speed increments
        # print("Performing gripper test in speed...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.finger_identifier = 1
        finger.value = speed

        ## TODO: Check to stop when the sensor detect an object



        self.base.SendGripperCommand(gripper_command)
        return True

    def gripper_force_test(self):
        base_command = BaseCyclic_pb2.Command()
        base_command.frame_id = 0
        base_command.interconnect.command_id.identifier = 0
        base_command.interconnect.gripper_command.command_id.identifier = 0

        motorcmd = base_command.interconnect.gripper_command.motor_cmd.add()

        base_feedback = self.base_cyclic.RefreshFeedback()
        motorcmd.position = base_feedback.interconnect.gripper_feedback.motor[0].position
        motorcmd.velocity = 0
        motorcmd.force = 0

        for actuator in base_feedback.actuators:
            actuator_command = base_command.actuators.add()
            actuator_command.position = actuator.position
            actuator_command.velocity = 0.0
            actuator_command.torque_joint = 0.0
            actuator_command.command_id = 0
            print("Position = ", actuator.position)

        previous_servoing_mode = self.base.GetServoingMode()

        servoing_mode_info = Base_pb2.ServoingModeInformation()
        servoing_mode_info.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
        self.base.SetServoingMode(servoing_mode_info)

        target_position = 100.0

        while True:
            try:
                base_feedback = self.base_cyclic.Refresh(base_command)
                # Calculate speed according to position error (target position VS current position)
                position_error = target_position - base_feedback.interconnect.gripper_feedback.motor[0].position
                print("position error: ", base_feedback.interconnect.gripper_feedback)

                # If positional error is small, stop gripper
                if abs(position_error) < 1.5:
                    position_error = 0
                    motorcmd.velocity = 0
                    self.base_cyclic.Refresh(base_command)
                    return True
                else:
                    motorcmd.velocity = 2.0 * abs(position_error)
                    if motorcmd.velocity > 10.0:
                        motorcmd.velocity = 10.0
                    motorcmd.position = target_position

            except Exception as e:
                print("Error in refresh: " + str(e))
                return False
            time.sleep(0.001)

        ### Final
        self.base.SetServoingMode(previous_servoing_mode)

        return True

    def cartesian_move_to(self, x, y, z, theta_x, theta_y, theta_z):

        print("Starting Cartesian Especific Movement ...")
        action = Base_pb2.Action()
        action.name = "Cartesian Especific movement"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = x  # (meters)
        cartesian_pose.y = y  # (meters)
        cartesian_pose.z = z  # (meters)
        cartesian_pose.theta_x = theta_x  # (degrees)
        cartesian_pose.theta_y = theta_y  # (degrees)
        cartesian_pose.theta_z = theta_z  # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Cartesian movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    def move_joints_with_speeds(self, speeds):
        # SPEED = 20.0
        # speeds = [SPEED, 0, -SPEED, 0, SPEED, 0, -SPEED]

        joint_speeds = Base_pb2.JointSpeeds()

        i = 0
        for speed in speeds:
            joint_speed = joint_speeds.joint_speeds.add()
            joint_speed.joint_identifier = i
            joint_speed.value = speed
            joint_speed.duration = 0
            i = i + 1

        self.base.SendJointSpeedsCommand(joint_speeds)
        # time.sleep(10)
        # self.base.Stop()

    def move_joints_to(self, joints):
        action = Base_pb2.Action()
        action.name = "Example angular action movement"
        action.application_data = ""

        actuator_count = self.base.GetActuatorCount()

        # angles = np.rad2deg([1.13, 4.71, np.pi / 2, 3.83, 0, 5.41, np.pi / 2])
        # angles = [65, 270, 90, 220, 0, 310, 90]

        # Place arm straight up
        for joint_id in range(actuator_count.count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = joints[joint_id]

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    #
    # Prints the extrinsic parameters on stdout
    #
    def print_extrinsic_parameters(self, extrinsics):
        print("Rotation matrix:")
        print("[{0: .6f} {1: .6f} {2: .6f}".format( \
            extrinsics.rotation.row1.column1, extrinsics.rotation.row1.column2, extrinsics.rotation.row1.column3))
        print(" {0: .6f} {1: .6f} {2: .6f}".format( \
            extrinsics.rotation.row2.column1, extrinsics.rotation.row2.column2, extrinsics.rotation.row2.column3))
        print(" {0: .6f} {1: .6f} {2: .6f}]".format( \
            extrinsics.rotation.row3.column1, extrinsics.rotation.row3.column2, extrinsics.rotation.row3.column3))
        print("Translation vector: [{0:.6f} {1:.6f} {2:.6f}]".format( \
            extrinsics.translation.t_x, extrinsics.translation.t_y, extrinsics.translation.t_z))

    #
    # Returns the device identifier of the Vision module, 0 if not found
    #
    def example_vision_get_device_id(self, device_manager):
        vision_device_id = 0

        # getting all device routing information (from DeviceManagerClient service)
        all_devices_info = device_manager.ReadAllDevices()

        vision_handles = [hd for hd in all_devices_info.device_handle if hd.device_type == DeviceConfig_pb2.VISION]
        if len(vision_handles) == 0:
            print("Error: there is no vision device registered in the devices info")
        elif len(vision_handles) > 1:
            print("Error: there are more than one vision device registered in the devices info")
        else:
            handle = vision_handles[0]
            vision_device_id = handle.device_identifier
            print("Vision module found, device Id: {0}".format(vision_device_id))

        return vision_device_id

    #
    # Example showing how to retrieve the extrinsic parameters
    #
    def example_routed_vision_get_extrinsics(self, vision_config, vision_device_id):
        print("\n\n** Example showing how to retrieve the extrinsic parameters **")

        print("\n-- Using Vision Config Service to get extrinsic parameters --")
        extrinsics = vision_config.GetExtrinsicParameters(vision_device_id)
        self.print_extrinsic_parameters(extrinsics)

    #
    # Returns a string matching the requested sensor
    #
    def sensor_to_string(self, sensor):
        return all_sensor_strings.get(sensor, "Unknown sensor")

    #
    # Returns a string matching the requested resolution
    #
    def resolution_to_string(self, resolution):
        return all_resolution_strings.get(resolution, "Unknown resolution")

    #
    # Prints the intrinsic parameters on stdout
    #
    def print_intrinsic_parameters(self, intrinsics):
        print("Sensor: {0} ({1})".format(intrinsics.sensor, self.sensor_to_string(intrinsics.sensor)))
        print("Resolution: {0} ({1})".format(intrinsics.resolution, self.resolution_to_string(intrinsics.resolution)))
        print("Principal point x: {0:.6f}".format(intrinsics.principal_point_x))
        print("Principal point y: {0:.6f}".format(intrinsics.principal_point_y))
        print("Focal length x: {0:.6f}".format(intrinsics.focal_length_x))
        print("Focal length y: {0:.6f}".format(intrinsics.focal_length_y))
        print("Distortion coefficients: [{0:.6f} {1:.6f} {2:.6f} {3:.6f} {4:.6f}]".format( \
            intrinsics.distortion_coeffs.k1, \
            intrinsics.distortion_coeffs.k2, \
            intrinsics.distortion_coeffs.p1, \
            intrinsics.distortion_coeffs.p2, \
            intrinsics.distortion_coeffs.k3))

    #
    # Example showing how to retrieve the intrinsic parameters of the Color and Depth sensors
    #
    def example_routed_vision_get_intrinsics(self, vision_config, vision_device_id):
        sensor_id = VisionConfig_pb2.SensorIdentifier()
        profile_id = VisionConfig_pb2.IntrinsicProfileIdentifier()

        print("\n\n** Example showing how to retrieve the intrinsic parameters of the Color and Depth sensors **")

        print("\n-- Using Vision Config Service to get intrinsic parameters of active color resolution --")
        sensor_id.sensor = VisionConfig_pb2.SENSOR_COLOR
        intrinsics = vision_config.GetIntrinsicParameters(sensor_id, vision_device_id)
        self.print_intrinsic_parameters(intrinsics)

        print("\n-- Using Vision Config Service to get intrinsic parameters of active depth resolution --")
        sensor_id.sensor = VisionConfig_pb2.SENSOR_DEPTH
        intrinsics = vision_config.GetIntrinsicParameters(sensor_id, vision_device_id)
        self.print_intrinsic_parameters(intrinsics)

        print("\n-- Using Vision Config Service to get intrinsic parameters for color resolution 1920x1080 --")
        profile_id.sensor = VisionConfig_pb2.SENSOR_COLOR
        profile_id.resolution = VisionConfig_pb2.RESOLUTION_1920x1080
        intrinsics = vision_config.GetIntrinsicParametersProfile(profile_id, vision_device_id)
        self.print_intrinsic_parameters(intrinsics)

        print("\n-- Using Vision Config Service to get intrinsic parameters for depth resolution 424x240 --")
        profile_id.sensor = VisionConfig_pb2.SENSOR_DEPTH
        profile_id.resolution = VisionConfig_pb2.RESOLUTION_424x240
        intrinsics = vision_config.GetIntrinsicParametersProfile(profile_id, vision_device_id)
        self.print_intrinsic_parameters(intrinsics)

    def get_camera_info(self):
        device_manager = DeviceManagerClient(self.router)
        vision_config = VisionConfigClient(self.router)

        vision_device_id = self.example_vision_get_device_id(device_manager)

        if vision_device_id != 0:
            self.example_routed_vision_get_extrinsics(vision_config, vision_device_id)
            self.example_routed_vision_get_intrinsics(vision_config, vision_device_id)

    def cartesian_move_relative(self, x, y, z, theta_x, theta_y, theta_z):

        print("Starting Cartesian action movement ...")
        action = Base_pb2.Action()
        action.name = "Example Cartesian action movement"
        action.application_data = ""

        feedback = self.base_cyclic.RefreshFeedback()

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = feedback.base.tool_pose_x + x  # (meters)
        cartesian_pose.y = feedback.base.tool_pose_y + y  # (meters)
        cartesian_pose.z = feedback.base.tool_pose_z + z  # (meters)
        cartesian_pose.theta_x = feedback.base.tool_pose_theta_x + theta_x  # (degrees)
        cartesian_pose.theta_y = feedback.base.tool_pose_theta_y + theta_y  # (degrees)
        cartesian_pose.theta_z = feedback.base.tool_pose_theta_z + theta_z  # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Cartesian movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    def move_gripper_speed_dest(self, dest_pos):

        gr_pos = self.get_gripper_state()

        direction = 1 if (gr_pos - dest_pos) > 0 else -1

        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        # Set speed to close gripper
        print("Closing gripper using speed command to dest pos")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 0.1 * direction
        self.base.SendGripperCommand(gripper_command)

        vel_gripper_request = Base_pb2.GripperRequest()
        # Wait for reported speed to be 0
        vel_gripper_request.mode = Base_pb2.GRIPPER_SPEED

        pos_gripper_request = Base_pb2.GripperRequest()
        # Wait for reported pos to be dest_pos
        pos_gripper_request.mode = Base_pb2.GRIPPER_POSITION

        # Speed is initially 0, to avoid premature stopping:
        time.sleep(.1)

        vel_gripper_measure = self.base.GetMeasuredGripperMovement(vel_gripper_request)
        pos_gripper_measure = self.base.GetMeasuredGripperMovement(pos_gripper_request)
        while abs(dest_pos - pos_gripper_measure.finger[0].value) > 0.01 and \
                vel_gripper_measure.finger[0].value != 0.00:
            vel_gripper_measure = self.base.GetMeasuredGripperMovement(vel_gripper_request)
            pos_gripper_measure = self.base.GetMeasuredGripperMovement(pos_gripper_request)

        finger.value = 0.0
        self.base.SendGripperCommand(gripper_command)

    def close_gripper_speed(self):

        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        # Set speed to close gripper
        print("Closing gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = -0.1
        self.base.SendGripperCommand(gripper_command)

        gripper_request = Base_pb2.GripperRequest()
        # Wait for reported speed to be 0
        gripper_request.mode = Base_pb2.GRIPPER_SPEED

        # Speed is initially 0, to avoid premature stopping:
        time.sleep(.1)
        while True:
            gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
            if len(gripper_measure.finger):
                print("Current speed is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value == 0.0:
                    break
            else:  # Else, no finger present in answer, end loop
                break

    def open_gripper_speed(self):

        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Set speed to open gripper
        print("Opening gripper using speed command...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.value = 0.1
        self.base.SendGripperCommand(gripper_command)
        gripper_request = Base_pb2.GripperRequest()

        # Wait for reported position to be opened
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len(gripper_measure.finger):
                print("Current position is : {0}".format(gripper_measure.finger[0].value))
                if gripper_measure.finger[0].value < 0.01:
                    break
            else:  # Else, no finger present in answer, end loop
                break