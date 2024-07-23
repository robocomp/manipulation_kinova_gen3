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

from cv2 import accumulateSquare

import utilities
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2, DeviceConfig_pb2, Session_pb2, DeviceManager_pb2, VisionConfig_pb2

from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient


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
    """
    Manages the opening and closing of a gripper using speed commands, as well as
    measuring the movement of the gripper during these operations.

    Attributes:
        connection (Base_pb2GripperCommand|Base_pb2GripperRequest): Used to send
            gripper commands to the Kinova Gen III robot. It allows you to specify
            the command mode, finger position, and other parameters for the gripper
            movement.
        router (Base_pb2GripperCommand): Used to send a gripper command to the
            Kinova Gen III robotic arm. It allows for sending different types of
            commands, such as speed or position, to control the movement of the gripper.
        base (Base_pb2Base): A reference to the underlying robot's base module,
            which provides methods for sending commands to the robot's end effectors.
        base_cyclic (Base|GripperRequest): Used to specify the cyclic movement
            pattern for the gripper, allowing for continuous opening or closing
            of the gripper.

    """
    def __init__(self):

        """
        Establishes connections to a device using TCP and creates two clients:
        `BaseClient` for direct interaction with the device, and `BaseCyclicClient`
        for cyclic communication.

        """
        ip = "192.168.1.10"
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
            """
            Prints the event name associated with a `notification` object and sets
            an instance variable `e` to a default value if the event is either
            `ACTION_END` or `ACTION_ABORT`.

            Args:
                notification (Base_pb2.Notification): Passed as an argument to the
                    function, providing information about an event that triggered
                    the function call.
                e (Base_pb2.Event): Set to an instance of Event upon calling the
                    function.

            """
            print("EVENT : " + \
                  Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
                    or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()

        return check

    def get_state(self):
        """
        Retrieves refresh feedback from the base cyclic component and returns it.

        Returns:
            Feedbackbase: A cyclic refresh feedback object created by calling the
            `RefreshFeedback` method.

        """
        feedback = self.base_cyclic.RefreshFeedback()
        # for j in feedback.actuators:
        #     print (j.command_id, j.position)
        # print("------")
        return feedback.base

    def get_joints(self):
        """
        Computes and returns the positions, velocities, and torques of the joints
        of a robotic arm.

        Returns:
            Dict[str,float]: A dictionary containing the positions, velocities,
            and torques of the joints in the system.

        """
        feedback = self.base_cyclic.RefreshFeedback()
        jointsPosition = [j.position for j in feedback.actuators]
        jointsVelocity = [j.velocity for j in feedback.actuators]
        jointsTorque = [j.torque for j in feedback.actuators]
        joints = {"position": jointsPosition, "velocity": jointsVelocity, "torque": jointsTorque}
        return joints

    def get_gripper_state(self):
        """
        Retrieves the current state of the gripper by sending a `GripperRequest`
        message to the base module and returning the measured movement of the
        gripper in the form of a `finger` object.

        Returns:
            float: The finger position of the gripper in meters.

        """
        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        return self.base.GetMeasuredGripperMovement(gripper_request).finger[0].value

    def get_pose(self):
        """
        Computes and returns the tool's pose (position and orientation) in a list
        of six elements.

        Returns:
            Tuple[float,float,float,float,float,float]: 6 floats representing the
            tool's pose in 3D space and orientation.

        """
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
        """
        Performs a gripper movement command to reach a specified position.

        Args:
            target_position (float): Representing the desired position of the
                gripper to move to.

        Returns:
            bool: True when the gripper has moved to the target position successfully,
            and False otherwise.

        """
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
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
        """
        Sets the gripper speed by creating a `GripperCommand` message, setting the
        gripper mode and finger value to the input speed, and sending the command
        to the base using `SendGripperCommand`.

        Args:
            speed (float): Represents the desired speed at which the gripper should
                move in the specified finger.

        Returns:
            bool: `True` if the gripper command was sent successfully, and `False`
            otherwise.

        """
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with speed increments
        print("Performing gripper test in speed...")
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.finger_identifier = 1
        finger.value = speed

        ## TODO: Check to stop when the sensor detect an object



        self.base.SendGripperCommand(gripper_command)
        return True

    def cartesian_move_to(self, x, y, z, theta_x, theta_y, theta_z):

        """
        Moves a robotic arm to a specified position and orientation using a Cartesian
        movement approach, waiting for the movement to finish before returning.

        Args:
            x (float): Used to set the x-coordinate of the target pose in the
                Cartesian space.
            y (float): Used to specify the target y-coordinate of the robot's end
                position after moving along the x-axis to the specified `x` coordinate.
            z (float): Representing the z-coordinate of the target position for
                the robot to move to.
            theta_x (float): Used to represent the rotation angle of the robot's
                end effector around its x-axis during movement.
            theta_y (float): Used to specify the yaw angle of the robot's end
                effector during the movement.
            theta_z (float): Representing the z-rotation angle of the robot end
                effector relative to its initial position, along the robot's Z axis.

        Returns:
            bool: 1 if the movement was completed successfully within the timeout
            duration, and 0 otherwise due to a timeout.

        """
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

        """
        Calculates and sends joint speeds to the robot.

        Args:
            speeds (List[float]): A list of joint speed values to be applied to
                the robot's joints.

        """
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
        """
        Moves joints to specified angles using an action object and waits for the
        movement to finish before returning.

        Args:
            joints (List[float]): Used to specify the angles of the joints to move
                to.

        Returns:
            bool: 1 if the angular movement was completed successfully within the
            specified timeout, and 0 otherwise due to a timeout.

        """
        action = Base_pb2.Action()
        action.name = "Example angular action movement"
        action.application_data = ""

        actuator_count = self.base.GetActuatorCount()

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
        """
        Prints the rotation and translation parameters of an extrinsic parameter
        set.

        Args:
            extrinsics (Extrinsic | RotationMatrix | TranslationVector): 3x3
                rotation matrix or 3x1 translation vector.

        """
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
        """
        Retrieves the device ID of the vision module from the devices info based
        on the device handle.

        Args:
            device_manager (DeviceManager | None): Used to represent the device
                manager object that provides information about the devices registered
                in the system.

        Returns:
            int: The device ID of the first Vision module detected in the system.

        """
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
        """
        Retrieves extrinsic parameters for a specific vision device using the
        Vision Config Service.

        Args:
            vision_config (VisionConfig): Used to retrieve extrinsic parameters
                for a specific vision device ID.
            vision_device_id (str): Used to identify the specific vision device
                for which the extrinsic parameters are being retrieved.

        """
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
        """
        Prints out various parameters of an intrinsic camera model, including
        sensor ID, resolution, principal point coordinates, focal length coordinates,
        and distortion coefficients.

        Args:
            intrinsics (IntrinsicParams): Used to access the intrinsic parameters
                of a camera sensor, including the principal point, resolution,
                focal length, and distortion coefficients.

        """
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
        """
        Retrieves intrinsic parameters of Color and Depth sensors using the Vision
        Config Service, and prints the retrieved values.

        Args:
            vision_config (VisionConfig_pb2.VisionConfig): Used to retrieve intrinsic
                parameters for different sensors and resolutions.
            vision_device_id (int): Used to identify the specific device to retrieve
                intrinsic parameters for.

        """
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
        """
        Retrieves device and intrinsic information for a vision camera connected
        to the router.

        """
        device_manager = DeviceManagerClient(self.router)
        vision_config = VisionConfigClient(self.router)

        vision_device_id = self.example_vision_get_device_id(device_manager)

        if vision_device_id != 0:
            self.example_routed_vision_get_extrinsics(vision_config, vision_device_id)
            self.example_routed_vision_get_intrinsics(vision_config, vision_device_id)

    def cartesian_move_relative(self, x, y, z, theta_x, theta_y, theta_z):

        """
        Performs a Cartesian movement action relative to a base tool, taking into
        account the specified x, y, z coordinates and theta angles. It executes
        the action, waits for it to finish, and returns whether the movement
        completed successfully or timed out.

        Args:
            x (float): Used to set the x-position of the tool relative to its
                initial position.
            y (int): Representing the relative movement along the y-axis of the
                robot's end effector.
            z (float): Representing the relative movement of the tool along the z-axis.
            theta_x (float): Representing the angular movement of the tool around
                the x-axis, measured in radians.
            theta_y (float): Representing the angle of rotation around the Y-axis
                of the tool coordinate system, which determines the orientation
                of the tool end effector during movement.
            theta_z (float): Representing the z-axis rotation angle of the tool
                relative to its current position, which is used by the action
                movement to achieve the desired orientation in the z-axis direction.

        Returns:
            bool: 1 if the cartesian movement was successful and 0 if it timed out.

        """
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

        """
        Calculates the gripper movement speed to reach a destination position,
        sends a speed command to the gripper, and measures the gripper's movement
        to ensure it reaches the desired position.

        Args:
            dest_pos (float): Representing the desired position of the gripper to
                move it to with the speed command.

        """
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

        """
        Controls the speed of the gripper using a speed command.

        """
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

        """
        Controls the gripper's speed using a command, and then measures its position
        to determine when it has reached the desired open state.

        """
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