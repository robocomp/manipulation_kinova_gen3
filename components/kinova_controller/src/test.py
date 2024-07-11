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
    Provides methods for getting camera information, intrinsic and extrinsic
    parameters, and movement actions using the Kinova Gen III SDK. It also includes
    functions for closing the gripper with a speed command and opening it with a
    speed command.

    Attributes:
        connection (object): Used to establish a connection with the Kinova Gen 3
            robot controller. It represents the result of calling the `connect()`
            method, which returns a connection object that can be used to send
            commands to the robot and receive data from it.
        router (RoombaRouter): Used to interact with the Robot's ROS2 node, sending
            and receiving messages.
        base (instance): A reference to an object of the `KinovaBase` class, which
            provides access to the robot's base functionality such as movement,
            gripper control, and sensor readings.
        base_cyclic (Base_pb2Base): Used to interact with the Cyclic robotic arm.
            It provides methods for moving the arm, closing the gripper, and opening
            the gripper.

    """
    def __init__(self):

        """
        Establishes connections to an IP address, username, and password using the
        `utilities.DeviceConnection` class, then creates instances of `BaseClient`
        and `BaseCyclicClient` using the connected router.

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
            Takes a `notification` object and an optional `e` parameter, and prints
            the action event name of the notification. If the action event is
            either `ACTION_END` or `ACTION_ABORT`, the `e` variable is set to a
            default value.

            Args:
                notification (Base_pb2Notification): Passed an event object
                    containing information about the event that triggered the
                    function, such as the action event name.
                e (Base_pb2Event): Set to an instance of that class by the line `e.set()`.

            """
            print("EVENT : " + \
                  Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
                    or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()

        return check

    def get_state(self):
        """
        Within the KinovaGen3 class retrieves refresh feedback from the base cyclic
        object and returns it.

        Returns:
            RefreshFeedback: An instance of a class that contains information about
            the state of the system.

        """
        feedback = self.base_cyclic.RefreshFeedback()
        # for j in feedback.actuators:
        #     print (j.command_id, j.position)
        # print("------")
        return feedback.base

    def get_joints(self):
        """
        Returns a dictionary containing the position, velocity, and torque of the
        joints of an object controlled by a Kinova Gen 3 robot.

        Returns:
            dict: A dictionary containing the positions, velocities, and torques
            of the joints.

        """
        feedback = self.base_cyclic.RefreshFeedback()
        jointsPosition = [j.position for j in feedback.actuators]
        jointsVelocity = [j.velocity for j in feedback.actuators]
        jointsTorque = [j.torque for j in feedback.actuators]
        joints = {"position": jointsPosition, "velocity": jointsVelocity, "torque": jointsTorque}
        return joints

    def get_gripper_state(self):
        """
        Retrieves the current state of a gripper, specifically the position of the
        gripper's finger, as measured by the `Base` class's `GetMeasuredGripperMovement`
        method.

        Returns:
            float: The measured movement of the gripper in the `finger[0]` position.

        """
        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        return self.base.GetMeasuredGripperMovement(gripper_request).finger[0].value

    def get_pose(self):
        """
        Retrieves the current position and orientation of a tool in a robotic
        system, returning a list of 6 values representing the tool's x, y, z
        coordinates and theta angles in each dimension.

        Returns:
            5element: A list of five floating-point numbers that represent the
            tool's pose (position and orientation) in the global coordinate system.

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
        Controls the movement of a gripper based on a target position, sending a
        command to the base to move the gripper to that position.

        Args:
            target_position (float): Representing the desired position of the
                gripper to move to.

        Returns:
            Boolean: True if the gripper moves to the target position successfully,
            otherwise False.

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


    def cartesian_move_to(self, x, y, z, theta_x, theta_y, theta_z):

        """
        Performs Cartesian specific movement of a robot, which involves moving the
        robot's end effector to a specified position and orientation using an
        action message.

        Args:
            x (float64): Used to set the x-coordinate of the target position for
                the Cartesian movement.
            y (int): Used to specify the y coordinate of the target position for
                the cartesian movement.
            z (float): Used to set the z-coordinate of the target pose in the
                Cartesian coordinate system.
            theta_x (float): Representing the x-angle of the robot's end effector
                at the specified position.
            theta_y (float): Representing the yaw angle of the robot, which
                determines the orientation of the robot's yaw axis relative to its
                base.
            theta_z (float): Used to specify the z-rotation angle of the robot's
                end effector during movement.

        Returns:
            bool: 1 if the movement was successful and 0 if it timed out.

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
        Within the KinovaGen3 class takes a list of joint speeds as input and
        creates a `JointSpeeds` message to send to the robot's base module. It
        then iterates through the list of speeds, adding each one to the message
        with the appropriate joint identifier and duration. Finally, it sends the
        completed message to the base module using the `SendJointSpeedsCommand` method.

        Args:
            speeds (Base_pb2JointSpeeds): An iterable containing joint speed values
                for each joint in a robot.

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
        Controls movement of joints in an angular action by creating an action
        object, adding joint angles to it, and executing the action using the
        `ExecuteAction` method. It also waits for the movement to finish and
        unsubscribes from the notification handle.

        Args:
            joints (list): Used to specify the joint angles for movement.

        Returns:
            bool: 1 if the angular movement completed within the specified timeout
            duration, or 0 if the movement did not complete before the timeout.

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
        Prints the extrinsic parameters (rotation and translation) of an object
        represented by the `KinovaGen3` class.

        Args:
            extrinsics (3x3): Used to represent a rotation matrix followed by a
                translation vector.

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
        Retrieves the device ID of the vision module in a Kinova Gen3 system. It
        first reads all devices information, then checks if there is only one
        vision device, and finally returns its device ID.

        Args:
            device_manager (DeviceManager): Used to retrieve information about all
                devices connected to the system.

        Returns:
            int: The device identifier of the first vision device found in the
            devices list or an error message if there are no vision devices
            registered or more than one vision device is registered.

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
        Retrieves extrinsic parameters for a given vision device ID using the
        `VisionConfigService`. It then prints the retrieved extrinsics to the console.

        Args:
            vision_config (VisionConfig): Used to retrieve extrinsic parameters
                for a specific vision device ID.
            vision_device_id (int): Used to identify a specific vision device for
                which extrinsic parameters are being retrieved.

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
        Within the KinovaGen3 class prints out various parameters associated with
        intrinsics.

        Args:
            intrinsics (IntrinsicParameters): Represented as a object that contains
                the intrinsic parameters of the camera sensor, including the
                principal point, resolution, focal length, and distortion coefficients.

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
        Config Service, and also retrieves intrinsic parameters for specific
        resolutions of Color and Depth sensors.

        Args:
            vision_config (VisionConfig_pb2VisionConfig): Used to retrieve intrinsic
                parameters from the vision config service.
            vision_device_id (int): Used to specify the device ID for which intrinsic
                parameters are being retrieved.

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
        Retrieves device information and extrinsic and intrinsic parameters for a
        vision device in Kinova Gen3.

        """
        device_manager = DeviceManagerClient(self.router)
        vision_config = VisionConfigClient(self.router)

        vision_device_id = self.example_vision_get_device_id(device_manager)

        if vision_device_id != 0:
            self.example_routed_vision_get_extrinsics(vision_config, vision_device_id)
            self.example_routed_vision_get_intrinsics(vision_config, vision_device_id)

    def cartesian_move_relative(self, x, y, z, theta_x, theta_y, theta_z):

        """
        Performs Cartesian movement based on user-inputted coordinates and angles,
        using the Kinova Gen3 robot's base API.

        Args:
            x (int): The relative movement of the tool along the x-axis.
            y (int): Represented as feedback.base.tool_pose_y + y, which indicates
                an additional movement along the Y axis of the tool's pose.
            z (32bit): Representing the relative movement of the tool's Z axis.
            theta_x (float): Part of the target pose's orientation, representing
                the yaw angle of the tool in the x-y plane.
            theta_y (float): Represented as the yaw angle of the tool relative to
                its parent link, indicating the direction of movement along the y-axis.
            theta_z (float): Representing the z-axis angle of the tool's orientation
                relative to its starting position, which is used in calculating
                the target pose of the tool for the Cartesian action movement.

        Returns:
            bool: 1 if the cartesian movement finishes within the given timeout
            duration, and 0 otherwise.

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
        Controls the movement of a gripper based on its destination position,
        moving it at a speed determined by the difference between its current
        position and the destined position.

        Args:
            dest_pos (float): Representing the desired position for the gripper
                to move towards.

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
        Controls the speed of a gripper using a speed command. It sets the value
        of the gripper to -0.1 and sends the command to the base using the
        `SendGripperCommand` method. The function then sleeps for 0.1 seconds
        before continuing to monitor the gripper's movement using the
        `GetMeasuredGripperMovement` method.

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
        Controls the speed and position of a gripper using a Basebot2 interface.
        It creates a gripper command message, sends it to the Basebot2, and retrieves
        the measured movement response from the Basebot2.

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