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
    def __init__(self):

        """
        Establishes a connection to a network device using the `utilities.DeviceConnection`
        class and creates an instance of `BaseClient` or `BaseCyclicClient`.

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
            Is called when an event occurs in the `on_completion` observer of a
            notification. It sets the state of the `e` variable based on the action
            event received in the notification.

            Args:
                notification (`Base_pb2.Notification` object.): notification payload
                    sent from the Firestore or Google Cloud Messaging services,
                    which contains information about the action event that triggered
                    the function.
                    
                    	* `action_event`: A member of the `Base_pb2.ActionEvent` enum,
                    which represents the event triggered by the notification. Its
                    value is determined by the field `notification.action_event`.
                    	* `set`: A method that sets the state of an object based on
                    the input provided. In this context, it sets the state of the
                    `e` object.
                e (`Event` object.): event, which is set to indicate that an action
                    has ended or been aborted.
                    
                    	* `e` is a `google.protobuf.Entity` object, indicating that
                    it is a protocol buffer message.
                    	* The type of `e` is specified by the value of its `message_type`
                    attribute, which is set to `"Event"`.
                    	* The fields of `e` are described by their names and attributes,
                    such as `name`, `action_event`, and others.
                    
                    	Overall, the function takes a serialized protocol buffer
                    message as input, parses it, and sets the resulting message
                    object in the `e` variable for further processing.

            """
            print("EVENT : " + \
                  Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
                    or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()

        return check

    def get_state(self):
        """
        Returns an instance of the `Feedback` class from a provided base object.

        Returns:
            undefined: a dictionary containing the current state of the actuators
            and their positions.
            
            	* `feedback`: The instance of the `RefreshFeedback` class.
            	* `actuators`: A list of actuator objects that contain information
            about the position of each actuator in the system. Each actuator object
            has a `command_id` attribute and a `position` attribute.

        """
        feedback = self.base_cyclic.RefreshFeedback()
        # for j in feedback.actuators:
        #     print (j.command_id, j.position)
        # print("------")
        return feedback.base

    def get_joints(self):
        """
        Generates high-quality documentation for code given to it by returning a
        dictionary containing the positions, velocities, and torques of the
        actuators' joints.

        Returns:
            undefined: a dictionary containing the current position, velocity, and
            torque of the actuators.

        """
        feedback = self.base_cyclic.RefreshFeedback()
        jointsPosition = [j.position for j in feedback.actuators]
        jointsVelocity = [j.velocity for j in feedback.actuators]
        jointsTorque = [j.torque for j in feedback.actuators]
        joints = {"position": jointsPosition, "velocity": jointsVelocity, "torque": jointsTorque}
        return joints

    def get_gripper_state(self):
        """
        Retrieves the current position of a gripper mechanism based on the specified
        mode and returns the value of the gripper's movement in a finger object.

        Returns:
            undefined: a measured value for the movement of the gripper in a
            specific mode.

        """
        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        return self.base.GetMeasuredGripperMovement(gripper_request).finger[0].value

    def get_pose(self):
        """
        Generates high-quality documentation for code by returning an array of
        five values containing the tool's pose (x, y, z, theta x, y, and z) based
        on the given state object.

        Returns:
            undefined: an array of 6 values representing the tool's position and
            orientation in 3D space.

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

    def cartesian_move_to(self, x, y, z, theta_x, theta_y, theta_z):

        """
        Moves a robot's end effector to a specified location using Cartesian
        coordinates, waiting for the movement to finish before returning.

        Args:
            x (float): 3D position of the robot's end effector in meters.
            y (meter(s) quantity.): 2D position of the robot's end effector along
                the y-axis, which is used to calculate the desired pose of the
                robot for the Cartesian movement.
                
                	* `x`: The position along the x-axis of the move (meters)
                	* `y`: The position along the y-axis of the move (meters)
                	* `z`: The position along the z-axis of the move (meters)
                	* `theta_x`, `theta_y`, and `theta_z`: The orientation angles
                about each axis (degrees)
            z (float): 3rd dimension of the Cartesian pose, specifying the position
                along the z-axis (in meters) for the movement to occur.
            theta_x (float): 3D rotational angle of the robot's end effector about
                its x-axis, measured in degrees.
            theta_y (degree(s).): 3D rotation angle around the y-axis of the end
                effector, which determines the orientation of the end effector
                relative to the base robot frame during the Cartesian movement.
                
                	* `theta_y`: This is a `float` representing the y-angle of the
                cartesian movement in degrees.
            theta_z (float): 3D rotation around the z-axis of the end effector,
                which is a part of the Cartesian movement.

        Returns:
            undefined: a boolean indicating whether the cartesian movement was
            completed within the given time frame.

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
        Generates high-quality documentation for given code by adding joint speeds
        to a base object and sending them as a command.

        Args:
            speeds (list): 3D joint angles to be applied to the robot, which are
                added to the `JointSpeeds` message sent by the function.

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
        Executes an angular action on a robot by setting joint angles and subscribing
        to notifications for the action's completion. It waits for the specified
        time duration for movement to finish before unsubscribing from the
        notification handle.

        Args:
            joints (int): 3D joint angles of a robot arm, which are used to configure
                the movement of the arm in the `reach_joint_angles` property of
                the `action` object.

        Returns:
            undefined: a boolean value indicating whether the angular movement
            completed within the specified time duration or timed out.

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
        Prints rotation and translation parameters in a matrix format.

        Args:
            extrinsics (float): 3D rotation and translation of a camera relative
                to a given reference frame, which is used to compute the view
                matrix and other properties of the camera's pose.

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
        Reads the device information from the DeviceManagerClient service and
        identifies the vision devices based on their device type. It returns the
        device identifier of the first vision device found or an error message if
        no vision devices are registered or more than one device is found.

        Args:
            device_manager (`DeviceManagerClient` object.): DeviceManagerClient
                service that provides information on all devices connected to the
                system, including their device handles and types.
                
                	* `ReadAllDevices()`: This is an instance method that retrieves
                all device information from the DeviceManagerClient service. The
                method returns a list of `DeviceHandle` objects, where each object
                contains attributes such as `device_type`, `device_identifier`,
                and `device_name`.
                
                	The `vision_handles` list is created by iterating over the
                `all_devices_info` list and filtering the items based on their
                `device_type` attribute. In this example, only devices with
                `device_type` equal to `DeviceConfig_pb2.VISION` are included in
                the list.
                
                	If the length of the `vision_handles` list is greater than or
                equal to 1, an error message is printed. Otherwise, the first
                element of the list (i.e., `vision_handles[0]`) is selected as the
                `handle` and its `device_identifier` attribute is assigned to the
                `vision_device_id`. Finally, a print statement outputs the
                `vision_device_id` value.

        Returns:
            undefined: a single device identifier for a vision module.

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
        Retrieves extrinsic parameters for a vision device using the Vision Config
        Service.

        Args:
            vision_config (str): 3D vision configuration that contains the parameters
                required to retrieve extrinsic parameters from the Vision Device
                ID.
            vision_device_id (str): unique identifier of the vision device for
                which the extrinsic parameters are being retrieved.

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
        Takes an intrinsics object as input and prints essential information about
        its properties such as the sensor name, resolution, principal point, focal
        length, and distortion coefficients.

        Args:
            intrinsics (int): 4x4 homography matrix and its corresponding distortion
                coefficients for the given camera sensor.

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
        Retrieves intrinsic parameters of active color and depth sensors, as well
        as for specific resolutions using the `Vision Config Service`.

        Args:
            vision_config (int): Vision Config service, which is used to retrieve
                intrinsic parameters for different sensor resolutions and device
                IDs.
            vision_device_id (int): device ID of the Vision device that is associated
                with the sensor, and is used to identify the correct intrinsic
                parameters for each sensor.

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
        Obtains camera information for a given vision device ID using the
        `DeviceManagerClient` and `VisionConfigClient` clients, retrieving extrinsic
        and intrinsic parameters.

        """
        device_manager = DeviceManagerClient(self.router)
        vision_config = VisionConfigClient(self.router)

        vision_device_id = self.example_vision_get_device_id(device_manager)

        if vision_device_id != 0:
            self.example_routed_vision_get_extrinsics(vision_config, vision_device_id)
            self.example_routed_vision_get_intrinsics(vision_config, vision_device_id)

    def cartesian_move_relative(self, x, y, z, theta_x, theta_y, theta_z):

        """
        Calculates the target pose for a Cartesian robotic arm movement based on
        user-provided x, y, and z coordinates, as well as rotational angle (theta_x,
        theta_y, theta_z) offsets from its current pose, and then executes the
        movement using the `ExecuteAction` method. The function also waits for a
        specified time duration for the movement to complete before checking if
        it has finished or timed out.

        Args:
            x (float): 3D position offset of the cartesian movement along the
                x-axis, which is added to the target pose's x-coordinate.
            y (meter(s).): 2nd coordinate of the target position for the cartesian
                movement, which is added to the feedback's base pose's tool pose's
                y-coordinate.
                
                	* `y`: The value of `y` is a meter value passed as a part of the
                input action.
                	* `feedback.base.tool_pose_x + y`: This is the calculated value
                of the `x` coordinate of the target pose, which represents the
                position of the tool tip relative to the base platform's pose.
            z (meter(s).): 3D position of the end effector's axis in the Z direction,
                which is updated by adding the value specified in `z` to the current
                position of the end effector's axis.
                
                	* `x`: The x-coordinate of the movement target in meters.
                	* `y`: The y-coordinate of the movement target in meters.
                	* `z`: The z-coordinate of the movement target in meters.
                	* `theta_x`, `theta_y`, and `theta_z`: The Euler angles representing
                the rotational part of the movement target, in degrees.
            theta_x (degree(s).): 3D orientation of the end effector relative to
                the world coordinate system around the X-axis, which is used in
                calculating the new position and orientation of the end effector
                during the Cartesian movement.
                
                	* `theta_x`: This is an angle in degrees, representing the rotation
                around the X-axis of the Cartesian coordinate system. It is a
                numerical value that is used to orient the end effector (i.e., the
                robot's tool) during movement.
            theta_y (degree(s).): 90-degree angle rotation around the y-axis of
                the robot's end effector during the Cartesian action movement,
                which is specified in degrees.
                
                	* `theta_y`: The angle of yaw, in degrees, of the tool's rotational
                motion around its z-axis.
            theta_z (int): 3D angle rotation around the z-axis of the robot's end
                effector, which is updated to match the desired movement pattern.

        Returns:
            undefined: a boolean value indicating whether the Cartesian movement
            was successful or timed out.

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
        Moves a gripper to a destination position using speed commands and then
        waits for the reported speed to reach zero before stopping.

        Args:
            dest_pos (float): 3D position that the gripper is aiming to reach and
                serves as a reference for determining the direction and speed of
                the gripper's movement.

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
        Sets a gripper command to close the gripper at a specific speed using the
        `GRIPPER_SPEED` mode and updates the measurement to check when the speed
        is zero, allowing the gripper to stop only after reaching the target speed.

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
        Sets the speed mode for a gripper and then sends a command to open it using
        the specified speed. It waits for the gripper's position to be opened
        before ending the loop.

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