#! /usr/bin/env python3
#
# Created by Jorge Calderon Gonzalez in Robolab at 28/05/2025
#

import sys
import threading

from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient

from kortex_api.autogen.messages import Session_pb2, Base_pb2, Common_pb2, VisionConfig_pb2, DeviceConfig_pb2

PORT = 10000
TIMEOUT_DURATION = 5000

class KinovaGen3():
    """Kinova Gen3 robot client for controlling the arm, gripper, and vision system."""
    def __init__(self, ip):
        """Initialize the Kinova Gen3 client with the given IP address."""
        self.error_callback = lambda kException: print("_________ callback error _________ {}".format(kException))
        self.transport = TCPTransport()
        self.router = RouterClient(self.transport, self.error_callback)
        self.transport.connect(ip, PORT)

        # Create session
        self.session_info = Session_pb2.CreateSessionInfo()
        self.session_info.username = "admin"
        self.session_info.password = "admin"
        self.session_info.session_inactivity_timeout = 60000  # (milliseconds)
        self.session_info.connection_inactivity_timeout = 2000  # (milliseconds)

        self.session_manager = SessionManager(self.router)
        self.session_manager.CreateSession(self.session_info)

        self.device_manager = DeviceManagerClient(self.router)
        self.device_config = DeviceConfigClient(self.router)
        self.vision_config = VisionConfigClient(self.router)
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)

        self.vision_device_id = self.get_vision_device_id()

    def __del__(self):
        """Destructor to close the session and disconnect the transport."""
        # self.session_manager.CloseSession()
        # self.transport.disconnect()

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

    def get_vision_device_id(self):
        """
        Retrieve the device identifier for the vision module.
        :return: The device identifier for the vision module, or 0 if not found.
        """
        vision_device_id = 0

        # Getting all device routing information (from DeviceManagerClient service)
        all_devices_info = self.device_manager.ReadAllDevices()

        vision_handles = [hd for hd in all_devices_info.device_handle if hd.device_type == DeviceConfig_pb2.VISION]
        if len(vision_handles) == 0:
            print("Error: there is no vision device registered in the devices info")
        elif len(vision_handles) > 1:
            print("Error: there are more than one vision device registered in the devices info")
        else:
            handle = vision_handles[0]
            vision_device_id = handle.device_identifier
            # print("Vision module found, device Id: {0}".format(vision_device_id))

        return vision_device_id

    def list_posibles_actions(self):
        """List all the possible actions that can be performed by the arm."""
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)

        print("\n=== Lista de Acciones (REACH_JOINT_ANGLES) ===")
        for i, action in enumerate(action_list.action_list, 1):
            print(f"{i}. {action.name}")
        print("=" * 40)

    def move_to_selected_pose(self, action_name):
        """ Move the arm to a specified pose using a predefined action.
        :param action_name: The name of the action to execute, to see the available actions use list_posibles_actions().
        """
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

        print(action_handle)

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

    def get_arm_state(self):
        """
        Get the current state of the arm including voltage, current, temperatures, and IMU data.
        :return: A dictionary containing the voltage, current, temperatures, and IMU data.
        """
        feedback = self.base_cyclic.RefreshFeedback()
        imu = [feedback.base.imu_acceleration_x,
               feedback.base.imu_acceleration_y,
               feedback.base.imu_acceleration_z,
               feedback.base.imu_angular_velocity_x,
               feedback.base.imu_angular_velocity_y,
               feedback.base.imu_angular_velocity_z]

        return {"arm_voltage": feedback.base.arm_voltage,
                "arm_current": feedback.base.arm_current,
                "temperature_cpu": feedback.base.temperature_cpu,
                "temperature_ambient": feedback.base.temperature_ambient,
                "imu": imu}

    def get_joints_state(self):
        """
        Get the current state of the joints including position, velocity, torque, current, voltage, and temperatures.
        :return: A dictionary containing the angles, velocities, torques, currents, voltages, motor temperatures,
        and core temperatures of the joints.
        """
        feedback = self.base_cyclic.RefreshFeedback()
        jointsPosition = [j.position for j in feedback.actuators]
        jointsVelocity = [j.velocity for j in feedback.actuators]
        jointsTorque = [j.torque for j in feedback.actuators]
        jointCurrent = [j.current_motor for j in feedback.actuators]
        jointsVoltage = [j.voltage for j in feedback.actuators]
        jointsMotorTemperature = [j.temperature_motor for j in feedback.actuators]
        jointsCoreTemperature = [j.temperature_core for j in feedback.actuators]

        return {"angles": jointsPosition,
                "velocities": jointsVelocity,
                "torques": jointsTorque,
                "currents": jointCurrent,
                "voltages": jointsVoltage,
                "motor_temperatures": jointsMotorTemperature,
                "core_temperatures": jointsCoreTemperature}

    def get_gripper_state(self):
        """
        Get the current state of the gripper.
        :return: The current position of the gripper finger.
        """
        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        return self.base.GetMeasuredGripperMovement(gripper_request).finger[0].value

    def get_tool_state(self):
        """
        Get the current state of the tool including pose, twist, and external wrench.
        :return: A dictionary containing the pose, twist, and external wrench of the tool.
        """
        feedback = self.base_cyclic.RefreshFeedback()
        tool_pose = [feedback.base.tool_pose_x,
                     feedback.base.tool_pose_y,
                     feedback.base.tool_pose_z,
                     feedback.base.tool_pose_theta_x,
                     feedback.base.tool_pose_theta_y,
                     feedback.base.tool_pose_theta_z]
        tool_twist = [feedback.base.tool_twist_linear_x,
                      feedback.base.tool_twist_linear_y,
                      feedback.base.tool_twist_linear_z,
                      feedback.base.tool_twist_angular_x,
                      feedback.base.tool_twist_angular_y,
                      feedback.base.tool_twist_angular_z]
        tool_external_wrench = [feedback.base.tool_external_wrench_force_x,
                                feedback.base.tool_external_wrench_force_y,
                                feedback.base.tool_external_wrench_force_z,
                                feedback.base.tool_external_wrench_torque_x,
                                feedback.base.tool_external_wrench_torque_y,
                                feedback.base.tool_external_wrench_torque_z]

        return {"pose": tool_pose,
                "twist": tool_twist,
                "external_wrench": tool_external_wrench}

    def gripper_move_to(self, target_position):
        """
            Move the gripper to a specific position.
            :arg target_position: The target position for the gripper, typically between 0.0 (open) and 1.0 (closed).
        """

        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        finger.value = target_position
        self.base.SendGripperCommand(gripper_command)

        return True

    def gripper_move_speed(self, speed: float):
        """
            Move the gripper with specified speed.
            :arg speed: The speed at which the gripper should move, for example, -0.005 for closing or 0.005 for opening.
        """
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with speed increments
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.finger_identifier = 1
        finger.value = speed

        self.base.SendGripperCommand(gripper_command)
        return True

    def move_joints_with_speeds(self, speeds):
        """Move the arm with specified joint speeds."""
        joint_speeds = Base_pb2.JointSpeeds()

        i = 0
        for speed in speeds:
            joint_speed = joint_speeds.joint_speeds.add()
            joint_speed.joint_identifier = i
            joint_speed.value = speed
            joint_speed.duration = 0
            i = i + 1

        self.base.SendJointSpeedsCommand(joint_speeds)

    def move_joints_with_angles(self, joints):
        """Move the arm to the specified joint angles."""
        action = Base_pb2.Action()
        action.name = "Angular action movement"
        action.application_data = ""

        print("join ", joints)


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

    def change_camera_autofocus(self, action: VisionConfig_pb2.SensorFocusAction):
        """Change the camera autofocus settings."""
        sensor_focus_action = action
        sensor_focus_action.sensor = VisionConfig_pb2.SENSOR_COLOR
        print("Change camera autofocus setting to :", sensor_focus_action.focus_mode)
        self.vision_config.DoSensorFocusAction(sensor_focus_action, self.vision_device_id)