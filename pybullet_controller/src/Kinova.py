# name: ur5_rf85.py
# content: robot+gripper class module
# author: BitMetrics (David Picado)
# date: 09-2019 (file created)
# Content Modification : ur5+gripper_rf85 class module (Modification of robot+gripper)
# author: BitMetrics (Sergi Ponsà)
# date: 11-2019 (file modified)
# ------------------------------------------------------------------------------------------------------

import os
import time
import math
import numpy as np
import pybullet as p
import pybullet_data
from scipy.optimize import fsolve
from collections import namedtuple
import attridict

from RobotDataBaseClass import RobotDataBase

# ------------------------------------------------------------------------------------------------------

class KinovaGen3():
    """Generic Robot class"""

    def __init__(self):

        """Initialization function

        urdf_root (str): root for URDF objects
        root (str): root directory for local files
        robot_urdf (str): name and location of robot URDF file,

        robot_launch_pos (list double | x,y,z in m): determine the robot base position
        robot_launch_orien (list double | rx,ry,rz in radiants): determine the robot base orientation


        last_robot_joint_name (str): URDF name of the las robot joint, the id it's the same than the last robot link
        robot_control_joints (list of str): name robot control joints in right order for inverse kinematics
        robot_mimic_joints_name (list of str): name robot joints which follow control joints in order to control them
        robot_mimic_joints_master (list of str): name joints which master each mimic joint,
        robot_mimic_multiplier (list of doubles): value of increase of the mimic respect the by increase of the master

        tcp_offset_pos (list of doubles | x,y,z in m): position offset referent to the robot_conection_to_tool_name
        tcp_offset_orien (list of doubles | rx,ry,rz in rad RPY): position offset referent to the robot_conection_to_tool_name

        nullspace (boolean): True if we want to find the inverse kinematicscloser to home
        home_position (list of doubles): joint angles in radiants of the initial joint position, I was also useing it to find solution near this configuration
        visual_inspection (boolean): If it's true it waits the real time to be able to see well the simulation

        save_database (boolean): If it's true create a database every step_simulation
        database_name (string): Name of the database

        """

        self.root = ""
        self.urdf_root = pybullet_data.getDataPath()
        self.robot_urdf = "/home/robocomp/robocomp/components/manipulation_kinova_gen3/pybullet_controller/gen3_robotiq_2f_140.urdf"

        self.robot_launch_pos = [0, 0.3, 0.6]
        self.robot_launch_orien = p.getQuaternionFromEuler([0, 0, 0])

        self.last_robot_joint_name = "EndEffector"
        self.robot_control_joints = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
        self.number_robot_control_joints = len(self.robot_control_joints)
        self.robot_mimic_joints_name = []
        self.robot_mimic_joints_master = []
        self.robot_mimic_multiplier = []
        self.tool_orient_e = [-3.14, 0, 1.57]

        self.tcp_offset_pos = [0.0, 0.0, 0.0]
        self.tcp_offset_orien_e = [0.0, 0.0, 0.0]

        self.nullspace = False
        self.home_angles = [1.57, 0.392, 0.0, 1.962, 0.0, 0.78, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
        val = 1.5
        self.position_gains = [val, val, val, val, val, val, val, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.visual_inspection = True

        self.save_database = False
        self.database_name = ""
        self.database_name_old = None
        self.database_list = []
        self.time_step = 0.001

        # launch robot in the world
        self.robot_id = p.loadURDF(self.robot_urdf, self.robot_launch_pos, self.robot_launch_orien, flags = p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT )
        print("Kinova Gen3 launched")
        p.setTimeStep(self.time_step)

        # robot data structure
        joint_type_list = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]

        joint_info = namedtuple("jointInfo",
                               ["id", "name", "type", "damping", "friction", "lower_limit", "upper_limit", "max_force", "max_velocity"])

        self.joints = attridict()
        self.joint_names = []

        # get robot data from urdf

        # get data of the joints. The id of the joint is the same as their children link
        self.num_joints = p.getNumJoints(self.robot_id)

        # initialize variables
        self.robot_control_joints_index = np.zeros(self.number_robot_control_joints, dtype=int)  # to avoid errors
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_id = info[0]
            joint_name = info[1].decode("utf-8")
            self.joint_names.append(joint_name) # I use it to search info in the dicctionary
            joint_type = joint_type_list[info[2]]
            joint_damping = info[6]
            joint_friction = info[7]
            joint_lower_limit = info[8]
            joint_upper_limit = info[9]
            joint_max_force = info[10]
            joint_max_velocity = info[11]
            single_info = joint_info(joint_id, joint_name, joint_type,joint_damping,joint_friction, joint_lower_limit, joint_upper_limit, joint_max_force,
                                   joint_max_velocity)
            self.joints[single_info.name] = single_info


            if joint_name == self.last_robot_joint_name:
                self.last_robot_joint_index = joint_id

            # while we get data of the joints, i gets the index of the control joints
            for k in range(len(self.robot_control_joints)):
                if (joint_name == self.robot_control_joints[k]):
                    self.robot_control_joints_index[k] = i

        # print the joint names
        print("Joint names: ", self.joint_names)

        # Null space parameters

        # parameters for the nullspace
        ll = [] #lower limit
        ul = [] #upper limit
        jr = [] # joint variance range
        rp = [] # the value it search to be closer to, the inverse kinematics

        # get robot data from the dicctionary
        numJoints = p.getNumJoints(self.robot_id)
        for i in range(numJoints):
            ll.append(self.joints[self.joint_names[i]].lower_limit)
            ul.append(self.joints[self.joint_names[i]].upper_limit)
            jr.append(
                abs(self.joints[self.joint_names[i]].lower_limit) + abs(self.joints[self.joint_names[i]].upper_limit))
            rp.append(0)

        # Tell that the solution has to be near to the home position
        for i in range(len(self.home_angles)):
            rp[i] = self.home_angles[i]

        self.lower_limit=ll
        self.upper_limit=ul
        self.joint_range=jr
        self.resting_pose=rp

    def move_joints_control_vel(self, joint_param_value = None, desired_force_per_one_list = [1],
                                desired_vel_per_one_list = [1] , wait=True, counter_max = 10**4,
                                error_threshold=10 ** -3,):
        """Class method to control robot position by passing joint angles
        joint_param_value (list): joint angles velocity aimed to reach
        desired_force_per_one (double): the value in per 1 of the maximum joint force  to be applied
        desired_vel_per_one (double): the value in per 1 of the maximum joint velocity to be applied
        wait (boolean): if we want to apply the control until the error is greater to the error threshold
                        or the control it's applied more than counter_max times
        counter_max: To apply maximum this amount of times the control
        error_threshold: The acceptable difference between the robot joints and the target joints
        """

        if (joint_param_value == None):
            joint_param_value = [0]*len(self.robot_control_joints)
        if (len(desired_force_per_one_list) == 1):
            desired_force_per_one_list = desired_force_per_one_list *self.number_robot_control_joints
        if (len(desired_vel_per_one_list) == 1):
            desired_vel_per_one_list = desired_vel_per_one_list *self.number_robot_control_joints

        reached = False
        counter = 0
        while not reached:

            counter += 1
            # Define the control to be applied
            for i in range(len(self.robot_control_joints)):

                #desired_force_per_one = desired_force_per_one_list[i]
                desired_force_per_one = 3
                desired_vel_per_one = desired_vel_per_one_list[i]

                #Control Joints
                p.setJointMotorControl2(self.robot_id, self.joints[self.robot_control_joints[i]].id,
                                        p.VELOCITY_CONTROL, targetVelocity=joint_param_value[i],
                                        force=self.joints[self.robot_control_joints[i]].max_force * desired_force_per_one,
                                        velocityGain=5)
                #Mimic joints
                if (len(self.robot_mimic_joints_name)>0):
                    for j in range(len(self.robot_mimic_joints_name)):
                        follow_joint = self.joints[self.robot_mimic_joints_name[j]]
                        master_joint = self.joints[self.robot_mimic_joints_master[j]]

                        if (master_joint == self.robot_control_joints[i]):

                            p.setJointMotorControl2(self.robot_id, joint.id, p.VELOCITY_CONTROL,
                                                    targetVelocity = joint_param_value[i] * self.robot_mimic_multiplier[i],
                                                    force = follow_joint.max_force * desired_force_per_one)

            #If we apply the control without care if another action modify it's trajectory and apply only 1 simulation
            if wait:
                # make step simulation
                self.step_simulation()
                # check position reached
                jointdiff = self.get_angle_difference(joint_param_value,data_by_joint = False)
                if (jointdiff <= error_threshold) or (counter > counter_max):
                    reached = True
                if (counter > counter_max):
                    if(counter == counter_max):
                        print("maximum iterations reach")
            else:
                reached = True

    def move_joints(self, joint_param_value=None, desired_force_per_one_list=[1], desired_vel_per_one_list=[1],
                    wait=True, counter_max=10**4, error_threshold=0.5):
        """Class method to control robot position by passing joint angles
        joint_param_value (list): joint angles aimed to reach
        desired_force_per_one (double): the value in per 1 of the maximum joint force  to be applied
        desired_vel_per_one (double): the value in per 1 of the maximum joint velocity to be applied
        wait (boolean): if we want to apply the control until the error is greater to the error threshold
                        or the control it's applied more than counter_max times
        counter_max: To apply maximum this amount of times the control
        error_threshold: The acceptable difference between the robot joints and the target joints
        """

        if (joint_param_value == None):
            joint_param_value = self.home_angles
        if (len(desired_force_per_one_list) == 1):
            desired_force_per_one_list = desired_force_per_one_list *self.number_robot_control_joints
        if (len(desired_vel_per_one_list) == 1):
            desired_vel_per_one_list = desired_vel_per_one_list *self.number_robot_control_joints

        reached = False
        counter = 0
        while not reached:
            counter += 1
            # Define the control to be applied
            for i in range(len(self.robot_control_joints)):

                #desired_force_per_one = desired_force_per_one_list[i]
                desired_force_per_one = 2
                desired_vel_per_one = desired_vel_per_one_list[i]

                #Control Joints
                p.setJointMotorControl2(self.robot_id, self.joints[self.robot_control_joints[i]].id,
                                        p.POSITION_CONTROL, targetPosition=joint_param_value[i],
                                        force=self.joints[self.robot_control_joints[i]].max_force * desired_force_per_one,
                                        maxVelocity=self.joints[self.robot_control_joints[i]].max_velocity * desired_vel_per_one,
                                        positionGain=self.position_gains[i], velocityGain=1.5)


                #Mimic joints
                if (len(self.robot_mimic_joints_name)>0):
                    for j in range(len(self.robot_mimic_joints_name)):
                        follow_joint = self.joints[self.robot_mimic_joints_name[j]]
                        master_joint = self.joints[self.robot_mimic_joints_master[j]]

                        if (master_joint == self.robot_control_joints[i]):

                            p.setJointMotorControl2(self.robot_id, joint.id, p.POSITION_CONTROL,
                                                    targetPosition = joint_param_value[i] * self.robot_mimic_multiplier[i],
                                                    force = follow_joint.max_force * desired_force_per_one,
                                                    maxVelocity = follow_joint.max_velocity * desired_vel_per_one,
                                                    positionGain = 2,
                                                    velocityGain = 1)

            #If we apply the control without care if another action modify it's trajectory and apply only 1 simulation
            if wait:
                # make step simulation
                self.step_simulation()
                # check position reached
                jointdiff = self.get_angle_difference(joint_param_value,data_by_joint = False)
                if (jointdiff <= error_threshold) or (counter > counter_max):
                    reached = True
                if (counter > counter_max):
                    print("maximum iterations reach")
            else:
                reached = True
            #print("Error:", jointdiff, error_threshold)
    def get_angle_difference(self,control_joints_target,data_by_joint = False):
        """
        compares the current joint state with a target angle and computes the
        difference for each joint, returning a list of differences or a single
        value depending on input.

        Args:
            control_joints_target (int): target joint angles of the robot for which
                the angle difference with the current joint states is calculated.
            data_by_joint (bool): data returned by the function, either as an array
                of length equal to the number of joints or as a scalar value
                indicating the overall angle difference.

        Returns:
            list: an array of distances representing the angle difference between
            each joint and its target value.

        """
        difference_by_joint = []
        for i in range(len(self.robot_control_joints)):
            jointstate_aux = p.getJointState(self.robot_id, self.robot_control_joints_index[i])
            if i == 0:
                jointstatepos = [jointstate_aux[0]]
                jointdiff = abs(jointstatepos[i] - control_joints_target[i])
            else:
                jointstatepos.append(jointstate_aux[0])
                jointdiff = jointdiff + abs(jointstatepos[i] - control_joints_target[i])

            difference_by_joint.append(abs(jointstatepos[i] - control_joints_target[i]))
        if(data_by_joint):
            return difference_by_joint
        else:
            return jointdiff

    def get_pose_difference(self,actual_position,actual_orientation_e,desired_position,desired_orientation_e):
        """
        takes in two lists of positions and orientations of a robot as input,
        compares them to the desired values, and outputs the differences between
        the actual and desired values for both position and orientation.

        Args:
            actual_position (list): 3D position of the object being analyzed
                relative to its previous position in the environment.
            actual_orientation_e (float): 3x3 rotation matrix that encodes the
                current orientation of the object being tracked, which is used to
                compute the difference between the current and desired orientations.
            desired_position (float): 2D position of the target pose that the code
                is trying to achieve, which is used to calculate the difference
                between the actual and desired positions.
            desired_orientation_e (float): 3D rotation vector of the target position,
                which is used to compute the difference between the actual and
                desired orientations.

        Returns:
            list: a list of differences between the actual and desired positions
            and orientations of a robot's end effector.

        """
        difference = []
        for i,j in zip(actual_position, desired_position):
            difference.append(i-j)
        for i,j in zip(actual_orientation_e, desired_orientation_e):
            difference.append(i-j)
        return difference

    def get_actual_control_joints_angle(self):
        """
        iterates over a list of robot control joints and appends the current
        position of each joint to a list called `joint_state_pos`.

        Returns:
            float: a list of joint angles for the robot's control joints.

        """
        for i in range(len(self.robot_control_joints)):
            joint_state_aux = p.getJointState(self.robot_id, self.robot_control_joints_index[i])
            if i == 0:
                joint_state_pos = [joint_state_aux[0]]  # these indexes are: [0]change joint anglesPosition, [1]Speed, [2]Reactive str, [3]Torque
            else:
                joint_state_pos.append(joint_state_aux[0])
        return joint_state_pos

    def get_actual_control_joints_velocity(self):
        """
        retrieves the current velocities of the control joints of a robot from the
        Robot Operating System (ROS) message broker. It does so by iterating through
        the length of the `robot_control_joints` list and appending the velocity
        values for each joint to a list called `joint_state_velocity`.

        Returns:
            list: a list of actual joint velocities.

        """
        joint_state_velocity = []
        for i in range(len(self.robot_control_joints)):
            joint_state_aux = p.getJointState(self.robot_id, self.robot_control_joints_index[i])
            joint_state_velocity.append(joint_state_aux[1])  # these indexes are: [0]change joint anglesPosition, [1]Speed, [2]Reactive str, [3]Torque
        return joint_state_velocity

    def get_actual_control_joints_torque(self):
        """
        iterates over a list of robot control joints and retrieves their current
        torque values from the robot's joint state. It returns a list of torque
        values for each joint.

        Returns:
            list: a list of torque values for each joint in the robot's control system.

        """
        for i in range(len(self.robot_control_joints)):
            joint_state_aux = p.getJointState(self.robot_id, self.robot_control_joints_index[i])
            if i == 0:
                joint_state_torque = [joint_state_aux[3]]  # these indexes are: [0]change joint anglesPosition, [1]Speed, [2]Reactive str, [3]Torque
            else:
                joint_state_torque.append(joint_state_aux[3])
        return joint_state_torque

    def get_actual_control_joints_full(self):
        """
        retrieves and returns a list of actual control joint states for a robot,
        obtained by iterating over the joint states and concatenating them into a
        single list.

        Returns:
            `numpy.ndarray`.: a 3D array of joint states for the robot, with each
            element representing the position of a joint in the robot's body.
            
            		- `jointsdata`: This is a list of length 3 \* number of joints in
            the robot, where each element is a tuple containing the current position
            (x, y, z) and orientation (roll, pitch, yaw) of a single joint.
            		- The list is initialized with zeros to represent the joint positions
            when the function is called.
            		- The function uses a for loop to populate the list with the actual
            values from the robot's joint states.
            		- Each joint's position and orientation are represented by separate
            elements in the tuple, which is consistent with the structure of the
            `JointState` class used by the `robot_control` module.
            

        """
        jointsdata=[0]*3*len(self.robot_control_joints)
        for i in range(len(self.robot_control_joints)):
            jointstate_aux = p.getJointState(self.robot_id, self.robot_control_joints_index[i])
            for j,k in enumerate([0,1,3]):
                jointsdata[i*3+j] = jointstate_aux[k]
        return jointsdata
    def forward_kinematics_7dof_kinova_gen3_1(self,current_joints):
        """
        computes the kinematic transformation matrix (TKinova) for a Kinova Gen
        III robot, given the current joint angles and parameters (dh_a and dh_d).
        The resulting TKinova matrix represents the end-effector coordinates in
        the world coordinate system, as calculated through the forward kinematics
        of the robot.

        Args:
            current_joints (float): 6-axis joint angles of the robot in real-time,
                which are used to compute the kinematic chain's pose and generate
                the final transformation matrix.

        Returns:
            4x4 rotation matrix.: a 4x4 rotation matrix representing the kinematic
            transformation of a RoboThespian arm.
            
            		- TKinova: This is an 8x4x4 numpy array representing the kinematic
            tree of the robot. Each element in the array corresponds to a joint
            in the robot, with the first three dimensions (0, 0, and 1) representing
            the position of the end effector, and the last three dimensions (2,
            3, and 4) representing the orientation of the end effector.
            		- dh_a: This is an 8-element numpy array representing the linear
            gain coefficients for each joint in the robot. These coefficients are
            used to correct the kinematic errors in the robot's movement.
            		- dh_d: This is an 8-element numpy array representing the rotational
            gain coefficients for each joint in the robot. These coefficients are
            used to correct the rotational kinematic errors in the robot's movement.
            		- robot_theta_zeros: This is a 7-element numpy array representing
            the initial angles of the robot's joints, measured in radians.
            		- robot_theta: This is an 8-element numpy array representing the
            final angles of the robot's joints, computed by adding the kinematic
            errors to the initial angles.
            		- TBaseEnd: This is a 4x4 numpy array representing the identity
            matrix, used as a starting point for computing the final kinematic
            tree of the robot.
            
            	The output of the `forward_kinematics_7dof_kinova_gen3_1` function
            can be destructured and analyzed further to understand its properties
            and attributes. For example, the values in the TKinova array represent
            the positions and orientations of the end effector in the robot's
            coordinate system, while the values in the dh_a and dh_d arrays represent
            the linear and rotational gain coefficients for each joint. The values
            in the robot_theta and TBaseEnd arrays can be used to compute the final
            angles of the robot's joints, taking into account any kinematic errors
            or other factors that may affect the robot's movement.
            

        """
        dh_a = [0]*8
        dh_d = [0,\
            -(0.1564+0.1284)\
            ,-(0.0054+0.0064)\
            ,-(0.2104+0.2104)\
            ,-(0.0064+0.0064)\
            ,-(0.2084+0.1059)\
            ,0\
            ,-(0.1059+0.0615)]
        dh_alpha = [3.14, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 3.14]
        robot_theta_zeros = [0,0, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
        robot_theta = np.zeros(8)
        robot_theta [1:] = list(\
            np.array(robot_theta_zeros[1:])\
             + np.array(current_joints)\
             )

        TKinova = np.zeros([8,4,4])
        for i in range(8):
            TKinova[i,0,0] = np.cos(robot_theta[i])
            TKinova[i,1,0] = np.sin(robot_theta[i])

            TKinova[i,0,1] = -np.cos(dh_alpha[i]) * np.sin(robot_theta[i])
            TKinova[i,1,1] = np.cos(dh_alpha[i]) * np.cos(robot_theta[i])
            TKinova[i,2,1] = np.sin(dh_alpha[i])

            TKinova[i,0,2] = np.sin(dh_alpha[i]) * np.sin(robot_theta[i])
            TKinova[i,1,2] = -np.sin(dh_alpha[i]) * np.cos(robot_theta[i])
            TKinova[i,2,2] = np.cos(dh_alpha[i])

            TKinova[i,0,3] = dh_a[i] * np.cos(robot_theta[i])
            TKinova[i,1,3] = dh_a[i] * np.sin(robot_theta[i])
            TKinova[i,2,3] = dh_d[i]
            TKinova[i,3,3] = 1

        TBaseEnd = np.identity(4)

        for i in range(0,8):
            TBaseEnd = np.dot(TBaseEnd,TKinova[i,:,:])

        return TBaseEnd

    def forward_kinematics_7dof_kinova_gen3_1_2(self,current_joints):
        """
        performs forward kinematics for a Kinova Gen3 robot with 7 degrees of
        freedom, given the current joint positions as input. It calculates the
        end-effector position and orientation in world coordinates using the
        kinematic tree.

        Args:
            current_joints (int): 8 joint angles of the robot, which are used to
                compute the kinematic transformation matrix `TKinova`.

        Returns:
            4x4 numpy array.: a 4x4 rotation matrix representing the kinematic
            transformation of a 7-DOF robot based on the current joint angles.
            
            		- `TBaseEnd`: A 4x4 matrix representing the base link transformation.
            It is a identity matrix.
            		- The elements of `TKinova`: These are the kinematic transformations
            for each joint in the robot. Each row represents the position and
            orientation of a joint in the world frame, while each column represents
            the corresponding position and orientation in the robot frame.
            		- `dh_a` and `dh_d`: These are two vectors representing the dynamic
            constraints of the robot. They are used to compute the final position
            and orientation of the end effector in the world frame.
            		- `robot_theta`: An 8-dimensional vector representing the current
            position and orientation of each joint in the robot.
            		- `TKinova[i,:,:]`: Each element of this matrix represents the
            kinematic transformation for a particular joint in the robot. The
            subscripts indicate the joint number (i), the rotation axis ([:]), and
            the translation vector ([:,:]) corresponding to each joint.
            

        """
        dh_a = [0]*8
        dh_d = [0\
            ,0\
            ,0\
            ,-(0.1564+0.1284+0.2104+0.2104)\
            ,-(0.0054+0.0064+0.0064+0.0064)\
            ,-(0.2084+0.1059+0.1059+0.0615)\
            ,0\
            ,0]
        #due to where I have the sensor the last 0.0615 I am no counting it
        dh_alpha = [3.14, 1.57, -1.57, 1.57, -1.57, 1.57, -1.57, 3.14]

        robot_theta = np.zeros(8)
        robot_theta [1:] = list(np.array(current_joints))

        TKinova = np.zeros([8,4,4])
        for i in range(8):
            TKinova[i,0,0] = np.cos(robot_theta[i])
            TKinova[i,1,0] = np.sin(robot_theta[i])

            TKinova[i,0,1] = -np.cos(dh_alpha[i]) * np.sin(robot_theta[i])
            TKinova[i,1,1] = np.cos(dh_alpha[i]) * np.cos(robot_theta[i])
            TKinova[i,2,1] = np.sin(dh_alpha[i])

            TKinova[i,0,2] = np.sin(dh_alpha[i]) * np.sin(robot_theta[i])
            TKinova[i,1,2] = -np.sin(dh_alpha[i]) * np.cos(robot_theta[i])
            TKinova[i,2,2] = np.cos(dh_alpha[i])

            TKinova[i,0,3] = dh_a[i] * np.cos(robot_theta[i])
            TKinova[i,1,3] = dh_a[i] * np.sin(robot_theta[i])
            TKinova[i,2,3] = dh_d[i]
            TKinova[i,3,3] = 1

        TBaseEnd = np.identity(4)

        for i in range(0,8):
            TBaseEnd = np.dot(TBaseEnd,TKinova[i,:,:])

        return TBaseEnd

    def forward_kinematics_7dof_kinova_gen3_2(self,current_joints):
        """
        calculates the end-effector position of a Kinova Gen3 arm based on the
        joint angles inputted as arguments, using a matrix multiplication approach.

        Args:
            current_joints (7-dimensional array.): 7-dimensional joint angles of
                the robot in the current state, which are used to compute the
                kinematic tree and solve the forward kinematics problem.
                
                		- `np.array([...])`: This line represents a 4D array with shape
                `(6, 10)` representing the joint angles of the robot in degrees.
                Each row corresponds to one of the 6 joints in the robot's kinematic
                chain, and each column represents the angle of that joint.
                		- `current_joints`: This is the input variable passed into the
                function, which deserializes the joint angles from a numpy array
                to a Python object. The name `current_joints` suggests that this
                variable may be updated within the function or used as part of its
                output.
                
                	Therefore, `current_joints` has the attributes and properties of
                a numpy array with 4 dimensions and shape `(6, 10)`, representing
                the joint angles of a 7-DOF robot in degrees.
                

        Returns:
            float: a 6x7 matrix representing the forward kinematic transformation
            of a 7-DoF robot arm given its current joint angles.

        """
        q = current_joints

        T01 = np.array([\
        [np.cos(q[0]),     -np.sin(q[0]),       0,          0],\
        [-np.sin(q[0]),    -np.cos(q[0]),       0,          0],\
        [0,                 0,                 -1,          0.1564],\
        [0,                 0,                  0,          1]\
        ])

        T12 = np.array([\
        [np.cos(q[1]),     -np.sin(q[1]),       0,          0],\
        [0,                 0,                 -1,          0.0054],\
        [np.sin(q[1]),     np.cos(q[1]),        0,         -0.1284],\
        [0,                 0,                  0,          1]\
        ])

        T23 = np.array([\
        [np.cos(q[2]),     -np.sin(q[2]),       0,          0],\
        [0,                 0,                  1,         -0.2104],\
        [-np.sin(q[2]),    -np.cos(q[2]),       0,         -0.0064],\
        [0,                 0,                  0,          1]\
        ])

        T34 = np.array([\
        [np.cos(q[3]),     -np.sin(q[3]),       0,          0],\
        [0,                 0,                 -1,         -0.0064],\
        [np.sin(q[3]),      np.cos(q[3]),       0,         -0.2104],\
        [0,                 0,                  0,          1]\
        ])

        T45 = np.array([\
        [np.cos(q[4]),     -np.sin(q[4]),       0,          0],\
        [0,                 0,                  1,         -0.2084],\
        [-np.sin(q[4]),    -np.cos(q[4]),       0,         -0.0064],\
        [0,                 0,                  0,          1]\
        ])

        T56 = np.array([\
        [np.cos(q[5]),     -np.sin(q[5]),       0,          0],\
        [0           ,      0           ,      -1,          0],\
        [np.sin(q[5]),      np.cos(q[5]),       0,         -0.1059],\
        [0,                 0,                  0,          1]\
        ])

        T67 = np.array([\
        [np.cos(q[6]),     -np.sin(q[6]),       0,          0],\
        [0,                 0,                 -1,         -0.1059],\
        [-np.sin(q[6]),    -np.cos(q[6]),       0,          0],\
        [0,                 0,                  0,          1]\
        ])

        T67m = np.array([\
        [np.cos(q[6]),     -np.sin(q[6]),       0,          0],\
        [0,                 0,                 -1,         -0.1059*1.4],\
        [-np.sin(q[6]),    -np.cos(q[6]),       0,          0],\
        [0,                 0,                  0,          1]\
        ])

        #print("T67", T67)
        #print("\n")

        T7E = np.array([\
        [1,                 0,                  0,          0],\
        [0,                -1,                  0,          0],\
        [0,                 0,                 -1,         -0.0615],\
        [0,                 0,                  0,          1]\
        ])

        TBaseEnd = np.dot(T01, T12)
        #print("TB2 ",TBaseEnd,"\n")
        TBaseEnd = np.dot(TBaseEnd, T23)
        #print("TB3 ",TBaseEnd,"\n")
        TBaseEnd = np.dot(TBaseEnd, T34)
        #print("TB4 ",TBaseEnd,"\n")
        TBaseEnd = np.dot(TBaseEnd, T45)
        #print("TB5 ",TBaseEnd,"\n")
        TBaseEnd = np.dot(TBaseEnd, T56)
        #print("TB6 ",TBaseEnd,"\n")
        TB7m = np.dot(TBaseEnd, T67m)
        #print("TB7m ",TB7m,"\n")
        TBaseEnd = np.dot(TBaseEnd, T67)
        #print("TB7 ",TBaseEnd,"\n")
        TBaseEnd = np.dot(TBaseEnd, T7E)

        return TB7m

    def inverse_kinematics_7dof_kinova_gen3_2(self,pose_desired,current_joints):
        #pose defined x,y,z | RPY

        #i don't take the accoun the base I computed later
        """
        solves the inverse kinematics problem for a 7-DoF Kinova Gen III robot,
        using a numerical optimization method to find the joint angles that achieve
        a specific end-effector pose.

        Args:
            pose_desired (3D rotation vector represented as three element vectors
                (x, y, z) or NumPy array.): 4D joint angles (position and orientation)
                of the end effector that the controller wants to reach as its
                desired configuration, and it is used to compute the desired
                rotation matrix and angular velocity vector for the manipulator's
                joints.
                
                		- ` pose_desired['q']`: 2D numpy array containing the desired
                positions of the robot's joints in radians. Each element in the
                array corresponds to one joint, and has shape (2,).
                		- `pose_desired['θ']`: 1D numpy array containing the desired
                orientation of the end effector in radians. Has shape (1,).
                		- `pose_desired['z']`: scalar containing the desired height of
                the end effector.
                		- `pose_desired['α']`: scalar containing the desired pitch angle
                of the robot's wrist joint.
                		- `pose_desired['β']`: scalar containing the desired yaw angle
                of the robot's shoulder joint.
                		- `pose_desired['γ']`: scalar containing the desired roll angle
                of the robot's elbow joint.
                		- `pose_desired['ρ']`: scalar containing the desired roll angle
                of the robot's wrist joint.
                
                	These properties/attributes of `pose_desired` are used in the
                function to define the system of equations that models the kinematics
                of the 7-DoF robot, and to compute the optimal solution for the
                end effector pose based on the desired positions of the joints.
                
            current_joints (float): 3D joint angles of the robot at the current
                time step, which are used as the initial conditions for the
                optimization problem to determine the optimal joint angles for the
                given task.

        Returns:
            list: a list of three angles, representing the positions of the end
            effector of a robotic arm.

        """
        dh_a = [0]*7
        dh_d = [0\
            ,0\
            ,-(0.1564+0.1284+0.2104+0.2104)\
            ,0\
            ,-(0.2084+0.1059+0.1059+0.0615)\
            ,0\
            ,0]
        dh_alpha = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 3.14]
        robot_theta_zeros = [0, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
        #Use the Paper Closedforminversekinematicssolutionforaredundantanthropomorphicrobotarm
        # http://iranarze.ir/wp-content/uploads/2016/11/E552.pdf
        theta_4 = []
        #Compute angle 4
        auxdividend_1 = (dh_d[2]**2 - 2*dh_d[2]*dh_d[4] + dh_d[4]**2 - pose_desired[0]**2 - pose_desired[1]**2 - pose_desired[2]**2)

        auxdivisor_1 = (dh_d[2]**2 + 2*dh_d[2]*dh_d[4] + dh_d[4]**2 - pose_desired[0]**2 - pose_desired[1]**2 - pose_desired[2]**2)
        theta_4.append(math.sqrt( -(auxdividend_1/auxdivisor_1) ))
        theta_4.append(-1*math.sqrt( -(auxdividend_1/auxdivisor_1) ))

        #Compute angle 1,2 & 3
        #I keep the second angle as it was
        theta_2 = current_joints[1]+robot_theta_zeros[1]
        theta_result = [0]*7
        theta_result[1] = theta_2
        #k1 & theta_3
        k1 = []
        theta_3 = []
        for th_4 in theta_4:
            k1_aux = theta_2**2 * th_4 **2 * (dh_d[2] + dh_d[4]-pose_desired[2])\
            + theta_2**2 * (dh_d[2] - dh_d[4] - pose_desired[2])\
            + th_4**2 * (-dh_d[2] - dh_d[4]-pose_desired[2])\
            - dh_d[2] + dh_d[4] - pose_desired[2]

            k1.append(k1_aux)

            auxdividend_1 = k1_aux + 4 * dh_d[4] * theta_2 *th_4
            auxdivisor_1 = k1_aux - 4 * dh_d[4] * theta_2 *th_4

            try:
                theta_3.append(math.sqrt( -(auxdividend_1/auxdivisor_1) ))
                theta_3.append(-1*math.sqrt( -(auxdividend_1/auxdivisor_1) ))
            except:
                print("Check the formulas above")
                theta_3.append(math.sqrt( (auxdividend_1/auxdivisor_1) ))
                theta_3.append(-1*math.sqrt( (auxdividend_1/auxdivisor_1) ))

        # We have 2 possible theta_4 and for each theta_4 2 possibles theta_3

        # k2 & theta_1
        k2 = []
        theta_1 = []
        for i in range(len(theta_4)):
            th_4 = theta_4[i]

            for j in range(2):
                th_3 = theta_3[i*2 + j]
                k2_aux = math.sqrt(pose_desired[0]**2 + pose_desired[1]**2)\
                    * (theta_2**2 + 1) * (th_3**2 +1) * (th_4**2 +1)\
                        + 2*th_4*dh_d[4] * (1-th_3**2) * (1-theta_2**2)\
                        + 2 *(th_3**2 +1) * theta_2 \
                            *(dh_d[2]*(th_4**2 + 1) + dh_d[4]*(1-th_4**2))

                k2.append(k2_aux)

                auxdividend_1 = 4 * th_3 *th_4 *dh_d[4]\
                    *(theta_2**2 + 1)\
                    *( math.sqrt(pose_desired[0]**2 + pose_desired[1]**2) + pose_desired[0] )\
                        +pose_desired[1] * k2_aux

                #Should be multiplied by the position theta2 but it's much worst
                auxdivisor_1 = -4 *th_3 *th_4 * dh_d[4]\
                    *(theta_2**2 + 1)\
                        +k2_aux\
                        * (math.sqrt(pose_desired[0]**2 + pose_desired[1]**2) + pose_desired[0])

                theta_1.append(auxdividend_1/auxdivisor_1)



        actual_joints_com = list(\
            np.array(current_joints)\
            + np.array(robot_theta_zeros)\
            )

        #chose the theta 4, then theta 3 and theta1
        if ( abs(actual_joints_com[3]-theta_4[0]) <= abs(actual_joints_com[3]-theta_4[1]) ):
            theta_result[3] = theta_4[0]
            if ( abs(actual_joints_com[2]-theta_3[0]) <= abs(actual_joints_com[2]-theta_3[1]) ):
                theta_result[2] = theta_3[0]
                theta_result[0] = theta_1[0]
            else:
                theta_result[2] = theta_3[1]
                theta_result[0] = theta_1[1]
        else:
            theta_result[3] = theta_4[1]
            if ( abs(actual_joints_com[2]-theta_3[2]) <= abs(actual_joints_com[2]-theta_3[3]) ):
                theta_result[2] = theta_3[2]
                theta_result[0] = theta_1[2]
            else:
                theta_result[2] = theta_3[3]
                theta_result[0] = theta_1[3]

        theta_result_ret = list(\
            np.array(theta_result)\
            - np.array(robot_theta_zeros)\
            )
        print("theta_result_ret 1st half ",theta_result_ret,"\n")
        #Computation of the possible thetas of the Wrist,

        q = theta_result_ret

        dh_a = [0]*8
        dh_d = [0\
            ,0\
            ,0\
            ,-(0.1564+0.1284+0.2104+0.2104)\
            ,-(0.0054+0.0064+0.0064+0.0064)\
            ,-(0.2084+0.1059+0.1059+0.0615)\
            ,0\
            ,0]
        #due to where I have the sensor the last 0.0615 I am no counting it
        dh_alpha = [3.14, 1.57, -1.57, 1.57, -1.57, 1.57, -1.57, 3.14]

        robot_theta = np.zeros(8)
        robot_theta [1:] = list(np.array(q))

        TKinova = np.zeros([5,4,4])
        for i in range(5):
            TKinova[i,0,0] = np.cos(robot_theta[i])
            TKinova[i,1,0] = np.sin(robot_theta[i])

            TKinova[i,0,1] = -np.cos(dh_alpha[i]) * np.sin(robot_theta[i])
            TKinova[i,1,1] = np.cos(dh_alpha[i]) * np.cos(robot_theta[i])
            TKinova[i,2,1] = np.sin(dh_alpha[i])

            TKinova[i,0,2] = np.sin(dh_alpha[i]) * np.sin(robot_theta[i])
            TKinova[i,1,2] = -np.sin(dh_alpha[i]) * np.cos(robot_theta[i])
            TKinova[i,2,2] = np.cos(dh_alpha[i])

            TKinova[i,0,3] = dh_a[i] * np.cos(robot_theta[i])
            TKinova[i,1,3] = dh_a[i] * np.sin(robot_theta[i])
            TKinova[i,2,3] = dh_d[i]
            TKinova[i,3,3] = 1

        TBaseEnd = np.identity(4)

        for i in range(0,5):
            TB4 = np.dot(TBaseEnd,TKinova[i,:,:])

        TB4i = np.linalg.inv(TB4)

        #Compute the desired rotational
        Euler = pose_desired[-3:].copy()
        Tran = pose_desired[:-3].copy()
        print("Euler ",Euler)
        print("Tran ",Tran)
        TBE = self.getHomogeniusFromEulTran(Euler,Tran)

        T4E = np.dot(TB4i,TBE)

        nx = T4E[0,0]
        ny = T4E[0,1]
        nz = T4E[0,2]

        sx = T4E[1,0]
        sy = T4E[1,1]
        sz = T4E[1,2]

        ax = T4E[2,0]
        ay = T4E[2,1]
        az = T4E[2,2]

        # Define the system of equation
        def sys_eq_sph_w_p(q):
            """
            computes the position and velocity of a spacecraft in a spherical
            coordinate system using a simple harmonic oscillator model. It takes
            in a single input vector `q` representing the current state of the
            spacecraft, and returns three equations representing the position,
            velocity, and acceleration of the spacecraft in spherical coordinates.

            Args:
                q (1-dimensional array of real numbers.): 3D position of a point
                    in space, and it is used to calculate the three components of
                    the spatial position vector in the return value of the function.
                    
                    		- `q[0]`: The x-component of the 3D vector representation
                    of the spherical coordinate system.
                    		- `q[1]`: The y-component of the 3D vector representation
                    of the spherical coordinate system.
                    		- `q[2]`: The z-component of the 3D vector representation
                    of the spherical coordinate system.
                    		- `q5`, `q6`, and `q7`: These are the values of the spherical
                    coordinates in the polar form, which are used to compute the
                    distances between points on the sphere.
                    

            Returns:
                float: a list of three angles representing the orientation of a
                spherical coordinate system in polar coordinates.

            """
            q5 = q[0]
            q6 = q[1]
            q7 = q[2]
            """
            nx = -np.sin(q5)*np.sin(q7)+np.cos(q5)*np.cos(q6)*np.cos(q7)
            ny = np.cos(q7)*np.sin(q6)
            nz = -np.cos(q5)*np.sin(q7)-np.cos(q6)*np.cos(q7)*np.sin(q5)

            sx = -np.cos(q7)*np.sin(q5)-np.cos(q5)*np.cos(q6)*np.sin(q7)
            sy = -np.sin(q6)*np.sin(q7)
            sz = -np.cos(q5)*np.cos(q7)+np.cos(q6)*np.sin(q5)*np.sin(q7)

            ax = -np.cos(q5)*np.sin(q6)
            ay = np.cos(q6)
            az = np.sin(q5)*np.sin(q6)
            """
            eq1 = -q6 + math.atan2(math.sqrt(ax**2+ay**2),az)
            eq2 = -q5 + math.atan2(ay,ax)
            eq3 = -q7 + math.atan2(sz,-nz)

            return [eq1,eq2,eq3]

        def sys_eq_sph_w_n(q):
            """
            calculates the spherical coordinates of a given vector. It takes a
            vector as input and returns three spherical coordinates (longitude,
            latitude, and altitude) representing the position of the vector in
            spherical coordinates.

            Args:
                q (ndarray.): 3D coordinates of a point in space, which are used
                    to compute the x, y, and z components of the position vector
                    and the angle between the position vector and the x-axis.
                    
                    		- `q[0]`: The x-coordinate of the center of mass of the system.
                    		- `q[1]`: The y-coordinate of the center of mass of the system.
                    		- `q[2]`: The z-coordinate of the center of mass of the system.
                    
                    	The function then calculates several related quantities, including:
                    
                    		- `nx`: The x-component of the angular momentum of the system.
                    		- `ny`: The y-component of the angular momentum of the system.
                    		- `nz`: The z-component of the angular momentum of the system.
                    
                    	The function also defines several other quantities, including:
                    
                    		- `sx`: The x-coordinate of the spin angular momentum of the
                    system.
                    		- `sy`: The y-coordinate of the spin angular momentum of the
                    system.
                    		- `sz`: The z-coordinate of the spin angular momentum of the
                    system.
                    
                    	Finally, the function returns an array containing the three
                    components of the general rotation angle (eq1), the angles of
                    rotation around the x, y, and z axes (eq2), and the angle of
                    rotation around the z axis (eq3).
                    

            Returns:
                list: three angles: `eq1`, `eq2`, and `eq3`, representing the
                spherical coordinates of a point in space.

            """
            q5 = q[0]
            q6 = q[1]
            q7 = q[2]

            """
            nx = -np.sin(q5)*np.sin(q7)+np.cos(q5)*np.cos(q6)*np.cos(q7)
            ny = np.cos(q7)*np.sin(q6)
            nz = -np.cos(q5)*np.sin(q7)-np.cos(q6)*np.cos(q7)*np.sin(q5)

            sx = np.cos(q7)*np.sin(q5)+np.cos(q5)*np.cos(q6)*np.sin(q7)
            sy = np.sin(q6)*np.sin(q7)
            sz = np.cos(q5)*np.cos(q7)-np.cos(q6)*np.sin(q5)*np.sin(q7)

            ax = np.cos(q5)*np.sin(q6)
            ay = -np.cos(q6)
            az = -np.sin(q5)*np.sin(q6)
            """

            eq1 = -q6 + math.atan2(-math.sqrt(ax**2+ay**2),az)
            eq2 = -q5 + math.atan2(-ay,-ax)
            eq3 = -q7 + math.atan2(-sz,nz)

            return [eq1,eq2,eq3]

        #solve the system closer to
        q567i = current_joints[-3:]
        theta_5_p,theta_6_p,theta_7_p = fsolve(sys_eq_sph_w_p,q567i)
        theta_5_n,theta_6_n,theta_7_n = fsolve(sys_eq_sph_w_n,q567i)




        #chosse theta 5 , 6 & 7

        diffp = abs(theta_5_p-actual_joints_com[4]) + abs(theta_6_p-actual_joints_com[5]) +abs(theta_7_p-actual_joints_com[6])
        diffn = abs(theta_5_n-actual_joints_com[4]) + abs(theta_6_n-actual_joints_com[5]) +abs(theta_7_n-actual_joints_com[6])

        if(diffp <= diffn):
            theta_result_ret [4] = theta_5_p
            theta_result_ret [5] = theta_6_p
            theta_result_ret [6] = theta_7_p
        else:
            theta_result_ret [4] = theta_5_n
            theta_result_ret [5] = theta_6_n
            theta_result_ret [6] = theta_7_n



        return theta_result_ret

    def get_jacobian(self,current_joints):
        """
        calculates the jacobian matrix for a given robot, based on its joint angles
        and velocity, and returns the tensor values.

        Args:
            current_joints (ndarray (a multi-dimensional NumPy array).): 6D joint
                states of the robot as provided by the user for calculating the
                Jacobian matrix.
                
                		- `current_joints`: This is the input tensor representing the
                current joint states of the robot.
                		- `pos`: This is a tensor representing the positions of the
                robot's end effector in global space.
                		- `vel`: This is a tensor representing the velocities of the
                robot's end effector in global space.
                		- `torq`: This is a tensor representing the torques applied to
                the robot's joints.
                		- `last_robot_joint_index`: This is an integer representing the
                index of the last robot joint in the system.
                		- `computeLinkVelocity` and `computeForwardKinematics`: These
                are optional boolean flags that determine whether the function
                should compute the link velocity or not, and whether it should
                perform forward kinematics or not.
                		- `link_trn`, `link_rot`, `com_trn`, `com_rot`, `frame_pos`,
                `frame_rot`, `link_vt`, and `link_vr`: These are tensors representing
                the torques, rotations, positions, and velocities of the robot's
                links and joints in global space.
                

        """
        pos, vel, torq = get_actual_control_joints_angle(self.robot_id)
        result = p.getLinkState(self.robot_id, self.last_robot_joint_index, computeLinkVelocity=1, computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
        jac_t, jac_r = p.calculateJacobian(self.robot_id, self.last_robot_joint_index, com_trn, pos, zero_vec, zero_vec)

    def inverse_kinematics_7dof_kinova_gen3(self,pose_desired,current_joints):
        #pose defined x,y,z | RPY

        #i don't take the accoun the base I computed later
        """
        performs inverse kinematics for a 7-DOF robotic arm, specifically the
        Kinova Gen III. It takes the current joint angles as input and calculates
        the optimal joint angles to reach a target position and orientation of the
        end effector based on the robot's Jacobian matrix.

        Args:
            pose_desired (int): 4x3 rotation matrix that specifies the desired end
                position and orientation of the end effector with respect to the
                robot's body, which is used to generate the Jacobian matrix and
                solve the optimization problem.
            current_joints (int): 7 joint angles of the robot's end effector in
                the current state, which is used to determine the initial conditions
                for the system's solution closer to the actual joint angles.

        Returns:
            float: a list of 7 joint angles that minimize the difference between
            the desired and actual robot positions.

        """
        dh_a = [0]*7
        dh_d = [-(0.1564+0.1284)\
            ,-(0.0054+0.0064)\
            ,-(0.2104+0.2104)\
            ,-(0.0064+0.0064)\
            ,-(0.2084+0.1059)\
            ,0\
            ,-(0.1059+0.0615)]
        dh_alpha = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 3.14]
        robot_theta_zeros = [0, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
        #Use the Paper Closedforminversekinematicssolutionforaredundantanthropomorphicrobotarm
        # http://iranarze.ir/wp-content/uploads/2016/11/E552.pdf
        theta_4 = []
        #Compute angle 4
        auxdividend_1 = (dh_d[2]**2 - 2*dh_d[2]*dh_d[4] + dh_d[4]**2 - pose_desired[0]**2 - pose_desired[1]**2 - pose_desired[2]**2)

        auxdivisor_1 = (dh_d[2]**2 + 2*dh_d[2]*dh_d[4] + dh_d[4]**2 - pose_desired[0]**2 - pose_desired[1]**2 - pose_desired[2]**2)
        theta_4.append(math.sqrt( -(auxdividend_1/auxdivisor_1) ))
        theta_4.append(-1*math.sqrt( -(auxdividend_1/auxdivisor_1) ))

        #Compute angle 1,2 & 3
        #I keep the second angle as it was
        theta_2 = current_joints[1]+robot_theta_zeros[1]
        theta_result = [0]*7
        theta_result[1] = theta_2
        #k1 & theta_3
        k1 = []
        theta_3 = []
        for th_4 in theta_4:
            k1_aux = theta_2**2 * th_4 **2 * (dh_d[2] + dh_d[4]-pose_desired[2])\
            + theta_2**2 * (dh_d[2] - dh_d[4] - pose_desired[2])\
            + th_4**2 * (-dh_d[2] - dh_d[4]-pose_desired[2])\
            - dh_d[2] + dh_d[4] - pose_desired[2]

            k1.append(k1_aux)

            auxdividend_1 = k1_aux + 4 * dh_d[4] * theta_2 *th_4
            auxdivisor_1 = k1_aux - 4 * dh_d[4] * theta_2 *th_4

            try:
                theta_3.append(math.sqrt( -(auxdividend_1/auxdivisor_1) ))
                theta_3.append(-1*math.sqrt( -(auxdividend_1/auxdivisor_1) ))
            except:
                print("Check the formulas above")
                theta_3.append(math.sqrt( (auxdividend_1/auxdivisor_1) ))
                theta_3.append(-1*math.sqrt( (auxdividend_1/auxdivisor_1) ))

        # We have 2 possible theta_4 and for each theta_4 2 possibles theta_3

        # k2 & theta_1
        k2 = []
        theta_1 = []
        for i in range(len(theta_4)):
            th_4 = theta_4[i]

            for j in range(2):
                th_3 = theta_3[i*2 + j]
                k2_aux = math.sqrt(pose_desired[0]**2 + pose_desired[1]**2)\
                    * (theta_2**2 + 1) * (th_3**2 +1) * (th_4**2 +1)\
                        + 2*th_4*dh_d[4] * (1-th_3**2) * (1-theta_2**2)\
                        + 2 *(th_3**2 +1) * theta_2 \
                            *(dh_d[2]*(th_4**2 + 1) + dh_d[4]*(1-th_4**2))

                k2.append(k2_aux)

                auxdividend_1 = 4 * th_3 *th_4 *dh_d[4]\
                    *(theta_2**2 + 1)\
                    *( math.sqrt(pose_desired[0]**2 + pose_desired[1]**2) + pose_desired[0] )\
                        +pose_desired[1] * k2_aux

                #Should be multiplied by the position theta2 but it's much worst
                auxdivisor_1 = -4 *th_3 *th_4 * dh_d[4]\
                    *(theta_2**2 + 1)\
                        +k2_aux\
                        * (math.sqrt(pose_desired[0]**2 + pose_desired[1]**2) + pose_desired[0])

                theta_1.append(auxdividend_1/auxdivisor_1)



        actual_joints_com = list(\
            np.array(current_joints)\
            + np.array(robot_theta_zeros)\
            )

        #chose the theta 4, then theta 3 and theta1
        if ( abs(actual_joints_com[3]-theta_4[0]) <= abs(actual_joints_com[3]-theta_4[1]) ):
            theta_result[3] = theta_4[0]
            if ( abs(actual_joints_com[2]-theta_3[0]) <= abs(actual_joints_com[2]-theta_3[1]) ):
                theta_result[2] = theta_3[0]
                theta_result[0] = theta_1[0]
            else:
                theta_result[2] = theta_3[1]
                theta_result[0] = theta_1[1]
        else:
            theta_result[3] = theta_4[1]
            if ( abs(actual_joints_com[2]-theta_3[2]) <= abs(actual_joints_com[2]-theta_3[3]) ):
                theta_result[2] = theta_3[2]
                theta_result[0] = theta_1[2]
            else:
                theta_result[2] = theta_3[3]
                theta_result[0] = theta_1[3]

        theta_result_ret = list(\
            np.array(theta_result)\
            - np.array(robot_theta_zeros)\
            )
        print("theta_result_ret 1st half ",theta_result_ret,"\n")
        #Computation of the possible thetas of the Wrist,

        q = theta_result_ret

        T01 = np.array([\
        [np.cos(q[0]),     -np.sin(q[0]),       0,          0],\
        [-np.sin(q[0]),    -np.cos(q[0]),       0,          0],\
        [0,                 0,                 -1,          0.1564],\
        [0,                 0,                  0,          1]\
        ])

        T12 = np.array([\
        [np.cos(q[1]),     -np.sin(q[1]),       0,          0],\
        [0,                 0,                 -1,          0.0054],\
        [np.sin(q[1]),     np.cos(q[1]),        0,         -0.1284],\
        [0,                 0,                  0,          1]\
        ])

        T23 = np.array([\
        [np.cos(q[2]),     -np.sin(q[2]),       0,          0],\
        [0,                 0,                  1,         -0.2104],\
        [-np.sin(q[2]),    -np.cos(q[2]),       0,         -0.0064],\
        [0,                 0,                  0,          1]\
        ])

        T34 = np.array([\
        [np.cos(q[3]),     -np.sin(q[3]),       0,          0],\
        [0,                 0,                 -1,         -0.0064],\
        [np.sin(q[3]),      np.cos(q[3]),       0,         -0.2104],\
        [0,                 0,                  0,          1]\
        ])

        TB4 = np.dot(T01,T12)
        TB4 = np.dot(TB4,T23)
        TB4 = np.dot(TB4,T34)
        TB4i = np.linalg.inv(TB4)

        #Compute the desired rotational
        Euler = pose_desired[-3:].copy()
        Tran = pose_desired[:-3].copy()
        print("Euler ",Euler)
        print("Tran ",Tran)
        TBE = self.getHomogeniusFromEulTran(Euler,Tran)

        T4E = np.dot(TB4i,TBE)

        nx = T4E[0,0]
        ny = T4E[0,1]
        nz = T4E[0,2]

        sx = T4E[1,0]
        sy = T4E[1,1]
        sz = T4E[1,2]

        ax = T4E[2,0]
        ay = T4E[2,1]
        az = T4E[2,2]

        # Define the system of equation
        def sys_eq_sph_w_p(q):
            """
            computes the spherical coordinates (nx, ny, nz) and their polar
            coordinates (x, y, z) from a given vector.

            Args:
                q (ndarray or NumPy array.): 3D cartesian coordinates of a point
                    in space, which are used to calculate the spherical coordinates
                    of that point.
                    
                    		- `q5`: The first element of `q`.
                    		- `q6`: The second element of `q`.
                    		- `q7`: The third element of `q`.
                    		- `nx`: The product of the sine and cosine of `q5`, the sine
                    of `q7`, and the cosine of the dot product of `q5` and `q6`.
                    		- `ny`: The product of the cosine and sinus of `q7`, and the
                    cosine of `q6`.
                    		- `nz`: The product of the cosine and sine of `q5`, and the
                    sinus of `q6`.
                    		- `sx`: The product of the cosine and sine of `q7`, and the
                    cosine of `q5`.
                    		- `sy`: The product of the cosine and sine of `q6`.
                    		- `sz`: The product of the cosine and sinus of `q5`, and the
                    sinus of `q6`.
                    		- `ax`: The product of the cosine and sine of `q5` and `q6`.
                    		- `ay`: The product of the cosine of `q6` and the sinus of
                    `q7`.
                    		- `az`: The product of the sinus of `q5` and the cosine of
                    `q6`.
                    
                    	In summary, `q` is a list of three elements representing the
                    position of a point in 3D space relative to some origin.
                    

            Returns:
                float: a list of three angles representing the positions of a point
                in spherical coordinates.

            """
            q5 = q[0]
            q6 = q[1]
            q7 = q[2]
            """
            nx = -np.sin(q5)*np.sin(q7)+np.cos(q5)*np.cos(q6)*np.cos(q7)
            ny = np.cos(q7)*np.sin(q6)
            nz = -np.cos(q5)*np.sin(q7)-np.cos(q6)*np.cos(q7)*np.sin(q5)

            sx = -np.cos(q7)*np.sin(q5)-np.cos(q5)*np.cos(q6)*np.sin(q7)
            sy = -np.sin(q6)*np.sin(q7)
            sz = -np.cos(q5)*np.cos(q7)+np.cos(q6)*np.sin(q5)*np.sin(q7)

            ax = -np.cos(q5)*np.sin(q6)
            ay = np.cos(q6)
            az = np.sin(q5)*np.sin(q6)
            """
            eq1 = -q6 + math.atan2(math.sqrt(ax**2+ay**2),az)
            eq2 = -q5 + math.atan2(ay,ax)
            eq3 = -q7 + math.atan2(sz,-nz)

            return [eq1,eq2,eq3]

        def sys_eq_sph_w_n(q):
            """
            computes the spherical coordinates of a point in a 3D space given its
            Cartesian coordinates. It returns the spherical coordinates (longitude,
            latitude, radius) of the point as a list of three elements.

            Args:
                q (list): 3D point of a system being transformed, and its values
                    are used to calculate the components of the transformation
                    matrix that maps the system's original coordinates to its new
                    coordinates.

            Returns:
                list: three equations representing the positions of a satellite
                in a spherical coordinate system.

            """
            q5 = q[0]
            q6 = q[1]
            q7 = q[2]

            """
            nx = -np.sin(q5)*np.sin(q7)+np.cos(q5)*np.cos(q6)*np.cos(q7)
            ny = np.cos(q7)*np.sin(q6)
            nz = -np.cos(q5)*np.sin(q7)-np.cos(q6)*np.cos(q7)*np.sin(q5)

            sx = np.cos(q7)*np.sin(q5)+np.cos(q5)*np.cos(q6)*np.sin(q7)
            sy = np.sin(q6)*np.sin(q7)
            sz = np.cos(q5)*np.cos(q7)-np.cos(q6)*np.sin(q5)*np.sin(q7)

            ax = np.cos(q5)*np.sin(q6)
            ay = -np.cos(q6)
            az = -np.sin(q5)*np.sin(q6)
            """

            eq1 = -q6 + math.atan2(-math.sqrt(ax**2+ay**2),az)
            eq2 = -q5 + math.atan2(-ay,-ax)
            eq3 = -q7 + math.atan2(-sz,nz)

            return [eq1,eq2,eq3]

        #solve the system closer to
        q567i = current_joints[-3:]
        theta_5_p,theta_6_p,theta_7_p = fsolve(sys_eq_sph_w_p,q567i)
        theta_5_n,theta_6_n,theta_7_n = fsolve(sys_eq_sph_w_n,q567i)




        #chosse theta 5 , 6 & 7

        diffp = abs(theta_5_p-actual_joints_com[4]) + abs(theta_6_p-actual_joints_com[5]) +abs(theta_7_p-actual_joints_com[6])
        diffn = abs(theta_5_n-actual_joints_com[4]) + abs(theta_6_n-actual_joints_com[5]) +abs(theta_7_n-actual_joints_com[6])

        if(diffp <= diffn):
            theta_result_ret [4] = theta_5_p
            theta_result_ret [5] = theta_6_p
            theta_result_ret [6] = theta_7_p
        else:
            theta_result_ret [4] = theta_5_n
            theta_result_ret [5] = theta_6_n
            theta_result_ret [6] = theta_7_n



        return theta_result_ret

    def inverse_kinematics_7dof_kinova_gen3(self,pose_desired,current_joints):
        #pose defined x,y,z | RPY

        #i don't take the accoun the base I computed later
        """
        solves the 7 DOF kinematic chain's inverse kinematics problem, given the
        actual joint angles and the desired end-effector positions, using the Gen
        3 algorithm from KINOVATM. It returns the calculated joint angles.

        Args:
            pose_desired (int): 4-tuple of Euler angles representing the desired
                end position and orientation of the robot's end effector, which
                is used to define the system of equations that are solved to compute
                the joint angles that achieve the desired end position and orientation.
            current_joints (int): 3D joint angles of the robot's arm at the current
                time step, which is used as the initial condition for solving the
                system of equations closer to the actual joint angles.

        Returns:
            list: a list of 7 joint angles that achieve a specific end-effector
            position and orientation, taking into account the constraints of a
            7-DOF kinematic chain.

        """
        dh_a = [0]*7
        dh_d = [-(0.1564+0.1284)\
            ,-(0.0054+0.0064)\
            ,-(0.2104+0.2104)\
            ,-(0.0064+0.0064)\
            ,-(0.2084+0.1059)\
            ,0\
            ,-(0.1059+0.0615)]
        dh_alpha = [1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 3.14]
        robot_theta_zeros = [0, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14]
        #Use the Paper Closedforminversekinematicssolutionforaredundantanthropomorphicrobotarm
        # http://iranarze.ir/wp-content/uploads/2016/11/E552.pdf
        theta_4 = []
        #Compute angle 4
        auxdividend_1 = (dh_d[2]**2 - 2*dh_d[2]*dh_d[4] + dh_d[4]**2 - pose_desired[0]**2 - pose_desired[1]**2 - pose_desired[2]**2)

        auxdivisor_1 = (dh_d[2]**2 + 2*dh_d[2]*dh_d[4] + dh_d[4]**2 - pose_desired[0]**2 - pose_desired[1]**2 - pose_desired[2]**2)
        theta_4.append(math.sqrt( -(auxdividend_1/auxdivisor_1) ))
        theta_4.append(-1*math.sqrt( -(auxdividend_1/auxdivisor_1) ))

        #Compute angle 1,2 & 3
        #I keep the second angle as it was
        theta_2 = current_joints[1]+robot_theta_zeros[1]
        theta_result = [0]*7
        theta_result[1] = theta_2
        #k1 & theta_3
        k1 = []
        theta_3 = []
        for th_4 in theta_4:
            k1_aux = theta_2**2 * th_4 **2 * (dh_d[2] + dh_d[4]-pose_desired[2])\
            + theta_2**2 * (dh_d[2] - dh_d[4] - pose_desired[2])\
            + th_4**2 * (-dh_d[2] - dh_d[4]-pose_desired[2])\
            - dh_d[2] + dh_d[4] - pose_desired[2]

            k1.append(k1_aux)

            auxdividend_1 = k1_aux + 4 * dh_d[4] * theta_2 *th_4
            auxdivisor_1 = k1_aux - 4 * dh_d[4] * theta_2 *th_4

            try:
                theta_3.append(math.sqrt( -(auxdividend_1/auxdivisor_1) ))
                theta_3.append(-1*math.sqrt( -(auxdividend_1/auxdivisor_1) ))
            except:
                print("Check the formulas above")
                theta_3.append(math.sqrt( (auxdividend_1/auxdivisor_1) ))
                theta_3.append(-1*math.sqrt( (auxdividend_1/auxdivisor_1) ))

        # We have 2 possible theta_4 and for each theta_4 2 possibles theta_3

        # k2 & theta_1
        k2 = []
        theta_1 = []
        for i in range(len(theta_4)):
            th_4 = theta_4[i]

            for j in range(2):
                th_3 = theta_3[i*2 + j]
                k2_aux = math.sqrt(pose_desired[0]**2 + pose_desired[1]**2)\
                    * (theta_2**2 + 1) * (th_3**2 +1) * (th_4**2 +1)\
                        + 2*th_4*dh_d[4] * (1-th_3**2) * (1-theta_2**2)\
                        + 2 *(th_3**2 +1) * theta_2 \
                            *(dh_d[2]*(th_4**2 + 1) + dh_d[4]*(1-th_4**2))

                k2.append(k2_aux)

                auxdividend_1 = 4 * th_3 *th_4 *dh_d[4]\
                    *(theta_2**2 + 1)\
                    *( math.sqrt(pose_desired[0]**2 + pose_desired[1]**2) + pose_desired[0] )\
                        +pose_desired[1] * k2_aux

                #Should be multiplied by the position theta2 but it's much worst
                auxdivisor_1 = -4 *th_3 *th_4 * dh_d[4]\
                    *(theta_2**2 + 1)\
                        +k2_aux\
                        * (math.sqrt(pose_desired[0]**2 + pose_desired[1]**2) + pose_desired[0])

                theta_1.append(auxdividend_1/auxdivisor_1)



        actual_joints_com = list(\
            np.array(current_joints)\
            + np.array(robot_theta_zeros)\
            )

        #chose the theta 4, then theta 3 and theta1
        if ( abs(actual_joints_com[3]-theta_4[0]) <= abs(actual_joints_com[3]-theta_4[1]) ):
            theta_result[3] = theta_4[0]
            if ( abs(actual_joints_com[2]-theta_3[0]) <= abs(actual_joints_com[2]-theta_3[1]) ):
                theta_result[2] = theta_3[0]
                theta_result[0] = theta_1[0]
            else:
                theta_result[2] = theta_3[1]
                theta_result[0] = theta_1[1]
        else:
            theta_result[3] = theta_4[1]
            if ( abs(actual_joints_com[2]-theta_3[2]) <= abs(actual_joints_com[2]-theta_3[3]) ):
                theta_result[2] = theta_3[2]
                theta_result[0] = theta_1[2]
            else:
                theta_result[2] = theta_3[3]
                theta_result[0] = theta_1[3]

        theta_result_ret = list(\
            np.array(theta_result)\
            - np.array(robot_theta_zeros)\
            )
        print("theta_result_ret 1st half ",theta_result_ret,"\n")
        #Computation of the possible thetas of the Wrist,

        q = theta_result_ret

        T01 = np.array([\
        [np.cos(q[0]),     -np.sin(q[0]),       0,          0],\
        [-np.sin(q[0]),    -np.cos(q[0]),       0,          0],\
        [0,                 0,                 -1,          0.1564],\
        [0,                 0,                  0,          1]\
        ])

        T12 = np.array([\
        [np.cos(q[1]),     -np.sin(q[1]),       0,          0],\
        [0,                 0,                 -1,          0.0054],\
        [np.sin(q[1]),     np.cos(q[1]),        0,         -0.1284],\
        [0,                 0,                  0,          1]\
        ])

        T23 = np.array([\
        [np.cos(q[2]),     -np.sin(q[2]),       0,          0],\
        [0,                 0,                  1,         -0.2104],\
        [-np.sin(q[2]),    -np.cos(q[2]),       0,         -0.0064],\
        [0,                 0,                  0,          1]\
        ])

        T34 = np.array([\
        [np.cos(q[3]),     -np.sin(q[3]),       0,          0],\
        [0,                 0,                 -1,         -0.0064],\
        [np.sin(q[3]),      np.cos(q[3]),       0,         -0.2104],\
        [0,                 0,                  0,          1]\
        ])

        TB4 = np.dot(T01,T12)
        TB4 = np.dot(TB4,T23)
        TB4 = np.dot(TB4,T34)
        TB4i = np.linalg.inv(TB4)

        #Compute the desired rotational
        Euler = pose_desired[-3:].copy()
        Tran = pose_desired[:-3].copy()
        print("Euler ",Euler)
        print("Tran ",Tran)
        TBE = self.getHomogeniusFromEulTran(Euler,Tran)

        T4E = np.dot(TB4i,TBE)

        nx = T4E[0,0]
        ny = T4E[0,1]
        nz = T4E[0,2]

        sx = T4E[1,0]
        sy = T4E[1,1]
        sz = T4E[1,2]

        ax = T4E[2,0]
        ay = T4E[2,1]
        az = T4E[2,2]

        # Define the system of equation
        def sys_eq_sph_w_p(q):
            """
            computes the position and orientation of a point on a sphere given
            three Cartesian coordinates. It returns the angle of the point's
            position, rotation around the z-axis, and rotation around the x-axis.

            Args:
                q (ndarray of shape (3,).): 3D position vector of a point in space
                    that undergoes a rotation about an arbitrary axis, which is
                    defined by the `q6` component.
                    
                    		- `q5`, `q6`, and `q7` are the x, y, and z components of
                    `q`, respectively.
                    		- `nx`, `ny`, and `nz` are calculated as the dot product of
                    `q5` and `q7`, minus the dot product of `q6` and `q7`.
                    		- `sx`, `sy`, and `sz` are calculated as the dot product of
                    `q5` and `q6`, minus the dot product of `q5` and `q7`, plus
                    the dot product of `q6` and `q7`.
                    		- `ax`, `ay`, and `az` are calculated as the dot product of
                    `q5` and `q6`, minus the dot product of `q5` and `q7`, and
                    minus the dot product of `q6` and `q7`, respectively.
                    
                    	These calculations are performed in a coordinate system defined
                    by the function, where each component of `q` represents a point
                    on a sphere. The final three elements of the output list
                    represent the angles of the sphere, measured counterclockwise
                    from the x-axis.
                    

            Returns:
                list: a list of three angular equations representing the position
                of a satellite in spherical coordinates.

            """
            q5 = q[0]
            q6 = q[1]
            q7 = q[2]
            """
            nx = -np.sin(q5)*np.sin(q7)+np.cos(q5)*np.cos(q6)*np.cos(q7)
            ny = np.cos(q7)*np.sin(q6)
            nz = -np.cos(q5)*np.sin(q7)-np.cos(q6)*np.cos(q7)*np.sin(q5)

            sx = -np.cos(q7)*np.sin(q5)-np.cos(q5)*np.cos(q6)*np.sin(q7)
            sy = -np.sin(q6)*np.sin(q7)
            sz = -np.cos(q5)*np.cos(q7)+np.cos(q6)*np.sin(q5)*np.sin(q7)

            ax = -np.cos(q5)*np.sin(q6)
            ay = np.cos(q6)
            az = np.sin(q5)*np.sin(q6)
            """
            eq1 = -q6 + math.atan2(math.sqrt(ax**2+ay**2),az)
            eq2 = -q5 + math.atan2(ay,ax)
            eq3 = -q7 + math.atan2(sz,-nz)

            return [eq1,eq2,eq3]

        def sys_eq_sph_w_n(q):
            """
            takes a vector of spherical coordinates (`q`) as input and computes
            three angular equations (equations) using those coordinates.

            Args:
                q (ndarray or NumPy array.): 3D point to be transformed through a
                    rotation around the x-, y-, and z-axes, respectively.
                    
                    		- `q[0]` represents the x-coordinate of the spherical position
                    vector.
                    		- `q[1]` represents the y-coordinate of the spherical position
                    vector.
                    		- `q[2]` represents the z-coordinate of the spherical position
                    vector.
                    		- `np.sin(q5)` and `np.cos(q5)` represent the sine and cosine
                    components of the spherical position vector in the x-y plane.
                    		- `np.sin(q7)` and `np.cos(q7)` represent the sine and cosine
                    components of the spherical position vector in the z-axis.
                    		- `np.cos(q6)` represents the cosine component of the spherical
                    position vector in the x-y plane.
                    		- `nx`, `ny`, and `nz` are calculated as the dot product of
                    the spherical position vector with the unit vectors in the x,
                    y, and z directions, respectively.
                    		- `sx`, `sy`, and `sz` are calculated as the dot product of
                    the spherical position vector with the unit vectors in the x,
                    y, and z directions, respectively.
                    		- `ax`, `ay`, and `az` are calculated as the cosine of the
                    angle between the spherical position vector and the x, y, and
                    z direction units, respectively.
                    		- `math.atan2()` calculates the arctangent of the product
                    of two vectors in a Cartesian coordinate system.
                    

            Returns:
                list: three equations representing the spherical coordinates of a
                point in a 3D space.

            """
            q5 = q[0]
            q6 = q[1]
            q7 = q[2]

            """
            nx = -np.sin(q5)*np.sin(q7)+np.cos(q5)*np.cos(q6)*np.cos(q7)
            ny = np.cos(q7)*np.sin(q6)
            nz = -np.cos(q5)*np.sin(q7)-np.cos(q6)*np.cos(q7)*np.sin(q5)

            sx = np.cos(q7)*np.sin(q5)+np.cos(q5)*np.cos(q6)*np.sin(q7)
            sy = np.sin(q6)*np.sin(q7)
            sz = np.cos(q5)*np.cos(q7)-np.cos(q6)*np.sin(q5)*np.sin(q7)

            ax = np.cos(q5)*np.sin(q6)
            ay = -np.cos(q6)
            az = -np.sin(q5)*np.sin(q6)
            """

            eq1 = -q6 + math.atan2(-math.sqrt(ax**2+ay**2),az)
            eq2 = -q5 + math.atan2(-ay,-ax)
            eq3 = -q7 + math.atan2(-sz,nz)

            return [eq1,eq2,eq3]

        #solve the system closer to
        q567i = current_joints[-3:]
        theta_5_p,theta_6_p,theta_7_p = fsolve(sys_eq_sph_w_p,q567i)
        theta_5_n,theta_6_n,theta_7_n = fsolve(sys_eq_sph_w_n,q567i)




        #chosse theta 5 , 6 & 7

        diffp = abs(theta_5_p-actual_joints_com[4]) + abs(theta_6_p-actual_joints_com[5]) +abs(theta_7_p-actual_joints_com[6])
        diffn = abs(theta_5_n-actual_joints_com[4]) + abs(theta_6_n-actual_joints_com[5]) +abs(theta_7_n-actual_joints_com[6])

        if(diffp <= diffn):
            theta_result_ret [4] = theta_5_p
            theta_result_ret [5] = theta_6_p
            theta_result_ret [6] = theta_7_p
        else:
            theta_result_ret [4] = theta_5_n
            theta_result_ret [5] = theta_6_n
            theta_result_ret [6] = theta_7_n



        return theta_result_ret

    def getRotationalFromEuler(self,euler_angles):

        """
        computes a 3x3 rotation matrix from given Euler angles (ry, rx, p) using
        the Rodrigues' formula and returns the resulting matrix as an numpy array.

        Args:
            euler_angles (ndarray.): 3D rotational angles (roll, pitch, and yaw)
                of a rigid body in Euler's angles representation.
                
                		- `r`, `p`, and `y` represent the Euler angles in the rotation
                matrix.
                		- `euler_angles` is a tuple of length 3 containing these three
                components.
                		- `np.array()` is used to convert the tuple into a numpy array
                for further manipulation.
                
                	In summary, `euler_angles` is an input tuple that contains the
                Euler angles in the following order: roll (r), pitch (p), and yaw
                (y). The function processes these angles using the rotation matrix
                to produce the rotational matrix representation of the Euler angles.
                

        Returns:
            float: a numpy array representing the rotation matrix in three dimensions.

        """
        r,p,y = euler_angles

        Rotational = np.array([\
            [np.cos(y)*np.cos(p),       np.cos(y)*np.sin(p)*np.sin(r) - np.sin(y)*np.cos(r),        np.cos(y)*np.sin(p)*np.cos(r) + np.sin(y)*np.sin(r)],\
            [np.sin(y)*np.cos(p),       np.sin(y)*np.sin(p)*np.sin(r) + np.cos(y)*np.cos(r),        np.sin(y)*np.sin(p)*np.cos(r) - np.cos(y)*np.sin(r)],\
            [-np.sin(p)         ,       np.cos(p)*np.sin(r)                                ,        np.cos(p)*np.cos(r)],\
            ])

        return Rotational

    def getHomogeniusFromRotTran(self,Rot,Tran):
        """
        combines two input matrices, `Rot` and `Tran`, to form a 3D matrix `Hom`.
        The resulting matrix is a concatenation of the rotation and translation
        vectors with an additional scale factor for each axis.

        Args:
            Rot (ndarray ( NumPy array).): 3x1 rotation matrix that transforms the
                3D tensor of tranquility coordinates into the reference frame.
                
                		- `np.array(Rot)` converts the rotational transformation `Rot`
                to an array of floating-point numbers.
                		- ` Rot = np.array(Rot)` is used to define the rotation matrix
                directly without using `np.array()`.
                		- `Tran = np.array(Tran)` converts the translational component
                `Tran` to an array of floating-point numbers, identical to `Rot`.
                		- `Tran = Tran.reshape([3,1])` rearranges the dimensions of
                `Tran` from [3, 4] to [3, 1], ensuring that all translations are
                stacked along the first axis (i.e., column-wise).
                		- `scale_factor = np.array([0,0,0,1])` creates a scalar array
                with values of [0, 0, 0, 1] to scale the `Hom` array later in the
                code.
                		- `np.vstack((Hom, scale_factor))` concatenates the `Hom` array
                with the scale factor along the first axis (i.e., row-wise),
                effectively forming a [3, 5] array that includes both `Rot` and `Tran`.
                
            Tran (1D NumPy array.): 3D transformation matrix for the translation
                of the object's coordinates from the world space to the body space.
                
                	1/ `Tran` is a numpy array with shape `(3, 1)`, indicating that
                it has three elements in each row and one element in total.
                	2/ `Tran` is deserialized from an input stream, as evidenced by
                its irregular shape.
                	3/ The values of `Tran` are not explicitly specified, suggesting
                that they may vary depending on the input data.
                

        Returns:
            array-shaped homogeneous tensor.: a numpy array with two dimensions,
            containing the homogenized rotation and translation vectors.
            
            		- The `Hom` array has shape `(4, 4)`, representing the homogeneous
            transformation matrix.
            		- The first three columns of `Hom` represent the rotation component
            of the transformation, while the fourth column represents the scaling
            component.
            		- The elements of `scale_factor` are set to `[0, 0, 0, 1]`, indicating
            that no scaling is applied to the output transformation.
            
            	The returned array can be used for further mathematical operations
            involving homogeneous transformations.
            

        """
        Tran = np.array(Tran)
        Tran = Tran.reshape([3,1])
        Rot = np.array(Rot)

        Hom = np.hstack((Rot,Tran))

        scale_factor = np.array([0,0,0,1])
        Hom = np.vstack((Hom,scale_factor))

        return Hom

    def getHomogeniusFromEulTran(self,Euler,Tran):
        """
        takes Euler and Tran angles as input and returns the homogeneous transformation
        matrix as output.

        Args:
            Euler (3D rotation matrix in the form of a numpy array.): 3D rotation
                transformation given as a list of three angles in radians, which
                is used to generate the homogeneous transformation matrix.
                
                		- `Euler`: This is a 3D vector containing the Euler angles
                representing the rotational part of the transformation. The
                dimensions of `Euler` are (3,) or (n,), where n is the number of
                degrees of freedom in the transformation.
                		- `Tran`: This is a 4th-dimensional tensor representing the
                translation part of the transformation. The dimensions of `Tran`
                are (1, 3), indicating that it contains a single element representing
                the displacement vector in 3D space.
                
            Tran (rotational transformation.): 3x3 rotation matrix used to transform
                the Euler angles into homogeneous coordinates.
                
                		- `Tran`: A rotation matrix represented as a 3x3 numpy array.
                It can be accessed using attribute access syntax (e.g., `Tran[0]`
                for the upper-left element).
                		- `Euler`: An Euler angle vector represented as a 3-element numpy
                array in radians.
                		- `Rot`: A rotation matrix obtained by applying the Euler angles
                to the identity matrix using matrix multiplication, represented
                as a 3x3 numpy array.
                

        Returns:
            `homogeneous matrix`.: a homogeneous transformation matrix.
            
            	Rotational transformation (Rot): This is a 3x3 rotation matrix obtained
            from the Euler angles provided in the input argument `Euler`. The
            rotation matrix represents a rotational transformation of the original
            coordinate system.
            	Homogenious tensor (Hom): This is an array of shape `(3, 3)` containing
            the homogenious transformation coefficients of the output. The homogenious
            tensor encodes the spatial transformation applied by the Euler angles
            to the input coordinates.
            

        """
        Rot = self.getRotationalFromEuler(Euler)
        Hom = self.getHomogeniusFromRotTran(Rot,Tran)

        return Hom

    def move_cartesian(self, pose, max_iterations = 10**8 ,nullspace = None, desired_force_per_one_list = [1], desired_vel_per_one_list = [1] , wait = True, counter_max = 10**4, error_threshold = 10 ** -3):

        """
        performs inverse kinematics on a robot using the `calculateInverseKinematics`
        method of a physics engine, and then moves the joints based on the calculated
        angles.

        Args:
            pose (list): 7D pose of the robot, consisting of position and orientation
                quaternion, which the function uses to calculate the correspondence
                to joint angles and perform control actions.
            max_iterations (int): maximum number of iterations for solving the
                inverse kinematics calculation in this function.
            nullspace (bool): joint angles corresponding to the pose in the rest
                position or nearest position and limits, skipping the computation
                of inverse kinematics if not needed, otherwise it's equivalent to
                finding the joint angles corresponding to the given pose.
            desired_force_per_one_list (list): force value that the robot should
                apply to each joint in order to reach the desired position.
            desired_vel_per_one_list (list): maximum desired velocity for each
                joint of the robot in the move_joints method call, which the
                algorithm uses to generate the control action.
            wait (bool): time for which the control action is executed before
                checking if the target position is reached.
            counter_max (int): maximum number of iterations allowed for solving
                the inverse kinematics, and it is used to terminate the process
                early if a satisfactory solution is found within that limit.
            error_threshold (`error_threshold`.): maximum allowable difference
                between the calculated joint angles and the desired values, and
                is used to stop the iterations when the difference falls below the
                threshold value.
                
                		- `error_threshold`: An instance of `float`. It represents the
                maximum tolerated difference between the actual joint angles and
                the desired values. If the difference is above this threshold, the
                control action is stopped.
                		- Unit: The unit of `error_threshold` is not explicitly stated
                in the function definition. However, it is assumed to be a dimensional
                value (e.g., meters, degrees) compatible with the joint angles and
                their computations.
                

        """
        if (nullspace == None):
            nullspace = self.nullspace

        """Class method to control the robot position by passing space coordinates
         and orientation and working out the correspondence to joint angles
         to call 'move_joints'

        pose (list): pose, i.e., position + orientation quaternion
        max_iterations (int): maximum number of iterations to solve the inverse kinematics
        nullspace (boolean): find the nearest to the defined position and limits or the nearest to the actual position

        The rest of parameters are the move_joints function parameters
        """

        # Equations to ensure it's inside the reachable area (NOT NEEDED!)

        if (nullspace == True):

            inv_result = p.calculateInverseKinematics(self.robot_id, self.last_robot_joint_index, pose[0], pose[1],\
                                                      maxNumIterations = max_iterations,\
                                                      lowerLimits = self.lower_limit,\
                                                      upperLimits = self.upper_limit,\
                                                      jointRanges = self.joint_range,\
                                                      restPoses = self.resting_pose)
        else:
            print("I don't use the null space")
            inv_result = p.calculateInverseKinematics(self.robot_id, self.last_robot_joint_index, pose[0], pose[1],\
                                                      maxNumIterations = max_iterations)
        joint_param_value = list(inv_result)

        # perform control action with 'joint_param_value'
        self.move_joints(joint_param_value = joint_param_value, wait = wait,desired_force_per_one_list=desired_force_per_one_list,desired_vel_per_one_list=desired_vel_per_one_list,counter_max=counter_max,error_threshold=error_threshold)

    def move_home(self):
        """Class method that sends robot to 'home' position"""
        self.move_joints(joint_param_value=self.home_angles)

    def set_home(self):
        """
        sets the joint angles for a robot to their home position using a loop that
        iterates through all joints and updates them individually with the target
        angle and velocity values provided in the function.

        """
        for i in range(len(self.robot_control_joints)):
            p.resetJointState(bodyUniqueId=self.robot_id, jointIndex=i,
                              targetValue=self.home_angles[i], targetVelocity=0)

    def get_actual_tcp_pose(self, print_value=False,referent_to_base = False):

        """
        calculates the position and orientation of a TCP (Tool-Class Position)
        relative to the base robot, taking into account the TCP's offset and the
        rotation of the base robot. It provides the world coordinates of the TCP
        end pose in a matrix format.

        Args:
            print_value (bool): whether or not to display the position and orientation
                of the TCP end in the response.
            referent_to_base (bool): 4x4 homogenous transformation matrix between
                the base link and the TCP link, which is applied to the position
                and orientation of the TCP end to obtain its world-relative pose.

        Returns:
            list: a set of transformation matrices that represent the pose of the
            TCP in the world frame, including its position and orientation.

        """
        last_robot_link_info = p.getLinkState(self.robot_id, self.last_robot_joint_index)
        world_last_robot_link_position = last_robot_link_info[0]
        world_last_robot_link_orientation_q = last_robot_link_info[1]
        world_last_robot_link_orientation_e = p.getEulerFromQuaternion(last_robot_link_info[1])

        if print_value:
            print("world's last robot joint position", world_last_robot_link_position,
                  world_last_robot_link_orientation_e)

        #Apply the rotation of the TCP, (the base of the TCP)
        last_robot_link_tcp_base_position = [0, 0, 0]
        last_robot_link_tcp_base_orientation_e = [self.tcp_offset_orien_e[0],
                                                  self.tcp_offset_orien_e[1],
                                                  self.tcp_offset_orien_e[2]]
        last_robot_link_tcp_base_orientation_q = p.getQuaternionFromEuler(last_robot_link_tcp_base_orientation_e)

        # transform from TCP base to world 0,0,0
        world_tcp_base_pose = p.multiplyTransforms(world_last_robot_link_position, world_last_robot_link_orientation_q,
                                                   last_robot_link_tcp_base_position,
                                                   last_robot_link_tcp_base_orientation_q)
        #Apply the translation to the tcp point
        tcp_base_tcp_end_position = self.tcp_offset_pos
        tcp_base_tcp_end_orientation_q = p.getQuaternionFromEuler([0, 0, 0])

        # transform from TCP end to world 0,0,0
        world_tcp_end_pose = p.multiplyTransforms(world_tcp_base_pose[0], world_tcp_base_pose[1],
                                                  tcp_base_tcp_end_position, tcp_base_tcp_end_orientation_q)


        if print_value:
            print("\n", "world to tcp end position", world_tcp_end_pose[0],
                     p.getEulerFromQuaternion(world_tcp_end_pose[1]))
        return world_tcp_end_pose

    def get_cartesian_offset_target_pose(self,desired_position_offset,desired_orientation_e_offset):

        """
        calculates the required position and orientation offset for a robot to
        reach a desired pose based on its current position and orientation.

        Args:
            desired_position_offset (3D position vector in homogeneous form,
                representing the desired offset to apply to the robot's actual
                position.): 3D position offset desired for the target pose relative
                to the actual TCP position.
                
                		- `desired_position_offset`: This is a 3D position vector in
                world coordinates representing the desired offset for the robot's
                position. Its components are represented by integers (`int`).
                
            desired_orientation_e_offset (3D vector containing the desired orientation
                offsets in euler angle representation.): 3D Euler angles that
                represent the desired orientation of the end effector with respect
                to its current position, which the function uses to calculate the
                appropriate move commands for the robot's end effector.
                
                		- `desired_position_offset`: a 3D vector representing the desired
                offset for the position of the target link in the global coordinate
                system.
                		- `desired_orientation_e_offset`: a 3D vector representing the
                desired orientation offset for the target link in the global
                coordinate system.
                

        Returns:
            float: an array containing the desired position and orientation of the
            end effector in Cartesian space, based on the inputs provided.

        """
        [actual_position,actual_orientation_q] = self.get_actual_tcp_pose()
        actual_orientation_e = p.getEulerFromQuaternion(actual_orientation_q)
        move_position = [actual_position[0]+desired_position_offset[0],actual_position[1]+desired_position_offset[1],\
                        actual_position[2]+desired_position_offset[2]]
        move_orientation_e = [actual_orientation_e[0]+desired_orientation_e_offset[0],\
                            actual_orientation_e[1]+desired_orientation_e_offset[1],\
                            actual_orientation_e[2]+desired_orientation_e_offset[2]]
        move_orientation_q = p.getQuaternionFromEuler(move_orientation_e)

        return [move_position,move_orientation_q]

    def move_cartesian_offset(self,desired_position_offset,desired_orientation_e_offset,max_iterations = 1000 ,nullspace = None, desired_force_per_one_list = [1], desired_vel_per_one_list = [1] , wait = True, counter_max = 20, error_threshold = 10 ** -3):

        """
        computes and moves a robot's position and orientation offset to a target
        pose while avoiding collisions and reaching the desired velocity using a
        maximum number of iterations, nullspace, and desired forces per iteration.

        Args:
            desired_position_offset (float): 3D position offset that the robot
                should move to, relative to its current position.
            desired_orientation_e_offset (float): 3D rotation matrix that the robot
                should apply to its end effector in addition to the translational
                offset requested in `move_cartesian_offset()`.
            max_iterations (int): maximum number of iterations to perform for each
                position and orientation adjustment during the motion planning process.
            nullspace (None): 4x4 transformation matrix used to transform the
                desired end position and orientation from world coordinates to
                robot frame, which is used in the motion planning process.
            desired_force_per_one_list (list): 1-dimensional force vector that the
                robot should apply to move towards the desired position in the
                absence of any other torques or forces.
            desired_vel_per_one_list (list): maximum desired velocity of the robot
                per joint list in the Cartesian space during the movement process,
                which is used as an input to the movement algorithm along with
                other control parameters to achieve the desired motion.
            wait (bool): duration for which the robot should pause after moving
                before restarting the movement sequence.
            counter_max (int): maximum number of iterations for the movement
                algorithm to reach the desired position and orientation, after
                which the method stops moving and reports the result.
            error_threshold (float): maximum acceptable error in the position and
                orientation of the end effector after reaching the desired position
                and orientation, and is used to determine whether the motion has
                been completed successfully.

        """
        [move_position,move_orientation_q] = self.get_cartesian_offset_target_pose(desired_position_offset,desired_orientation_e_offset)

        self.move_cartesian([move_position,move_orientation_q],max_iterations=max_iterations\
                            ,nullspace=nullspace,desired_force_per_one_list=desired_force_per_one_list\
                            ,desired_vel_per_one_list=desired_vel_per_one_list,wait=wait\
                            ,counter_max=counter_max,error_threshold=error_threshold)

    def get_robot_base_pose_from_world_pose(world_position,world_orientation_q):

        """
        computes the pose of a robot base in world coordinates using a set of
        pre-defined parameters. It performs inverse and direct transforms, and
        multiplies the transform matrices to obtain the final result.

        Args:
            world_position (4D vector.): 3D position of the world relative to the
                robot's base, which is transformed to get the robot's base position
                in the world frame.
                
                		- `world_position`: A 4D vector representing the position of the
                robot in the world coordinate system. It has values in meters for
                each dimension (x, y, z, and w).
                		- `world_orientation_q`: A 4D quaternion representing the
                orientation of the robot in the world coordinate system. It is a
                unit-length vector that describes the rotation of the robot around
                its center in the world frame.
                
            world_orientation_q (𝜙 (quaternion).): 4x4 quaternion matrix that
                describes the orientation of the robot's base in world coordinates,
                which is used to transform the robot's base position and orientation
                from world coordinates to robot base coordinates.
                
                		- `world_orientation_q`: A 4x3 homogeneous transformation matrix
                that represents the world orientation of the robot relative to its
                base coordinate system. The first three columns represent the
                rotation around the x, y, and z axes, respectively, while the
                fourth column represents the scalar value of the translation vector
                (i.e., the position of the robot's base with respect to its original
                location).
                

        Returns:
            instance of the `pinv.Pose` class, which represents the transformed
            robot base position and orientation in the world coordinate frame based
            on the input world pose and orientation.: the transformed position and
            orientation of the robot's base in world coordinates.
            
            		- `robot_base_world_position`: The position of the robot's base in
            the world coordinate system.
            		- `robot_base_world_orientation_q`: The orientation of the robot's
            base in the world coordinate system, represented as a quaternion.
            		- `world_position`: The position of the robot's base in the environment
            coordinate system.
            		- `world_orientation_q`: The orientation of the robot's base in the
            environment coordinate system, represented as a quaternion.
            		- `invertTransform`: A function used to invert a transformation matrix.
            		- `multiplyTransforms`: A function used to multiply two transformation
            matrices together.
            

        """
        [world_robot_base_position, world_robot_base_orientation_q] = p.getBasePositionAndOrientation(self.robot_id)
        [robot_base_world_position, robot_base_world_orientation_q] = p.invertTransform (world_robot_base_position, world_robot_base_orientation_q)

        return p.multiplyTransforms(robot_base_world_position, robot_base_world_orientation_q,world_position,world_orientation_q)

    def get_actual_tcp_pose_world_oriented(self, print_value=False):
        """ I want world_tcp_end_worldoriented_pose = world_tcp_end_pose *tcp_end_tcp_end_worldoriented_pose
         object position and TCP position are the same, the only difference is a rotation """

        world_tcp_end_pose = self.get_actual_tcp_pose()
        world_tcp_end_worldoriented_pose = p.multiplyTransforms(world_tcp_end_pose[0], world_tcp_end_pose[1],
                                                                [0.0, 0.0, 0.0],
                                                                p.getQuaternionFromEuler([-3.14, 0.0, 0.0]))

        return world_tcp_end_worldoriented_pose

    def tcp_go_pose(self, target_pos, target_orien_q, tool_orien_e=None, print_value=False):
        """Class method that controls robot position by pose (i.e., position + orientation)
        target_pos (list): position
        target_orien_q (list): target orientation, in quaternions

        return the position to be given to the tcp looking to that object"""

        if(tool_orien_e == None):
            tool_orien_e = self.tool_orient_e

        world_object_position = target_pos
        world_object_orientation_q = target_orien_q

        # object position and TCP position are the same, the only difference is a rotation
        object_tcp_end_position = [0.0, 0.0, 0.0]
        object_tcp_end_orientation_q = p.getQuaternionFromEuler(tool_orien_e)

        # get the world_TCP pose (rotate)
        world_tcp_end_pose = p.multiplyTransforms(world_object_position, world_object_orientation_q,
                                                  object_tcp_end_position,
                                                  object_tcp_end_orientation_q)
        if print_value:
            print("\n", "world to tcp pose", world_tcp_end_pose[0], p.getEulerFromQuaternion(world_tcp_end_pose[1]))

        # from TCP to base, there is only a translation, the TCP offset
        tcp_end_base_position = [-1 * self.tcp_offset_pos[0], -1 * self.tcp_offset_pos[1], -1 * self.tcp_offset_pos[2]]
        tcp_end_base_orientation_q = p.getQuaternionFromEuler([0.0, 0.0, 0.0])

        # get the world TCP pose (translate)
        world_base_pose = p.multiplyTransforms(world_tcp_end_pose[0], world_tcp_end_pose[1], tcp_end_base_position,
                                               tcp_end_base_orientation_q)
        if print_value:
            print("\n", "world to base pose", world_base_pose[0], p.getEulerFromQuaternion(world_base_pose[1]))

        # from base to last link there is only one rotatiton
        base_lastlink_position = [0.0, 0.0, 0.0]
        base_lastlink_orientation_q = p.getQuaternionFromEuler([0.0,0.0,0.0])

        # get world's last link pose
        world_lastlink_pose = p.multiplyTransforms(world_base_pose[0], world_base_pose[1], base_lastlink_position,
                                                   base_lastlink_orientation_q)
        if print_value:
            print("\n", "world to lastlink pose", world_lastlink_pose[0],
                  p.getEulerFromQuaternion(world_lastlink_pose[1]))

        return world_lastlink_pose

    def get_object_position(self, objectID, modify_center=[0.0, 0.0, 0.0], orientation_correction_e=[-1.57, 0, 1.57]):
        """
        calculates and returns an object's position and orientation in world space,
        based on its ID and optional modification center and orientation correction
        values.

        Args:
            objectID (int): 3D object whose position and orientation are to be retrieved.
            modify_center (float): 3D position where the world coordinate system
                should be translated and scaled to account for any desired offset
                or rotation of the object's center of mass relative to its actual
                position.
            orientation_correction_e (3D Euler angles sequence.): 3D Euler angles
                in Earth-centered coordinates to correct the object's orientation
                calculated from its world position and orientation, expressed as
                a list of three values in degrees [-1.57, 0, 1.57].
                
                		- `-1.57`, `0`, and `1.57`: These values correspond to the X,
                Y, and Z components of the quaternion's angle axis, respectively.
                They serve as a rotation offset for the object's orientation,
                helping to correct for potential misalignments in the scene.
                
                	Therefore, `orientation_correction_e` is an array of three numbers
                that help refine the object's orientation estimate through this
                quaternion-based correction.
                

        Returns:
            list: a list containing the position and orientation of an object in
            the world reference frame, after applying modifications to the center
            and orientation correction.

        """
        self.objectIDpick = objectID
        object_link_info = p.getBasePositionAndOrientation(self.objectIDpick)
        object_position = object_link_info[0]  # refered to world
        object_orientation = object_link_info[1]  # refered to world
        print("\n", "world object pose", object_position, object_orientation)
        object_position_pick = list(object_position)  # refered to world
        object_position_pick = [object_position_pick[0] - modify_center[0], object_position_pick[1] - modify_center[1],
                                object_position_pick[2] - modify_center[2]]
        object_orientation_pick_e = p.getEulerFromQuaternion(object_orientation)
        object_orientation_pick_e = [object_orientation_pick_e[0] - orientation_correction_e[0],
                                     object_orientation_pick_e[1] - orientation_correction_e[1],
                                     object_orientation_pick_e[2] - orientation_correction_e[2]]
        object_orientation_pick_q = p.getQuaternionFromEuler(object_orientation_pick_e)
        return [object_position_pick, object_orientation_pick_q]

    def wait(self, time_wait):
        """
        waits for a specified time interval before executing the next iteration
        of the simulation.

        Args:
            time_wait (int): duration of simulation to be waited before proceeding
                with the next iteration of the loop in milliseconds.

        """
        if self.visual_inspection:
            t = int(time_wait/self.time_step)
        else:
            t = int(time_wait * 20)
        for i in range(t):
            self.step_simulation()

    def step_simulation(self):
        """Step simulation method"""
        p.stepSimulation()
        if self.visual_inspection:
            time.sleep(self.time_step)

        if self.save_database:
            self.record_database()

    def create_empty_file(self,path):
        "Create a new file where you can write without losing data"
        aux_path = path [:-5] #copy all except the .urdf
        for  i in range(256):
            aux_path_2 = aux_path + "_" + str(i) + ".urdf"
            try:
                f = open(aux_path_2, "x")
                f.close
                return aux_path_2
                break
            except:
                print(aux_path_2 + " already exist" )
        return "error"

    def Copy_file(self,path2read,path2copy):
        """
        reads a file located at `path2read` and copies its contents to a new file
        located at `path2copy`.

        Args:
            path2read (str): file that will be read from and its contents will be
                written to the `path2copy`.
            path2copy (str): location where the contents of the file read from
                `path2read` are written.

        """
        readf = open(path2read,"r")
        writef = open(path2copy,"w")
        for line in readf:
            writef.write(line)
        readf.close()
        writef.close()

    def get_modify_elements_urdf_joint(self):
        "provide the options to modify the joints"
        joint_mod = ["damping","friction","lower", "upper", "effort", "velocity"]
        return joint_mod

    def get_modify_elements_urdf_link(self):
        "provide the options to modify the links"
        link_mod = ["mass","inertia"]
        return link_mod
    def get_modify_elements_robot(self):
        """
        returns an array of 18 dynamic element names in robot simulations.

        Returns:
            list: a list of 17 string names for attributes that can be modified
            to customize the robot's dynamics.

        """
        elements_changeDynamics = ["mass","lateral_friction","spinning_friction","rolling_friction",\
                                "restitution","linear_damping","angular_damping","contact_stiffness",\
                                "contact_damping","friction_anchor","inertia","collision_sphere_radius",\
                                "collision_distance_threshold","activation_state","damping","anisotropic_friction",\
                                "velocity","collision_margin"]
        return elements_changeDynamics

    def modify_urdf(self,path2read,path2write,element_to_modify, value , link_or_joint_name = None):
        """
        To work with an specific name it's need it to be all together without space name=something >
        """


        #load the options
        joint_opt = self.get_modify_elements_urdf_joint()
        link_opt = self.get_modify_elements_urdf_link()

        #elements field
        inertial_elements = ["mass","inertia"]
        dynamics_elements = ["damping","friction"]
        limit_elements = ["lower", "upper", "effort", "velocity"]

        #1rst and 2nd element to search
        #Check if we have to search a joint or a link
        if (element_to_modify in joint_opt):
            opening_search_1 = "<joint"
            closing_search_1 = "</joint"
            if(link_or_joint_name == None):
                opening_search_2 = "name"
            else:
                opening_search_2 = "name=\""+str(link_or_joint_name)+"\""
            closing_search_2 = "</joint"

        elif((element_to_modify in link_opt)):
            opening_search_1 = "<link"
            closing_search_1 = "</link"
            if(link_or_joint_name == None):
                opening_search_2 = "name"
            else:
                opening_search_2 = "name=\""+str(link_or_joint_name)+"\""
                print(opening_search_2)
            closing_search_2 = "</link"
        else:
            print("Doesn't exist the field you are asking for, check modify_elements_link and modify_elements_joint ")
            return

        print("\n"+opening_search_2+"\n")

        #3rd element to search
        if (element_to_modify in inertial_elements):
            opening_search_3 = "<inertial"
            closing_search_3 = "</inertial"
        elif((element_to_modify in dynamics_elements)):
            opening_search_3 = "<dynamics"
            closing_search_3 = "/>"
        elif((element_to_modify in limit_elements)):
            opening_search_3 = "<limit"
            closing_search_3 = "/>"
        else:
            print("Doesn't exist an element field, please add it to the function")
            return

        #4rd element to search and write
        if (element_to_modify == "mass"):
            opening_search_4 = "value"
            closing_search_4 = "/>"
            writevalue = " =  \"" + str(value) + "\" "

        elif(element_to_modify == "inertia"):
            opening_search_4 = "ixx"
            closing_search_4 = "/>"
            print(value)
            if(len(value) != 6):
                print("The given inertia is wrong please insert a list of the 6 elements of inertia")
                print("ixx, ixy, ixz, iyy, iyz, izz")
                return
            else:
                writevalue = " = \"" + str(value[0]) + "\" "  + \
                        "ixy = \"" + str(value[1]) + "\" "  + \
                        "ixz = \"" + str(value[2]) + "\" "  + \
                        "iyy = \"" + str(value[3]) + "\" "  + \
                        "iyz = \"" + str(value[4]) + "\" "  + \
                        "izz = \"" + str(value[5]) + "\" "
        elif(element_to_modify == "damping"):
            opening_search_4 = "damping"
            closing_search_4 = "friction"
            writevalue = " = \"" + str(value)+"\" "

        elif(element_to_modify == "friction"):
            opening_search_4 = "friction"
            closing_search_4 = "/>"
            writevalue = " = \"" + str(value) + "\" "

        elif(element_to_modify == "lower"):
            opening_search_4 = "lower"
            closing_search_4 = "upper"
            writevalue = " = \"" + str(value)+"\" "

        elif(element_to_modify == "upper"):
            opening_search_4 = "upper"
            closing_search_4 = "effort"
            writevalue = " = \"" + str(value)+"\" "

        elif(element_to_modify == "effort"):
            opening_search_4 = "effort"
            closing_search_4 = "velocity"
            writevalue = " = \"" + str(value) + "\" "

        elif(element_to_modify == "velocity"):
            opening_search_4 = "velocity"
            closing_search_4 = "/>"
            writevalue = " = \"" + str(value)+ "\" "

        else:
            print("An error happend check this function")

        #Everything it's defined

        #DEfinition of files to use
        if(path2read == path2write):
            #Create a dummy file
            dummyf = open("dummy_urdf.urdf","w")
            dummyf.close()
            self.Copy_file(path2read,"dummy_urdf.urdf")
            path2read = "dummy_urdf.urdf"

        readf = open(path2read, "r")
        writef = open(path2write, "w")

        print ("Overwriting " + path2write + " Reading " + path2read )

        #Process of modification
        auxtext = ""
        for line in readf :
            auxtext = auxtext + line
            if(opening_search_1 in auxtext):
                #print("I find the first")
                auxindex_open_1 = auxtext.find(opening_search_1)
                auxindex_open_2 = auxtext.find(opening_search_2,auxindex_open_1 )
                auxindex_close_1 = auxtext.find(closing_search_1,auxindex_open_2 )

                #If it has found the opening and the closing do something else continue
                if((auxindex_open_2 != -1) and (auxindex_close_1 != -1)):
                    #print("I find the second")
                    #it's not the one I am searching so write until the close
                    if(auxindex_close_1 < auxindex_open_2):
                        #split the data to free memory
                        textwrite = auxtext[:auxindex_close_1]
                        auxtext = auxtext[auxindex_close_1:]
                        writef.write(textwrite)
                        textwrite = ""
                    else:
                        auxindex_open_3 = auxtext.find(opening_search_3,auxindex_open_2)
                        auxindex_close_2 = auxtext.find(closing_search_2,auxindex_open_3)

                        #If it has found the opening and the closing do something, else continue
                        if((auxindex_open_3 != -1) and (auxindex_close_2 != -1)):
                            # print("I find the third")
                            #it's not the one I am searching so write until the close
                            if(auxindex_close_2 < auxindex_open_3):
                                #split the data to free memory
                                textwrite = auxtext[:auxindex_close_2]
                                auxtext = auxtext[auxindex_close_2:]
                                writef.write(textwrite)
                                textwrite = ""
                            else:
                                auxindex_open_4 = auxtext.find(opening_search_4,auxindex_open_3)
                                auxindex_close_3 = auxtext.find(closing_search_3,auxindex_open_4)

                                #If it has found the opening and the closing do something, else continue
                                if((auxindex_open_4 != -1) and (auxindex_close_3 != -1)):
                                    print("I find the fourth")
                                    #it's not the one I am searching so write until the close
                                    if(auxindex_close_3 < auxindex_open_4):
                                        #split the data to free memory
                                        textwrite = auxtext[:auxindex_close_3]
                                        auxtext = auxtext[auxindex_close_3:]
                                        writef.write(textwrite)
                                        textwrite = ""
                                    else:
                                        auxindex_close_4 = auxtext.find(closing_search_4,auxindex_open_4)
                                        #If it has found the opening and the closing do something, else continue
                                        if((auxindex_open_4 != -1) and (auxindex_close_4 != -1)):
                                            print("I find all")
                                            textwrite = auxtext[:auxindex_open_4]
                                            textwrite = textwrite + opening_search_4
                                            #print(textwrite)
                                            writef.write(textwrite)

                                            #print(writevalue)
                                            writef.write(writevalue)

                                            textwrite = auxtext[auxindex_close_4:]
                                            #print(textwrite)
                                            writef.write(textwrite)
                                            #clean to read from that point
                                            auxtext = ""
        writef.write(auxtext)
        readf.close()
        writef.close()
        print("Files closed")

    def modify_urdf_list(self,path2read,path2write,joints_names_2modify_list,element_to_modify_list, value_list ):

        """
        checks and modifies URDF joint values based on a list of expected values
        and a list of links to modify.

        Args:
            path2read (str): file path to read the urdf joints information from.
            path2write (str): path to write modified URDF joints to an external file.
            joints_names_2modify_list (str): list of joints or links that should
                be modified in the UrDF file based on their expected values.
            element_to_modify_list (list): list of urdf joints to be modified based
                on their expected values from the external file.
            value_list (list): list of values that need to be applied to the urdf
                joints, and it is used to check if the expected number of values
                is matching with the actual values provided.

        """
        possible_elements_to_modify = self.get_modify_elements_urdf_joint()
        possible_elements_to_modify.extend(self.get_modify_elements_urdf_link())

        print (value_list)
        #time.sleep(10)

        dict_expected_values = {}
        for possible in possible_elements_to_modify:
            if (possible != "inertia"):
                dict_expected_values[possible] = 1
            else:
                dict_expected_values[possible] = 6

        expected_values = 0

        for i in element_to_modify_list:
            expected_values += element_to_modify_list.count(i) * dict_expected_values[i]

        if( (expected_values == 0) or ( (expected_values * len(joints_names_2modify_list)) != len(value_list) ) ):
            print("Expected " + str(expected_values) \
            + "and given " + str(len(value_list)/len(joints_names_2modify_list)) + "values"+ "modify urdf joints" )

            print("element_to_modify_list",element_to_modify_list)

            print("values",value_list)

            time.sleep(10)
        else:
            #The main program once it's checked

            # The second one and de following ones it's saved to the first external file, but first it's copied to a dummy
            for joint_name in joints_names_2modify_list :

                for element in element_to_modify_list:

                    #Get the link name or joint name
                    if(element in self.get_modify_elements_urdf_link()):
                        info = p.getJointInfo(self.robot_id,self.joints[joint_name].id)
                        LinkName = str(info[12], "utf-8")
                    else:
                        LinkName = joint_name

                    print(LinkName)

                    if (dict_expected_values[element] ==1):
                        element_value = value_list.pop(0)
                        self.modify_urdf(path2read,path2write,element,element_value,\
                                        link_or_joint_name=LinkName)
                    else:
                        element_value_list = []

                        for i in range( dict_expected_values[element] ):
                            element_value_list = value_list.pop(0)
                        self.modify_urdf(path2read,path2write,element,element_value_list,\
                                        link_or_joint_name=LinkName)
            print("created")

    def modify_robot_pybullet(self,joints_names_2modify_list,element_to_modify_list, value_list ):

        """
        modifies the robot's dynamics by changing the specified joint properties.

        Args:
            joints_names_2modify_list (list): list of names of robot joints to
                modify their parameters.
            element_to_modify_list (list): list of joint elements to be modified
                based on the corresponding values provided in the `value_list`.
            value_list (list): 3D list of values that will be assigned to the
                specified joint parameters.

        """
        possible_elements_to_modify = self.get_modify_elements_robot()

        #Check the elements lenght it's right before do nothing

        dict_expected_values = {}
        for possible in possible_elements_to_modify:
            if (possible == "inertia"):
                dict_expected_values[possible] = 3
            else:
                dict_expected_values[possible] = 1

        expected_values = 0
        for i in element_to_modify_list:
            expected_values += element_to_modify_list.count(i) * dict_expected_values[i]

        if( (expected_values == 0) or ( (expected_values * len(joints_names_2modify_list)) != len(value_list) ) ):

            print("Expected " + str(expected_values* len(joints_names_2modify_list))  \
            + " and given " + str(len(value_list)) + " values" +"modify robot joints" )

            print("element_to_modify_list",element_to_modify_list)

            print("values",value_list)

            time.sleep(10)
        else:

            # The second one and the following ones it's saved to the first external file, but first it's copied to a dummy
            for joint_name in joints_names_2modify_list :

                joint_index = self.joints[joint_name].id

                for element in element_to_modify_list:

                    if (element in possible_elements_to_modify):
                        if (element == "mass"):
                            p.changeDynamics(self.robot_id,joint_index, mass = value_list.pop(0))
                        elif (element == "lateral_friction"):
                            p.changeDynamics(self.robot_id,joint_index, lateralFriction = value_list.pop(0))
                        elif (element == "spinning_friction"):
                            p.changeDynamics(self.robot_id,joint_index, spinningFriction = value_list.pop(0))
                        elif (element == "rolling_friction"):
                            p.changeDynamics(self.robot_id,joint_index, rollingFriction = value_list.pop(0))
                        elif (element == "restitution"):
                            p.changeDynamics(self.robot_id,joint_index, restitution = value_list.pop(0))
                        elif (element == "linear_damping"):
                            p.changeDynamics(self.robot_id,joint_index, linearDamping = value_list.pop(0))
                        elif (element == "angular_damping"):
                            p.changeDynamics(self.robot_id,joint_index, angularDamping = value_list.pop(0))
                        elif (element == "contact_stiffness"):
                            p.changeDynamics(self.robot_id,joint_index, contactStiffness = value_list.pop(0))
                        elif (element == "friction_anchor"):
                            p.changeDynamics(self.robot_id,joint_index, frictionAnchor = value_list.pop(0))
                        elif (element == "inertia"):
                            func_value_list = []
                            for i in range(3):
                                func_value_list.append(value_list.pop(0))
                            p.changeDynamics(self.robot_id,joint_index, localInertiaDiagonal = func_value_list )
                        elif (element == "collision_sphere_radius"):
                            p.changeDynamics(self.robot_id,joint_index, ccdSweptSphereRadiu = value_list.pop(0))
                        elif (element == "collision_distance_threshold"):
                            p.changeDynamics(self.robot_id,joint_index, contactProcessingThreshold = value_list.pop(0))
                        elif (element == "activation_state"):
                            p.changeDynamics(self.robot_id,joint_index, activationState = value_list.pop(0))
                        elif (element == "damping"):
                            p.changeDynamics(self.robot_id,joint_index, jointDamping = value_list.pop(0))
                        elif (element == "anisotropic_friction"):
                            p.changeDynamics(self.robot_id,joint_index, anisotropicFriction = value_list.pop(0))
                        elif (element == "max_velocity"):
                            p.changeDynamics(self.robot_id,joint_index, maxJointVelocity = value_list.pop(0))
                        elif (element == "collision_margin"):
                            p.changeDynamics(self.robot_id,joint_index, collisionMargin = value_list.pop(0))
                    else:
                        print("the parameter "+ element+" it's not a parameter of the changeDynamics parameters")
    def modify_robot_pybullet_base(self,element_to_modify_list, value_list ):

        """
        checks if any of the specified base parameters need to be updated based
        on the given list of values. If any parameter needs updating, it changes
        the corresponding value for that parameter.

        Args:
            element_to_modify_list (int): list of parameters to modify in the
                robot's dynamics, which are selected from a set of possible
                parameters defined in `possible_elements_to_modify`.
            value_list (list): 0-based index list of the values to be updated for
                each corresponding parameter of the robot's dynamics in the
                `changeDynamics` method call.

        """
        possible_elements_to_modify = ["mass_base","inertia_base"]

        #Check the elements lenght it's right before do nothing

        dict_expected_values = {}
        for possible in possible_elements_to_modify:
            if (possible == "inertia_base"):
                dict_expected_values[possible] = 3
            else:
                dict_expected_values[possible] = 1

        expected_values = 0
        for i in element_to_modify_list:
            expected_values += element_to_modify_list.count(i) * dict_expected_values[i]

        if( (expected_values == 0) or ( expected_values != len(value_list) ) ):
            print("Expected " + str(expected_values)  \
            + " and given " + str(len(value_list)) + " values" + "modify base" )

            print("element_to_modify_list",element_to_modify_list)

            print("values",value_list)

            time.sleep(10)

        else:

            joint_index = 0

            for element in element_to_modify_list:

                if (element in possible_elements_to_modify):
                    if (element == "mass_base"):
                        p.changeDynamics(self.robot_id,joint_index, mass = value_list.pop(0))
                    elif (element == "lateral_friction_base"):
                        p.changeDynamics(self.robot_id,joint_index, lateralFriction = value_list.pop(0))
                    elif (element == "spinning_friction_base"):
                        p.changeDynamics(self.robot_id,joint_index, spinningFriction = value_list.pop(0))
                    elif (element == "rolling_friction_base"):
                        p.changeDynamics(self.robot_id,joint_index, rollingFriction = value_list.pop(0))
                    elif (element == "restitution_base"):
                        p.changeDynamics(self.robot_id,joint_index, restitution = value_list.pop(0))
                    elif (element == "linear_damping_base"):
                        p.changeDynamics(self.robot_id,joint_index, linearDamping = value_list.pop(0))
                    elif (element == "angular_damping_base"):
                        p.changeDynamics(self.robot_id,joint_index, angularDamping = value_list.pop(0))
                    elif (element == "contact_stiffness_base"):
                        p.changeDynamics(self.robot_id,joint_index, contactStiffness = value_list.pop(0))
                    elif (element == "friction_anchor_base"):
                        p.changeDynamics(self.robot_id,joint_index, frictionAnchor = value_list.pop(0))
                    elif (element == "inertia_base"):
                        func_value_list = []
                        for i in range(3):
                            func_value_list.append(value_list.pop(0))
                        p.changeDynamics(self.robot_id,joint_index, localInertiaDiagonal = func_value_list )
                    elif (element == "collision_sphere_radius_base"):
                        p.changeDynamics(self.robot_id,joint_index, ccdSweptSphereRadiu = value_list.pop(0))
                    elif (element == "collision_distance_threshold_base"):
                        p.changeDynamics(self.robot_id,joint_index, contactProcessingThreshold = value_list.pop(0))
                    elif (element == "activation_state_base"):
                        p.changeDynamics(self.robot_id,joint_index, activationState = value_list.pop(0))
                    elif (element == "damping_base"):
                        p.changeDynamics(self.robot_id,joint_index, jointDamping = value_list.pop(0))
                    elif (element == "anisotropic_friction_base"):
                        p.changeDynamics(self.robot_id,joint_index, anisotropicFriction = value_list.pop(0))
                    elif (element == "max_velocity_base"):
                        p.changeDynamics(self.robot_id,joint_index, maxJointVelocity = value_list.pop(0))
                    elif (element == "collision_margin_base"):
                        p.changeDynamics(self.robot_id,joint_index, collisionMargin = value_list.pop(0))
                else:
                    print("the parameter "+ element+" it's not a parameter of the changeDynamics base parameters")

    def get_robot_pybullet_param_dynamics(self,joint_names_2read_list):

        """
        retrieves dynamics information for a robot using PyBullet. It iterates
        over a list of joint names and extracts the dynamics data for each joint,
        vstacking the data into a single NumPy array.

        Args:
            joint_names_2read_list (str): list of joint names to read data for,
                which determines the joints for which the dynamics information is
                retrieved from the PyBullet robot simulation environment.

        Returns:
            `numpy array`.: a NumPy array containing the dynamic data of the robot.
            
            	1/ Data: This variable contains an array with the dynamics information
            of the robot's joints. The array has multiple dimensions representing
            the time and space variables.
            	2/ shape: This attribute provides the number of rows and columns in
            the Data array.
            
            	Without referring to the function or any other external data, this
            description of the output properties is provided in under 100 words.
            

        """
        first = True
        for joint_name in joint_names_2read_list :
            joint_index = self.joints[joint_name].id
            if (first == True):
                Data = np.array(p.getDynamicsInfo(self.robot_id,joint_index))
                first = False
                #print(Data)
                #print(Data.shape)
                #time.sleep(10)
            else:
                Data = np.vstack(( Data,np.array(p.getDynamicsInfo(self.robot_id,joint_index)) ))
        return Data
    def get_robot_pybullet_param_joints(self,joint_names_2read_list):

        """
        retrieves joint parameters from a Bullet robot simulation using PyBullet
        and stacks them into a NumPy array.

        Args:
            joint_names_2read_list (list): list of joint names that are to be read
                from the Bullet simulation environment using the `p.getJointInfo()`
                function.

        Returns:
            `ndarray`.: a numpy array containing joint information of the robot.
            
            		- `Data`: A numpy array containing the joint parameters for each
            joint in the list `joint_names_2read_list`. Each element in the array
            is a 4-dimensional vector representing the position and orientation
            of the joint.
            		- `shape`: The shape of the output array, which is `(N,3)` where `N`
            is the number of joints in the list `joint_names_2read_list`.
            
            	Note: No summary is provided at the end as requested.
            

        """
        first = True
        for joint_name in joint_names_2read_list :
            joint_index = self.joints[joint_name].id
            if (first == True):
                Data = np.array(p.getJointInfo(self.robot_id,joint_index))
                first = False
                #print(Data)
                #print(Data.shape)
                #time.sleep(10)
            else:
                Data = np.vstack(( Data,np.array(p.getJointInfo(self.robot_id,joint_index)) ))
        return Data

    def record_database(self):
        """
        updates a database with the current robot's joint angles, velocities, and
        torque, as well as its current TCP pose and orientation. It also saves the
        current time.

        """
        if(self.database_name != self.database_name_old):

            if(self.database_name_old != None):
                auxdatabase = self.database
                self.database_list.append(auxdatabase)

            self.database_name_old = self.database_name
            self.database = RobotDataBase(self.database_name,time_step = self.time_step)

        self.database.joints_angles_rad.append( self.get_actual_control_joints_angle() )
        self.database.joint_angles_vel_rad.append( self.get_actual_control_joints_velocity() )
        self.database.joint_torques.append( self.get_actual_control_joints_torque() )

        [tcp_position, tcp_orientation_q] = self.get_actual_tcp_pose()
        self.database.tcp_position.append(tcp_position)
        self.database.tcp_orientation_q.append(tcp_orientation_q)
        self.database.tcp_orientation_e.append(p.getEulerFromQuaternion(tcp_orientation_q))
        self.database.save_time()


# ------------------------------------------------------------------------------------------------------