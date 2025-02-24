//
// Created by robolab on 27/11/24.
//

#ifndef API_KINOVA_CONTROLLER_H
#define API_KINOVA_CONTROLLER_H

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <string>
#include <TransportClientTcp.h>
#include <KinovaArm.h>
#include <chrono>

#include <cmath>

namespace k_api = Kinova::Api;

#define PORT 10000
#define IP "192.168.1.11"
#define USERNAME "admin"
#define PASSWORD "admin"

constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

class api_kinova_controller {
private:
    function<void(Kinova::Api::KError)> error_callback;
    k_api::TransportClientTcp *transport;
    k_api::RouterClient *router;

    k_api::Session::CreateSessionInfo create_session_info;
    k_api::SessionManager *session_manager;

    k_api::Base::BaseClient *base;
    k_api::BaseCyclic::BaseCyclicClient *base_cyclic;

public:

    /**
     * @brief Default constructor that initializes the connection to the predefined ip
     */
    api_kinova_controller();

    /**
     * @brief Parameterized constructor to initialize the controller at the specified ip
     *
     * @param ip The ip you want the controller to connect to
     */
    api_kinova_controller(std::string ip);

    /**
     * @brief Destructor of the class
     */
    ~api_kinova_controller();

    /**
     * @brief List the preset poses
     */
    void list_poses();

    /** @brief Moves the arm to one of the preset poses
     *
     * @param pose name of the pose
     *
     * @return True in case the pose is found and can move the arm to that pose,
     * False in case is not able to move the arm or do not find the selected pose,
     * in the latter case the possible poses will be listed.
     */
    bool move_to_selected_pose(std::string pose);

    /**
     * @brief Return the state of the arm joints
     *
     * @return The Robocomp struct that represents the joints info
     */
    RoboCompKinovaArm::TJoints get_joints_info();

    /**
     * @brief Print the state of the arm joints
     */
    void print_joints_info();

    /**
    * @brief Return the state of the arm gripper
    *
    * @return The Robocomp struct that represents the gripper info
    */
    RoboCompKinovaArm::TGripper get_gripper_state();

    /**
    * @brief Move the gripper to a position
    *
    * @param pos Position to move the gripper (0.0 - totally open, 1.0 - totally closed)
    *
    * @return True if can move the gripper, False otherwise
    */
    bool move_gripper_with_pos(float pos);

    /**
    * @brief Move the gripper with a velocity
    *
    * @param vel velocity to move the gripper (vel > 0 - close the gripper, vel < 0 - open the gripper)
    *
    * @return True if can move the gripper, False otherwise
    */
    bool move_gripper_with_vel(float vel);

    /**
    * @brief Move the arm joints with velocities
    *
    * @param speeds vector with the velocity to each joint
    *
    * @return True if can move the joints, False otherwise
    */
    bool move_joints_with_speeds(std::vector<float> speeds);

    /**
    * @brief Move the arm joints with angles
    *
    * @param angles vector with the angle to each joint
    *
    * @return True if the joints can reach the angle, False otherwise
    */
    bool move_joints_with_angles(std::vector<float> angles);

    /**
     * @brief Get the forward kinematics for the end-effector's arm
     *
     * @return Tuple with two vector, first represents the translations and second represent the rotations
     */
    std::tuple<std::vector<float>, std::vector<float>> get_forward_kinematics();

};



#endif //API_KINOVA_CONTROLLER_H
