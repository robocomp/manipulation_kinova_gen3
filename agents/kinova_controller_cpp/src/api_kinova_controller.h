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

// #include "utilities.h"

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
     * @struct Joint_info
     * @brief Stores the position, speed and torque of each of an arm joint
     */
    struct Joint_info {
        int id;
        float position;
        float velocity;
        float torque;
        float current;
        float voltage;
        float motor_temperature;
        float core_temperature;
    };

    typedef vector<Joint_info> Joints_info;

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
     * @return Structure with three lists for storing the position, speed and torsion of each joint
     */
    Joints_info get_joints_info();

    /**
     * @brief Print the state of the arm joints
     */
    void print_joints_info();



};



#endif //API_KINOVA_CONTROLLER_H
