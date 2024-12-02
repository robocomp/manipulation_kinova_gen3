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
    api_kinova_controller();
    api_kinova_controller(std::string ip);
    ~api_kinova_controller();

    bool move_to_selected_position(std::string pos);

};



#endif //API_KINOVA_CONTROLLER_H
