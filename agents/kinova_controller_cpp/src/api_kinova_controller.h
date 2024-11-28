//
// Created by robolab on 27/11/24.
//

#ifndef API_KINOVA_CONTROLLER_H
#define API_KINOVA_CONTROLLER_H

#include <BaseClientRpc.h>
#include <SessionManager.h>
#include <DeviceConfigClientRpc.h>

#include <RouterClient.h>
#include <string>
#include <TransportClientTcp.h>

#include "utilities.h"

namespace k_api = Kinova::Api;

#define PORT 10000
#define IP "192.168.1.10"
#define USERNAME "admin"
#define PASSWORD "admin"

class api_kinova_controller {
private:
    k_api::kError error_callback;
    k_api::TransportClientTcp transport;
    k_api::RouterClient router;

    k_api::CreateSesionInfo create_session_info;
    k_api::SessionManager session_manager;

    k_api::Base::BaseCliente base;
    k_api::BaseCyclic::BaseCyclicClient base_cyclic;

public:
    api_kinova_controller();
    api_kinova_controller(std::string ip);
    ~api_kinova_controller();

    void move_to_selected_position(std::string pos);

};



#endif //API_KINOVA_CONTROLLER_H
