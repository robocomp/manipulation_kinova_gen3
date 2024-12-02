//
// Created by robolab on 27/11/24.
//

#include "api_kinova_controller.h"

api_kinova_controller::api_kinova_controller() {
    error_callback = [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); };
    transport = new k_api::TransportClientTcp();
    cout << "Transport client connected" << endl;
    router = new k_api::RouterClient(transport, error_callback);
    cout << "Router created" << endl;
    transport->connect(IP, PORT);

    cout << "Connected" << endl;
    create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(USERNAME);
    create_session_info.set_password(PASSWORD);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    base = new k_api::Base::BaseClient(router);
    base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);
}

api_kinova_controller::api_kinova_controller(std::string ip) {
    error_callback = [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); };
    transport = new k_api::TransportClientTcp();
    router = new k_api::RouterClient(transport, error_callback);
    transport->connect(ip, PORT);

    create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(USERNAME);
    create_session_info.set_password(PASSWORD);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    base = new k_api::Base::BaseClient(router);
    base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);
}

api_kinova_controller::~api_kinova_controller() {
    session_manager->CloseSession();
    router->SetActivationStatus(false);
    transport->disconnect();

    delete session_manager;
    delete base_cyclic;
    // delete router;
    delete transport;
    delete base;
}

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)>
    create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
            break;
            default:
                break;
        }
    };
}

// Create an event listener that will set the sent reference to the exit value
// Will set to either END or ABORT
// Read the value of returnAction until it is set
std::function<void(k_api::Base::ActionNotification)>
    create_event_listener_by_ref(k_api::Base::ActionEvent& returnAction)
{
    return [&returnAction](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                returnAction = action_event;
            break;
            default:
                break;
        }
    };
}

bool api_kinova_controller::move_to_selected_position(std::string pos) {
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();

        std::cout << "Move to Home completed" << std::endl;
        std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

        return true;
    }
}


