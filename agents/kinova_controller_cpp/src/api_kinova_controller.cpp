//
// Created by robolab on 27/11/24.
//

#include "api_kinova_controller.h"

#include <ranges>

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


void api_kinova_controller::list_poses() {
    std::cout << "List Positions" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    for (auto action_name : action_list.action_list()) {
        std::cout << "[" << action_name.name() << "]" << std::endl;
    }
}

bool api_kinova_controller::move_to_selected_pose(std::string pos) {
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
    bool pos_found = false;
    for (auto action : action_list.action_list())
    {
        if (action.name() == pos)
        {
            action_handle = action.handle();
            pos_found = true;
        }
    }
    if (!pos_found){
        list_poses();
        return false;
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

RoboCompKinovaArm::TJoints api_kinova_controller::get_joints_info() {
    RoboCompKinovaArm::TJoints joints_info;
    auto feedback = base_cyclic->RefreshFeedback();
    int actuator_id = 0;
    for (auto actuator : feedback.actuators()) {
        joints_info.joints.push_back(RoboCompKinovaArm::TJoint({actuator_id ,actuator.position(), actuator.velocity(), actuator.torque(),
            actuator.current_motor(), actuator.voltage(), actuator.temperature_motor(), actuator.temperature_core()}));
        actuator_id++;
    }
    const auto now = std::chrono::system_clock::now();
    joints_info.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    return joints_info;
}

void api_kinova_controller::print_joints_info() {
    std::cout << std::endl << "Printing Joints info" << std::endl << std::endl;
    auto joints_info = get_joints_info();
    for (auto joint: joints_info.joints) {
        std::cout << "----Joint " << joint.id << "----" << std::endl;
        std::cout << "Angle: " << joint.angle << std::endl;
        std::cout << "Velocity: " << joint.velocity << std::endl;
        std::cout << "Torque: " << joint.torque << std::endl;
        std::cout << "Current: " << joint.current << std::endl;
        std::cout << "Voltage: " << joint.voltage << std::endl;
        std::cout << "Motor temperature: " << joint.motorTemperature << std::endl;
        std::cout << "Motor core: " << joint.coreTemperature << std::endl << std::endl;
    }
    std::cout << "----Timestamp: " << joints_info.timestamp << "----" << std::endl << std::endl;
}

RoboCompKinovaArm::TGripper api_kinova_controller::get_gripper_state() {
    RoboCompKinovaArm::TGripper gripper_info;
    k_api::Base::GripperRequest gripper_request;
    gripper_request.set_mode(k_api::Base::GRIPPER_POSITION);
    gripper_info.distance = base->GetMeasuredGripperMovement(gripper_request).finger(0).value();
    return gripper_info;
}

bool api_kinova_controller::move_gripper_with_pos(const float pos) {
    k_api::Base::GripperCommand gripper_command;
    gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
    const auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
    finger->set_value(pos);
    base->SendGripperCommand(gripper_command);
    return true;
}

bool api_kinova_controller::move_gripper_with_vel(const float vel) {
    k_api::Base::GripperCommand gripper_command;
    gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);
    const auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
    finger->set_value(vel);
    base->SendGripperCommand(gripper_command);
    return true;
}

bool api_kinova_controller::move_joints_with_speeds(std::vector<float> speeds) {
    k_api::Base::JointSpeeds joint_speeds;
    int joint_id = 0;
    for (const auto speed: speeds) {
        const auto joint_speed = joint_speeds.add_joint_speeds();
        joint_speed->set_joint_identifier(joint_id);
        joint_speed->set_value(speed);
        joint_speed->set_duration(1);
        joint_id++;
    }
    base->SendJointSpeedsCommand(joint_speeds);

    return true;
}

bool api_kinova_controller::move_joints_with_angles(std::vector<float> angles) {
    auto action = k_api::Base::Action();
    const auto reach_joint_angles = action.mutable_reach_joint_angles();
    const auto joint_angles = reach_joint_angles->mutable_joint_angles();
    int joint_id = 0;
    for (const auto angle: angles) {
        const auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(joint_id);
        joint_angle->set_value(angle);
        joint_id++;
    }

    std::promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        k_api::Common::NotificationOptions()
    );

    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    const auto status = finish_future.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if(status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }
    const auto promise_event = finish_future.get();

    std::cout << "Angular movement completed" << std::endl;
    std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

    return true;
}

std::tuple<std::vector<float>, std::vector<float>>api_kinova_controller::get_forward_kinematics()
{
    auto joints_angles = base->GetMeasuredJointAngles();
    const auto pose = base->ComputeForwardKinematics(joints_angles);
    return std::make_tuple(std::vector<float>{pose.x(), pose.y(), pose.z()},
                           std::vector<float>{pose.theta_x(),pose.theta_y(), pose.theta_z()});
}




