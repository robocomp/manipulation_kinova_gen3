/*
 *    Copyright (C) 2024 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

#include <ranges>
#include <rapplication/rapplication.h>
#include <cppitertools/sliding_window.hpp>
#include "api_kinova_controller.h"

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{


	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		rt = G->get_rt_api();
		inner_eigen = G->get_inner_eigen_api();

		//dsr update signals
		//connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
		//connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_node_attrs_slot);
		//connect(G.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &SpecificWorker::modify_edge_attrs_slot);
		//connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);
		
		/***
		Custom Widget
		In addition to the predefined viewers, Graph Viewer allows you to add various widgets designed by the developer.
		The add_custom_widget_to_dock method is used. This widget can be defined like any other Qt widget,
		either with a QtDesigner or directly from scratch in a class of its own.
		The add_custom_widget_to_dock method receives a name for the widget and a reference to the class instance.
		***/
		//graph_viewer->add_custom_widget_to_dock("CustomWidget", &custom_widget);

		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	//G->write_to_json_file("./"+agent_name+".json");

	delete api_controller;
}

void SpecificWorker::initialize()
{
	std::cout << "Initialize worker" << std::endl;

	// 2D widget

	if (widget_2d != nullptr)
	{
		connect(widget_2d, SIGNAL(mouse_left_click(QPointF)), this, SLOT(new_target_from_mouse(QPointF)));
	}

	new_speeds = std::vector<float>(7, 0.0);

	auto ip = configLoader.get<std::string>("ip");

	std::cout << "Trying to create the api_controller" << std::endl;
	api_controller = new api_kinova_controller(ip);
	std::cout << "api_controller created" << std::endl;

	// api_controller->move_to_selected_pose("Home");
	joints = api_controller->get_joints_info();
	gripper = api_controller->get_gripper_state();
	api_controller->print_joints_info();

	// TEST TO THE CONTACTILE SENSOR DISCOMMENT ONLY IN CASE YOU WANT TO TEST IT
	// test_contactile();
}

void SpecificWorker::compute()
{
	// TEST TO THE GRIPPER SPEED MOVEMENT DISCOMMENT ONLY IN CASE YOU WANT TO TEST IT
	// gripper_test_loop();

	// THE SPEED MOVE TEST DISCOMMENT ONLY IN CASE YOU WANT TO TEST IT
	// test_speed_move();

	joints = api_controller->get_joints_info();
	gripper = api_controller->get_gripper_state();

	if (speed_check_flag) {
		const auto now = std::chrono::system_clock::now();
		const auto ms_now = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
		if (abs(ms_now - last_time_speed_check) > 1000) {
			new_speeds = std::vector<float>(7, 0.0);
			speed_check_flag = false;
		}
		api_controller->move_joints_with_speeds(new_speeds);
	}

	// write joint values to G
	//for (const auto &names: joint_list_names | iter::sliding_window(2))
	//{
	//	names[0], names[1]

	if (auto parent = G->get_node("joint_0"); parent.has_value())
		if (auto child = G ->get_node("joint_1"); child.has_value())
		{
			if (auto edge = rt->get_edge_RT(parent.value(), child.value().id()); edge.has_value())
			{
				if (auto rot = G->get_attrib_by_name<rt_rotation_euler_xyz_att>(edge.value()); rot.has_value())
				{
					auto rot_val = rot.value().get();
					// cambiar valor
					rt->insert_or_assign_edge_RT(parent.value(), child.value().id(), {}, rot_val);
				}
			}



}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void SpecificWorker::new_target_from_mouse(QPointF p)
{
	qInfo() << "KinovaArm new_target_from_mouse" << p;
}

void SpecificWorker::gripper_test_loop() {
	static bool close = true;
	RoboCompKinovaArm::TGripper gripper_state = api_controller->get_gripper_state();
	std::cout << "Gripper state: " << gripper_state.distance << std::endl;
	if (close) {
		api_controller->move_gripper_with_vel(0.005);
		if (gripper_state.distance <= 0.05)
			close = false;
	}
	else {
		api_controller->move_gripper_with_vel(-0.005);
		if (gripper_state.distance >= 0.95)close = true;
	}
}

void SpecificWorker::test_contactile() {
	std::cout << "Test contactile" << std::endl;
	const bool close = KinovaArm_closeGripper();
	cout << "Gripper close: " << close << endl;
}

void SpecificWorker::test_speed_move() {
	std::cout << "Test speed_move" << std::endl;
	RoboCompKinovaArm::TJointSpeeds kinova_speeds;
	kinova_speeds.jointSpeeds = std::vector<float> (7, 0.0);
	kinova_speeds.jointSpeeds[6] = 5.0;
	for (auto speed : kinova_speeds.jointSpeeds) {
		cout << "Speed: " << speed << endl;
	}
	KinovaArm_moveJointsWithSpeed(kinova_speeds);
}


std::vector<float> SpecificWorker::get_joints_angles() {
	std::vector<float> angles(joints.joints.size());
	for (auto joint:joints.joints)
		angles.push_back(joint.angle);

	return angles;
}

bool SpecificWorker::KinovaArm_closeGripper()
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	float force = 0;
	float gripper_dist = gripper.distance;
	while (force < 10.0 && gripper_dist < 0.9) {
		api_controller->move_gripper_with_vel(-0.005);
		auto tactileValues = contactile_proxy->getValues();
		force = fabs(tactileValues.left.x) + fabs(tactileValues.left.y) + fabs(tactileValues.left.z)
		+ fabs(tactileValues.right.x) + fabs(tactileValues.right.y) + fabs(tactileValues.right.z);
		gripper = api_controller->get_gripper_state();
		gripper_dist = gripper.distance;
	}

	api_controller->move_gripper_with_vel(0.0);

	if (force > 2.0)
		return true;

	return false;
}

RoboCompKinovaArm::TPose SpecificWorker::KinovaArm_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo)
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	RoboCompKinovaArm::TPose ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TGripper SpecificWorker::KinovaArm_getGripperState()
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	return gripper;
}

RoboCompKinovaArm::TJoints SpecificWorker::KinovaArm_getJointsState()
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	return joints;
}

void SpecificWorker::KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles)
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	api_controller->move_joints_with_angles(angles.jointAngles);
}

void SpecificWorker::KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	new_speeds = speeds.jointSpeeds;
	speed_check_flag = true;
	const auto now = std::chrono::system_clock::now();
	last_time_speed_check = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

void SpecificWorker::KinovaArm_openGripper()
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	std::cout << "Opening the gripper" << std::endl;
	api_controller->move_gripper_with_pos(0.0);
}

void SpecificWorker::KinovaArm_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo)
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	//implementCODE
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
	std::cout << "Emergency worker" << std::endl;
	//emergencyCODE
	//
	//if (SUCCESSFUL) //The componet is safe for continue
	//  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
	std::cout << "Restore worker" << std::endl;
	//restoreCODE
	//Restore emergency component
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}


/**************************************/
// From the RoboCompContactile you can call this methods:
// RoboCompContactile::FingerTips this->contactile_proxy->getValues()

/**************************************/
// From the RoboCompContactile you can use this types:
// RoboCompContactile::FingerTip
// RoboCompContactile::FingerTips

/**************************************/
// From the RoboCompKinovaArm you can use this types:
// RoboCompKinovaArm::TPose
// RoboCompKinovaArm::TGripper
// RoboCompKinovaArm::TJoint
// RoboCompKinovaArm::TJoints
// RoboCompKinovaArm::TJointSpeeds
// RoboCompKinovaArm::TJointAngles


