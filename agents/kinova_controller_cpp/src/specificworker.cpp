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

	auto ip = configLoader.get<std::string>("ip");

	std::cout << "Trying to create the api_controller" << std::endl;
	api_controller = new api_kinova_controller();
	std::cout << "api_controller created" << std::endl;

	api_controller->move_to_selected_pose("Home");
	api_controller->print_joints_info();

}

void SpecificWorker::compute()
{
    // std::cout << "Compute worker" << std::endl;

	// TEST TO THE GRIPPER SPEED MOVEMENT DISCOMMENT ONLY IN CASE YOU WANT TO TEST IT
	// gripper_test_loop();

	joints = api_controller->get_joints_info();
	gripper = api_controller->get_gripper_state();

	//computeCODE
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
    	//    if (img.empty())
    	//        emit goToEmergency()
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	
	
}

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

bool SpecificWorker::KinovaArm_closeGripper()
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	bool ret{};
	//implementCODE

	return ret;
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
	//implementCODE
}

void SpecificWorker::KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	//implementCODE
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


