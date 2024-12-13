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

/**
	\brief
	@author Jorge Calderon Gonzalez
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>

#include "api_kinova_controller.h"

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	/**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();

	bool KinovaArm_closeGripper();
	RoboCompKinovaArm::TPose KinovaArm_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo);
	RoboCompKinovaArm::TGripper KinovaArm_getGripperState();
	RoboCompKinovaArm::TJoints KinovaArm_getJointsState();
	void KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles);
	void KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds);
	void KinovaArm_openGripper();
	void KinovaArm_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo);


public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();
	void modify_node_slot(std::uint64_t, const std::string &type){};
	void modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){};
	void modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){};
	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){};
	void del_node_slot(std::uint64_t from){};
	void new_target_from_mouse(QPointF);

private:

	std::unique_ptr<DSR::RT_API> rt;
	std::shared_ptr<DSR::InnerEigenAPI> inner_eigen;

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

	/**
	*  @brief Object to control the kinova api
	*/
	api_kinova_controller *api_controller;

	/**
	* @brief Object storing the state of the gripper
	*/
	RoboCompKinovaArm::TGripper gripper{};

	/**
	* @brief Object storing the state of the arm joints
	*/
	RoboCompKinovaArm::TJoints joints{};

	/**
	* @brief Vector that contains the new speed to move the arm
	*/
	std::vector<float> new_speeds;

	/**
	* @brief Flag indicating when to move the arm with speeds
	*/
	bool speed_check_flag = false;

	/**
	* @brief Timestamp to know when to stop move the arm with speed
	*/
	long long last_time_speed_check;

	/**
	* @brief List with the links names on the json
	*/
	const std::vector<string> json_links_names{
		"base_link",
		"shoulder_link",
		"half_arm_1_link",
		"half_arm_2_link",
		"forearm_link",
		"spherical_wrist_1_link",
		"spherical_wrist_2_link",
		"bracelet_link"
	};


	//----------------------------METHODS----------------------------//

	/**
	* @brief Test method for the gripper, closing and opening it constantly
	*/
	void gripper_test_loop();

	/**
	* @brief Test method for the contactile sensors
	*/
	void test_contactile();

	/**
	* @brief Test method for the movement with joints speeds
	*/
	void test_speed_move();

	/** @brief Get the joints angles in a vector
	*
	* @return The vector with the joints angles
	*/
	std::vector<float> get_joints_angles();

	/**
	* @brief Update the DSR with the joints info
	*/
	void update_dsr_joints();


signals:
	//void customSignal();
};

#endif
