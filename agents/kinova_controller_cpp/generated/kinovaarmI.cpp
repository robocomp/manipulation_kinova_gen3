/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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
#include "kinovaarmI.h"

KinovaArmI::KinovaArmI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	closeGripperHandlers = {
		[this]() -> bool {if (worker != nullptr) return worker->KinovaArm_closeGripper(); else throw std::runtime_error("Worker is null");}
	};

	getCenterOfToolHandlers = {
		[this](auto &a) -> RoboCompKinovaArm::TPose {if (worker != nullptr) return worker->KinovaArm_getCenterOfTool(a); else throw std::runtime_error("Worker is null");}
	};

	getGripperStateHandlers = {
		[this]() -> RoboCompKinovaArm::TGripper {if (worker != nullptr) return worker->KinovaArm_getGripperState(); else throw std::runtime_error("Worker is null");}
	};

	getJointsStateHandlers = {
		[this]() -> RoboCompKinovaArm::TJoints {if (worker != nullptr) return worker->KinovaArm_getJointsState(); else throw std::runtime_error("Worker is null");}
	};

	getToolInfoHandlers = {
		[this]() -> RoboCompKinovaArm::TToolInfo {if (worker != nullptr) return worker->KinovaArm_getToolInfo(); else throw std::runtime_error("Worker is null");}
	};

	moveJointsWithAngleHandlers = {
		[this](auto &a) {if (worker != nullptr) worker->KinovaArm_moveJointsWithAngle(a); else throw std::runtime_error("Worker is null");}
	};

	moveJointsWithSpeedHandlers = {
		[this](auto &a) {if (worker != nullptr) worker->KinovaArm_moveJointsWithSpeed(a); else throw std::runtime_error("Worker is null");}
	};

	openGripperHandlers = {
		[this]() {if (worker != nullptr) worker->KinovaArm_openGripper(); else throw std::runtime_error("Worker is null");}
	};

	setCenterOfToolHandlers = {
		[this](auto &a, auto &b) {if (worker != nullptr) worker->KinovaArm_setCenterOfTool(a, b); else throw std::runtime_error("Worker is null");}
	};

	setGripperPosHandlers = {
		[this](auto &a) -> bool {if (worker != nullptr) return worker->KinovaArm_setGripperPos(a); else throw std::runtime_error("Worker is null");}
	};

	stopArmHandlers = {
		[this]() {if (worker != nullptr) worker->KinovaArm_stopArm(); else throw std::runtime_error("Worker is null");}
	};

}

KinovaArmI::~KinovaArmI()
{
}

bool KinovaArmI::closeGripper(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return closeGripperHandlers.at(id)();
}

RoboCompKinovaArm::TPose KinovaArmI::getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getCenterOfToolHandlers.at(id)(referencedTo);
}

RoboCompKinovaArm::TGripper KinovaArmI::getGripperState(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getGripperStateHandlers.at(id)();
}

RoboCompKinovaArm::TJoints KinovaArmI::getJointsState(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getJointsStateHandlers.at(id)();
}

RoboCompKinovaArm::TToolInfo KinovaArmI::getToolInfo(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getToolInfoHandlers.at(id)();
}

void KinovaArmI::moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	moveJointsWithAngleHandlers.at(id)(angles);
}

void KinovaArmI::moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	moveJointsWithSpeedHandlers.at(id)(speeds);
}

void KinovaArmI::openGripper(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	openGripperHandlers.at(id)();
}

void KinovaArmI::setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	setCenterOfToolHandlers.at(id)(pose, referencedTo);
}

bool KinovaArmI::setGripperPos(float pos, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return setGripperPosHandlers.at(id)(pos);
}

void KinovaArmI::stopArm(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	stopArmHandlers.at(id)();
}