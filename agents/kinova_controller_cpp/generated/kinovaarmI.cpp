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
#include "kinovaarmI.h"

KinovaArmI::KinovaArmI(GenericWorker *_worker)
{
	worker = _worker;
}


KinovaArmI::~KinovaArmI()
{
}


bool KinovaArmI::closeGripper(const Ice::Current&)
{
	return worker->KinovaArm_closeGripper();
}

RoboCompKinovaArm::TPose KinovaArmI::getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo, const Ice::Current&)
{
	return worker->KinovaArm_getCenterOfTool(referencedTo);
}

RoboCompKinovaArm::TGripper KinovaArmI::getGripperState(const Ice::Current&)
{
	return worker->KinovaArm_getGripperState();
}

RoboCompKinovaArm::TJoints KinovaArmI::getJointsState(const Ice::Current&)
{
	return worker->KinovaArm_getJointsState();
}

void KinovaArmI::moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles, const Ice::Current&)
{
	worker->KinovaArm_moveJointsWithAngle(angles);
}

void KinovaArmI::moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds, const Ice::Current&)
{
	worker->KinovaArm_moveJointsWithSpeed(speeds);
}

void KinovaArmI::openGripper(const Ice::Current&)
{
	worker->KinovaArm_openGripper();
}

void KinovaArmI::setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo, const Ice::Current&)
{
	worker->KinovaArm_setCenterOfTool(pose, referencedTo);
}

