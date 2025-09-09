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
#ifndef KINOVAARM_H
#define KINOVAARM_H

// Ice includes
#include <Ice/Ice.h>
#include <KinovaArm.h>

#include "genericworker.h"


class KinovaArmI : public virtual RoboCompKinovaArm::KinovaArm
{
public:
	KinovaArmI(GenericWorker *_worker);
	~KinovaArmI();

	bool closeGripper(const Ice::Current&);
	RoboCompKinovaArm::TPose getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo, const Ice::Current&);
	RoboCompKinovaArm::TGripper getGripperState(const Ice::Current&);
	RoboCompKinovaArm::TJoints getJointsState(const Ice::Current&);
	void moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles, const Ice::Current&);
	void moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds, const Ice::Current&);
	void openGripper(const Ice::Current&);
	void setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
