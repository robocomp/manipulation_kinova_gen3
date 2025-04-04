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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include <stdint.h>

#if Qt5_FOUND
	#include <QtWidgets>
#else
	#include <QtGui>
#endif
#include <ui_mainUI.h>
#include <grafcetStep/GRAFCETStep.h>
#include <ConfigLoader/ConfigLoader.h>
#include <QStateMachine>
#include <QEvent>
#include <QString>
#include <functional>
#include <atomic>
#include <QtCore>
#include <variant>
#include <unordered_map>

#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>

#include <Contactile.h>
#include <KinovaArm.h>

#define BASIC_PERIOD 100

using TuplePrx = std::tuple<RoboCompContactile::ContactilePrxPtr>;


class GenericWorker : public QMainWindow, public Ui_guiDlg
{
Q_OBJECT
public:
	GenericWorker(const ConfigLoader& configLoader, TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();

	void setPeriod(const std::string& state, int period);
	int getPeriod(const std::string& state);

	QStateMachine statemachine;
	QTimer hibernationChecker;
	std::atomic_bool hibernation = false;


	RoboCompContactile::ContactilePrxPtr contactile_proxy;

	virtual bool KinovaArm_closeGripper() = 0;
	virtual RoboCompKinovaArm::TPose KinovaArm_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo) = 0;
	virtual RoboCompKinovaArm::TGripper KinovaArm_getGripperState() = 0;
	virtual RoboCompKinovaArm::TJoints KinovaArm_getJointsState() = 0;
	virtual void KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles) = 0;
	virtual void KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds) = 0;
	virtual void KinovaArm_openGripper() = 0;
	virtual void KinovaArm_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo) = 0;

protected:
	std::unordered_map<std::string, std::unique_ptr<GRAFCETStep>> states;
	ConfigLoader configLoader;
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;

	//DSR params
	std::string agent_name;
	int agent_id;
	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;

	DSR::QScene2dViewer* widget_2d;

private:

public slots:
	virtual void initialize() = 0;
	virtual void compute() = 0;
	virtual void emergency() = 0;
	virtual void restore() = 0;
	void hibernationCheck();
	
signals:
	void kill();
	void goToEmergency();
	void goToRestore();
};

#endif
