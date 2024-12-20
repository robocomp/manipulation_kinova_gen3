#
#    Copyright (C) 2024 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, Ice

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'
if len(ROBOCOMP)<1:
    raise RuntimeError('ROBOCOMP environment variable not set! Exiting.')


Ice.loadSlice("-I ./src/ --all ./src/KinovaArm.ice")

from RoboCompKinovaArm import *

class KinovaArmI(KinovaArm):
    def __init__(self, worker):
        self.worker = worker


    def closeGripper(self, c):
        return self.worker.KinovaArm_closeGripper()

    def getCenterOfTool(self, referencedTo, c):
        return self.worker.KinovaArm_getCenterOfTool(referencedTo)

    def getGripperState(self, c):
        return self.worker.KinovaArm_getGripperState()

    def getJointsState(self, c):
        return self.worker.KinovaArm_getJointsState()

    def moveJointsWithAngle(self, angles, c):
        return self.worker.KinovaArm_moveJointsWithAngle(angles)

    def moveJointsWithSpeed(self, speeds, c):
        return self.worker.KinovaArm_moveJointsWithSpeed(speeds)

    def openGripper(self, c):
        return self.worker.KinovaArm_openGripper()

    def setCenterOfTool(self, pose, referencedTo, c):
        return self.worker.KinovaArm_setCenterOfTool(pose, referencedTo)
