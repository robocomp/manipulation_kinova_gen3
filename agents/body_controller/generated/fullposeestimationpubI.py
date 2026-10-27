#
<<<<<<<< HEAD:components/camera_kinova/generated/camerargbdsimpleI.py
#    Copyright (C) 2026 by YOUR NAME HERE
========
#    Copyright (C) 2025 by YOUR NAME HERE
>>>>>>>> P3Bot:agents/body_controller/generated/fullposeestimationpubI.py
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


<<<<<<<< HEAD:components/camera_kinova/generated/camerargbdsimpleI.py
Ice.loadSlice("-I ./generated/ --all ./generated/CameraRGBDSimple.ice")
========
Ice.loadSlice("-I ./generated/ --all ./generated/FullPoseEstimationPub.ice")
>>>>>>>> P3Bot:agents/body_controller/generated/fullposeestimationpubI.py

from RoboCompFullPoseEstimationPub import *

<<<<<<<< HEAD:components/camera_kinova/generated/camerargbdsimpleI.py
class CameraRGBDSimpleI(CameraRGBDSimple):
========
class FullPoseEstimationPubI(FullPoseEstimationPub):
>>>>>>>> P3Bot:agents/body_controller/generated/fullposeestimationpubI.py
    def __init__(self, worker, id:str):
        self.worker = worker
        self.id = id


<<<<<<<< HEAD:components/camera_kinova/generated/camerargbdsimpleI.py
    def getAll(self, camera, ice):
        return getattr(self.worker, f"CameraRGBDSimple{self.id}_getAll")(camera)

    def getDepth(self, camera, ice):
        return getattr(self.worker, f"CameraRGBDSimple{self.id}_getDepth")(camera)

    def getImage(self, camera, ice):
        return getattr(self.worker, f"CameraRGBDSimple{self.id}_getImage")(camera)

    def getPoints(self, camera, ice):
        return getattr(self.worker, f"CameraRGBDSimple{self.id}_getPoints")(camera)
========
    def newFullPose(self, pose, ice):
        return getattr(self.worker, f"FullPoseEstimationPub{self.id}_newFullPose")(pose)
>>>>>>>> P3Bot:agents/body_controller/generated/fullposeestimationpubI.py
