#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from genericworker import *
import cv2
import apriltag
import numpy as np

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            # image
            self.image = []
            self.depth = []
            self.camera_name = "camera_arm"

            # apriltags
            self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        self.read_camera()
        return True

    # ===================================================================
    def read_camera(self):
        all = self.camerargbdsimple_proxy.getAll(self.camera_name)
        self.draw_image(all.image)

    def draw_image(self, color_):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        cv2.imshow("Gen3", color)
        cv2.waitKey(1)

    def compute_april_taga(self, color, depth):
        image = np.frombuffer(color.image, np.uint8).reshape(color.height, color.width, color.depth)
        tags = self.detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        if len(tags) > 0:
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(image, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
                    cv2.putText(image, str(tag.tag_id),
                               org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                               fontFace=cv.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255))
                    cv2.rectangle(image, tuple(tag.center.astype(int) - 1), tuple(tag.center.astype(int) + 1), (255, 0, 255))
        else:
            print("Compute_april_tags: No tags detected")

        return image, tags

    # def observar(self):
    #     pos = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
    #     pos.x = pos.x + 5
    #     pos.y = pos.y
    #     pos.z = 0
    #     self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)
    #
    # def posicionInicial(self):
    #     try:
    #         pose = RoboCompCoppeliaUtils.PoseType()
    #         pose.x = self.posicion_inicial.x
    #         pose.y = self.posicion_inicial.y
    #         pose.z = self.posicion_inicial.z
    #         pose.rx = self.posicion_inicial.rx
    #         pose.ry = self.posicion_inicial.ry
    #         pose.rz = self.posicion_inicial.rz
    #         #print(self.posicion_inicial)
    #         self.coppeliautils_proxy.addOrModifyDummy(RoboCompCoppeliaUtils.TargetTypes.Info, "target", pose)
    #     except:
    #         pass
    #
    # def mover(self, box):
    #     pos = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
    #     pos.x = pos.x - box[0] / 100
    #     pos.y = pos.y - box[1] / 100
    #     pos.z = 0
    #     self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)
    #
    # def pinza(self, centre,obj):
    #     tip = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
    #     pos = RoboCompKinovaArm.TPose()
    #     self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)

    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # self.camerargbdsimple_proxy.getAll(...)
    # self.camerargbdsimple_proxy.getDepth(...)
    # self.camerargbdsimple_proxy.getImage(...)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # self.kinovaarm_proxy.closeGripper(...)
    # self.kinovaarm_proxy.getCenterOfTool(...)
    # self.kinovaarm_proxy.openGripper(...)
    # self.kinovaarm_proxy.setCenterOfTool(...)
    # self.kinovaarm_proxy.setGripper(...)

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)