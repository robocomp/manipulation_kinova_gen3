#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
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
from cv2 import norm
from rich.console import Console
from genericworker import *
import interfaces as ifaces

import numpy as np
import cv2
import time 

import apriltag

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 31
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)
        self.depth = None

        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)

        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        if self.color_raw is not None and self.depth_raw is not None:
            # print (type(self.depth), self.depth.shape)

            self.depth = np.frombuffer(self.depth_raw, dtype=np.uint16)
            self.depth = self.depth.reshape((270, 480))

            # depth[ depth > 2000] = 0
            # print (depth.dtype, depth[135, 240])

            # depth = cv2.convertScaleAbs (depth, depth, 255.0/np.max(depth))

            # print ("max", depth[135, 240], np.max(depth))

            self.color = np.frombuffer(self.color_raw, dtype=np.uint8)
            self.color = self.color.reshape((480, 640, 3))
            self.tags = self.compute_april_tags()
            print (self.tags[0])

            
            cv2.imshow('Color', cv2.cvtColor(self.depth.astype(np.uint8), cv2.COLOR_RGB2BGR)) #depth.astype(np.uint8))

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        # console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')
        pass

    def update_node(self, id: int, type: str):
        # console.print(f"UPDATE NODE: {id} {type}", style='green')

        if type=='rgbd':
            updated_node = self.g.get_node(id)
            self.depth_raw = updated_node.attrs['cam_depth'].value
            self.color_raw = updated_node.attrs['cam_rgb'].value

            self.focal_x = updated_node.attrs['cam_rgb_focalx'].value
            self.focal_y = updated_node.attrs['cam_rgb_focaly'].value

    def delete_node(self, id: int):
        # console.print(f"DELETE NODE:: {id} ", style='green')
        pass

    def update_edge(self, fr: int, to: int, type: str):
        # console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')
        pass

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        # console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')
        pass

    def delete_edge(self, fr: int, to: int, type: str):
        # console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
        pass

    def compute_april_tags(self):
        tags = self.detector.detect(cv2.cvtColor(self.color, cv2.COLOR_RGB2GRAY))
        if len(tags) > 0:
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(self.color, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
                    cv2.putText(self.color, str(tag.tag_id),
                               org = (tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                               fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255))
                    cv2.rectangle(self.color, tuple(tag.center.astype(int) - 1), tuple(tag.center.astype(int) + 1), (255, 0, 255))
        
        
        else:
            print("Compute_april_tags: No tags detected")
            pass
    
        tags = self.simplify_tags(tags)

        return tags

    def simplify_tags(self, tags):
        s_tags = []
        for tag in tags:
            # print (tag.center)
            # print (int(tag.center[1] / 1.77), int(tag.center[0] / 1.33))
            pos_z = self.depth[int(tag.center[1] / 1.91 + 10)][int((tag.center[0]) / 1.96 + 100)]
            pos_x = - (tag.center[1] * pos_z) / self.focal_x  
            pos_y = - (tag.center[0] * pos_z) / self.focal_y
            pos = [pos_x, pos_y, pos_z]

            ori = [0, 0, 0]

            s_tag = [tag.tag_id, pos, ori]
            s_tags.append(s_tag)

            cv2.drawMarker(self.depth, (int((tag.center[0]) / 1.96 + 100), int(tag.center[1] / 1.91 + 10)), (0, 255, 0), cv2.MARKER_CROSS, 150, 1)

            self.depth = cv2.rectangle (self.depth, (95, 15), (430, 260), (0, 255, 0))
        return s_tags
        
