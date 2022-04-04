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
from rich.console import Console
from genericworker import *
import interfaces as ifaces

import numpy as np
import cv2
import mediapipe as mp
import time

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
        self.Period = 25

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 222
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)

        self.color = []
        self.has_image = False

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=False,
                                        max_num_hands=2,
                                        min_detection_confidence=0.5,
                                        min_tracking_confidence=0.5)
        self.mpDraw = mp.solutions.drawing_utils

        try:
            # signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            # signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            # signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            # signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            # signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)

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
        if not self.has_image:
            return True
        self.hand_color = np.frombuffer(self.color, dtype=np.uint8)
        img = self.hand_color.reshape((480, 640, 3))
        # imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img)
        #print(results.multi_hand_landmarks)
        triangle = []
        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                for id, lm in enumerate(handLms.landmark):
                    # print(id,lm)
                    h, w, c = img.shape
                    cx, cy = int(lm.x *w), int(lm.y*h)
                    if id in [0, 5, 17]:
                        triangle.append ((cx, cy))
                        cv2.circle(img, (cx,cy), 3, (255,0,255), cv2.FILLED)

                # self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)
        if triangle:
            pos = self.get_hand_position(img, triangle, [300, 300])
            self.insert_or_update_hand (pos)
            print (pos)
            
        cv2.imshow("Image", img)
        cv2.waitKey(1)

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def insert_or_update_hand (self, pos):
        tf = inner_api(self.g)
        new_pos = tf.transform_axis ("world", pos + [0,0,0], "hand_camera")

        if (hand_node := self.g.get_node("human_hand")) is None:
            hand_node = Node(44, "box", "human_hand")
            self.g.insert_node (hand_node)
            hand_node = new_node
        
        # print ("Inserted cube " + str(cube_id))

        # if is_top:
        #     print ("top ", new_pos[3:5])
        # else:
        #     print ("hand", new_pos[3:5])

        rt = rt_api(self.g)

        # print ("update")
        world = self.g.get_node ("world")
        rt.insert_or_assign_edge_RT(world, hand_node.id, new_pos[:3], new_pos[3:])
        self.g.update_node(world)

    def get_hand_position (self, img, points, focals):

        center_x = (points[0][0] + points[1][0] + points[2][0]) // 3
        center_y = (points[0][1] + points[1][1] + points[2][1]) // 3

        pos_z = 100 # np.mean(depth[index_x-10:index_x+10,index_y-10:index_y+10])
        pos_x = - ((center_y - img.shape[0]) * pos_z) / focals[0]  
        pos_y = - ((center_x - img.shape[1]) * pos_z) / focals[1]

        cv2.circle(img, (center_x,center_y), 3, (0,0,255), cv2.FILLED)

        return [pos_x, pos_y, pos_z]



    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        # console.print(f"UPDATE NODE: {id} {type}", style='green')
        if type=='rgbd' and id == 63693811452215316: # 62842907933016084:
            self.has_image = True
            updated_node = self.g.get_node(id)
            self.color = updated_node.attrs['cam_rgb'].value


    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):

        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
