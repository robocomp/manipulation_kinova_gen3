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

from csv import excel_tab
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
        self.Period = 100

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 222
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)

        self.grasped_cube = None

        self.color_raw = None
        self.hand_pos = [0,0,0]
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=False,
                                        max_num_hands=1,
                                        min_detection_confidence=0.5,
                                        min_tracking_confidence=0.5)
        self.mpDraw = mp.solutions.drawing_utils

        try:
            # signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            # signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            # signals.connect(self.g, signals.UPDATE_EDGE_ATTR, self.update_edge_att)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
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
        if self.color_raw is None:
            print ("No Image received")
            return True

        self.color = np.frombuffer(self.color_raw, dtype=np.uint8)
        img = self.color.reshape((480, 640, 3))

        self.depth = np.frombuffer(self.depth_raw, dtype=np.uint16)
        depth = self.depth.reshape((480, 640))

        results = self.hands.process(img)

        # #### Center position of the hand #######
        # triangle = []
        # if results.multi_hand_landmarks:
        #     for handLms in results.multi_hand_landmarks:
        #         for id, lm in enumerate(handLms.landmark):
        #             h, w, c = img.shape
        #             cx, cy = int(lm.x *w), int(lm.y*h)
        #             if id in [0, 5, 17]:
        #                 triangle.append ((cx, cy))
        #                 cv2.circle(img, (cx,cy), 3, (255,0,255), cv2.FILLED)
        # if triangle:
        #     pos = self.get_hand_position(img, depth, triangle, [self.focal_x, self.focal_y])
        #     if pos is not None:
        #         self.pos = self.insert_or_update_hand (pos)
        #         # self.check_grasp()
        #     else:
        #         print ("no hand")
        # ##########################################

        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                for id, lm in enumerate(handLms.landmark):
                    h, w, c = img.shape
                    cx, cy = int(lm.x *w), int(lm.y*h)
                    if id == 5:
                        pos = self.get_hand_position(img, depth, (cx, cy), [self.focal_x, self.focal_y])
                        if pos is not None:
                            self.pos = self.insert_or_update_hand (pos)
                        cv2.circle(img, (cx,cy), 3, (255,0,255), cv2.FILLED)

        
        cv2.imshow("Image", img)
        cv2.waitKey(1)


        return True

    def check_grasp (self):

        cubes = self.g.get_nodes_by_type("box")
        hand = self.g.get_node ("human_hand")

        for cube in cubes:    
            tf = inner_api(self.g)
            try:
                cube_pos = tf.transform_axis ("world", cube.name)[:3]
            except:
                self.g.delete_edge (hand.id, cube.id, "graspping")
                continue

            dist = np.linalg.norm(self.pos[:3] - cube_pos)
            print (dist)

            if dist < 100:
                g_rt = Edge (cube.id, hand.id, "graspping", self.agent_id)
                self.g.insert_or_assign_edge (g_rt)
                self.grasped_cube = cube.id
                # print ("graspping cube", cube.name)
            else:
                # print ("not grasping cube", cube.name)
                self.g.delete_edge (hand.id, cube.id, "graspping")

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def insert_or_update_hand (self, pos):
        tf = inner_api(self.g)
        new_pos = tf.transform_axis ("world", pos + [0,0,0], "hand_camera")

        if (hand_node := self.g.get_node("human_hand")) is None:
            hand_node = Node(44, "left_hand", "human_hand")
            hand_node.attrs['pos_x'] = Attribute(float(67), self.agent_id)
            hand_node.attrs['pos_y'] = Attribute(float(160), self.agent_id)
            self.g.insert_node (hand_node)
        
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

        return new_pos

    def get_hand_position (self, img, depth, points, focals):

        center_x = points[0] # (points[0][0] + points[1][0] + points[2][0]) // 3
        center_y = points[1] # (points[0][1] + points[1][1] + points[2][1]) // 3

        try:
            pos_z = depth[center_y, center_x] # np.mean(depth[index_x-10:index_x+10,index_y-10:index_y+10])
            pos_y = - ((center_y - img.shape[0]//2) * pos_z) / focals[0]  
            pos_x = - ((center_x - img.shape[1]//2) * pos_z) / focals[1]
            cv2.circle(img, (center_x,center_y), 3, (0,0,255), cv2.FILLED)
        except:
            print ("hand center out of frame")
            return None

        return [pos_x, pos_y, pos_z]



    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        # console.print(f"UPDATE NODE: {id} {type}", style='green')
        if type=='rgbd' and id == 62842907933016084:
            self.has_image = True
            
            updated_node = self.g.get_node(id)
            self.depth_raw = updated_node.attrs['cam_depth'].value
            self.color_raw = updated_node.attrs['cam_rgb'].value

            self.focal_x = updated_node.attrs['cam_rgb_focalx'].value
            self.focal_y = updated_node.attrs['cam_rgb_focaly'].value


    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        pass

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        pass
        
