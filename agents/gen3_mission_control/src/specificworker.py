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

from pynput import keyboard

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

        self.CUBE_PREFIX = 1000

        self.tf = inner_api(self.g)
        self.tag_detection_count = {}
        self.tags = {}
        self.cube_pos = {}

        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

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
        pass

    def on_press(self, key):
        pass
        # try:
        #     print('Alphanumeric key pressed: {0} '.format(
        #         key.char))
        # except AttributeError:
        #     print('special key pressed: {0}'.format(
        #         key))

    def on_release(self, key):
        print('Key released: {0}'.format(
            key))
        
        try:
            if key.char == 'c':
                self.close_gripper()
                return True
            cube_id = int (key.char)
            self.pick_cube (cube_id)
        except:
            print ("not an int")

        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    def close_gripper  (self):
        gripper = self.g.get_node ("gripper")
        gripper.attrs["gripper_target_finger_distance"].value = 0.0
        self.g.update_node (gripper)

    def pick_cube (self, cube_id):

        if cube_id == 0:
            dest_pose = [400, 0, 400, np.pi, 0, np.pi/2]

        elif cube_id not in self.tags.keys():
            return
        else:
        # cube_node = self.g.get_node ("cube_" + str(cube_id))
            dest_pose = self.g.get_edge ("world", "cube_" + str(cube_id), "RT")
            dest_pose = np.concatenate((dest_pose.attrs["rt_translation"].value, dest_pose.attrs["rt_rotation_euler_xyz"].value))
        print ("Grabbin in ", dest_pose)
        gripper = self.g.get_node ("gripper")

        dest_pose[3] = 0.0
        dest_pose[4] = np.pi

        print (dest_pose)
        gripper.attrs["gripper_target_finger_distance"].value = 1.0
        gripper.attrs["target"].value = dest_pose
        self.g.update_node (gripper)

    def insert_or_update_cube (self, cube_id):
        cube = self.tags[cube_id]
       
        
        new_pos = self.tf.transform_axis ("world", cube["pos"] + cube["rot"], "hand_camera")

        if (cube_node := self.g.get_node("cube_" + str(cube_id))) is None:
            new_node = Node(cube_id + self.CUBE_PREFIX, "box", "cube_" + str(cube_id))
            self.g.insert_node (new_node)

            cube_node = new_node
        
        # print ("Inserted cube " + str(cube_id))

        world = self.g.get_node ("world")
        self.rt = rt_api(self.g)
        self.rt.insert_or_assign_edge_RT(world, cube_node.id, new_pos[:3], new_pos[3:])
        self.g.update_node(world)

    def delete_cube (self, cube_id):
        if (cube := self.g.get_node ("cube_" + str(cube_id))):
            self.g.delete_node (cube.id)

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
            self.inmediate_tags = self.compute_april_tags()

            for id in self.inmediate_tags.keys():
                if id not in self.tag_detection_count.keys():
                    self.tag_detection_count[id] = 0

            for id in self.tag_detection_count.keys():
                if id in self.inmediate_tags.keys():
                    self.tag_detection_count[id] = np.minimum (30, self.tag_detection_count[id]+1)
                else:
                    self.tag_detection_count[id] = np.maximum (0, self.tag_detection_count[id]-1)
                
                if (self.tag_detection_count[id] > 20):
                    self.tags[id] = self.inmediate_tags[id]
                    self.insert_or_update_cube (id)
                else:
                    self.delete_cube (id)

            
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
                print (tag)
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
        s_tags = {}
        for tag in tags:
            # print (tag.center)
            # print (int(tag.center[1] / 1.77), int(tag.center[0] / 1.33))
            index_x = int(tag.center[1] / 1.91 + 10 )
            index_y = int(tag.center[0] / 1.96 + 100)
            pos_z = self.depth[index_x][index_y]
            pos_x = - ((tag.center[1] - self.color.shape[0] // 2) * pos_z) / self.focal_x  
            pos_y = - ((tag.center[0] - self.color.shape[1] // 2) * pos_z) / self.focal_y

            pos = [pos_y, pos_x, pos_z] # Acording to gripper reference frame

            r_x = 0
            r_y = 0
            r_z = (np.arctan2(tag.corners[0][1] - tag.center[1], tag.corners[0][0] - tag.center[0]) + ( 3 * np.pi )/4) # % 360
            ori = [r_x, r_y, r_z]

            s_tags[tag.tag_id] = {"pos": pos, "rot": ori}

            cv2.drawMarker(self.depth, (int((tag.center[0]) / 1.96 + 100), int(tag.center[1] / 1.91 + 10)), (0, 255, 0), cv2.MARKER_CROSS, 150, 1)

            self.depth = cv2.rectangle (self.depth, (95, 15), (430, 260), (0, 255, 0))
        return s_tags
        
