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
from attr import Attribute
from cv2 import norm
from rich.console import Console
from genericworker import *
import interfaces as ifaces

import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

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

        print ("Insertede dummie edge")

        self.CUBE_PREFIX = 1000

        self.hand_tag_detection_count = {}
        self.hand_tags = {}

        self.top_tag_detection_count = {}
        self.top_tags = {}

        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

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

        elif cube_id not in self.hand_tags.keys():
            return
        else:
        # cube_node = self.g.get_node ("cube_" + str(cube_id))
            dest_pose = self.g.get_edge ("world", "cube_" + str(cube_id), "RT")
            dest_pose = np.concatenate((dest_pose.attrs["rt_translation"].value, dest_pose.attrs["rt_rotation_euler_xyz"].value))
        print ("Grabbin in ", dest_pose)
        gripper = self.g.get_node ("gripper")

        dest_pose[3] = 0.0
        dest_pose[4] = np.pi

        # -45 para minizar giro, + 90 para cuadrar con el gripper
        dest_pose[5] = np.degrees(dest_pose[5]) % 90
        if dest_pose[5] < 45:
            dest_pose[5] += 90

        dest_pose[5] -= 45
        dest_pose[5] = (90 - dest_pose[5]) % 90 + 45
        dest_pose[5] = np.radians(dest_pose[5])

        print (dest_pose)
        gripper.attrs["gripper_target_finger_distance"].value = 1.0
        gripper.attrs["target"].value = dest_pose
        self.g.update_node (gripper)

    def insert_or_update_cube (self, tags, cube_id, is_top = "", offset = 0):
        cube = tags[cube_id]
        cube_name = "cube_" + str(cube_id) + is_top
        tf = inner_api(self.g)
        camera_id = "top_view_camera" if is_top else "hand_camera"
        new_pos = tf.transform_axis ("world", cube["pos"] + cube["rot"], camera_id)

        if (cube_node := self.g.get_node(cube_name)) is None:
            new_node = Node(cube_id + self.CUBE_PREFIX + offset, "box", cube_name)
            new_node.attrs['pos_x'] = Attribute(float(-280 + 90 * cube_id), self.agent_id)
            new_node.attrs['pos_y'] = Attribute(float(90), self.agent_id)
            self.g.insert_node (new_node)

            cube_node = new_node
        
        # print ("Inserted cube " + str(cube_id))

        # if is_top:
        #     print ("top ", new_pos[3:5])
        # else:
        #     print ("hand", new_pos[3:5])

        rt = rt_api(self.g)
        try:
            current_pos = tf.transform_axis ("world", cube_name)
            pos_diff = np.linalg.norm (new_pos[:3]-current_pos[:3])
            rot_diff = np.linalg.norm (new_pos[3:]-current_pos[3:])
            if pos_diff < 5 and rot_diff < 0.1:
                # print ("Not updating pose", pos_diff, rot_diff)
                return
        except:
            print ("Does not exist")

        # print ("update")
        world = self.g.get_node ("world")
        rt.insert_or_assign_edge_RT(world, cube_node.id, new_pos[:3], new_pos[3:])
        self.g.update_node(world)

    def delete_cube_rt (self, cube_id, is_top=""):
        if (cube := self.g.get_node ("cube_" + str(cube_id) + is_top)):
            world = self.g.get_node ("world")
            self.g.delete_edge (world.id, cube.id, "RT")

    @QtCore.Slot()
    def compute(self):

        if self.hand_color_raw is not None and self.hand_depth_raw is not None:
            # print (type(self.depth), self.depth.shape)

            self.hand_depth = np.frombuffer(self.hand_depth_raw, dtype=np.uint16)
            self.hand_depth = self.hand_depth.reshape((480, 640))

            # depth[ depth > 2000] = 0
            # print (depth.dtype, depth[135, 240])

            # depth = cv2.convertScaleAbs (depth, depth, 255.0/np.max(depth))

            # print ("max", depth[135, 240], np.max(depth))

            self.hand_color = np.frombuffer(self.hand_color_raw, dtype=np.uint8)
            self.hand_color = self.hand_color.reshape((480, 640, 3))
            self.hand_color = cv2.resize(self.hand_color, (1280, 960), interpolation=cv2.INTER_NEAREST)

            complete_tags = self.compute_april_tags(self.hand_color, (self.hand_focal_x, self.hand_focal_y))
            simplified_inmediate_tags = self.simplify_tags(complete_tags, self.hand_color, (self.hand_focal_x, self.hand_focal_y), self.hand_depth)

            for id in simplified_inmediate_tags.keys():
                if id not in self.hand_tag_detection_count.keys():
                    self.hand_tag_detection_count[id] = 0

            for id in self.hand_tag_detection_count.keys():
                if id in simplified_inmediate_tags.keys():
                    self.hand_tag_detection_count[id] = np.minimum (30, self.hand_tag_detection_count[id]+1)
                else:
                    self.hand_tag_detection_count[id] = np.maximum (0, self.hand_tag_detection_count[id]-1)
                
                if (self.hand_tag_detection_count[id] > 20):
                    self.hand_tags[id] = simplified_inmediate_tags[id]
                    self.insert_or_update_cube (self.hand_tags, id)
                else:
                    self.delete_cube_rt (id)
        # cv2.imshow('Color', self.hand_color) #depth.astype(np.uint8))

        if self.top_color_raw is not None and self.top_depth_raw is not None:
            # print (type(self.depth), self.depth.shape)

            self.top_depth = np.frombuffer(self.top_depth_raw, dtype=np.uint16)
            self.top_depth = self.top_depth.reshape((480, 640))

            # depth[ depth > 2000] = 0
            # print (depth.dtype, depth[135, 240])

            # depth = cv2.convertScaleAbs (depth, depth, 255.0/np.max(depth))

            # print ("max", depth[135, 240], np.max(depth))

            self.top_color = np.frombuffer(self.top_color_raw, dtype=np.uint8)
            self.top_color = self.top_color.reshape((480, 640, 3))
            self.top_color = cv2.resize(self.top_color, (1280, 960), interpolation=cv2.INTER_NEAREST)

            top_complete_tags = self.compute_april_tags(self.top_color, (self.top_focal_x, self.top_focal_y))
            top_simplified_inmediate_tags = self.simplify_tags(top_complete_tags, self.top_color, (self.top_focal_x, self.top_focal_y), self.top_depth)

            for id in simplified_inmediate_tags.keys():
                if id not in self.top_tag_detection_count.keys():
                    self.top_tag_detection_count[id] = 0

            for id in self.top_tag_detection_count.keys():
                if id in top_simplified_inmediate_tags.keys():
                    self.top_tag_detection_count[id] = np.minimum (30, self.top_tag_detection_count[id]+1)
                else:
                    self.top_tag_detection_count[id] = np.maximum (0, self.top_tag_detection_count[id]-1)
                
                if (self.top_tag_detection_count[id] > 20):
                    self.top_tags[id] = top_simplified_inmediate_tags[id]
                    self.insert_or_update_cube (self.top_tags, id, "**", 40)
                else:
                    self.delete_cube_rt (id, "**")

            dept_show = cv2.applyColorMap(cv2.convertScaleAbs(self.top_depth, alpha=0.03), cv2.COLORMAP_JET)
            # dept_show = cv2.rectangle (dept_show, (450, 268), (118, 81), (255, 255, 255))  
            # dept_show = cv2.resize(dept_show, dsize=(1280, 720))          

        # self.display_cube_diff (self.top_tags, self.hand_tags)

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

        if type=='rgbd' and id == 62842907933016084:
            updated_node = self.g.get_node(id)
            self.hand_depth_raw = updated_node.attrs['cam_depth'].value
            self.hand_color_raw = updated_node.attrs['cam_rgb'].value

            self.hand_focal_x = updated_node.attrs['cam_rgb_focalx'].value
            self.hand_focal_y = updated_node.attrs['cam_rgb_focaly'].value

        elif type=='rgbd' and id == 63693811452215316:
            updated_node = self.g.get_node(id)
            self.top_depth_raw = updated_node.attrs['cam_depth'].value
            self.top_color_raw = updated_node.attrs['cam_rgb'].value

            self.top_focal_x = updated_node.attrs['cam_rgb_focalx'].value
            self.top_focal_y = updated_node.attrs['cam_rgb_focaly'].value

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

    def display_cube_diff (self, top_tags, hand_tags):
        tf = inner_api(self.g)
        try:
            hand_pos = tf.transform_axis ("world", "cube_3")
            top_pos  = tf.transform_axis ("world", "cube_3**")

            print ("diff=", hand_pos[:3] - top_pos[:3])
        except:
            print ("Couldn't")

    def compute_april_tags(self, img, focals):
        tags = self.detector.detect(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
        if len(tags) > 0:
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(self.hand_color, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
                    cv2.putText(self.hand_color, str(tag.tag_id),
                               org = (tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                               fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255))
                    cv2.rectangle(self.hand_color, tuple(tag.center.astype(int) - 1), tuple(tag.center.astype(int) + 1), (255, 0, 255))
        else:
            # print("Compute_april_tags: No tags detected")
            pass
    
        # tags = self.simplify_tags(tags)

        return tags

    def simplify_tags(self, tags, img, focals, depth):
        s_tags = {}
        for tag in tags:
            m = self.detector.detection_pose(tag,[focals[0], focals[1], 640, 480], 0.04)

            rot = m[0][:3,:3]
            r = R.from_matrix(rot)
            
            index_x = int(tag.center[1]) // 2
            index_y = int(tag.center[0]) // 2
            pos_z = np.mean(depth[index_x-10:index_x+10,index_y-10:index_y+10])


            pos_x = - ((tag.center[1]//2 - img.shape[0] // 4) * pos_z) / focals[0]  
            pos_y = - ((tag.center[0]//2 - img.shape[1] // 4) * pos_z) / focals[1]

            pos = [pos_y, pos_x, pos_z] # Acording to gripper reference frame
            # pos = [pos_x, pos_y, pos_z] # Acording to world reference frame

            offset = 90 if tag.tag_id == 1 else 20
            pos[2] += offset

            
            ori = np.multiply(r.as_euler('XYZ'), -1).tolist()
            ori[-1] *= -1

            # ori = np.multiply(r.as_euler('zyx'), -1).tolist()
            # print ("zyx: ", np.degrees(ori))

            s_tags[tag.tag_id] = {"pos": pos, "rot": ori}

        return s_tags
        
