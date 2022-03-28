#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by Daniel Peix del Río
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

import pyrealsense2 as rs
import numpy as np
import cv2

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100

        self.agent_id = 280322
        self.g = DSRGraph(0, "gen3_strips", self.agent_id)

        self.arm = kinovaControl.KinovaGen3()

        self.HAND_CAMERA_SN = '839112060624'

        # Hand camera configuration
        self.pipeline_hand = rs.pipeline()
        config_hand = rs.config()
        config_hand.enable_device(self.HAND_CAMERA_SN)
        config_hand.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config_hand.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline_hand.start(config_hand)

        self.camera_node = self.g.get_node('hand_camera')
        
        if self.camera_node is not None:
            self.camera_node.attrs['cam_rgb'         ] = Attribute (np.zeros((480,640,3), np.uint8), self.agent_id)
            self.camera_node.attrs['cam_rgb_width'   ] = Attribute (640,        self.agent_id)
            self.camera_node.attrs['cam_rgb_height'  ] = Attribute (480,        self.agent_id)
            self.camera_node.attrs['cam_rgb_depth'   ] = Attribute (3,          self.agent_id)
            self.camera_node.attrs['cam_rgb_focalx'  ] = Attribute (653.68229,  self.agent_id)
            self.camera_node.attrs['cam_rgb_focaly'  ] = Attribute (651.855994, self.agent_id)
            self.camera_node.attrs['cam_rgb_cameraID'] = Attribute (0,          self.agent_id)

            self.camera_node.attrs['cam_depth'         ] = Attribute (np.zeros((480,640,2), np.uint8), self.agent_id)
            self.camera_node.attrs['cam_depth_width'   ] = Attribute (480,           self.agent_id)
            self.camera_node.attrs['cam_depth_height'  ] = Attribute (640,           self.agent_id)
            self.camera_node.attrs['cam_depth_depth'   ] = Attribute (2,             self.agent_id)
            self.camera_node.attrs['cam_depth_focalx'  ] = Attribute (360.01333,     self.agent_id)
            self.camera_node.attrs['cam_depth_focaly'  ] = Attribute (360.013366699, self.agent_id)
            self.camera_node.attrs['cam_depthFactor'   ] = Attribute (0.01,          self.agent_id)
            self.camera_node.attrs['cam_depth_cameraID'] = Attribute (1,             self.agent_id)
            


            self.g.update_node(self.camera_node)

        align_to = rs.stream.color
        self.align = rs.align(align_to)

        self.gripper = self.g.get_node('gripper')
        self.gripper.attrs['gripper_finger_distance'] = Attribute (float(0.0), self.agent_id)
        self.gripper.attrs['gripper_target_finger_distance'] = Attribute (float(0.0), self.agent_id)
        self.g.update_node(self.gripper)

        self.GRIPPER_ID = self.gripper.id

        self.moving = False

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
        print('SpecificWorker.compute...')
        frames = self.pipeline_hand.wait_for_frames()
        aligned_frames = self.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            print ('No hand frame')
            return True
        

         # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # images = np.hstack((color_image, depth_colormap))
        # images = np.hstack((color_image_h, depth_colormap_h))

        # Show images
        # cv2.namedWindow('RealSense hand', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense hand', images)
        # cv2.waitKey(1)

        # Suboptimal, should treat them independently
        if color_image is not None and depth_image is not None: 
            
            depth = depth_image.tobytes()
            depth = np.frombuffer(depth, dtype=np.uint8)
            depth = depth.reshape((480, 640, 2)) 

            self.camera_node = self.g.get_node('hand_camera')

            if self.camera_node is not None:
                self.camera_node.attrs['cam_rgb'].value = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                self.camera_node.attrs['cam_rgb_height'].value = color_image.shape[0]
                self.camera_node.attrs['cam_rgb_width'].value  = color_image.shape[1]
                self.camera_node.attrs['cam_rgb_depth'].value  = color_image.shape[2]
                self.camera_node.attrs['cam_rgb_focalx'].value = float(color_intrin.fx)
                self.camera_node.attrs['cam_rgb_focaly'].value = float(color_intrin.fy)

                self.camera_node.attrs['cam_depth'].value = depth
                self.camera_node.attrs['cam_depth_height'].value = depth.shape[0]
                self.camera_node.attrs['cam_depth_width'].value  = depth.shape[1]
                # self.camera_node.attrs['cam_ed_depth'].value = color.shape[2]
                self.camera_node.attrs['cam_depth_focalx'].value = float(depth_intrin.fx)
                self.camera_node.attrs['cam_depth_focaly'].value = float(depth_intrin.fy)

                self.g.update_node(self.camera_node)
        

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)






    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):

        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
