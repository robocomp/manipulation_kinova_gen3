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

import kinovaControl

import multiprocessing as mp
import time
import vid_streamv32 as vs
import cv2
import numpy as np

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

        self.agent_id = 30
        self.g = DSRGraph(0, "gen3 DSR", self.agent_id)
        self.rt = rt_api(self.g)

        self.arm = kinovaControl.KinovaGen3()

        #Current Cam
        self.camProcess = None
        self.cam_queue = None
        self.stopbit = None
        self.camlink = 'rtsp://192.168.1.10/depth' # Add your RTSP cam link
        self.framerate = 15

        #Current Cam
        self.colorCamProcess = None
        self.color_queue = None
        self.colorStopbit = None
        self.colorLink = 'rtsp://192.168.1.10/color' # Add your RTSP cam link
        self.colorFramerate = 15

        #set  queue size
        self.cam_queue = mp.Queue(maxsize=1)
        self.color_queue = mp.Queue(maxsize=1)

        #get all cams
        time.sleep(3)


        self.stopbit = mp.Event()
        self.camProcess = vs.StreamCapture( self.camlink,
                                            self.stopbit,
                                            self.cam_queue,
                                            self.framerate)

                            
        self.camProcess.start()
        
        self.colorStopbit = mp.Event()
        self.colorCamProcess = vs.StreamCapture(self.colorLink,
                                                self.colorStopbit,
                                                self.color_queue,
                                                self.colorFramerate)
        self.colorCamProcess.start()

        self.camera_node = self.g.get_node('hand_camera')
        
        if self.camera_node is not None:
            self.camera_node.attrs['cam_rgb'         ] = Attribute (np.zeros((480,640,3), np.uint8), self.agent_id)
            self.camera_node.attrs['cam_rgb_width'   ] = Attribute (640,        self.agent_id)
            self.camera_node.attrs['cam_rgb_height'  ] = Attribute (480,        self.agent_id)
            self.camera_node.attrs['cam_rgb_depth'   ] = Attribute (3,          self.agent_id)
            self.camera_node.attrs['cam_rgb_focalx'  ] = Attribute (653.68229,  self.agent_id)
            self.camera_node.attrs['cam_rgb_focaly'  ] = Attribute (651.855994, self.agent_id)
            self.camera_node.attrs['cam_rgb_cameraID'] = Attribute (0,          self.agent_id)

            self.camera_node.attrs['cam_depth'         ] = Attribute (np.zeros((270,480,3), np.uint8), self.agent_id)
            self.camera_node.attrs['cam_depth_width'   ] = Attribute (270,           self.agent_id)
            self.camera_node.attrs['cam_depth_height'  ] = Attribute (480,           self.agent_id)
            self.camera_node.attrs['cam_depth_depth'   ] = Attribute (2,             self.agent_id)
            self.camera_node.attrs['cam_depth_focalx'  ] = Attribute (360.01333,     self.agent_id)
            self.camera_node.attrs['cam_depth_focaly'  ] = Attribute (360.013366699, self.agent_id)
            self.camera_node.attrs['cam_depthFactor'   ] = Attribute (0.01,          self.agent_id)
            self.camera_node.attrs['cam_depth_cameraID'] = Attribute (1,             self.agent_id)
            


            self.g.update_node(self.camera_node)

        self.gripper = self.g.get_node('gripper')
        self.gripper.attrs['gripper_finger_distance'] = Attribute (float(0.0), self.agent_id)
        self.gripper.attrs['gripper_target_finger_distance'] = Attribute (float(0.0), self.agent_id)
        self.g.update_node(self.gripper)

        self.GRIPPER_ID = self.gripper.id


        try:
            signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
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
        if self.stopbit is not None:
            self.stopbit.set()
            while not self.cam_queue.empty():
                try:
                    _ = self.cam_queue.get()
                except:
                    break
                self.cam_queue.close()

            self.camProcess.join()

        if self.colorStopbit is not None:
            self.colorStopbit.set()
            while not self.color_queue.empty():
                try:
                    _ = self.color_queue.get()
                except:
                    break
                self.color_queue.close()

            self.colorCamProcess.join()

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    def update_gripper_state(self):

        self.gripper = self.g.get_node('gripper')
        gr_state = self.arm.get_gripper_state()            
        self.gripper.attrs['gripper_finger_distance'].value = 1 - gr_state
        self.g.update_node (self.gripper)
        

    def update_arm_pose(self):

        new_pos = None
        arm_pose = self.arm.get_pose()

        world   = self.g.get_node('arm')

        if self.gripper is not None and world is not None:
            arm_rot = np.radians(arm_pose[3:])
            arm_rot[0],arm_rot[1] = arm_rot[1],arm_rot[0] 
            new_pos = [self.gripper.id, np.multiply(arm_pose[:3], 1000), arm_rot]
            self.rt.insert_or_assign_edge_RT(world, *new_pos)
            self.g.update_node(world)

    def move_arm_to (self, target_position):
        
        target_xyz = np.multiply(target_position[:3], 0.001)
        target_rpy = np.degrees (target_position[3:])

        target_rpy[0],target_rpy[1] = target_rpy[1],target_rpy[0] 

        print (target_xyz, target_rpy)

        self.arm.cartesian_move_to (*target_xyz, *target_rpy)

    def move_gripper_to (self, dest_distance):
        inv_dest = 1 - dest_distance
        self.arm.move_gripper_speed_dest(inv_dest)


    @QtCore.Slot()
    def compute(self):
        # print('SpecificWorker.compute...')

        self.update_arm_pose()
        # self.update_gripper_state()
        
        depth, color = self.get_rgbd_stream()

        # Suboptimal, should treat them independently
        if color is not None and depth is not None: 
            
            depth = depth.tobytes()
            depth = np.frombuffer(depth, dtype=np.uint8)
            depth = depth.reshape((270, 480, 2)) 

            self.camera_node = self.g.get_node('hand_camera')

            if self.camera_node is not None:
                self.camera_node.attrs['cam_rgb'].value = color
                self.camera_node.attrs['cam_rgb_height'].value = color.shape[0]
                self.camera_node.attrs['cam_rgb_width'].value  = color.shape[1]
                self.camera_node.attrs['cam_rgb_depth'].value  = color.shape[2]

                self.camera_node.attrs['cam_depth'].value = depth
                self.camera_node.attrs['cam_depth_height'].value = depth.shape[0]
                self.camera_node.attrs['cam_depth_width'].value  = depth.shape[1]
                # self.camera_node.attrs['cam_ed_depth'].value = color.shape[2]

                self.g.update_node(self.camera_node)

                # camera_node.attrs['cam_rgb'].value = color.data
                # camera_node.attrs['cam_rgb_width'].value = color.cols
                # camera_node.attrs['cam_rgb_height'].value = color.rows

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def get_rgbd_stream(self):
        depth = None 
        color_color = None
        if not self.cam_queue.empty():
            # print('Got frame')
            cmd, val = self.cam_queue.get()

            # if cmd == vs.StreamCommands.RESOLUTION:
            #     pass #print(val)

            if cmd == vs.StreamCommands.FRAME:
                if val is not None:
                    
                    # print ("depth: ", val[240][127])

                    # val = val.astype(np.uint8)

                    depth = val # cv2.cvtColor(val, cv2.COLOR_GRAY2RGB)

                    # cv2.imshow('depth', color)
                    # qt_color = QImage(color, val.shape[1], val.shape[0], QImage.Format_RGB888)
                    # pix_color = QPixmap.fromImage(qt_color).scaled(self.ui.depth.width(), self.ui.depth.height())
                    # self.ui.depth.setPixmap(pix_color)

                    # print (val.shape, val)


        if not self.color_queue.empty():
            # print('Got frame')
            color_cmd, color_val = self.color_queue.get()

            # if cmd == vs.StreamCommands.RESOLUTION:
            #     pass #print(val)

            if color_cmd == vs.StreamCommands.FRAME:
                if color_val is not None:
                    
                    # print ("img: ", val.shape[1], val.shape[0], " UI: ", self.ui.color.width(), self.ui.color.height() )

                    color_color = cv2.cvtColor(color_val, cv2.COLOR_BGR2RGB)
                    # color_qt_color = QImage(color_color, color_val.shape[1], color_val.shape[0], QImage.Format_RGB888)
                    # color_pix_color = QPixmap.fromImage(color_qt_color).scaled(self.ui.color.width(), self.ui.color.height())
                    # self.ui.color.setPixmap(color_pix_color)
                    # cv2.imshow('color', color_color)

                    # print (val.shape, val)

        return depth, color_color



    # =============== DSR SLOTS  ================
    # =============================================

    def update_node_att(self, id: int, attribute_names: [str]):
        # console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

        if id == self.GRIPPER_ID and 'target' in attribute_names:
            updated_node = self.g.get_node(id)
            target_position  = updated_node.attrs['target'].value
            print ("Received target position", target_position)
            self.move_arm_to (target_position)

        if id == self.GRIPPER_ID and 'gripper_target_finger_distance' in attribute_names:
            updated_node = self.g.get_node(id)
            target_distance  = updated_node.attrs['gripper_target_finger_distance'].value
            print ("Received target finger distance", target_distance)
            self.move_gripper_to (target_distance)

    def update_node(self, id: int, type: str):
        # console.print(f"UPDATE NODE: {id} {type}", style='green')
        pass

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
