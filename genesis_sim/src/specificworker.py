#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by YOUR NAME HERE
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
import time
from typing import Tuple
import cv2
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
import torch
import numpy as np
from shapely.geometry import MultiPoint, Point, LineString, Polygon

from genericworker import *
import interfaces as ifaces

from gs_sim import GS_SIM
from cube import Cube
from parser import ChatWindow
from model import Model
from gen3 import Gen3

console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 10
        if startup_check:
            self.startup_check()
        else:
            print("Compiling scene...")
            self.sim = GS_SIM()

            # list of model cubes
            self.gen3 = Gen3(self.sim.gen3)
            self.model = Model()
            self.model.add_robot(self.gen3)

            # arm affordance
            self.first_time = True  # for aff_circular_trajectory
            self.traj = None
            self.current_affordance = self.gen3.aff_exploring_from_above
            self.current_params = None

            self.target_cube = None
            self.default_tip_pose = self.sim.gen3.get_link(self.sim.tip_name).get_pos()
            self.number_of_real_cubes = 3

            # chat
            self.ui.frame_chat.resize(800, 400)
            self.chat = ChatWindow(self.ui.frame_chat, self.model)
            self.chat.parser.cmd_select_signal.connect(self.select_affordance)

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        # get image from camera
        seg = self.get_image_from_camera()

        # update model
        self.model = self.model.update_model(self.sim.scene, seg, self.model)

        # execute one step of the affordance
        self.execute_affordance(self.current_affordance, self.current_params)

        # update sim and camera in hand
        self.sim.step()

    ###################################################################################################
    def get_image_from_camera(self):
        rgb, _, seg, _ = self.sim.camera.render(rgb=True, segmentation=True)
        rgb = np.ascontiguousarray(rgb)
        height, width, channel = rgb.shape
        bytes_per_line = 3 * width
        # convert to QImage
        q_image = QImage(rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        self.ui.label.setPixmap(QPixmap.fromImage(q_image))
        return seg

    def select_affordance(self, affordance, params=None):
        print("Affordance selected: ", affordance)
        self.current_affordance = affordance
        self.current_params = params
        self.first_time = True

    def execute_affordance(self, affordance, params):

        if self.first_time:
           self.traj  = affordance(params)
           self.first_time = False

        # call the generator
        try:
            x, y = next(self.traj)
            tip_target = torch.tensor([x, y, self.default_tip_pose[2]])
            qpos = self.sim.gen3.inverse_kinematics(link=self.sim.gen3.get_link(self.sim.tip_name),
                                                    pos=tip_target,
                                                    quat=self.sim.rotate_quaternion_wrt_z([1, 0, 0, 0], np.pi / 2))
            self.sim.gen3.control_dofs_position(qpos)
        except StopIteration:
            time.sleep(0.5)
            self.first_time = True

    ####################################################333
    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)





