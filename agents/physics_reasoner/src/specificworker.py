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
        self.agent_id = 136
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)

        try:
            # signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            # signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
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
        plot = np.zeros((300, 800, 3), np.uint8)
        plot2 = np.zeros((300, 800, 3), np.uint8)
        print('SpecificWorker.compute...')
        
        cubes = self.g.get_nodes_by_type ("box")
        for cube in cubes:
            plot = np.zeros((300, 800, 3), np.uint8)
            rt   = self.g.get_edge ("world", cube.name, "RT")
            v_rt = self.g.get_edge ("world", cube.name, "virtual_RT")

            if rt is None or v_rt is None:
                continue

            trans_diffs = np.absolute(rt.attrs["rt_translation"].value - v_rt.attrs["rt_translation"].value)
            rot_diffs   = np.absolute(rt.attrs["rt_rotation_euler_xyz"].value - v_rt.attrs["rt_rotation_euler_xyz"].value)

            trans_diff = np.linalg.norm (trans_diffs)
            rot_diff   = np.linalg.norm (rot_diffs)

            print ("----", cube.name, "----")
            print (rt.attrs["rt_translation"].value, v_rt.attrs["rt_translation"].value, "\n")
            print (trans_diffs, rot_diffs)

            self.plot_bars (plot, trans_diffs, 50, 10)
            cv2.imshow(cube.name + " trans diff", plot)

            # self.plot_bars (plot2, rot_diffs, 30, 5)
            # cv2.imshow(cube.name +  " rot diff", plot2)

    
        print ("\n \n \n")
        
        
        cv2.waitKey(1)
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)




    def plot_bars (self, img, values, max, steps):
        height = img.shape[0]
        width  = img.shape[1]
        cant = len(values)
        size = width//cant

        for i in range(max//steps):
            h = ((i*steps) * height) // max
            cv2.line(img, (0, height-h), (width, height-h), (255,255,255), 1)

        offset = 0
        for i in range(len(values)):
            x1 = offset + size//3
            x2 = offset + (2*size)//3 
            y = height - ((int (values[i]) * height) // max)

            # cv2.putText(img, (x1, 100), names[i], cv2.FONT_HERSHEY_SIMPLEX, 1, (150, 0, 0), 1, 2)
            cv2.rectangle(img, (x1, y),(x2, height), (255, 0, 255), -1)
            offset += size

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
