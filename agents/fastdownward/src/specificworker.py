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
from planifier import *
import time
from pynput import keyboard


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
        self.Period = 2000

        self.agent_id = 183
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)


        self.planner = Planifier()
        self.init_state = []
        self.end_state = []
        
        self.step_finished = False

        listener = keyboard.Listener(
            on_press=None,
            on_release=self.on_release)
        listener.start()

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


        time.sleep(0.5)

        cube_nodes = self.g.get_nodes_by_type("box")
        cube_names = []
        for c in cube_nodes:
            cube_names.append(c.name[-1])
        print (cube_names)
        self.init_state, self.cubes = self.planner.create_initial_state_cubes(cube_names)
        self.end_state = self.planner.create_final_state([])
        time.sleep(0.5)

        self.planner.save_to_file(self.init_state, self.end_state, self.cubes)
        self.planner.exec_planner()
        time.sleep(0.5)
        
        self.plan = self.planner.load_plan()

        print (self.plan)

    def __del__(self):
        """Destructor"""

    def on_release(self, key):
        
        try:

            if key.char == 's':
                print ("A step was made")
                self.step_finished = True



        except:
            print ("Not a valid character")

    

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True

    def impact_effects (self, step):
        action = step[0]
        params = step[1]

        if action == 'stack':
            upper = self.g.get_node ("cube_" + str(params[0]))
            lower = self.g.get_node ("cube_" + str(params[1]))
            on_e = Edge (lower.id, upper.id, "on", self.agent_id)
            self.g.insert_or_assign_edge (on_e)


    @QtCore.Slot()
    def compute(self):
        print('My plan is to ')
        print (self.plan)

        print ("Now, execute", self.plan[0])
        try:
            while not self.step_finished:
                pass
        except KeyboardInterrupt:
            quit()
        
        print ("Well done")
        self.step_finished = False

        self.impact_effects(self.plan[0])

        self.plan = self.plan[1:]

        # computeCODE
        # try:
        #   self.differentialrobot_proxy.setSpeedBase(100, 0)
        # except Ice.Exception as e:
        #   traceback.print_exc()
        #   print(e)

        # The API of python-innermodel is not exactly the same as the C++ version
        # self.innermodel.updateTransformValues('head_rot_tilt_pose', 0, 0, 0, 1.3, 0, 0)
        # z = librobocomp_qmat.QVec(3,0)
        # r = self.innermodel.transform('rgbd', z, 'laser')
        # r.printvector('d')
        # print(r[0], r[1], r[2])

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
