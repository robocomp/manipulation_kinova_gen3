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
from Simulation import *

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 194
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)
        self.boxes_ids =  []
        self.already_added = []

        self.sim = Simulation()
        self.sim.load_scene ("/home/robocomp/robocomp/components/manipulation_kinova_gen3/etc/gen3_cubes.ttt")
        self.sim.start_simulation()

        try:
            # signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            # signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
            # signals.connect(self.g, signals.DELETE_NODE, self.delete_node)
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
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
        self.sim.stop_simulation()
        print ( "Stopped" )

    def setParams(self, params):
        return True


    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')
        self.update_simulated_arm ()

        for id in self.boxes_ids:
            cube = self.g.get_node (id)
            tf = inner_api(self.g)
            if cube:
                pos = tf.transform_axis ("world", cube.name)
                pos[2] -= 20
                if id not in self.already_added:
                    self.sim.insert_cube (cube.name, pos[:3], "gen3")
                    self.already_added.append(id)

                    print ("Created new cube", id, self.boxes_ids, self.already_added)
                else:
                    self.sim.set_object_pose(cube.name, pos, "gen3")

        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


    def update_simulated_arm (self):
        tf = inner_api(self.g)
        new_pos = tf.transform_axis ("world", "gripper")
        self.sim.set_arm_position (new_pos)


    # =============== DSR SLOTS  ================
    # ===========================================

    def update_node_att(self, id: int, attribute_names: [str]):
        console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        console.print(f"UPDATE NODE: {id} {type}", style='green')

    def delete_node(self, id: int):
        console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        dest = self.g.get_node(to)
        if dest.type == 'box' and dest.name[-1] != '*' and (dest.name not in self.boxes_ids):
            self.boxes_ids.append (dest.name)
        # console.print(f"UPDATE EDGE: {fr} to {to}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
