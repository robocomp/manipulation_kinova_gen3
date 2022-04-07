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
from scipy.spatial.transform import Rotation as R


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
        self.updated_cubes = []

        self.grasped_cube = None
        self.last_grasp   = None
        self.new_grasp = False
        self.grasp_released = False

        self.sim = Simulation()
        self.sim.load_scene ("/home/robocomp/robocomp/components/manipulation_kinova_gen3/etc/gen3_cubes.ttt")
        self.sim.start_simulation()

        self.sim.insert_hand ("human_hand", [0,0,0], "gen3")

        # self.sim.set_object_pose("goal", [400, 0, 400, np.pi, 0, np.pi/2], "gen3")

        # time.sleep (7)

        # self.sim.set_object_pose("goal", [400, 0, 400, np.pi/2, 0, np.pi/2], "gen3")

        try:
            # signals.connect(self.g, signals.UPDATE_NODE_ATTR, self.update_node_att)
            # signals.connect(self.g, signals.UPDATE_NODE, self.update_node)
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
        self.sim.stop_simulation()
        print ( "Stopped" )

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        # print('SpecificWorker.compute...')


        # goal = self.sim.get_object_pose("goal")
        # rot = R.from_quat(goal[1]).as_euler('xyz')
        # print (np.degrees(rot))

        # return True



        self.update_simulated_arm ()


        if self.updated_cubes:
            for id in self.updated_cubes:
                # print ("Updating cube", id)
                cube = self.g.get_node (id)
                tf = inner_api(self.g)
                if cube:
                    pos = tf.transform_axis ("world", cube.name)
                    
                    ### Trying to get all rts ######
                    # rt = rt_api(self.g)
                    # edge = self.g.get_edge ("world", cube.name, "RT")
                    # print ("a verte", rt.get_edge_RT_as_rtmat (edge))

                    int_rot = pos[3:]
                    ext_rot = R.from_euler('XYZ', int_rot).as_euler('xyz')
                    pos[3:] = ext_rot
                    if id not in self.already_added:
                        if cube.name == "cube_1":
                            self.sim.insert_box (cube.name, pos[:3], "gen3")
                        else:
                            self.sim.insert_cube (cube.name, pos[:3], "gen3")
                        self.already_added.append(id)
                        print ("Created new cube", id, self.boxes_ids, self.already_added)
                    else:
                        # pass
                        self.sim.set_object_pose(cube.name, pos, "gen3")
            # print ("Updating simulation")
            self.updated_cubes = []
        
        
        #### grasp detection w/distance ####
        if self.last_grasp != self.grasped_cube:
            if self.grasped_cube is None:
                self.cube_released (self.last_grasp)
            else:
                self.cube_grasped (self.grasped_cube)
            self.last_grasp = self.grasped_cube
        #####################################

        
        self.update_cubes_beliefs ()

        self.update_hand()
        
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


    def update_simulated_arm (self):
        tf = inner_api(self.g)
        new_pos = tf.transform_axis ("world", "gripper")

        int_rot = new_pos[3:]
        ext_rot = R.from_euler('XYZ', int_rot).as_euler('xyz')
        new_pos[3:] = ext_rot

        self.sim.set_object_pose("goal", new_pos, "gen3")
        # self.sim.set_arm_position (new_pos)

    def update_cubes_beliefs (self):

        for id in self.boxes_ids:
            cube = self.g.get_node(id)
            pos, rot = self.sim.get_object_pose(cube.name)

            rot = R.from_quat(rot).as_euler('xyz')

            world = self.g.get_node("world")

            v_rt = Edge (cube.id, world.id, "virtual_RT", self.agent_id)
            
            v_rt.attrs["rt_rotation_euler_xyz"] = Attribute(rot, self.agent_id)
            v_rt.attrs["rt_translation"]        = Attribute(pos, self.agent_id)
            self.g.insert_or_assign_edge (v_rt)

    def update_hand (self):
        tf = inner_api(self.g)
        try:
            pos = tf.transform_axis ("world", "human_hand")
            self.sim.set_object_pose ("human_hand", pos, "gen3")
        except:
            return

        colliding = self.sim.check_colisions("human_hand")
        # for c in colliding:
        hand = self.g.get_node ("human_hand")
        if colliding:
            cube = self.g.get_node (colliding)
            g_rt = Edge (cube.id, hand.id, "graspping", self.agent_id)
            self.g.insert_or_assign_edge (g_rt)
        elif self.last_grasp:
            cube = self.g.get_node (self.last_grasp)
            self.g.delete_edge (hand.id, cube.id, "graspping")

        self.grasped_cube = colliding


    def cube_grasped (self, name):
        print ("Changing to static", name)
        self.sim.change_static (name, 1)

    def cube_released (self, name):
        print ("Changing to dynamic", name)
        self.grasped_cube = None
        self.sim.change_static (name, 0)

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
        if dest.type == 'box' and type == "RT" and dest.name[-1] != '*':
            self.updated_cubes.append (dest.name)
            if (dest.name not in self.boxes_ids):
                self.boxes_ids.append (dest.name)

        # if dest.type == 'box' and type == "graspping" and dest.name[-1] != '*':
        #     self.grasped_cube = dest.name
            # if dest.name != self.grasped_cube:
            #     self.new_grasp = True
            #     self.grasped_cube = dest.name
        # console.print(f"UPDATE EDGE: {fr} to {to}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        pass
        # dest = self.g.get_node(to)
        # if dest.type == 'box' and type == "graspping" and dest.name[-1] != '*':
        #     self.grasped_cube = None
            # self.grasp_released = True
