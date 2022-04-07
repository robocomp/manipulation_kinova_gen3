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

from concurrent.futures import thread
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from attr import Attribute
from rich.console import Console
from genericworker import *
import interfaces as ifaces

import pyrealsense2 as rs
import kinovaControl
import numpy as np
import cv2
from pynput import keyboard
from scipy.spatial.transform import Rotation as R

from image_processor import *
from planifier import *
from constants import *
import threading, queue
import time

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)

from pydsr import *


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        # CONSTANTS
        self.Period = 100
        self.CUBE_PREFIX = 1000
        self.agent_id = 2803

        # EVENTS HANDLING [basic]
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        # EVENT HANDLING [pro]
        self.thread_event = None
        self.queue = queue.Queue()
        self.do_action = False
        self.do_next_action = False
        
        # IMPORTANT ATTRIBUTES
        self.img_proc = ImageProcessor()
        self.planner = Planifier()
        self.g = DSRGraph(0, "gen3_strips", self.agent_id)
        # self.observation_pose = [534.254, -0.419066, 68.6622, 3.13773, -0.802042, -1.58157]
        # self.working_pose = [544, 0, 400, 3.14159, 0, -1.57089]
        self.hand_tag_detection_count = {}
        self.hand_tags = {}

        # PLANNING
        self.init_state = []
        self.end_state = []
        self.ui.generar.clicked.connect(self.final_state)
        self.begin_plan = False
        self.end = True
        self.plan = []
        self.step = 0
        self.tags = []

        # TODO: GO TO OBSERVATION POSE
        # self.thread_event = threading.Thread(name='OBSERVATING', target=self.move_arm, args=[self.observation_pose])
        # self.thread_event.start()
        
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
        return True


    @QtCore.Slot()
    def compute(self):
        print('SpecificWorker.compute...')       
        if self.hand_color_raw is not None and self.hand_depth_raw is not None:
            color, depth = self.img_proc.extract_color_and_depth(self.hand_color_raw, self.hand_depth_raw)
            self.tags, simple_tags = self.img_proc.compute_april_tags(color, depth, (self.hand_focal_x, self.hand_focal_y))
            current_cubes = self.cubes_to_dsr(simple_tags)
            
            # self.init_state, cubes = self.planner.create_initial_state(tags)
            # print("STATE", self.init_state)
            # print("CUBES", cubes)

        if self.end:
            print("PLAN ENDED")
        else:
            if self.begin_plan:
                self.begin_plan = False
                self.init_state, cubes = self.planner.create_initial_state(self.tags)
                for rule in self.init_state:
                    print(rule)
                self.planner.save_to_file(self.init_state, self.end_state, cubes)

                time.sleep(1)

                self.planner.exec_planner()

                time.sleep(1)
                
                self.plan = self.planner.load_plan()
                print(self.plan)

                self.move_arm(WORKING_POSE)

                self.do_action = True

            else:
                if self.plan != []:
                    print(self.plan)
                    time.sleep(2)
                    '''current_step = self.plan[self.step]
                    params = current_step[1] if len(current_step) > 1 else None                                                             # --
                    arg_list = []                                                                                                           # --

                    if self.do_action:
                        if self.thread_event == None:
                            if current_step[0] == "pick_up":                                                                                # --
                                self.thread_event = threading.Thread(name='pick_up', target=self.pick_up, args=[self.observation_pose])     # --
                            elif current_step[0] == "put_down":                                                                             # --
                                self.thread_event = threading.Thread(name='put_down', target=self.put_down, args=[self.observation_pose])   # --
                            elif current_step[0] == "stack":                                                                                # --
                                self.thread_event = threading.Thread(name='stack', target=self.stack, args=[self.observation_pose])         # --
                            elif current_step[0] == "unstack":                                                                              # --
                                self.thread_event = threading.Thread(name='unstack', target=self.unstack, args=[self.observation_pose])     # --
                            self.thread_event.start()                                                                                       # --

                            # Josemi's way
                            # self.thread_event = threading.Thread(name=current_step[0], target=self.__dict__[current_step[0]], args=[params])
                            # self.thread_event.start()
                        else:
                            try:
                                state = self.queue.get_nowait()
                                print(state)
                                if "Finish" in state:
                                    self.do_action = False
                                    self.thread_event = None
                                    self.step += 1
                                    if self.step == len(self.plan):
                                        self.end = True
                                    else:
                                        self.do_next_action = True
                            except:
                                pass    '''                       
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    # ARM MOVEMENTS
    def move_arm(self, pose):
        gripper = self.g.get_node ("gripper")
        gripper.attrs["target"].value = pose
        self.g.update_node(gripper)

    def open_gripper(self):
        gripper = self.g.get_node ("gripper")
        gripper.attrs["gripper_target_finger_distance"].value = OPEN
        self.g.update_node (gripper)
    
    def close_gripper(self):
        gripper = self.g.get_node ("gripper")
        gripper.attrs["gripper_target_finger_distance"].value = CLOSE
        self.g.update_node (gripper)

    # EVENT FUNCTION TRIGGERS
    def on_press(self, key):
        pass

    def final_state(self):
        self.end_state = self.planner.create_final_state(self.ui)
        self.begin_plan = True
        self.end = False
        self.step = 0

    def on_release(self, key):
        print('Key released: {0}'.format(key))
        try:
            # if key.char == 'c':
            #     self.close_gripper()
            #     return True
            # if key.char == 'o':
            #     self.open_gripper()
            #     return True
            # if key.char == 's':
            #     self.put_down(int(key.char))
            #     return True
            # if key.char == 'w':
            #     self.move_arm(self.working_pose)
            #     return True
            # if key.char == 'r':
            #     self.move_arm(self.observation_pose)
            #     return True
            self.pick_up(int(key.char))
        except:
            print ("Not an INT")

        if key == keyboard.Key.esc:
            # Stop listener
            return False

    # ACTIONS
    def pick_up(self, cube_id):
        # TODO Reimplementar o reorganizar
        if cube_id == 0:
            dest_pose = [400, 0, 400, np.pi, 0, np.pi/2]

        elif cube_id not in self.hand_tags.keys():
            return
        else:
            dest_pose = self.g.get_edge ("world", "cube_" + str(cube_id), "RT")
            dest_pose = np.concatenate((dest_pose.attrs["rt_translation"].value, dest_pose.attrs["rt_rotation_euler_xyz"].value))
        print ("Grabbin in ", dest_pose)

        dest_pose[3] = 0.0
        dest_pose[4] = np.pi

        # -45 para minizar giro, + 90 para cuadrar con el gripper
        dest_pose[5] = np.degrees(dest_pose[5]) % 90
        if dest_pose[5] < 45:
            dest_pose[5] += 90

        dest_pose[5] -= 45
        dest_pose[5] = (90 - dest_pose[5]) % 90 + 45
        dest_pose[5] = np.radians(dest_pose[5])

        self.open_gripper()
        self.move_arm(dest_pose)
        self.close_gripper()
        self.move_arm(WORKING_POSE)

    def put_down(self, cube_id):
        pass

    def unstack(self, cube_id, cube_aux):
        self.pick_up(cube_id)
    
    def stack(self, cube_id, cube_aux):
        pass


    # INTERFACE CUBES-DSR
    def cubes_to_dsr(self, tags):
        cubes = []
        for id in tags.keys():
            if id not in self.hand_tag_detection_count.keys():
                self.hand_tag_detection_count[id] = 0

        for id in self.hand_tag_detection_count.keys():
            if id in tags.keys():
                self.hand_tag_detection_count[id] = np.minimum (30, self.hand_tag_detection_count[id]+1)
            else:
                self.hand_tag_detection_count[id] = np.maximum (0, self.hand_tag_detection_count[id]-1)
            
            if (self.hand_tag_detection_count[id] > 20):
                self.hand_tags[id] = tags[id]
                self.__insert_or_update_cube (tags, id)
                cubes.append(id)
            else:
                self.__delete_cube (id)
        return cubes
        

    def __insert_or_update_cube (self, tags, cube_id, offset = 0):
        cube = tags[cube_id]
        cube_name = "cube_" + str(cube_id)
        tf = inner_api(self.g)
        new_pos = tf.transform_axis ("world", cube["pos"] + cube["rot"], "hand_camera")
        
        if (cube_node := self.g.get_node(cube_name)) is None:
            new_node = Node(cube_id + self.CUBE_PREFIX + offset, "box", cube_name)
            new_node.attrs['pos_x'] = Attribute(float(REFERENCE_COORD + CUBE_OFFSET * cube_id), self.agent_id)
            new_node.attrs['pos_y'] = Attribute(float(CUBE_OFFSET), self.agent_id)
            
            self.g.insert_node (new_node)
            cube_node = new_node

        rt = rt_api(self.g)
        try:
            current_pos = tf.transform_axis ("world", cube_name)
            pos_diff = np.linalg.norm (new_pos[:3]-current_pos[:3])
            rot_diff = np.linalg.norm (new_pos[3:]-current_pos[3:])
            if pos_diff < MAX_POS_DIFF and rot_diff < MAX_ROT_DIFF:
                return
        except:
            print ("Does not exist")

        world = self.g.get_node ("world")
        rt.insert_or_assign_edge_RT(world, cube_node.id, new_pos[:3], new_pos[3:])
        self.g.update_node(world)

    def __delete_cube (self, cube_id):
        if (cube := self.g.get_node ("cube_" + str(cube_id))):
            self.g.delete_node (cube.id)

    # DSR SLOTS
    def update_node_att(self, id: int, attribute_names: [str]):
        pass
        # console.print(f"UPDATE NODE ATT: {id} {attribute_names}", style='green')

    def update_node(self, id: int, type: str):
        # console.print(f"UPDATE NODE: {id} {type}", style='green')
        if type=='rgbd' and id == CAMERA_ID:
            updated_node = self.g.get_node(id)
            self.hand_depth_raw = updated_node.attrs['cam_depth'].value
            self.hand_color_raw = updated_node.attrs['cam_rgb'].value

            self.hand_focal_x = updated_node.attrs['cam_rgb_focalx'].value
            self.hand_focal_y = updated_node.attrs['cam_rgb_focaly'].value

    def delete_node(self, id: int):
        pass
        # console.print(f"DELETE NODE:: {id} ", style='green')

    def update_edge(self, fr: int, to: int, type: str):
        pass
        # console.print(f"UPDATE EDGE: {fr} to {type}", type, style='green')

    def update_edge_att(self, fr: int, to: int, type: str, attribute_names: [str]):
        pass
        # console.print(f"UPDATE EDGE ATT: {fr} to {type} {attribute_names}", style='green')

    def delete_edge(self, fr: int, to: int, type: str):
        pass
        # console.print(f"DELETE EDGE: {fr} to {type} {type}", style='green')
