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
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize


import time
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
        self.Period = 2000

        # YOU MUST SET AN UNIQUE ID FOR THIS AGENT IN YOUR DEPLOYMENT. "_CHANGE_THIS_ID_" for a valid unique integer
        self.agent_id = 136
        self.g = DSRGraph(0, "pythonAgent", self.agent_id)
        self.hadcoded_poses = [ [330, -20,  500,     0, np.pi,  np.pi/2], 
                                [120, -20,  350, np.pi,    0.5, -np.pi/2],
                                [320, 156, 460,   2.90, 0,  -np.pi/2] ]
                                # [300,  230, 380,   2.7,      0, -np.pi/2]]

        
        self.cube_rts = {}

        # self.b_rt = np.array([0, 0, 0, 0, 0, 0])
        self.original_rt = np.array([10, 100, -150, 0, 0, 0])
        self.b_rt = np.copy(self.original_rt)


        self.load_cube_rts()

        print ("Starting initial optimization")
        print (self.b_rt, self.transformation_error_2(self.b_rt))

        ini = time.time()

        self.b_rt = self.compute_and_publish_best_rt(self.b_rt)
        print ("Finished initial optimization in", time.time()-ini)
        self.old_error = self.transformation_error_2(self.b_rt)
        print (self.b_rt, self.old_error)

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


    def transformation_error (self, rt_gr_cam):
        rot_m = R.from_euler("XYZ", rt_gr_cam[3:])

        res_mat = np.hstack([rot_m.as_matrix(), rt_gr_cam[:3].reshape(3,1)])
        res_mat = np.vstack([res_mat, [0,0,0,1]])
        tf = inner_api(self.g)
        rmat_w_gr = tf.get_transformation_matrix("world", "gripper")
        mat = rmat_w_gr @ res_mat

        mat2 = np.linalg.inv(mat[:3,:3])


        print (mat,"\n\n", mat2, "\n\n", mat[:3,:3]@mat2, "\n\n", mat2@mat2)

        cubes = self.g.get_nodes_by_type ("box")
        diff = 0
        for cube in cubes:
            cube1 = self.g.get_edge ("hand_camera", cube.name, "RT")
            cube1_trans = cube1.attrs["rt_translation"].value
            cube1_rot   = cube1.attrs["rt_rotation_euler_xyz"].value

            v_rt = self.g.get_edge ("world", cube.name, "virtual_RT")

            print(cube.name, cube1_rot, cube1_rot @ mat2, v_rt.attrs["rt_rotation_euler_xyz"].value)

            cube1_trans = np.append(cube1_trans, 1)
            cube1_res = mat @ cube1_trans


            diff += np.linalg.norm(cube1_res[:3] - v_rt.attrs["rt_translation"].value)
        # print (diff)
        return diff

    def angle_diff (self, v1, v2):

        v1_q = R.from_euler("xyz", v1).as_quat()
        v2_q = R.from_euler("xyz", v2).as_quat()

        return 1 - np.inner(v1_q, v2_q)**2 # np.linalg.norm(v1_q - v2_q)

    def get_cube_error (self, name):
        rt = rt_api(self.g)
        tf = inner_api(self.g)

        rt = tf.transform_axis ("world", self.cube_rts[name], "hand_camera")
        v_rt = self.g.get_edge ("world", name, "virtual_RT")

        rot_diff   = self.angle_diff(rt[3:],  v_rt.attrs["rt_rotation_euler_xyz"].value) * 1000
        trans_diff = np.linalg.norm (rt[:3] - v_rt.attrs["rt_translation"].value)

        return rot_diff, trans_diff
        # print (cube.name, trans_diff, rot_diff)


    def update_camera_rt (self, rt_gr_cam):
        rt = rt_api(self.g)

        griper = self.g.get_node ("gripper")
        h_camera = self.g.get_node ("hand_camera")
        rt.insert_or_assign_edge_RT(griper, h_camera.id, rt_gr_cam[:3], rt_gr_cam[3:])
        self.g.update_node(griper)

    def load_cube_rts (self):
        print  ("--> Updated rts to avoid noise")
        cubes = self.g.get_nodes_by_type ("box")
        tf = inner_api(self.g)
        
        for cube in cubes:
            rt = self.g.get_edge ("hand_camera", cube.name, "RT")
            self.cube_rts[cube.name] = np.concatenate((rt.attrs["rt_translation"].value, rt.attrs["rt_rotation_euler_xyz"].value))

        print (self.cube_rts)

    def transformation_error_2 (self, rt_gr_cam):

        self.update_camera_rt(rt_gr_cam)

        cubes = self.g.get_nodes_by_type ("box")
        diff = 0
        for cube in cubes:

            rot_diff ,trans_diff = self.get_cube_error(cube.name)

            # rt = tf.transform_axis ("world", cube.name)
            # v_rt = self.g.get_edge ("world", cube.name, "virtual_RT")

            # rot_diff   = self.angle_diff(rt[3:],  v_rt.attrs["rt_rotation_euler_xyz"].value) * 1000
            # trans_diff = np.linalg.norm (rt[:3] - v_rt.attrs["rt_translation"].value)


            # print (cube.name, trans_diff, rot_diff)

            diff += rot_diff + trans_diff

        return diff

    def move_to_hardcoded_pose (self, index):
        dest_pose = self.hadcoded_poses [index]

        gripper = self.g.get_node ("gripper")
        gripper.attrs["gripper_target_finger_distance"].value = 1.0
        gripper.attrs["target"].value = dest_pose
        self.g.update_node (gripper)

    def compute_and_publish_best_rt (self, initial_guess):
        rt = rt_api(self.g)

        res = minimize(self.transformation_error_2, initial_guess, method='Nelder-Mead')
        
        # griper = self.g.get_node ("gripper")
        # h_camera = self.g.get_node ("hand_camera")
        # rt.insert_or_assign_edge_RT(griper, h_camera.id, res.x[:3], res.x[3:])
        # self.g.update_node(griper)
        print ("optimization success:", res.success)
        return res.x

    @QtCore.Slot()
    def compute(self):


        print ("Moving to home")
        self.move_to_hardcoded_pose(0)
        time.sleep(5)
        print ("Moving to home - done")
        
        
        rt = rt_api(self.g)
        tf = inner_api(self.g)

        # camera_rt = self.g.get_edge ("gripper", "hand_camera", "RT")     
        # print (camera_rt)
        # rot_m = R.from_euler("XYZ", camera_rt.attrs["rt_rotation_euler_xyz"].value)

        # res_mat = np.hstack([rot_m.as_matrix(), camera_rt.attrs["rt_translation"].value.reshape(3,1)])
        # res_mat = np.vstack([res_mat, [0,0,0,1]])

        # cube1 = self.g.get_edge ("hand_camera", "cube_1", "RT")
        # cube1_trans = cube1.attrs["rt_translation"].value

        # rmat_1 = tf.get_transformation_matrix("gripper", "hand_camera")
        # rmat_2 = tf.get_transformation_matrix("world", "gripper")


        # cube1_trans = np.append(cube1_trans, 1)
        # test_2 = rmat_2 @ res_mat @ cube1_trans

        # self.transformation_error_2(self.b_rt)

        # self.b_rt = self.compute_and_publish_best_rt(self.b_rt)
        # print (self.b_rt, self.transformation_error(self.b_rt))
        


        # print (self.transformation_error(np.array( [ 15.8171056,  77.6368291, -150.756363,-0.0545380167,  0.0111418872,  0.0372585873])))
        self.load_cube_rts()
        self.current_error = self.transformation_error_2(self.b_rt)
        print (self.b_rt, self.current_error)

        if self.current_error > 20:

            last_rt = np.copy(self.b_rt) 

            trans_noise = np.random.normal (0, 5  , 3)
            rot_noise   = np.random.normal (0, 0.5, 3)

            print ("Starting optimization", trans_noise, rot_noise)

            # self.b_rt[:3] += trans_noise
            # self.b_rt[3:] += rot_noise

            ini = time.time()
            self.b_rt = self.compute_and_publish_best_rt(self.b_rt)
            print ("Finished optimization in", time.time()-ini)

            new_error = self.transformation_error_2(self.b_rt)
            

            if (new_error > self.current_error):
                print ("keeping the old one", self.current_error, new_error)
                self.update_camera_rt(last_rt)
                self.b_rt = last_rt

        else:
            print ("Not optmizing")

        # print (self.transformation_error_2(self.b_rt))
        # cubes = self.g.get_nodes_by_type ("box")
        # for cube in cubes:
        #     print (cube.name, self.get_cube_error(cube.name))
        # return True

        for i in range(len(self.hadcoded_poses)):
            self.move_to_hardcoded_pose(i)
            time.sleep(7)
            print ("---- Evaluation in pose", i, "------")
            self.load_cube_rts()
            print (self.transformation_error_2(self.b_rt))
            cubes = self.g.get_nodes_by_type ("box")
            for cube in cubes:
                print (cube.name, self.get_cube_error(cube.name))
            time.sleep(1)

            print (self.transformation_error_2(self.original_rt))
            cubes = self.g.get_nodes_by_type ("box")
            for cube in cubes:
                print (cube.name, self.get_cube_error(cube.name))
            time.sleep(1)
            print ("-------------------------------------")

        return True

        plot = np.zeros((300, 800, 3), np.uint8)
        plot2 = np.zeros((300, 800, 3), np.uint8)

        positional_errors = []
        
        cubes = self.g.get_nodes_by_type ("box")
        for cube in cubes:
            plot = np.zeros((300, 800, 3), np.uint8)
            # rt   = self.g.get_edge ("world", cube.name, "RT")
            rt = tf.transform_axis ("world", cube.name)
            v_rt = self.g.get_edge ("world", cube.name, "virtual_RT")

            if rt is None or v_rt is None:
                continue


            # trans_diffs = np.absolute(rt.attrs["rt_translation"].value - v_rt.attrs["rt_translation"].value)
            # rot_diffs   = np.absolute(rt.attrs["rt_rotation_euler_xyz"].value - v_rt.attrs["rt_rotation_euler_xyz"].value)

            trans_diffs = rt[:3] - v_rt.attrs["rt_translation"].value# np.absolute(rt[:3]- v_rt.attrs["rt_translation"].value)
            rot_diffs   = rt[3:] - v_rt.attrs["rt_rotation_euler_xyz"].value

            positional_errors.append (trans_diffs)

            trans_diff = np.linalg.norm (trans_diffs)
            rot_diff   = np.linalg.norm (rot_diffs)

            print ("----", cube.name, "----")
            # print (rt[:3], v_rt.attrs["rt_translation"].value, "\n")
            print (self.get_cube_error(cube.name))

            # self.plot_bars (plot, trans_diffs, 50, 10, 2)
            # cv2.imshow(cube.name + " trans diff", plot)

            # self.plot_bars (plot2, rot_diffs, 30, 5)
            # cv2.imshow(cube.name +  " rot diff", plot2)

    
        print ("\n \n \n")
        
        
        cv2.waitKey(1)
        return True

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)




    def plot_bars (self, img, values, max, steps, signed = 1):
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
            cv2.rectangle(img, (x1, y),(x2, height//signed), (255, 0, 255), -1)
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
