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
from parser import Parser

sys.path.append('/opt/robocomp/lib')
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
            self.model_cubes = []

            # arm affordance
            self.first_time = True  # for aff_circular_trajectory
            self.traj = None

            self.target_cube = None
            self.default_tip_pose = self.sim.gen3.get_link(self.sim.tip_name).get_pos()
            self.number_of_real_cubes = 3

            # parser
            interpreter = Parser()

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    def setParams(self, params):
        return True


    @QtCore.Slot()
    def compute(self):
        # get image from camera
        _, _, seg, _ = self.sim.camera.render(rgb=False, segmentation=True)

        # update model
        model_cubes = self.update_model(self.sim.scene, seg, self.model_cubes)
        for cube in model_cubes:
            print("cube pos", cube.pos)
        print("------------------")

        # collect affordances from the model
        #affordances = self.collect_affordances(model_cubes, self.sim.gen3)

        # select and affordance. For now, select the arm's default affordance that moves around in the x-y plane
        #curr_affordance = affordances[0]

        # execute one step of the affordance
        self.execute_affordance(None)

        # update camera in hand
        self.sim.camera.set_pose(transform=self.sim.move_camera(self.sim.gen3))

        # update sim
        self.sim.scene.step()


    ###################################################################################################
    def execute_affordance(self, affordance):

        if self.first_time:
            #self.traj = self.aff_circular_trajectory(0.1, 0, self.default_tip_pose[:2])  # Create the generator
            self.traj = self.aff_exploring_from_above(0.003)
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
            #print("Trajectory finished")
            time.sleep(0.5)
            self.sim.move_arm_to_default_pose()
            time.sleep(0.5)
            self.first_time = True


    def collect_affordances(self, model_cubes, gen3):
        affordances = [self.gen3_get_affordances()]
        for cube in model_cubes:
            affordances.append(cube.get_affordances())
        return affordances

    def gen3_get_affordances(self):
        return 0

    def update_model(self, scene, seg_image, model_cubes):
        # prediction: match-insert-delete cycle
        local_cubes = []
        for i in range(1, self.number_of_real_cubes+1):
            if i in seg_image:
                local_cubes.append(
                    [scene.rigid_solver.entities[i].get_pos(), scene.rigid_solver.entities[i].get_quat()])

        local_cubes_to_remove = []
        for pcube in model_cubes:
            pcube.updated = False
            for lcube in local_cubes:
                if pcube.is_equal(lcube):  # successfull prediction. remove lcube from local_cubes
                    local_cubes_to_remove.append(lcube)
                    pcube.updated = True

        # do like this because each element is a tuple of two tensors
        remaining_local_cubes = [t for t in local_cubes if
                                 not any(
                                     torch.equal(t[0], r[0]) and torch.equal(t[1], r[1]) for r in
                                     local_cubes_to_remove)]

        # add the remaining cubes to the model
        for lcube in remaining_local_cubes:
            model_cubes.append(Cube(pos=lcube[0], quat=lcube[1]))
            model_cubes[-1].updated = True
            print("Cube updated", model_cubes[-1].pos)
        # remove the cubes that are not in the local_cubes
        for cube in model_cubes:
            if not cube.updated:  # cube was not updated in the current frame
                model_cubes.remove(cube)
                print("Cube removed", cube.pos)

        return model_cubes

    import numpy as np
    import matplotlib.pyplot as plt

    def aff_circular_trajectory(self, r: float, theta: float, initial_xy_pos) -> Tuple[float, float]:
        """
        Generates the x and y coordinates of the trajectory iteratively.

        Args:
          R: The radius of the circle.
          theta: The random angle in radians.

        Yields:
          A tuple (x, y) representing the coordinates at each step.
        """
        # Radial movement (t from 0 to 1)
        for t in np.linspace(0, 1, 200):  # You can adjust the number of steps
            x = (r * np.cos(theta)) * t + initial_xy_pos[0]
            y = (r * np.sin(theta)) * t + initial_xy_pos[1]
            yield x, y

        # Circular movement (t from 1 to 2)
        for t in np.linspace(1 + 1 / 100, 2,
                             100):  # Start slightly after 1 to avoid duplicate point, adjust steps as needed
            x = r * np.cos(theta + 2 * np.pi * (t - 1)) + initial_xy_pos[0]
            y = r * np.sin(theta + 2 * np.pi * (t - 1)) + initial_xy_pos[1]
            yield x, y

        for t in np.linspace(1 + 1 / 100, 2,
                             100):  # Start slightly after 1 to avoid duplicate point, adjust steps as needed
            x = r * np.cos(theta + 2 * np.pi * (t - 1)) + initial_xy_pos[0]
            y = r * np.sin(theta + 2 * np.pi * (t - 1)) + initial_xy_pos[1]
            yield x, y

    def aff_exploring_from_above(self, min_step: float) -> Tuple[float, float]:
        """
        Generates a trajectory that explores the workspace from above.
        It  does so by analyzing the list of model_cubes and creating an stochastic trajectory that moves the end effector
        visiting briefley each cube.
        """
        # read model_cubes  and  compute a convex  hull that contains all the cubes
        points = np.array([cube.pos.cpu() for cube in self.model_cubes])
        multi_point = MultiPoint(points)
        hull = multi_point.convex_hull
        res = []
        if isinstance(hull, Polygon):
            res = list(hull.exterior.coords)[:-1]  # Exclude the last point (duplicate)
        elif isinstance(hull, LineString):
            res = list(hull.coords)
        elif isinstance(hull, Point):
            res = [(hull.x, hull.y)]

        # Generate an eternal trajectory that visits each point in the convex hull
        i=0
        while True:
            start = np.array(res[i])
            end = np.array(res[(i+1) % len(res)])
            dist = np.linalg.norm(start - end)
            density = int(dist / min_step)
            for t in np.linspace(0, 1, density):
                point = (1 - t) * start + t * end
                yield point[0], point[1]
            i = (i + 1) % len(res)

    ####################################################333


    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)





