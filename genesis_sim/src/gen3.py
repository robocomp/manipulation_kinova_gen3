'''
This file contains the Gen3 class, which is a model of the Kinova Gen3 robot arm.
'''

import numpy as np
from shapely.geometry import MultiPoint, Point, LineString, Polygon
from typing import Tuple

class Gen3:
    def __init__(self, sim_gen3, dofs_idx):
        self.pos = [0, 0, 0]
        self.model = None
        self.sim_gen3 = sim_gen3    # low-level simulation object
        self.dofs_idx = dofs_idx
        self.min_step = 0.003
        self.tip_name = "bracelet_link"
        self.default_tip_pose = sim_gen3.get_link(self.tip_name).get_pos()


    def get_current_tip_pose(self):
        return self.sim_gen3.get_link(self.tip_name).get_pos().cpu().numpy()

    def print_affordances(self):
        # get the names of methods in this class starting with "aff_"
        methods = [func for func in dir(self) if callable(getattr(self, func)) and func.startswith("aff_")]
        res = ""
        for i, e in enumerate(methods):
            res += f"{i}: {e.replace('aff_', '')}<br>"
        return res

    ###############################################
    ################# Affordances #################
    ###############################################

    def aff_exploring_from_above(self, params=None) -> Tuple[float, float]:
        """
        Generates a trajectory that explores the workspace from above.
        It  does so by analyzing the list of model_cubes and creating a convex hull that contains all the cubes.
        """
        print("AFFORDANCE: aff_exploring_from_above")
        # if tip not in default position, move to default position
        current_tip_pose = self.sim_gen3.get_link(self.tip_name).get_pos().cpu()
        if not np.allclose(current_tip_pose, self.default_tip_pose.cpu(), atol=1e-3):
            start = current_tip_pose
            end = self.default_tip_pose.cpu()
            dist = np.linalg.norm(start - end)
            density = int(dist / self.min_step)
            for t in np.linspace(0, 1, density):
                point = (1 - t) * start + t * end
                yield point[0], point[1], point[2]

        # read model_cubes  and  compute a convex  hull that contains all the cubes
        points = np.array([cube.pos.cpu() for cube in self.model.elements])
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
        i = 0
        while True:
            start = np.array(res[i])
            end = np.array(res[(i + 1) % len(res)])
            dist = np.linalg.norm(start - end)
            density = int(dist / self.min_step)
            for t in np.linspace(0, 1, density):
                point = (1 - t) * start + t * end
                yield point[0], point[1], self.default_tip_pose[2].cpu()
            i = (i + 1) % len(res)

    def aff_select_cube(self, params: dict) -> Tuple[float, float]:
        """
        Moves end effector over the selected cube.
        """
        print("AFFORDANCE: aff_select_cube")
        # get current end effector pose
        start = self.sim_gen3.get_link(self.tip_name).get_pos().cpu()
        end = params["cube"].pos.cpu()
        end[2] = start[2]
        dist = np.linalg.norm(start - end)
        density = int(dist / self.min_step)
        for t in np.linspace(0, 1, density):
            point = (1 - t) * start + t * end
            yield point[0], point[1], self.default_tip_pose[2].cpu()

    def aff_reach_cube(self, params: dict) -> Tuple[float, float]:
        """
        Moves end effector down close to the selected cube.
        """
        print("AFFORDANCE: aff_reach_cube")
        # get current end effector pose
        start = self.sim_gen3.get_link(self.tip_name).get_pos().cpu()
        end = params["cube"].get_top_reach_position().cpu()
        dist = np.linalg.norm(start - end)
        print("dist", dist, "start", start, "end", end)
        density = int(dist / self.min_step)
        for t in np.linspace(0, 1, density):
            point = (1 - t) * start + t * end
            yield point[0], point[1], point[2]

    def aff_lift(self, params: dict) -> Tuple[float, float]:
        """
        Lifts the end effector.
        """
        print("AFFORDANCE: aff_lift")
        start = self.sim_gen3.get_link(self.tip_name).get_pos().cpu()
        end = self.sim_gen3.get_link(self.tip_name).get_pos().cpu()
        end[2] = self.default_tip_pose[2].cpu()
        dist = np.linalg.norm(start - end)
        print("dist", dist, "start", start, "end", end)
        density = int(dist / self.min_step)
        for t in np.linspace(0, 1, density):
            point = (1 - t) * start + t * end
            yield point[0], point[1], point[2]

    def aff_idle(self, pparams:dict):
        """
        Does nothing.
        """
        #print("AFFORDANCE: aff_idle")
        yield self.sim_gen3.get_link(self.tip_name).get_pos().cpu()

    def aff_open_gripper(self, params: dict):
        print("AFFORDANCE: aff_open_gripper")
        self.sim_gen3.control_dofs_position(np.array([0, 0, 0, 0, 0, 0]), [7, 8, 9, 10, 11, 12])
        for t in range(100):
            yield None, None, None

    def aff_close_gripper(self, params: dict):
        print("AFFORDANCE: aff_close_gripper")
        self.sim_gen3.control_dofs_position(np.array([-0.1, -0.1, -0.1, -0.1, 0.1, 0.1])*8, [7, 8, 9, 10, 11, 12])
        for t in range(100):
            yield None, None, None