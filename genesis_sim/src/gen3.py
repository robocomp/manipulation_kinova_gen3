'''
This file contains the Gen3 class, which is a model of the Kinova Gen3 robot arm.
'''

import numpy as np
from shapely.geometry import MultiPoint, Point, LineString, Polygon
from typing import Tuple

class Gen3:
    def __init__(self, sim_gen3):
        self.pos = [0, 0, 0]
        self.model = None
        self.sim_gen3 = sim_gen3    # low-level simulation object
        self.min_step = 0.003

    def print_affordances(self):
        # get the names of methods in this class starting with "aff_"
        methods = [func for func in dir(self) if callable(getattr(self, func)) and func.startswith("aff_")]
        res = ""
        for i, e in enumerate(methods):
            res += f"{i}: {e.replace('aff_', '')}<br>"
        return res

    def aff_exploring_from_above(self, params=None) -> Tuple[float, float]:
        """
        Generates a trajectory that explores the workspace from above.
        It  does so by analyzing the list of model_cubes and creating a convex hull that contains all the cubes.
        """
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
                yield point[0], point[1]
            i = (i + 1) % len(res)

    def aff_select_cube(self, params: dict) -> Tuple[float, float]:
        """
        Moves end effector over the selected cube.
        """
        # get current end effector pose
        start = self.sim_gen3.get_link("bracelet_link").get_pos().cpu() #TODO: get name from above
        end = params["cube"].pos.cpu()
        end[2] = start[2]
        dist = np.linalg.norm(start - end)
        density = int(dist / self.min_step)
        for t in np.linspace(0, 1, density):
            point = (1 - t) * start + t * end
            yield point[0], point[1]