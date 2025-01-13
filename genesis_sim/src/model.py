'''
This file contains the Model class, which is responsible for storing the state of the application.
'''

import torch
from cube import Cube
from basket import Basket

class Model:
    def __init__(self):
        self.elements = []
        self.gen3 = None
        self.number_of_real_cubes = 3
        self.last_selected_element = None

    def  add_robot(self, gen3):
        self.gen3 = gen3
        self.gen3.model = self

    def update_model(self, scene, seg_image, model):
        # prediction: match-insert-delete cycle

        ############## Cubes ################################3
        local_cubes = []
        local_cubes_to_remove = []
        for i in range(1, self.number_of_real_cubes+1):
            if i in seg_image:
                local_cubes.append(
                    [scene.rigid_solver.entities[i].get_pos(), scene.rigid_solver.entities[i].get_quat()])

        for pcube in model.elements:
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
            model.elements.append(Cube(pos=lcube[0], quat=lcube[1]))
            model.elements[-1].updated = True
            #print("Cube updated", model_cubes[-1].pos)
        # remove the cubes that are not in the local_cubes
        for cube in model.elements:
            if not cube.updated:  # cube was not updated in the current frame
                model.elements.remove(cube)
                #print("Cube removed", cube.pos)

        ############## Basket ################################3
        basket = [scene.rigid_solver.entities[4].get_pos(), scene.rigid_solver.entities[4].get_quat()] if 4 in seg_image else None
        if basket is not None and not basket in model.elements:
            model.elements.append(Basket(pos=basket[0], quat=basket[1]))
        #TODO: remove basket if it is not in the scene
        #TODO: update basket position if it is in the scene and has moved

        return model

    def get_element_by_index(self, index):
        # given a sorting by creation_time, return the element at index
        if len(self.elements) >= index:
            elements = sorted(self.elements, key=lambda x: x.creation_time)
            return elements[index]
        else:
            return None

    def get_index_of_element(self, element):
        # given a sorting by creation_time, return the index of the element
        elements = sorted(self.elements, key=lambda x: x.creation_time)
        for i, e in enumerate(elements):
            if e == element:
                return i
        return None

    def print_model(self):
        # Assign a number to each element in elements so that the oldest one is the first one
        # sort elements by creation_time
        elements = sorted(self.elements, key=lambda x: x.creation_time)
        res = ""
        for i, e in enumerate(elements):
            res += f"{e.type} id={i}, [{e.pos[0]:.2f}, {e.pos[1]:.2f}, {e.pos[2]:.2f}]<br>"
        return res
