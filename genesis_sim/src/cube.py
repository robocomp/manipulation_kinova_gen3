import torch
import numpy as np
import uuid
import time

class Cube:
    def __init__(self, pos, quat):
        self.pos = pos
        self.quat = quat
        self.updated = False
        self.id = uuid.uuid4()
        self.creation_time = time.time()
        self.pos_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.type = "cube"

    def project(self):
        return [self.pos, self.quat]

    def is_equal(self, lcube):
        return (torch.sqrt(torch.sum(torch.square(self.pos - lcube[0])))) < 1e-4 and \
                (torch.sqrt(torch.sum(torch.square(self.quat - lcube[1])))) < 1e-4

    def get_top_reach_position(self):
        return self.pos + torch.tensor([0, 0, 0.19], device=self.pos.device)

    def get_affordances(self):
        pass

    def aff_reach(self):
        pass

