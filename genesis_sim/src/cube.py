import torch
import numpy as np
import uuid

class Cube:
    def __init__(self, pos, quat):
        self.pos = pos
        self.quat = quat
        self.updated = False
        self.id = uuid.uuid4()

    def project(self):
        return [self.pos, self.quat]

    def is_equal(self, lcube):
        return (torch.sqrt(torch.sum(torch.square(self.pos - lcube[0])))) < 1e-4 and \
                (torch.sqrt(torch.sum(torch.square(self.quat - lcube[1])))) < 1e-4

    def get_affordances(self):
        pass

    def aff_reach(self):
        pass
