import numpy as np
from numpy import linalg as LA
import os

class Planifier():
    def __init__(self):
        return

    def create_initial_state(self, original_tags, cubes):
        initState = ["  (handempty)"]
        

        # tags = []
        # for tag in original_tags:
        #     if self.is_horizontal(tag):
        #         initState.append(f"  (clear {tag.tag_id})")
        #     elif tag.tag_id in cubes:
        #         tags.append(tag)
        # ret_tags = tags
        # tags.sort(key=lambda x: x.center[1], reverse=True)
        # tag_base = tags[0]
        # tags_aux = []
        # for tag in tags:
        #     if abs(tag_base.center[1] - tag.center[1]) < 30:
        #         initState.append(v)
        #         tags_aux.append(tag)
        #     else:
        #         for tag_aux in tags_aux:
        #             if abs(tag.center[0] - tag_aux.center[0]) < 30 and tag.tag_id != tag_aux.tag_id:
        #                 initState.append(f"  (on {tag.tag_id} {tag_aux.tag_id})")
        #                 tags_aux.remove(tag_aux)
        #                 tags_aux.append(tag)
        # return initState, ret_tags
        initState.append(f"  (ontable {3})")
        initState.append(f"  (on {3} {2})")
        initState.append(f"  (clear {2})")
        return initState

    def is_horizontal(self, tag):
        up_line = LA.norm(np.array(tag.corners[3]) - np.array(tag.corners[2]))
        down_line = LA.norm(np.array(tag.corners[1]) - np.array(tag.corners[0]))
        return down_line > up_line

    def create_final_state(self, ui):
        end_state = ["  (and(handempty)"]
        
        # b1 = (1, ui.cubo1.x(), ui.cubo1.y())
        # b2 = (2, ui.cubo2.x(), ui.cubo2.y())
        # b3 = (3, ui.cubo3.x(), ui.cubo3.y())
        # b4 = (4, ui.cubo4.x(), ui.cubo4.y())
        # b5 = (5, ui.cubo5.x(), ui.cubo5.y())
        # b6 = (6, ui.cubo6.x(), ui.cubo6.y())
        # cubos = [b1, b2, b3, b4, b5, b6]
        # cubos.sort(key=lambda x: x[2], reverse=True)
        
        # cubo_mesa = cubos[0]
        # cubos_aux = []
        # for cubo in cubos:
        #     if abs(cubo_mesa[2] - cubo[2]) < self.constants["block_threshold"]:
        #         self.end_state.append(f"     (ontable {cubo[0]})")
        #         cubos_aux.append(cubo)
        #     else:
        #         for c_aux in cubos_aux:
        #             if abs(cubo[1] - c_aux[1]) < self.constants["block_threshold"] and cubo[0] != c_aux[0]:
        #                 self.end_state.append(f"     (on {cubo[0]} {c_aux[0]})")
        #                 cubos_aux.remove(c_aux)
        #                 cubos_aux.append(cubo)
        # for cubo in cubos_aux:
        #     self.end_state.append(f"     (clear {cubo[0]})")
        # self.begin_plan = True
        # self.end = False
        # self.step = 0
        # print(self.end_state)
        end_state.append(f"  (ontable {3})")
        end_state.append(f"  (clear {3})")
        end_state.append(f"  (ontable {2})")
        end_state.append(f"  (clear {2})")
        return end_state

    def save_to_file(self, init_state, end_state, tag_list):
        tag_list.sort()
        blocks = ""
        for tag in tag_list:
            blocks += str(tag.tag_id) + " "

        for r in init_state:
            print(r)
        for r in end_state:
            print(r)
        lines = [
            "(define (problem BLOCKS-{}-1)".format(len(tag_list)),
            "(:domain blocks)",
            "(:objects {}- block)".format(blocks),
            "(:init",
            *init_state,
            ")",
            "(:goal",
            *end_state, 
            "   )",
            ")",
            ")"
        ]
        with open("init_state.pdll", 'w+') as f:
            f.write('\n'.join(lines))
    
    def exec_planner(self):
        print("PLANNER EXECUTION")
        #os.system("cd /home/robocomp/software/fast-downward-20.06/ && ls && \
        #./fast-downward.py --alias lama-first /home/robocomp/robocomp/components/manipulation_kinova_gen3/etc/domain.pddl {}"
        #.format("/home/robocomp/robocomp/components/manipulation_kinova_gen3/components/controllerDani/init_state.pdll"))
    
    def load_plan(self):
        #with open("init_state.pdll", 'r+') as f:
        #    lines = f.readlines()
        #    for line in lines:
        #        if ";" not in line:
        #            action, *rest = line[1:-2].split()
        #            self.plan.append([action, list(map(lambda x: int(x), rest))])
        return [["pick-up", 2], ["put_dow", 2]]
