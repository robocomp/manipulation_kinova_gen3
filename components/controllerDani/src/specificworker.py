#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
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
from genericworker import *
import cv2
import apriltag
import time
import numpy as np
import random
from numpy import linalg as LA
import threading, queue

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            # image
            self.image = []
            self.depth = []
            self.camera_name = "camera_arm"

            # apriltags
            self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))

            # get current position
            ref_base = RoboCompKinovaArm.ArmJoints.base
            self.tool_initial_pose = self.kinovaarm_proxy.getCenterOfTool(ref_base)
            print("Initial pose:", self.tool_initial_pose)
            self.plan = [["pick_up", 2], ["put_down"]]
            self.step = 0
            self.end = False
            self.actions = {
                "pick_up" : {
                    "do_action" : True,
                    "thread": None,
                    "queue": queue.Queue(),
                    "func": self.pick_up,
                },
                "put_down" : {
                    "do_action" : False,
                    "thread": None,
                    "queue": queue.Queue(),
                    "func": self.put_down
                },
                "stack": {
                    "do_action" : False,
                    "thread": None,
                    "queue": queue.Queue(),
                    "func": self.stack
                },
                "unstack": {
                    "do_action" : False,
                    "thread": None,
                    "queue": queue.Queue(),
                    "func": self.unstack
                }

            }

            #  set of candidate positions for blocks on the table
            # a = RoboCompKinovaArm.TPose([74, 450, 30])

            self.timer.timeout.connect(self.compute)
            #self.timer.setSingleShot(True)
            self.timer.start(self.Period)


    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        color, depth, all = self.read_camera()
        color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        color, tags = self.compute_april_tags(color, depth)

        if self.end:
            print("PLAN ENDED")
        else:
            current_step = self.plan[self.step]
            current_action = self.actions[current_step[0]]
            params = current_step[1] if len(current_step) > 1 else None
            args_list = []
            if current_action["do_action"]:
                if current_action["thread"] == None:
                    if current_step[0] == "pick_up":
                        args_list = [params, tags, all.image]
                    elif current_step[0] == "put_down":
                        args_list = [color, all.image]
                    elif current_step[0] == "stack":
                        pass
                    elif current_step[0] == "unstack":
                        pass
                    current_action["thread"] = threading.Thread(name=current_step[0], target=current_action["func"], args=tuple(args_list))
                    current_action["thread"].start()
                else:
                    try:
                        state = current_action["queue"].get_nowait()
                        print(state)
                        if "Finish" in state:
                            current_action["do_action"] = False
                            current_action["thread"] = None
                            print(f"{current_step[0]} DONE")
                            self.step += 1
                            if self.step == len(self.plan):
                                self.end = True
                            else:
                                next_step = self.plan[self.step]
                                next_action = self.actions[next_step[0]]
                                next_action["do_action"] = True
                    except:
                        pass

        self.draw_image(color)

    # ===================================================================
    # locate x and do ballistic approach
    def pick_up(self, x, tags, image):
        tx = [t for t in tags if t.tag_id == x]
        if tx:
            tx_pose = self.detector.detection_pose(tx[0], [image.focalx, image.focaly, image.width / 2,
                                                           image.height / 2], tag_size=0.04, z_sign=1)
            tr = tx_pose[0][:, 3][:-1]*1000  # first three elements of last column, aka translation
            self.actions["pick_up"]["queue"].put("PICK_UP: Target position in camera ref system: " + str(tr))
            ref_base = RoboCompKinovaArm.ArmJoints.base
            current_pose = self.kinovaarm_proxy.getCenterOfTool(ref_base)
            target = RoboCompKinovaArm.TPose()
            target.x = current_pose.x - tr[0]*1.2    # possible focus error
            target.y = current_pose.y + tr[1] - 40   # plus distance from camera to top along Y
            target.z = current_pose.z - tr[2] + 110  # distance from camera to tip along Z
            self.actions["pick_up"]["queue"].put("PICK_UP: Tip sent to target")
            self.kinovaarm_proxy.setCenterOfTool(target, ref_base)
            target_pose = np.array([target.x, target.y, target.z])

            # wait for the arm to complete the movement
            dist = sys.float_info.max
            while dist > 30:
                pose = self.kinovaarm_proxy.getCenterOfTool(ref_base)
                dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - target_pose)
                #print("dist", dist)

            # grasp. The tag is not visible from here. Should be driven by gripper sensors
            self.actions["pick_up"]["queue"].put("PICK_UP: Initiating grasping")
            pre_grip_pose = self.kinovaarm_proxy.getCenterOfTool(ref_base)
            pre_grip_pose.z -= 80    # block semi height
            pre_grip_pose.x -= 10  #
            self.kinovaarm_proxy.setCenterOfTool(pre_grip_pose, ref_base)
            time.sleep(3)
            self.kinovaarm_proxy.openGripper()
            time.sleep(3)

            # move to initial position
            self.kinovaarm_proxy.setCenterOfTool(current_pose, ref_base)
            current_pose_np = np.array([current_pose.x, current_pose.y, current_pose.z])
            self.actions["pick_up"]["queue"].put("PICK_UP: Tip sent to initial position")
            dist = sys.float_info.max
            while dist > 50:  # mm
                pose = self.kinovaarm_proxy.getCenterOfTool(ref_base)
                dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - current_pose_np)

            self.actions["pick_up"]["queue"].put("PICK_UP: Finish")
            return True
        else:
            self.actions["pick_up"]["queue"].put("PICK_UP: Finish without finding target cube")
            return False

    def put_down(self, color, image):
        # search a free spot in the table
        # sample the image with rectangles for a blue-filled one
        # the size of the square in pixels is  x = (u - cx) * z / fx
        current_pose = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        z = current_pose.z + 110
        x1 = (-image.width/2.0) * z / image.focalx
        x2 = (90 - image.width/2.0) * z / image.focalx   # 90 is the expanded size of the block
        side = int(abs(x1-x2))
        print("side", side)
        # compute the color of the table (blue)
        hist = cv2.calcHist([color], [2], None, [256], [0, 256])
        table_color = np.argmax(hist)
        print("table_color", table_color)

        # choose a random point close to the tip projection point
        np.random.seed()
        result = False
        counter = 0
        while not result and counter < 100:
            cx, cy = random.randint(side, image.width-side), random.uniform(side*4, image.height-side)
            # get the block - roi in the current image
            roi = color[int(cx-side/2):int(cx+side/2), int(cy-side/2.0):int(cy+side/2)]
            # check that all points are close to the table color
            blue = np.array(roi[:, :, 2])  # blue channel
            hits = np.count_nonzero(abs(blue - table_color) < 50)
            print(result, hits, roi.shape[0]*roi.shape[1], int(cx), int(cy))
            counter += 1
            result = hits >= roi.shape[0]*roi.shape[1] * 0.9

        # compute target coordinates. cx, cy, z in camera CS
        # color = cv2.rectangle(color, (cx-side/2, cy-side/2), (cx+side/2, cy+side/2), (255, 0, 0), 6)
        # cv2.imshow("roi", roi)
        target = RoboCompKinovaArm.TPose()
        z = current_pose.z + 110
        target.x = cx * image.focalx / z + (image.width/2)
        target.y = -cy * image.focaly / z + (image.height/2)
        if target.y < -500: target.y = -500
        target.z = 10
        print("target:", int(target.x), int(target.y), target.z)
        #target.x = 0

        self.actions["put_down"]["queue"].put("PUT_DOWN: Target position in camera ref system: " + str([target.x, target.y, target.z]))
        self.actions["put_down"]["queue"].put("PUT_DOWN: Tip sent to target")
        self.kinovaarm_proxy.setCenterOfTool(target, RoboCompKinovaArm.ArmJoints.base)
        target_np = np.array([target.x, target.y, target.z])

        # wait for the arm to complete the movement
        dist = sys.float_info.max
        while dist > 30:
            pose = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
            dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - target_np)
            # print("dist", dist)

        # rekease block
        self.actions["put_down"]["queue"].put("PUT_DOWN: Initiating release")
        self.kinovaarm_proxy.closeGripper()
        time.sleep(3)

        # move to initial position
        self.kinovaarm_proxy.setCenterOfTool(current_pose, RoboCompKinovaArm.ArmJoints.base)
        current_pose_np = np.array([current_pose.x, current_pose.y, current_pose.z])
        self.actions["put_down"]["queue"].put("PUT_DOWN: Tip sent to initial position")
        dist = sys.float_info.max
        while dist > 50:  # mm
            pose = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
            dist = LA.norm(np.array([pose.x, pose.y, pose.z]) - current_pose_np)

        self.actions["put_down"]["queue"].put("PUT_DOWN: Finish")
        return True

    def stack(self, x, y):
        pass

    def unstack(self, x, y):
        pass

    # ===================================================================
    def read_camera(self):
        all = self.camerargbdsimple_proxy.getAll(self.camera_name)
        color = np.frombuffer(all.image.image, np.uint8).reshape(all.image.height, all.image.width, all.image.depth)
        depth = np.frombuffer(all.depth.depth, np.float32).reshape(all.depth.height, all.depth.width)
        return color, depth, all

    def draw_image(self, color):
        cv2.imshow("Gen3", color)
        cv2.waitKey(1)

    def compute_april_tags(self, color, depth):
        tags = self.detector.detect(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY))
        if len(tags) > 0:
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv2.line(color, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
                    cv2.putText(color, str(tag.tag_id),
                               org = (tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                               fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255))
                    cv2.rectangle(color, tuple(tag.center.astype(int) - 1), tuple(tag.center.astype(int) + 1), (255, 0, 255))
        else:
            #print("Compute_april_tags: No tags detected")
            pass

        return color, tags

    # def observar(self):
    #     pos = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
    #     pos.x = pos.x + 5
    #     pos.y = pos.y
    #     pos.z = 0
    #     self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)
    #
    # def posicionInicial(self):
    #     try:
    #         pose = RoboCompCoppeliaUtils.PoseType()
    #         pose.x = self.posicion_inicial.x
    #         pose.y = self.posicion_inicial.y
    #         pose.z = self.posicion_inicial.z
    #         pose.rx = self.posicion_inicial.rx
    #         pose.ry = self.posicion_inicial.ry
    #         pose.rz = self.posicion_inicial.rz
    #         #print(self.posicion_inicial)
    #         self.coppeliautils_proxy.addOrModifyDummy(RoboCompCoppeliaUtils.TargetTypes.Info, "target", pose)
    #     except:
    #         pass
    #
    # def mover(self, box):
    #     pos = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
    #     pos.x = pos.x - box[0] / 100
    #     pos.y = pos.y - box[1] / 100
    #     pos.z = 0
    #     self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)
    #
    # def pinza(self, centre,obj):
    #     tip = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
    #     pos = RoboCompKinovaArm.TPose()
    #     self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)

    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # self.camerargbdsimple_proxy.getAll(...)
    # self.camerargbdsimple_proxy.getDepth(...)
    # self.camerargbdsimple_proxy.getImage(...)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # self.kinovaarm_proxy.closeGripper(...)
    # self.kinovaarm_proxy.getCenterOfTool(...)
    # self.kinovaarm_proxy.openGripper(...)
    # self.kinovaarm_proxy.setCenterOfTool(...)
    # self.kinovaarm_proxy.setGripper(...)

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)


# self.future = concurrent.futures.Future()
# print(self.future.running(), self.future.done())
# if not self.future.running():
#     print("if1")
#     with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
#         self.future = executor.submit(self.pick_up, tx[0].tag_id, tx_pose)
