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
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from yolov4 import Detector
from PIL import Image
import time

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        # image
        self.image = []
        self.depth = []
        self.camera_name = "camera_arm"
        self.d = Detector(gpu_id=0)
        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
        self.position = 0
        self.objetos = {}
        self.centros= False
        self.cogido = False
        self.ejecucion = False
        self.posicion_inicial = []
        self.logCogidos = []
        self.bajando = True
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        return True


    @QtCore.Slot()
    def compute(self):
        if not self.posicion_inicial:
            self.posicion_inicial = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        all = self.camerargbdsimple_proxy.getAll(self.camera_name)
        aux = self.camerargbdsimple_proxy.getImage(self.camera_name)
        detections = self.yolo(all.image,all.depth)
        self.coger(detections, all.image)
        return True

    # ===================================================================
    def draw_image(self, color_,all):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        plt.figure(1)
        plt.clf()
        plt.imshow(color)
        plt.title('Front Camera ')
        plt.pause(.1)

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def yolo(self, color_,depth_):

        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        depth = np.frombuffer(depth_.depth, np.float32).reshape(depth_.height, depth_.width)

        img = Image.fromarray(color)
        img_arr = np.array(img.resize((self.d.network_width(), self.d.network_height())))
        detections = self.d.perform_detect(image_path_or_buf=img_arr, show_image=False)
        for detection in detections:
            if detection.class_confidence * 100 > 60:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')
            else:
                pass
                #print("Objeto no detectado")
        print("-"*100)

        return detections

    def procesarImagen(self, color_):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        color = cv.medianBlur(color, 5)
        color = cv.cvtColor(color, cv.COLOR_BGR2GRAY)
        cimg = cv.cvtColor(color, cv.COLOR_GRAY2BGR)
        circles = cv.HoughCircles(color, cv.HOUGH_GRADIENT, 1, 20, param1=60, param2=60, minRadius=0,maxRadius=0)
        centre = []
        if type(circles) is not type(None):
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)
                centre.append(i[0])
                centre.append(i[1])
                centre.append(i[2])
            #print(centre[0], centre[1])
        plt.figure(1)
        plt.clf()
        plt.imshow(cimg)
        plt.title('Front Camera ')
        plt.pause(.1)
        plt.imshow(cimg)
            #self.draw_image(cimg)
        return centre

    def posiciones(self, box):
        """ eje x: 640
            eje y: 480"""
        """
            si moverse en x, y primer cuadrante:  - -
            si moverse en x, y segundo cuadrante:  + -
            si moverse en x, y tercero cuadrante:  + +
            si moverse en x, y cuarto cuadrante:  - +
        """
        y, x = box[0], box[1]
        #print(x,y)
        if y < 320:
            box[0] = -y
        if x >= 240:
            box[1] = -x
        return box


    def coger(self, detections, image):
        centre = []
        if self.ejecucion == False:
            for detection in detections:
                box = detection.top_y+detection.width/4, detection.left_x
                if detection.class_name == 'cup':
                    if 'cup' not in self.logCogidos:
                        self.objetos[detection.class_name] = box
                        self.ejecucion = True
                if detection.class_name == 'knife':
                    if 'knife' not in self.logCogidos:
                        self.objetos[detection.class_name] = box
                        self.ejecucion = True
                if detection.class_name == 'spoon':
                    if 'spoon' not in self.logCogidos:
                        self.objetos[detection.class_name] = box
                        self.ejecucion = True
        if self.ejecucion == False:
            #self.observar()
            pass
        if "cup" not in self.logCogidos and "cup" in self.objetos.keys():
            self.cup(image)
        elif "spoon" not in self.logCogidos and "spoon" in self.objetos.keys():
            self.spoon(image)
        elif "knife" not in self.logCogidos and "knife" in self.objetos.keys():
            self.ejecucion = True
            self.knife(image)
            if self.cogido:
                self.pinza([],"knife")
    def cup(self,image):
        print("estoy en cup")
        centre = self.procesarImagen(image)
        if centre or self.centros==True:
            self.centros=True
            self.pinza(centre,"cup")
        elif self.centros==False:
            self.mover(self.objetos["cup"])

    def spoon(self, image):
        print("estoy en spoon")
        pos = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        box=self.objetos["spoon"]
        if pos.x - box[0] / 100<0.1:
            pos.x = pos.x + box[0] / 100
        else:
            pos.x=0
        pos.y = pos.y - box[1] / 100
        pos.z = 0
        self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)
    def knife(self, image):
        print("estoy en knife")
        pos = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        box = self.objetos["knife"]
        pose = RoboCompCoppeliaUtils.PoseType()
        pose.x = pos.x
        pose.y = pos.y
        pose.z = pos.z
        print(pos.z)
        pose.rx = self.posicion_inicial.rx
        pose.ry = self.posicion_inicial.ry
        pose.rz = self.posicion_inicial.rz
        lineas = self.lineas(image)
        if abs(pos.y) < box[1]/1000-0.18 or abs(pos.y) > box[1]/1000+0.18:
            pose.x = pos.x - box[0]/100000
            pose.y = pos.y - box[1]/100000
            pose.z = self.posicion_inicial.z
        elif abs(pos.x) < box[0]/1000-0.12 or abs(pos.x) > box[0]/1000+0.12:
            pose.x = pos.x - box[0]/10000
            pose.y = pos.y
            pose.z = self.posicion_inicial.z
        elif abs(pos.z) > 0.001:
            print("tamaño")
            print(pos.z)
            pose.x = pos.x
            pose.y = pos.y
            pose.z = pos.z - 0.0025
        else:
            self.kinovaarm_proxy.closeGripper()
            time.sleep(4)
            self.cogido = True
        self.coppeliautils_proxy.addOrModifyDummy(RoboCompCoppeliaUtils.TargetTypes.Info, "target", pose)


    def lineas(self,image):
        color = np.frombuffer(image.image, np.uint8).reshape(image.height, image.width, image.depth)
        color = cv.medianBlur(color, 5)
        color = cv.cvtColor(color, cv.COLOR_BGR2GRAY)
        cimg = cv.cvtColor(color, cv.COLOR_GRAY2BGR)
        edges = cv.Canny(color, 50, 150, apertureSize=3)
        lines = cv.HoughLines(edges, 1, np.pi / 180, 175)
        if lines is not None:
            print(lines)
            for rho, theta in lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))

                cv.line(cimg, (x1, y1), (x2, y2), (0, 0, 255), 2)

            plt.figure(1)
            plt.clf()
            plt.imshow(cimg)
            plt.title('Front Camera ')
            plt.pause(.1)
            plt.imshow(cimg)

        return lines

    def observar(self):
        pos = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        pos.x = pos.x + 5
        pos.y = pos.y
        pos.z = 0
        self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)
    def posicionInicial(self):
        try:
            pose = RoboCompCoppeliaUtils.PoseType()
            pose.x = self.posicion_inicial.x
            pose.y = self.posicion_inicial.y
            pose.z = self.posicion_inicial.z
            pose.rx = self.posicion_inicial.rx
            pose.ry = self.posicion_inicial.ry
            pose.rz = self.posicion_inicial.rz
            #print(self.posicion_inicial)
            self.coppeliautils_proxy.addOrModifyDummy(RoboCompCoppeliaUtils.TargetTypes.Info, "target", pose)
        except:
            pass
    def mover(self, box):
        pos = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)

        pos.x = pos.x - box[0] / 100
        pos.y = pos.y - box[1] / 100
        pos.z = 0
        self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)

    def pinza(self, centre,obj):
        tip = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        pos = RoboCompKinovaArm.TPose()

        if self.cogido == False:
            if centre:
                pos.x=tip.x - (centre[0])/400
                pos.y=tip.y - (centre[1])/400
                print(pos.x-tip.x)
                print(pos.y-tip.y)
                pos.z=0
            elif tip.z > 0.07:
                pos.x=0
                pos.y=0
                pos.z=-2
            else:
                pos.x=0
                pos.y=0
                pos.z=0
                self.kinovaarm_proxy.closeGripper()
                time.sleep(4)
                self.cogido = True

        else:
            if tip.z<0.5:
                pos.x = 0
                pos.y = 0
                pos.z = 15
            else:
                self.kinovaarm_proxy.openGripper()
                time.sleep(4)
                self.cogido=False
                self.logCogidos.append(obj)
                self.ejecucion = False
                self.posicionInicial()
        self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)

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