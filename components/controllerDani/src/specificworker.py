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

import time
import apriltag
import cv2 as cv
import numpy as np
from PIL import Image
from bloques import *
from genericworker import *
import matplotlib.pyplot as plt
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))
        self.camera_name = "camera_arm"
        self.state = "IDLE"
        self.cogido = False
        self.posicion_inicial = []
        self.posicion_bloque = []
        self.estado_inicial = {}
        self.estafo_final = {}
        self.plan = []
        self.fixer_plan = []
        self.logica = MundoBloques()

    def __del__(self):
        print('SpecificWorker destructor')

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)
    
    def setParams(self, params):
        return True

    @QtCore.Slot()
    def compute(self):
        """APRILTAGS"""
        all = self.camerargbdsimple_proxy.getAll(self.camera_name)
        aux = self.camerargbdsimple_proxy.getImage(self.camera_name)
        image, tags = self.process_image(aux)
        # cv.imshow("Apriltags", cv.resize(image, (720, 460)))
        # TODO : Pasar la imagen resultado al widget en vez de plotearla

        """MÁQUINA DE ESTADOS: PRUEBA"""
        # self.state = state_machine(self.state)

        """COGER: PRUEBA"""
        if self.posicion_inicial == []:
            self.posicion_inicial = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        if self.posicion_bloque == []:
            self.posicion_bloque = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
            self.posicion_bloque.x = tags[0].center[0]
            self.posicion_bloque.y = tags[0].center[1]
        print(self.posicion_bloque)

        pos = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        if pos != self.posicion_bloque:
            self.kinovaarm_proxy.setCenterOfTool(self.posicion_bloque, RoboCompKinovaArm.ArmJoints.base)
            self.kinovaarm_proxy.openGripper()
        elif not self.cogido:
            self.kinovaarm_proxy.closeGripper()
            time.sleep(4)
            self.cogido = True
        else:
            self.kinovaarm_proxy.setCenterOfTool(self.posicion_inicial, RoboCompKinovaArm.ArmJoints.base)
        
        return True


    ###########################################################
    ###########################################################
    ####                                                   ####
    ####   Una vez funcione una primitiva simple (coger)   ####  
    ####                                                   ####
    ###########################################################
    ###########################################################

    # ========= UI =========
    """
    Interfaz que permita generar un estado final, deslizando los bloques para colocarlos de la forma deseada 
    """

    # ======================= Máquina de estados =======================
    def idle(self):
        """
        Si el estado final se ha creado en la UI, extraerlo y guardarlo
        """
        # if hayEstadoFinal: 
        #   self.estado_final = obtenerEstadoFinal
        #   return "INIT"
        # else
        #   return "IDLE"
        pass

    def init(self):
        """
        Desplaza el brazo a su posición inicial, observa el entorno y genera el estado inicial
        """
        # if pos_actual != pos_observacion:
        #   self.move_to_pos(pos_observacion)
        # else:
        #   coger imagen y procesarla
        #   si numero de bloques == numero esperado:
        #     self.estado_actual = self.create_state()
        #     return "PLANNING"
        #   else:
        #     print("NO SE DETECTAN TODOS LOS BLOQUES")
        #     return "INIT"     

        pass

    def planning(self):
        """
        Genera el plan para pasar del inical al final
        """
        # self.plan = self.generate_plan(Self.estado_inicial, self.estado_final)
        # return "CHECK"
        pass

    def check(self):
        """
        Comprueba que el estado actual logico coincide con el físico.
        En caso negativo, se planifica para hacer que coincidan ambos estados.
        """

        pass

    def move(self):
        """
        Hace el siguiente movimiento
        """
        pass

    state_machine = {"IDLE": idle, "INIT": init, "PLANNING": planning, "CHECK": check, "MOVE": move}


    # ==================== Procesamiento de imágenes ====================

    def process_image(self, aux):
        """
        Se extraen de la imagen los datos relacionados con la lectura de las apriltags leídas en la misma.
        Devuelve la imagen marcando las apriltags con su ID y un cuadrado
        """
        image = np.frombuffer(aux.image, np.uint8).reshape(aux.height, aux.width, aux.depth)
        tags = self.detector.detect(cv.cvtColor(image, cv.COLOR_BGR2GRAY))
        if len(tags) > 0:
            for tag in tags:
                for idx in range(len(tag.corners)):
                    cv.line(image, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
                    cv.putText(image, str(tag.tag_id), org=(tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
                        fontFace=cv.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255))
                    cv.rectangle(image, tuple(tag.center.astype(int)-1), tuple(tag.center.astype(int)+1), (255, 0, 255))
        else:
            print("No tags detected")
        # TODO: transformas la posicion de las tags si fuese necesario
        return image, tags


    # ========== Funciones físicas de interacción con bloques ==========
    def create_state(self):
        """
        Observa los cubos en el mundo y codifica el estado del conjunto
        """
        pass

    def generate_plan(self, inicia, final):
        """
        Genera los pasos necesarior para ir del estado inicial al estado final
        """
        pass

    def coger(self, bloque):
        """
        Se coge el bloque especificado de la mesa
        """
        self.move_to_pos(bloque.center)
        pass

    def dejar(self, bloque):
        """
        Se deja el bloque especificado en la mesa
        """
        pass

    def apilar(self, bloque, destino):
        """
        Se deja el bloque especificado encima del bloque destino
        """
        pass

    def desapilar(self, bloque, fuente):
        """
        Se coge el bloque especificado, que está situado encima del bloque fuente
        """
        pass

    # =============== Funciones de movimiento del robot ===============
    def grip_block(self):
        """
        Coge el bloque que tiene delante
        """
        pass

    def move_to_pos(self, posicion):
        """
        Mueve el brazo y coloca la mano en la posición especificada
        """
        pass

    # def posicionInicial(self):
    #     try:
    #         pose = RoboCompCoppeliaUtils.PoseType()
    #         pose.x = self.posicion_inicial.x
    #         pose.y = self.posicion_inicial.y
    #         pose.z = self.posicion_inicial.z
    #         pose.rx = self.posicion_inicial.rx
    #         pose.ry = self.posicion_inicial.ry
    #         pose.rz = self.posicion_inicial.rz
    #         # print(self.posicion_inicial)
    #         self.coppeliautils_proxy.addOrModifyDummy(RoboCompCoppeliaUtils.TargetTypes.Info, "target", pose)
    #     except:
    #         pass

    """
    Funciones de las interfaces
    """
    # From the RoboCompCameraRGBDSimple you can call this methods:
    #  - self.camerargbdsimple_proxy.getAll(...)
    #  - self.camerargbdsimple_proxy.getDepth(...)
    #  - self.camerargbdsimple_proxy.getImage(...)

    # From the RoboCompCameraRGBDSimple you can use this types:
    #  - RoboCompCameraRGBDSimple.TImage
    #  - RoboCompCameraRGBDSimple.TDepth
    #  - RoboCompCameraRGBDSimple.TRGBD

    # From the RoboCompKinovaArm you can call this methods:
    #  - self.kinovaarm_proxy.closeGripper(...)
    #  - self.kinovaarm_proxy.getCenterOfTool(...)
    #  - self.kinovaarm_proxy.openGripper(...)
    #  - self.kinovaarm_proxy.setCenterOfTool(...)
    #  - self.kinovaarm_proxy.setGripper(...)
    
    """
    Uso de funciones
    """
    # pos = RoboCompKinovaArm.TPose()
    # pos = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
    # self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)
    # self.kinovaarm_proxy.closeGripper()
    # self.kinovaarm_proxy.openGripper()

