#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by Jorge Calderón
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
import sys

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from orca.debug import pidOf
from rich.console import Console
from genericworker import *
import interfaces as ifaces

#import multiprocessing as mp
import threading
import queue
import time
#import vid_streamv32 as vs
import cv2
import numpy as np

console = Console(highlight=False)

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]
        self.hide()

        #set  queue size
        self.depth_queue = queue.Queue(maxsize=1)
        self.color_queue = queue.Queue(maxsize=1)
        self.init_timestamp = int(time.time()*1000)

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

        self.killThreads()
        self.color_stream.release()
        self.depth_stream.release()

        print("Destructor")


    def setParams(self, params):
        self.ip = params["ip"]
        self.run = True
        # create the video capture objects
        print(f"Connecting to {self.ip}")
        # self.color_stream = cv2.VideoCapture(
        #     f"gst-launch-1.0 rtspsrc location=rtsp://{self.ip}/color latency=30 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true",
        #     cv2.CAP_GSTREAMER)
        print("Antes de intentar abrir")
        self.color_stream = cv2.VideoCapture(
            f"gst-launch-1.0 rtspsrc location=rtsp://{self.ip}/color latency=30 ! rtph264depay ! h264parse ! nvh264dec ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true",
            cv2.CAP_GSTREAMER)
        print("Despues de intentar abrir", self.color_stream.isOpened())

        # self.depth_stream = cv2.VideoCapture(
        #     f"gst-launch-1.0 rtspsrc location=rtsp://{self.ip}/depth latency=30 ! rtpgstdepay ! videoconvert n-threads=2 ! video/x-raw,format=GRAY16_LE ! queue ! appsink drop=true",
        #     cv2.CAP_GSTREAMER)
        # print(self.depth_stream.isOpened())

        # create a thread to capture the stream and start it
        self.color_thread = threading.Thread(target=self.video_color_thread, args=(self.color_stream, self.color_queue))
        if (not self.color_stream.isOpened()):
            print("color stream not opened")
            sys.exit()
        self.color_thread.start()

        # self.depth_thread = threading.Thread(target=self.video_depth_thread, args=(self.depth_stream, self.depth_queue))
        # if (not self.depth_stream.isOpened()):
        #     print("depth stream not opened")
        #     sys.exit()
        # self.depth_thread.start()

        print("Reading threads started")

        return True


    @QtCore.Slot()
    def compute(self):
        # print('SpecificWorker.compute...')
        #
        # color_frame = self.color_queue.get()
        # color_frame = cv2.resize(color_frame, (480, 270))
        # qt_color = QImage(color_frame, color_frame.shape[1], color_frame.shape[0], QImage.Format_RGB888)
        # pix_color = QPixmap.fromImage(qt_color).scaled(self.ui.color.width(), self.ui.color.height())
        # self.ui.color.setPixmap(pix_color)
        #
        # depth_frame = self.depth_queue.get()
        # depth_frame = cv2.resize(depth_frame, (480, 270))
        # qt_depth = QImage(depth_frame, depth_frame.shape[1], depth_frame.shape[0], QImage.Format_Grayscale16)
        # pix_depth = QPixmap.fromImage(qt_depth).scaled(self.ui.depth.width(), self.ui.depth.height())
        # self.ui.depth.setPixmap(pix_depth)
        #
        # if self.isHidden():
        #     sys.exit()
        #return True
        pass

################################################################################################################
    def video_color_thread(self, cap, inqueue):
        try:
            while cap.isOpened() and self.run:
                # print("color", int(time.time()*1000)-self.init_timestamp)
                ret, frame = cap.read()
                if ret:
                    # inqueue.put_nowait(frame)
                    try:
                        # Intenta agregar la imagen a la cola sin bloquear
                        inqueue.put_nowait(frame)
                    except queue.Full:
                        # Si la cola está llena, descarta la imagen más antigua y agrega la nueva
                        inqueue.get_nowait()
                        inqueue.put_nowait(frame)
            print("color finish")
        except KeyboardInterrupt:
            print("hilo finish")
        cap.release()

    def video_depth_thread(self, cap, inqueue):
        #print("inicio bucle")
        try:
            while cap.isOpened() and self.run:
                print("depth", int(time.time()*1000)-self.init_timestamp)
                ret, frame = cap.read()
                if ret:
                    # inqueue.put_nowait(frame)
                    try:
                        # Intenta agregar la imagen a la cola sin bloquear
                        inqueue.put_nowait(frame)
                    except queue.Full:
                        # Si la cola está llena, descarta la imagen más antigua y agrega la nueva
                        inqueue.get_nowait()
                        inqueue.put_nowait(frame)
            print("depth finish")
        except KeyboardInterrupt:
            print("hilo finish")
        cap.release()

    def killThreads(self):
        self.run = False
        self.color_thread.join()
        self.depth_thread.join()
        print("threads killed")


    def startup_check(self):
        print(f"Testing RoboCompCameraRGBDSimple.Point3D from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.Point3D()
        print(f"Testing RoboCompCameraRGBDSimple.TPoints from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TPoints()
        print(f"Testing RoboCompCameraRGBDSimple.TImage from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TImage()
        print(f"Testing RoboCompCameraRGBDSimple.TDepth from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TDepth()
        print(f"Testing RoboCompCameraRGBDSimple.TRGBD from ifaces.RoboCompCameraRGBDSimple")
        test = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getAll method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getAll(self, camera):
        try:
            ret = ifaces.RoboCompCameraRGBDSimple.TRGBD()
            #ret.depth.depth = cv2.resize(self.depth_queue.get(), (480, 270))
            ret.depth.depth = self.depth_queue.get_nowait()
            ret.depth.height, ret.depth.width = ret.depth.depth.shape
            ret.depth.alivetime = int(time.time()*1000)
            #ret.image.image = cv2.resize(self.color_queue.get(), (480, 270))
            ret.image.image = self.color_queue.get_nowait()
            ret.image.height, ret.image.width, ret.image.depth = ret.image.image.shape
            ret.image.alivetime = int(time.time()*1000)
            print("get all")
            return ret
        except queue.Empty:
            print("Empty queue")
            return None
    #
    # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getDepth(self, camera):
        try:
            img = self.depth_queue.get()
            # img = cv2.resize(img, (480, 270))
            ret = ifaces.RoboCompCameraRGBDSimple.TDepth()
            ret.height, ret.width = img.shape
            ret.depth = img

            return ret
        except queue.Empty:
            return None
    #
    # IMPLEMENTATION of getImage method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getImage(self, camera):
        try:
            img = self.color_queue.get()
            # img = cv2.resize(img, (480, 270))
            ret = ifaces.RoboCompCameraRGBDSimple.TImage()
            ret.height, ret.width, ret.depth = img.shape
            ret.image = img
            ret.alivetime = int(time.time()*1000)

            return ret
        except queue.Empty:
            return None

    #
    # IMPLEMENTATION of getPoints method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getPoints(self, camera):

        #
        # write your CODE here
        #
        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # ifaces.RoboCompCameraRGBDSimple.Point3D
    # ifaces.RoboCompCameraRGBDSimple.TPoints
    # ifaces.RoboCompCameraRGBDSimple.TImage
    # ifaces.RoboCompCameraRGBDSimple.TDepth
    # ifaces.RoboCompCameraRGBDSimple.TRGBD


