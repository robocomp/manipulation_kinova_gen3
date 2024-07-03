#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2022 by YOUR NAME HERE
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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
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
    """
    Manages two video streams (color and depth) from a robotic camera, queues the
    frames for processing, and provides methods to retrieve the frames as `TImage`
    or `TDepth` objects.

    Attributes:
        Period (int): 100, which represents the interval between consecutive updates
            of the worker's internal state.
        hide (instance): Used to hide or show the worker's GUI element when it is
            not needed, which helps improve performance by reducing flickering and
            minimizing CPU usage.
        depth_queue (queueQueue): Used to store depth images read from a video
            capture device.
        color_queue (queueQueue): Used to store the color frames read from the
            video capture devices.
        color_stream (cv2VideoCapture): Used to capture color video streams.
        depth_stream (cv2VideoCapture): Used to capture depth stream from a RTSP
            source.
        color_thread (threadingThread): Used to represent a thread that runs in
            parallel with the main thread of the program, handling the color stream
            of the camera.
        video_color_thread (threadingThread): Responsible for processing the color
            stream of the camera in a separate thread.
        depth_thread (threadingThread): Used to start a separate thread for
            processing depth images.
        video_depth_thread (threadingThread): Used to run a separate thread for
            reading depth frames from the camera.
        startup_check (algorithm): Called when the object is initialized. It tests
            if the RoboCompCameraRGBDSimple interfaces are available.
        timer (QTimer): Used to schedule a function call every `Period` milliseconds
            to update the worker's state.
        compute (Python): Defined as a slot function that is called by the timer.
            It performs no operation for now.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes member variables and starts two threads to handle video streams
        for color and depth sensors, respectively.

        Args:
            proxy_map (dict): Used to store a mapping of proxy servers for each
                worker instance, allowing for flexible configuration of proxy
                servers for different workers.
            startup_check (bool): Used to check if the worker has started correctly
                or not.

        """
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 100
        self.hide()

        #set  queue size
        self.depth_queue = queue.Queue(maxsize=1)
        self.color_queue = queue.Queue(maxsize=1)

        self.color_stream = cv2.VideoCapture(
           "gst-launch-1.0 rtspsrc location=rtsp://192.168.1.10/color latency=30 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true",cv2.CAP_GSTREAMER)

        self.depth_stream = cv2.VideoCapture(
            "gst-launch-1.0 rtspsrc location=rtsp://192.168.1.10/depth latency=30 ! rtpgstdepay ! videoconvert n-threads=2 ! video/x-raw,format=GRAY16_LE ! queue ! appsink drop=true",cv2.CAP_GSTREAMER)
        #print(self.depth_stream.isOpened())

        # create a thread to capture the stream and start it
        self.color_thread = threading.Thread(target=self.video_color_thread, args=(self.color_stream, self.color_queue))
        if(not self.color_stream.isOpened()):
            print("color stream not opened")
            sys.exit()
        self.color_thread.start()

        self.depth_thread = threading.Thread(target=self.video_depth_thread, args=(self.depth_stream, self.depth_queue))
        if(not self.depth_stream.isOpened()):
            print("depth stream not opened")
            sys.exit()
        self.depth_thread.start()

        print("Reading threads started")

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""
        print("Destructor")


    def setParams(self, params):
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
        """
        Processes two image-like objects, `pix_color` and `pix_depth`, scaling
        them to the size of the widgets `ui.color` and `ui.depth`. If the object
        is hidden, it returns `True`.

        """
        pass

################################################################################################################
    def video_color_thread(self, cap, inqueue):
        """
        Reads frames from an opened camera and adds them to a queue for processing,
        while handling exceptions and keyboard interrupts.

        Args:
            cap (Capture): Represented by the object `cap`.
            inqueue (Queue): Used to store the frames read from the video capture
                device during the video processing thread execution.

        """
        try:
            while cap.isOpened():
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
        except KeyboardInterrupt:
            print("hilo finish")
        cap.release()


    def video_depth_thread(self, cap, inqueue):
        #print("inicio bucle")
        """
        Reads frames from a given video capture object `cap` and enqueues them in
        a queue `inqueue`. It repeatedly reads frames until the `cap` is closed,
        then releases the resource.

        Args:
            cap (OpenCVVideoCapture): Used to capture video frames from a video
                file or device.
            inqueue (queueQueue): Used to store frames read from the video capture
                device.

        """
        try:
            while cap.isOpened():
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
        except KeyboardInterrupt:
            print("hilo finish")
        cap.release()


    def startup_check(self):
        """
        Tests various components of a class called `ifaces.RoboCompCameraRGBDSimple`.
        These include the `TImage`, `TDepth`, and `TRGBD` classes, as well as the
        `QApplication.instance().quit()` method.

        """
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
        """
        Retrieves RGB-D data from a camera and stores it in a queue for processing.
        It returns the entire RGB-D data or `None` if the queue is empty.

        Args:
            camera (ifacesRoboCompCameraRGBDSimple): Used to retrieve RGB-D data
                from a RoboComp camera.

        Returns:
            RoboCompCameraRGBDSimple: A RGB-D image representing the depth and
            color information of a camera.

        """
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
            return ret
        except queue.Empty:
            return None
    #
    # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getDepth(self, camera):
        """
        Retrieves depth data from a queue and returns it in the form of a `TDepth`
        object with dimensions and depth value.

        Args:
            camera (ifacesRoboCompCameraRGBDSimple): An instance of the
                RoboCompCameraRGBDSimple class, which represents a camera for depth
                sensing.

        Returns:
            ifacesRoboCompCameraRGBDSimpleTDepth: A struct containing height, width
            and depth information of a image.

        """
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
        """
        Retrieves an image from a color queue and returns a `TImage` object with
        the appropriate dimensions and contents.

        Args:
            camera (ifacesRoboCompCameraRGBDSimple): An instance of a class
                representing a camera device.

        Returns:
            ifacesRoboCompCameraRGBDSimpleTImage: A struct containing height,
            width, depth and image values of a RGB-D camera frame.

        """
        try:
            img = self.color_queue.get()
            # img = cv2.resize(img, (480, 270))
            ret = ifaces.RoboCompCameraRGBDSimple.TImage()
            ret.height, ret.width, ret.depth = img.shape
            ret.image = img

            return ret
        except queue.Empty:
            return None
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD


