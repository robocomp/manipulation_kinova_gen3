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
    Manages and processes video data from two sources, color and depth, using
    separate threads. It provides methods for getting images or depth information
    from the queues and returning them as iface objects.

    Attributes:
        Period (int): 100 by default, which represents the time interval between
            video frames captured by the camera.
        hide (instance): Used to hide the worker object from the Python interpreter,
            preventing it from being garbage collected.
        depth_queue (queueQueue): Used to store depth images from the camera stream.
        color_queue (queueQueue): Used to store the color frames captured by the
            camera. It is used to feed the color frames to the video display thread.
        color_stream (cv2VideoCapture): A video capture object for capturing color
            images.
        depth_stream (cv2VideoCapture): Used to capture depth images from a RTSP
            stream.
        color_thread (Thread): Used to represent a thread that runs in parallel
            to the main thread, handling the color stream of the camera.
        video_color_thread (threadingThread): Responsible for reading color frames
            from the video capture and adding them to a queue for processing.
        depth_thread (QtCoreQRunnable): Used to manage a separate thread for
            processing depth images.
        video_depth_thread (threadingThread): Responsible for reading depth frames
            from a video capture device and storing them in a queue for processing.
        startup_check (Python): A method that checks if the RoboCompCameraRGBDSimple
            interface is available and functions correctly.
        timer (QTimer): Used to schedule a function call at a specific interval
            (in this case, every 100 milliseconds).
        compute (Python): Defined as a slot that will be called at regular intervals,
            as determined by the `Timer` object passed to its constructor. It has
            no effect on the code's behavior.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes an instance of the SpecificWorker class by setting up video
        capture streams for color and depth, creating threads to process the
        streams, and starting a timer for computing depth values.

        Args:
            proxy_map (dict): Used to store the proxy map for the SpecificWorker
                class.
            startup_check (bool): Used to check if the thread-based video reader
                has already been started.

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
        Computes and returns a boolean value based on the visibility of an object.

        """
        pass

################################################################################################################
    def video_color_thread(self, cap, inqueue):
        """
        Reads frames from a video capture object `cap` and puts them into a queue
        `inqueue`. It continuously reads frames until the video is closed or an
        interrupt is encountered, at which point it prints "hilo finish" and
        releases the capture.

        Args:
            cap (OpenCVVideoCapture): Used to capture video frames from a video
                file or device.
            inqueue (queue): Used to store images read from the camera in a
                non-blocking manner.

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
        Reads frames from an opened Capture object `cap` and puts them into a queue
        `inqueue` until the queue is full, then it waits for the thread to retrieve
        the frame from the queue and repeats the process.

        Args:
            cap (OpenCVVideoCapture): Used to capture video frames from a video source.
            inqueue (queue): Used for putting frames from the camera into the queue
                for processing.

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
        Performs startup checks for a RoboCompCameraRGBDSimple instance by printing
        the class name and creating instances of its child classes (TImage, TDepth,
        TRGBD). It also schedules the application to quit after 200 milliseconds.

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
        Retrieves and combines depth and color information from queues within the
        `SpecificWorker` class, returning a complete RGB-D image with alivetime metadata.

        Args:
            camera (ifacesRoboCompCameraRGBDSimple): Used to represent a RoboComp
                camera object.

        Returns:
            RoboCompCameraRGBDSimple: A RGBD image with depth and color information.

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
            print(ret.image.alivetime)
            return ret
        except queue.Empty:
            return None
    #
    # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getDepth(self, camera):
        """
        Retrieves an RGB-D image from a depth queue and returns the corresponding
        depth value.

        Args:
            camera (ifacesRoboCompCameraRGBDSimple): A reference to a camera object.

        Returns:
            ifacesRoboCompCameraRGBDSimpleTDepth: A struct containing height,
            width, and depth values of a image.

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
        height, width, and depth properties set to the image's dimensions, as well
        as the actual image data.

        Args:
            camera (ifacesRoboCompCameraRGBDSimple): Passed as an instance of RoboCompCameraRGBDSimple.

        Returns:
            ifacesRoboCompCameraRGBDSimpleTImage: A struct containing height, width
            and depth of an image along with the image itself.

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


