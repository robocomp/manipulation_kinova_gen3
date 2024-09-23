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
    """
    Captures and processes RGB-D video streams from a camera, storing frames in
    queues for further processing or retrieval by other components. It manages
    threads for color and depth stream capture and provides methods to retrieve
    images and depth data.

    Attributes:
        Period (int): 1000, indicating a period or interval (in milliseconds) for
            various operations to be performed by the worker, such as connecting
            to streams and processing frames.
        hide (bool): Set to `self.hide()` in the `__init__` method, but its exact
            purpose or effect is not clear without additional context.
        depth_queue (queueQueue[int]): Initialized with a maximum size of 1, which
            allows it to store one frame from the depth stream at a time.
        color_queue (Queue[Any]): Used to hold color frames from a video stream,
            with a maximum size of one frame at a time. It follows first-in-first-out
            (FIFO) order.
        init_timestamp (int): Initialized with the current time (in milliseconds)
            at the instance creation, obtained using `int(time.time()*1000)`.
        startup_check (bool): Used to determine whether a startup check should be
            performed when initializing the worker. The check tests various data
            structures from the RoboComp library.
        timer (QTimer): Connected to the compute method, which it calls every
            Period milliseconds.
        compute (None|Callable[[],None]): Annotated with @QtCore.Slot(). It
            represents a slot that gets called when a timer event occurs. The
            function does not contain any operation, it only passes.

    """
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes object properties, such as time period and queues for depth
        and color data, sets up event handling, and starts the worker process upon
        initialization or startup check.

        Args:
            proxy_map (Dict[str, Any]): Passed to the parent class's constructor
                using `super(SpecificWorker, self).__init__(proxy_map)`, indicating
                it serves as a configuration or setup map.
            startup_check (bool): Optional, with a default value of False. It
                determines whether to run startup checks or not when the worker
                is initialized.

        """
        super(SpecificWorker, self).__init__(proxy_map)
        self.Period = 1000
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
        """
        Initializes parameters for video stream capture, starts capturing color
        stream from an IP address, and creates a thread to process the captured
        frames. It also checks if the stream is opened successfully before proceeding.

        Args:
            params (Dict[str, str | int | bool]): Expected to contain key-value
                pairs representing various settings such as IP address, latency
                and others necessary for connecting to an RTSP stream.

        Returns:
            bool: Set to True when the execution is successful, indicating that
            the video streams have been successfully opened and a thread has been
            created to capture them.

        """
        self.ip = params["ip"]
        self.run = True
        # create the video capture objects
        print(f"Connecting to {self.ip}")
        # self.color_stream = cv2.VideoCapture(
        #     f"gst-launch-1.0 rtspsrc location=rtsp://{self.ip}/color latency=30 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true",
        #     cv2.CAP_GSTREAMER)
        self.color_stream = cv2.VideoCapture(
            f"gst-launch-1.0 rtspsrc location=rtsp://{self.ip}/color latency=30 ! rtph264depay ! h264parse ! nvh264dec ! videoconvert n-threads=2 ! video/x-raw,format=BGR ! queue ! appsink drop=true",
            cv2.CAP_GSTREAMER)

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
        """
        Computes color and depth pixel values, but these lines are currently
        commented out, rendering them inactive. The method also checks if the
        worker is hidden and returns True immediately if so. This could be used
        to implement lazy computation.

        """
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
        """
        Captures video frames from an OpenCV camera, puts them into a queue, and
        releases the camera when it finishes or is interrupted by a keyboard signal.

        Args:
            cap (cv2.VideoCapture): Referenced as an object, likely representing
                a video capture device or a video file. It provides functionality
                for reading frames from the captured video.
            inqueue (Queue[Any]): Used to store frames from the camera for further
                processing, allowing for efficient handling of video data and
                preventing buffer overflows due to full queue condition.

        """
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
        """
        Reads frames from an opened camera capture object (cap), converts them
        into depth data, and pushes them into a shared queue for processing. It
        runs until the camera is closed or the thread is stopped.

        Args:
            cap (cv2.VideoCapture | None): A video capture object that provides
                read access to video frames from the device's camera or other video
                source.
            inqueue (Queue): Used to store frames from the camera capture (`cap`)
                as they are read, enabling thread-safe input queuing.
                
                Used for handling full queue conditions by removing an item before
                adding the new one.

        """
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
        """
        Terminates two threads, color_thread and depth_thread, when its run variable
        is set to False, allowing the program to exit safely by joining these
        threads before proceeding.

        """
        self.run = False
        self.color_thread.join()
        self.depth_thread.join()
        print("threads killed")


    def startup_check(self):
        """
        Tests three types of images from the ifaces module, printing a message for
        each. After testing, it schedules a single shot to quit the application
        after 200 milliseconds using QTimer and QApplication's instance.

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
        Retrieves data from two queues and returns it in a structured format,
        specifically an instance of TRGBD from the RoboComp library. It handles
        exceptions where either queue is empty.

        Args:
            camera (CameraRGBDSimple): Not used within the function itself. It
                seems to be an unused parameter possibly intended for future use
                or method overriding.

        Returns:
            TRGBD|None: An object containing depth and image data along with their
            corresponding timestamps, unless a queue is empty. In that case it
            returns None.

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
            print("get all")
            return ret
        except queue.Empty:
            print("Empty queue")
            return None
    #
    # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getDepth(self, camera):
        """
        Dequeues an image from a queue, extracts its dimensions and pixel data,
        and returns them as a TDepth object representing depth information.

        Args:
            camera (RoboCompCameraRGBDSimple): Passed to the method as an argument,
                however its usage within the method is unclear and appears redundant
                since it's not used.

        Returns:
            ifacesRoboCompCameraRGBDSimpleTDepth: A structured data object containing
            three elements: ret.height, ret.width and ret.depth where ret.height
            and ret.width represent the image dimensions and ret.depth represents
            the depth image itself.

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
        Retrieves an image from a color queue, packages it into a TImage object,
        and returns the packaged image along with its height, width, depth, and
        alivetime stamp. It handles empty queues by returning None.

        Args:
            camera (RoboCompCameraRGBDSimple.Camera): Used to pass a camera object
                to the function.

        Returns:
            RoboCompCameraRGBDSimpleTImage: An object containing image data along
            with its dimensions, alive time and the image itself represented as a
            numpy array.

        """
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
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD


