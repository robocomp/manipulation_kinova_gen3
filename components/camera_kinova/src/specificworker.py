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
    def __init__(self, proxy_map, startup_check=False):
        """
        Initializes various queues, video captures, and a thread to capture color
        and depth streams simultaneously. It also starts a timer to call the
        `compute` function every `Period` milliseconds.

        Args:
            proxy_map (dict): 3DProxy class instance, which is used to initialize
                the worker object's properties and start the capture threads.
            startup_check (int): execution of an additional check at startup, which
                the function implementation has yet to reveal.

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
        pass

################################################################################################################
    def video_color_thread(self, cap, inqueue):
        """
        Captures video frames and adds them to an internal queue for processing.

        Args:
            cap (open-file object.): opencv video capture object that is used to
                read frames from a video file or camera.
                
                	* `isOpened`: This attribute indicates whether the `cap` is
                currently open or not.
                	* `read()`: The `read()` method returns a tuple containing the
                captured frame and a boolean value indicating whether the read was
                successful.
                	* `frame`: The frame data captured by the camera.
                	* `ret`: A boolean value indicating whether the read operation
                was successful.
            inqueue (queue of bytes (frame) during its usage in this provided
                function code.): queue where frames read from the capture device
                are added.
                
                	* `ret`: This variable stores the result of reading from the
                webcam using the `cap.read()` method, which can be either a tuple
                containing the frame image and an error code (i.e., `None`) or an
                error message if there was one during reading.
                	* `frame`: This variable stores the image data read from the
                webcam as a `numpy` array, with dimensions (height, width, 3) for
                a standard RGB color image.
                	* `inqueue`: This variable stores a queue of deserialized frame
                images collected from the webcam during the reading process. The
                queue can be accessed and retrieved using the `get()` method.

        """
        try:
            while cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    inqueue.put(frame)
        except KeyboardInterrupt:
            print("hilo finish")
        cap.release()


    def video_depth_thread(self, cap, inqueue):
        #print("inicio bucle")
        """
        Reads and enqueues frames from a video capture device while it is open,
        until the user interrupts the thread with `KeyboardInterrupt`. The function
        then releases the video capture device.

        Args:
            cap (opensocket client object.): 4GB Visuelle Capture device, which
                is being read and its frame data is stored in a queue `inqueue`.
                
                	* `isOpened`: A boolean attribute that indicates whether the
                camera is open or not.
                	* `read()`: A method that reads a frame from the camera and returns
                the decoded frame data as a tuple containing the frame image (`ret`)
                and its associated frame ID (`frame`).
                	* `release()`: A method that releases the camera resource,
                preventing any further reads or writes to the camera.
            inqueue (Queue.): buffer where frames are stored while being read from
                the webcam using the `cap.read()` method.
                
                	1/ `inqueue`: This is an instance of the `queue` class from the
                `queue` module. It stores a sequence of Python objects (frames)
                read from the OpenCV `cap` object using the `read()` method.
                	2/ `ret`: This is a Boolean value indicating whether the read
                operation was successful or not. If `ret` is `True`, it means that
                the function successfully read data from the input. Otherwise, it
                indicates an error occurred during the read operation.

        """
        try:
            while cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    inqueue.put(frame)
        except KeyboardInterrupt:
            print("hilo finish")
        cap.release()


    def startup_check(self):
        """
        Tests and initializes three classes from the `ifaces` module: `TImage`,
        `TDepth`, and `TRGBD`. These classes represent image, depth, and RGBD data
        from a robotic camera.

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
        Generates a dictionary object containing the RGB and depth information
        from a robotic camera. The RGB image is obtained from a queue, while the
        depth information is retrieved from the camera itself. The returned
        dictionary includes both the RGB and depth data, along with their dimensions.

        Args:
            camera (instance of `RoboCompCameraRGBDSimple`.): 3D point cloud data
                to be processed by the function.
                
                	* `TRGBD`: This is the type of the returned output, which represents
                a single frame from the RGB-D camera stream.
                	* `depth`: This attribute contains the depth map of the current
                frame, with shape `(height, width, 2)` where height and width are
                the dimensions of the frame, and 2 represents the number of color
                channels (red and blue).
                	* `image`: This attribute contains the color image of the current
                frame, with shape `(height, width, 3)` where height and width are
                the dimensions of the frame, and 3 represents the number of color
                channels (red, green, and blue).
                	* `depth_queue`: This is a queue that holds the depth map for
                each frame, which is serialized from the original RGB-D stream.
                	* `color_queue`: This is a queue that holds the color image for
                each frame, which is serialized from the original RGB-D stream.

        Returns:
            undefined: a struct containing depth and image information of a given
            size.

        """
        ret = ifaces.RoboCompCameraRGBDSimple.TRGBD()
        #ret.depth.depth = cv2.resize(self.depth_queue.get(), (480, 270))
        ret.depth.depth = self.depth_queue.get()
        ret.depth.height, ret.depth.width = ret.depth.depth.shape
        #ret.image.image = cv2.resize(self.color_queue.get(), (480, 270))
        ret.image.image = self.color_queue.get()
        ret.image.height, ret.image.width, ret.image.depth = ret.image.image.shape
        return ret
    #
    # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getDepth(self, camera):
        """
        Processes a single RGB image and returns its corresponding depth map, using
        the provided instance of `RoboCompCameraRGBDSimple`.

        Args:
            camera (3D-array with shape `(height, width, depth)`.): 2D image data
                to be processed and transformed into a depth map by the function.
                
                	* `height`: The height of the image in pixels.
                	* `width`: The width of the image in pixels.
                	* `depth`: The depth map of the image in pixels.

        Returns:
            undefined: a depth map of the input image with a resolution of (480,
            270).
            
            	* `ret`: The output is stored in a variable named `ret`.
            	* `height`: The height of the image is specified in the attribute `height`.
            	* `width`: The width of the image is specified in the attribute `width`.
            	* `depth`: The depth value of the image is set to the same value as
            the input image.

        """
        img = self.depth_queue.get()
        # img = cv2.resize(img, (480, 270))
        ret = ifaces.RoboCompCameraRGBDSimple.TDepth()
        ret.height, ret.width = img.shape
        ret.depth = img

        return ret
    #
    # IMPLEMENTATION of getImage method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getImage(self, camera):
        """
        Retrieves a single image from a color queue and resizes it to a smaller
        size while maintaining its original height and width dimensions. The
        resulting resized image is returned as an instance of the `TImage` class.

        Args:
            camera (ndarray (array-like object) of size (1,) representing a single
                color image frame.): 4D numpy array containing the color image
                data that is to be processed and returned as an instance of the
                `TImage` class.
                
                	* `height`: The height of the image in pixels.
                	* `width`: The width of the image in pixels.
                	* `depth`: The number of color channels in the image (3 for RGB).

        Returns:
            undefined: a Python image object containing the retrieved RGBD image
            data.
            
            	* `height`: The height of the output image in pixels.
            	* `width`: The width of the output image in pixels.
            	* `depth`: The depth of the output image, which is always 32 bits
            (one integer for each color channel).
            	* `image`: The actual pixel data of the output image, represented as
            a Python tensor with shape `(height, width, depth)`

        """
        img = self.color_queue.get()
        # img = cv2.resize(img, (480, 270))
        ret = ifaces.RoboCompCameraRGBDSimple.TImage()
        ret.height, ret.width, ret.depth = img.shape
        ret.image = img

        return ret
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD


