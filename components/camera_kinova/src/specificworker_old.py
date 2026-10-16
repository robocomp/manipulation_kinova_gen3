#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2026 by YOUR NAME HERE
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
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from PySide6.QtCore import QTimer, Qt, Slot
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QApplication
from genericworker import *
import interfaces as ifaces

import threading
import time
import cv2
import numpy as np


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        self.init_timestamp = int(time.time()*1000)
        self.color_stream = None
        self.depth_stream = None
        self.color_thread = None
        self.depth_thread = None
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.latest_color_timestamp = None
        self.latest_depth_timestamp = None
        self.color_frame_count = 0
        self.depth_frame_count = 0
        self.color_latency_ms = 0.0
        self.depth_latency_ms = 0.0

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

        self.timer.stop()
        self.run = False
        self.killThreads()
        if self.color_stream is not None:
            self.color_stream.release()
        if self.depth_stream is not None:
            self.depth_stream.release()

        print("Destructor")

    def closeEvent(self, event):
        self.timer.stop()
        self.run = False
        self.killThreads()
        if self.color_stream is not None:
            self.color_stream.release()
        if self.depth_stream is not None:
            self.depth_stream.release()
        self.ui = None
        super(SpecificWorker, self).closeEvent(event)


    def setParams(self, params):
        self.ip = self.configData["Arm"]["ip"]
        self.run = True
        print(f"Connecting to {self.ip}")
        color_url = f"rtsp://{self.ip}/color"
        depth_url = f"rtsp://{self.ip}/depth"

        self.color_stream = cv2.VideoCapture(color_url)
        if self.color_stream.isOpened():
            self.color_stream.set(cv2.CAP_PROP_BUFFERSIZE, 0)
        if not self.color_stream.isOpened():
            print("color stream not opened with default backend, trying GStreamer fallback")
            gst_pipeline = (
                f"rtspsrc location={color_url} protocols=tcp latency=1 drop-on-latency=true ! rtph264depay ! h264parse ! avdec_h264 ! "
                "videoconvert n-threads=1 ! video/x-raw,format=BGR ! queue leaky=2 max-size-buffers=1 ! appsink drop=true"
            )
            self.color_stream = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.color_stream.isOpened():
            print("color stream not opened")
            return False

        self.depth_stream = cv2.VideoCapture(depth_url)
        if self.depth_stream.isOpened():
            self.depth_stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.depth_stream.isOpened():
            print("depth stream not opened with default backend, trying GStreamer fallback")
            gst_pipeline = (
                f"rtspsrc location={depth_url} protocols=tcp latency=30 ! rtph264depay ! h264parse ! avdec_h264 ! "
                "videoconvert ! video/x-raw,format=GRAY8 ! queue leaky=2 max-size-buffers=1 ! appsink drop=true"
            )
            self.depth_stream = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.depth_stream.isOpened():
            print("depth stream not opened, continuing without depth")
            self.depth_stream = None

        self.color_thread = threading.Thread(
            target=self.video_color_thread,
            args=(self.color_stream,),
            daemon=True,
        )
        self.color_thread.start()

        if self.depth_stream is not None:
            self.depth_thread = threading.Thread(
                target=self.video_depth_thread,
                args=(self.depth_stream,),
                daemon=True,
            )
            self.depth_thread.start()

        print("Reading threads started")

        return True


    def _to_display_image(self, frame):
        if frame is None:
            return None

        if frame.ndim == 3:
            if frame.dtype != np.uint8:
                frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            display_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            if frame.dtype != np.uint8:
                frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            display_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

        return display_frame

    @Slot()
    def compute(self):
        if self.latest_color_frame is None and self.latest_depth_frame is None:
            return True

        if not hasattr(self, 'ui') or self.ui is None:
            self.timer.stop()
            return True

        try:
            if self.latest_color_frame is not None:
                color_frame = self._to_display_image(self.latest_color_frame)
                height, width = color_frame.shape[:2]
                image = QImage(color_frame.data, width, height, width * 3, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(image).scaled(
                    max(1, self.ui.color.width()),
                    max(1, self.ui.color.height()),
                    Qt.KeepAspectRatio,
                )
                self.ui.color.setPixmap(pixmap)

            if self.latest_depth_frame is not None:
                depth_frame = self._to_display_image(self.latest_depth_frame)
                height, width = depth_frame.shape[:2]
                image = QImage(depth_frame.data, width, height, width * 3, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(image).scaled(
                    max(1, self.ui.depth.width()),
                    max(1, self.ui.depth.height()),
                    Qt.KeepAspectRatio,
                )
                self.ui.depth.setPixmap(pixmap)

            if self.latest_color_timestamp is not None and time.time() - self.last_latency_print >= 1.0:
                self.color_latency_ms = (time.time() - self.latest_color_timestamp) * 1000.0
                print(f'color delay={self.color_latency_ms:.1f}ms')
                self.last_latency_print = time.time()
        except Exception:
            self.timer.stop()
            return True
        return True

################################################################################################################
    def video_color_thread(self, cap):
        try:
            while cap.isOpened() and self.run:
                ret, frame = cap.read()
                if ret:
                    self.latest_color_frame = frame
                    self.latest_color_timestamp = time.time()
                    self.color_frame_count += 1
                else:
                    print('No frame from color stream')
                    time.sleep(0.1)
            print("color finish")
        except KeyboardInterrupt:
            print("hilo finish")
        finally:
            cap.release()

    def video_depth_thread(self, cap):
        try:
            while cap.isOpened() and self.run:
                ret, frame = cap.read()
                if ret:
                    self.latest_depth_frame = frame
                    self.latest_depth_timestamp = time.time()
                    self.depth_frame_count += 1
                else:
                    print('No frame from depth stream')
                    time.sleep(0.1)
            print("depth finish")
        except KeyboardInterrupt:
            print("hilo finish")
        finally:
            cap.release()

    def killThreads(self):
        self.run = False
        if self.color_thread is not None and self.color_thread.is_alive():
            self.color_thread.join(timeout=1.0)
        if self.depth_thread is not None and self.depth_thread.is_alive():
            self.depth_thread.join(timeout=1.0)
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
        ret = ifaces.RoboCompCameraRGBDSimple.TRGBD()

        color_frame = self.latest_color_frame
        if color_frame is None:
            return None

        color_bytes, color_shape = self._frame_to_bytes(color_frame)
        ret.image.image = color_bytes
        ret.image.height, ret.image.width = color_shape[:2]
        ret.image.depth = color_shape[2] if len(color_shape) > 2 else 1
        ret.image.alivetime = int(time.time()*1000)

        depth_frame = self.latest_depth_frame
        if depth_frame is not None:
            depth_bytes, depth_shape = self._frame_to_bytes(depth_frame)
            ret.depth.depth = depth_bytes
            ret.depth.height, ret.depth.width = depth_shape[:2]
            ret.depth.alivetime = int(time.time()*1000)
        else:
            ret.depth.depth = b""
            ret.depth.height = 0
            ret.depth.width = 0
            ret.depth.alivetime = int(time.time()*1000)

        return ret
    #
    # IMPLEMENTATION of getDepth method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getDepth(self, camera):
        img = self.latest_depth_frame
        if img is None:
            return None

        ret = ifaces.RoboCompCameraRGBDSimple.TDepth()
        depth_bytes, depth_shape = self._frame_to_bytes(img)
        ret.height, ret.width = depth_shape[:2]
        ret.depth = depth_bytes
        ret.alivetime = int(time.time()*1000)
        return ret
    #
    # IMPLEMENTATION of getImage method from CameraRGBDSimple interface
    #
    def CameraRGBDSimple_getImage(self, camera):
        img = self.latest_color_frame
        if img is None:
            return None

        ret = ifaces.RoboCompCameraRGBDSimple.TImage()
        image_bytes, image_shape = self._frame_to_bytes(img)
        ret.height, ret.width = image_shape[:2]
        ret.depth = image_shape[2] if len(image_shape) > 2 else 1
        ret.image = image_bytes
        ret.alivetime = int(time.time()*1000)

        return ret


    def CameraRGBDSimple_getPoints(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TPoints()
        ret.points = []
        ret.alivetime = int(time.time()*1000)
        ret.period = 0.0
        ret.compressed = False
        return ret

    def _frame_to_bytes(self, frame):
        if frame is None:
            return b"", (0, 0)

        frame = np.ascontiguousarray(frame)
        return frame.tobytes(), frame.shape

    def get_metrics(self):
        return {
            'color_frames': self.color_frame_count,
            'depth_frames': self.depth_frame_count,
            'color_latency_ms': round(self.color_latency_ms, 2),
            'depth_latency_ms': round(self.depth_latency_ms, 2),
            'depth_available': self.depth_stream is not None,
        }
    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # ifaces.RoboCompCameraRGBDSimple.Point3D
    # ifaces.RoboCompCameraRGBDSimple.TPoints
    # ifaces.RoboCompCameraRGBDSimple.TImage
    # ifaces.RoboCompCameraRGBDSimple.TDepth
    # ifaces.RoboCompCameraRGBDSimple.TRGBD


