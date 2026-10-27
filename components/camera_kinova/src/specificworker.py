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

# Límite de visualización de profundidad; los datos publicados no se recortan.
MAX_DEPTH_DISPLAY_MM = 5000.0
DEPTH_FACTOR_M_PER_UNIT = 0.001
COLOR_READ_TIMEOUT_MS = 5000
COLOR_RECONNECT_FAILURES = 5
DEPTH_RECONNECT_FAILURES = 5
DEPTH_RECONNECT_INITIAL_DELAY = 0.25
DEPTH_RECONNECT_MAX_DELAY = 2.0
DEBUG_FRAME_CHECK_INTERVAL = 15.0


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        stream_mode = configData.get("Camera", {}).get("stream", "both")
        if stream_mode not in ("both", "color", "depth"):
            raise ValueError(f"unsupported stream mode: {stream_mode}")
        self.Period = configData["Period"]["Compute"]
        self.stream_mode = stream_mode
        self.display = configData.get("Display", {}).get("display", False) is True
        self.ui.color.setVisible(stream_mode in ("color", "both"))
        self.ui.depth.setVisible(stream_mode in ("depth", "both"))
        if not self.display:
            self.hide()

        self.init_timestamp = int(time.time() * 1000)
        self.color_stream = None
        self.depth_stream = None
        self.color_thread = None
        self.depth_thread = None
        self.run = False
        self.stop_event = threading.Event()
        self._stopping = False
        self.latest_color_frame = None
        self.latest_depth_frame = None          # muestras GRAY16_LE originales recibidas del sensor
        self.latest_color_timestamp = None
        self.latest_depth_timestamp = None
        self.color_frame_count = 0
        self.depth_frame_count = 0
        self.color_latency_ms = 0.0
        self.depth_latency_ms = 0.0
        self.last_latency_print = time.time()   # bug anterior: no estaba inicializado

        if startup_check:
            self.startup_check()
        elif self.display:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def closeEvent(self, event):
        self.shutdown()
        self.ui = None
        super(SpecificWorker, self).closeEvent(event)

    def shutdown(self):
        """Stop stream readers and release resources, including on application quit."""
        self.timer.stop()
        self.run = False
        self.stop_event.set()

        if not self._stopping:
            self._stopping = True
            streams = (self.color_stream, self.depth_stream)
            self.color_stream = None
            self.depth_stream = None
            for stream in streams:
                if stream is not None:
                    stream.release()

        for thread in (self.color_thread, self.depth_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)
                if thread.is_alive():
                    print(f"ERROR: stream reader thread {thread.name} did not stop")

    @staticmethod
    def _has_gstreamer_backend():
        try:
            return cv2.videoio_registry.hasBackend(cv2.CAP_GSTREAMER)
        except AttributeError:
            return "GStreamer:                   YES" in cv2.getBuildInformation()

    def _open_color_stream(self):
        if self._has_gstreamer_backend():
            print("trying color stream with low-latency GStreamer pipeline")
            capture = cv2.VideoCapture(self.color_pipeline, cv2.CAP_GSTREAMER)
            if capture.isOpened():
                print(f"color stream opened with {capture.getBackendName()}")
                return capture
            capture.release()

        print("GStreamer color stream unavailable; falling back to FFmpeg")
        timeout_params = [
            cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, COLOR_READ_TIMEOUT_MS,
            cv2.CAP_PROP_READ_TIMEOUT_MSEC, COLOR_READ_TIMEOUT_MS,
        ]
        used_default_backend = False
        try:
            if cv2.videoio_registry.hasBackend(cv2.CAP_FFMPEG):
                capture = cv2.VideoCapture(self.color_url, cv2.CAP_FFMPEG, timeout_params)
            else:
                capture = cv2.VideoCapture(self.color_url)
                used_default_backend = True
        except cv2.error as error:
            print(f"Unable to set FFmpeg stream timeouts: {error}")
            capture = cv2.VideoCapture(self.color_url)
            used_default_backend = True

        if capture.isOpened():
            print(f"color stream opened with {capture.getBackendName()}")
            return capture
        capture.release()

        if not used_default_backend:
            print("bounded FFmpeg open failed; skipping unbounded default-backend retry")

        print(f"ERROR: unable to open color stream at {self.color_url} with OpenCV backends")
        return None

    def setParams(self, params):
        self.ip = self.configData["Arm"]["ip"]
        print(f"Connecting to {self.ip}")
        self.color_url = f"rtsp://{self.ip}/color"
        depth_url = f"rtsp://{self.ip}/depth"

        self.color_pipeline = (
            f"rtspsrc location={self.color_url} latency=30 "
            "do-rtsp-keep-alive=true drop-on-latency=true ! "
            "rtph264depay ! h264parse ! avdec_h264 max-threads=2 ! videoconvert ! "
            "video/x-raw,format=BGR ! queue leaky=downstream max-size-buffers=1 ! "
            "appsink drop=true sync=false max-buffers=1"
        )
        self.depth_pipeline = (
            f"rtspsrc location={depth_url} latency=30 "
            "do-rtsp-keep-alive=true drop-on-latency=true ! "
            "rtpgstdepay ! videoconvert ! video/x-raw,format=GRAY16_LE ! "
            "queue leaky=downstream max-size-buffers=1 ! appsink drop=true sync=false max-buffers=1"
        )

        if self.stream_mode in ("color", "both"):
            self.color_stream = self._open_color_stream()
            if self.color_stream is None:
                return False

        if self.stream_mode in ("depth", "both"):
            self.depth_stream = self._open_depth_stream()

        self.run = True
        self.stop_event.clear()
        if self.color_stream is not None:
            self.color_thread = threading.Thread(
                target=self.video_color_thread,
                args=(self.color_stream,),
                daemon=True,
            )
            self.color_thread.start()

        if self.stream_mode in ("depth", "both"):
            self.depth_thread = threading.Thread(
                target=self.video_depth_thread,
                args=(self.depth_stream,),
                daemon=True,
            )
            self.depth_thread.start()

        print("Reading threads started")
        return True

    def _open_depth_stream(self):
        if not self._has_gstreamer_backend():
            print("ERROR: OpenCV has no GStreamer backend; cannot open the depth stream")
            return None

        try:
            capture = cv2.VideoCapture(self.depth_pipeline, cv2.CAP_GSTREAMER)
        except cv2.error as error:
            print(f"ERROR: failed to create depth stream capture: {error}")
            return None

        if not capture.isOpened():
            print("depth stream not opened")
            capture.release()
            return None

        print(f"depth stream opened with {capture.getBackendName()}")
        return capture

    @staticmethod
    def _log_frame_health(stream_name, last_check, valid_frames, empty_frames, last_status):
        now = time.monotonic()
        if now - last_check < DEBUG_FRAME_CHECK_INTERVAL:
            return last_check, valid_frames, empty_frames

        print(
            f"[{stream_name}] DEBUG últimos {DEBUG_FRAME_CHECK_INTERVAL:.0f}s: "
            f"no vacíos={valid_frames}, vacíos={empty_frames}, última lectura={last_status}"
        )
        return now, 0, 0

    # ------------------------------------------------------------------
    # Normalización de profundidad
    # ------------------------------------------------------------------
    def _depth_to_display(self, frame_mm):
        """Convierte profundidad cruda (mm) a una imagen RGB visualizable,
        usando una escala fija de visualización sin modificar las muestras
        originales del sensor."""
        if frame_mm is None:
            return None
        # Los valores 0 (sin medida) se muestran igual que la distancia máxima.
        scaled = (frame_mm.astype(np.float32) / MAX_DEPTH_DISPLAY_MM * 255.0)
        scaled = np.clip(scaled, 0, 255).astype(np.uint8)
        scaled[frame_mm == 0] = 255
        colored = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
        return cv2.cvtColor(colored, cv2.COLOR_BGR2RGB)

    def _to_display_image(self, frame):
        """Solo para el color; el depth usa _depth_to_display."""
        if frame is None:
            return None
        if frame.dtype != np.uint8:
            frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    @Slot()
    def compute(self):
        if not self.display:
            return True

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
                depth_display = self._depth_to_display(self.latest_depth_frame)
                height, width = depth_display.shape[:2]
                image = QImage(depth_display.data, width, height, width * 3, QImage.Format_RGB888)
                pixmap = QPixmap.fromImage(image).scaled(
                    max(1, self.ui.depth.width()),
                    max(1, self.ui.depth.height()),
                    Qt.KeepAspectRatio,
                )
                self.ui.depth.setPixmap(pixmap)

            if self.latest_color_timestamp is not None and time.time() - self.last_latency_print >= 1.0:
                self.color_latency_ms = (time.time() - self.latest_color_timestamp) * 1000.0
                print(f'color frame age={self.color_latency_ms:.1f}ms')
                self.last_latency_print = time.time()
        except Exception as e:
            print(f"Error en compute: {e}")
            self.timer.stop()
            return True
        return True

    ################################################################################################################
    def video_color_thread(self, cap):
        fail_count = 0
        reconnect_delay = 1.0
        last_health_check = time.monotonic()
        valid_frames_since_check = 0
        empty_frames_since_check = 0
        last_frame_status = "sin lecturas"
        try:
            while self.run:
                ret, frame = cap.read() if cap is not None and cap.isOpened() else (False, None)
                frame_valid = ret and frame is not None and frame.size > 0
                if frame_valid:
                    self.latest_color_frame = frame
                    self.latest_color_timestamp = time.time()
                    self.color_frame_count += 1
                    valid_frames_since_check += 1
                    last_frame_status = f"no vacío, shape={frame.shape}"
                    fail_count = 0
                    reconnect_delay = 1.0
                else:
                    if ret:
                        empty_frames_since_check += 1
                        last_frame_status = "vacío"
                    else:
                        last_frame_status = "sin frame"
                    fail_count += 1
                    if fail_count >= COLOR_RECONNECT_FAILURES:
                        if cap is not None:
                            cap.release()
                        cap = None
                        self.color_stream = None
                        if not self.run:
                            break
                        print(f"[color] {fail_count} lecturas fallidas; reconectando en {reconnect_delay:.0f}s")
                        if self.stop_event.wait(reconnect_delay) or not self.run:
                            break
                        cap = self._open_color_stream()
                        if cap is not None and self.run:
                            self.color_stream = cap
                        elif cap is not None:
                            cap.release()
                            cap = None
                        reconnect_delay = min(reconnect_delay * 2.0, 10.0)
                        fail_count = 0
                    else:
                        if self.stop_event.wait(0.05):
                            break
                last_health_check, valid_frames_since_check, empty_frames_since_check = (
                    self._log_frame_health(
                        "color",
                        last_health_check,
                        valid_frames_since_check,
                        empty_frames_since_check,
                        last_frame_status,
                    )
                )
            print("color finish")
        except KeyboardInterrupt:
            print("hilo finish")
        finally:
            if cap is not None:
                cap.release()

    def video_depth_thread(self, cap):
        fail_count = 0
        reconnect_delay = DEPTH_RECONNECT_INITIAL_DELAY
        last_health_check = time.monotonic()
        valid_frames_since_check = 0
        empty_frames_since_check = 0
        last_frame_status = "sin lecturas"
        try:
            while self.run:
                if cap is None:
                    if self.stop_event.wait(reconnect_delay) or not self.run:
                        break
                    print(f"[depth] intentando reconectar (espera {reconnect_delay:.2f}s)")
                    cap = self._open_depth_stream()
                    if cap is None:
                        reconnect_delay = min(reconnect_delay * 2.0, DEPTH_RECONNECT_MAX_DELAY)
                        last_frame_status = "sin frame"
                        last_health_check, valid_frames_since_check, empty_frames_since_check = (
                            self._log_frame_health(
                                "depth",
                                last_health_check,
                                valid_frames_since_check,
                                empty_frames_since_check,
                                last_frame_status,
                            )
                        )
                        continue
                    self.depth_stream = cap
                    fail_count = 0
                    reconnect_delay = DEPTH_RECONNECT_INITIAL_DELAY

                try:
                    ret, frame = cap.read() if cap.isOpened() else (False, None)
                except Exception as error:
                    print(f"[depth] error leyendo el stream: {error}")
                    ret, frame = False, None
                frame_valid = ret and frame is not None and frame.size > 0
                if frame_valid:
                    self.latest_depth_frame = frame  # crudo, GRAY16_LE en mm
                    self.latest_depth_timestamp = time.time()
                    self.depth_frame_count += 1
                    valid_frames_since_check += 1
                    last_frame_status = f"no vacío, shape={frame.shape}"
                    fail_count = 0
                    reconnect_delay = DEPTH_RECONNECT_INITIAL_DELAY
                else:
                    if ret:
                        empty_frames_since_check += 1
                        last_frame_status = "vacío"
                    else:
                        last_frame_status = "sin frame"
                    fail_count += 1
                    if fail_count >= DEPTH_RECONNECT_FAILURES:
                        print(f"[depth] {fail_count} lecturas fallidas; cerrando captura para reconectar")
                        cap.release()
                        cap = None
                        self.depth_stream = None
                        self.latest_depth_frame = None
                        self.latest_depth_timestamp = None
                        fail_count = 0
                    elif self.stop_event.wait(0.05):
                        break
                last_health_check, valid_frames_since_check, empty_frames_since_check = (
                    self._log_frame_health(
                        "depth",
                        last_health_check,
                        valid_frames_since_check,
                        empty_frames_since_check,
                        last_frame_status,
                    )
                )
            print("depth finish")
        except KeyboardInterrupt:
            print("hilo finish")
        finally:
            if cap is not None:
                cap.release()

    def killThreads(self):
        self.shutdown()

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

    def CameraRGBDSimple_getAll(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TRGBD()

        color_frame = self.latest_color_frame
        if color_frame is not None:
            color_bytes, color_shape = self._frame_to_bytes(color_frame)
            ret.image.image = color_bytes
            ret.image.height, ret.image.width = color_shape[:2]
            ret.image.depth = color_shape[2] if len(color_shape) > 2 else 1
            ret.image.alivetime = int(time.time() * 1000)
        else:
            ret.image.image = b""
            ret.image.height = 0
            ret.image.width = 0
            ret.image.depth = 0
            ret.image.alivetime = int(time.time() * 1000)

        depth_frame = self.latest_depth_frame
        if color_frame is None and depth_frame is None:
            return None

        if depth_frame is not None:
            depth_bytes, depth_shape = self._frame_to_bytes(depth_frame)
            ret.depth.depth = depth_bytes
            ret.depth.height, ret.depth.width = depth_shape[:2]
            ret.depth.depthFactor = DEPTH_FACTOR_M_PER_UNIT
            ret.depth.alivetime = int(time.time() * 1000)
        else:
            ret.depth.depth = b""
            ret.depth.height = 0
            ret.depth.width = 0
            ret.depth.depthFactor = DEPTH_FACTOR_M_PER_UNIT
            ret.depth.alivetime = int(time.time() * 1000)

        return ret

    def CameraRGBDSimple_getDepth(self, camera):
        img = self.latest_depth_frame
        if img is None:
            return None

        ret = ifaces.RoboCompCameraRGBDSimple.TDepth()
        depth_bytes, depth_shape = self._frame_to_bytes(img)
        ret.height, ret.width = depth_shape[:2]
        ret.depth = depth_bytes
        ret.depthFactor = DEPTH_FACTOR_M_PER_UNIT
        ret.alivetime = int(time.time() * 1000)
        return ret

    def CameraRGBDSimple_getImage(self, camera):
        img = self.latest_color_frame
        if img is None:
            return None

        ret = ifaces.RoboCompCameraRGBDSimple.TImage()
        image_bytes, image_shape = self._frame_to_bytes(img)
        ret.height, ret.width = image_shape[:2]
        ret.depth = image_shape[2] if len(image_shape) > 2 else 1
        ret.image = image_bytes
        ret.alivetime = int(time.time() * 1000)

        return ret

    def CameraRGBDSimple_getPoints(self, camera):
        ret = ifaces.RoboCompCameraRGBDSimple.TPoints()
        ret.points = []
        ret.alivetime = int(time.time() * 1000)
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
