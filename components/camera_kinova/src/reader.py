#!/usr/bin/env python3
import argparse
import threading
import time

import cv2
import numpy as np


def build_pipeline(url, stream_kind):
    if stream_kind == "color":
        return (
            f"rtspsrc location={url} latency=30 "
            "do-rtsp-keep-alive=true drop-on-latency=true ! "
            "rtph264depay ! h264parse ! avdec_h264 max-threads=2 ! videoconvert ! "
            "video/x-raw,format=BGR ! queue leaky=downstream max-size-buffers=1 ! "
            "appsink drop=true sync=false max-buffers=1"
        )
    else:  # depth
        return (
            f"rtspsrc location={url} latency=30 "
            "do-rtsp-keep-alive=true drop-on-latency=true ! "
            "rtpgstdepay ! videoconvert ! video/x-raw,format=GRAY16_LE ! "
            "queue leaky=downstream max-size-buffers=1 ! appsink drop=true sync=false max-buffers=1"
        )


def open_stream(url, stream_kind):
    pipeline = build_pipeline(url, stream_kind)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    return cap, "gstreamer" if cap.isOpened() else "failed"


def normalize_depth_for_display(frame):
    if frame is None or frame.size == 0:
        return frame
    normalized = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


class StreamReader:
    """Lee un stream en su propio hilo y guarda siempre el último frame."""

    def __init__(self, cap, name):
        self.cap = cap
        self.name = name
        self.lock = threading.Lock()
        self.frame = None
        self.ok = False
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        fail_count = 0
        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                fail_count += 1
                if fail_count == 30:
                    print(f"[{self.name}] posible freeze: 30 lecturas fallidas seguidas")
                time.sleep(0.05)
            else:
                fail_count = 0
            with self.lock:
                self.ok = ok
                self.frame = frame

    def get(self):
        with self.lock:
            return self.ok, self.frame

    def stop(self):
        self.running = False
        self.cap.release()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            print(f"ERROR: stream reader thread {self.name} did not stop")


def run_step(step, ip, frames=5, show=False):
    color_url = f"rtsp://{ip}/color"
    depth_url = f"rtsp://{ip}/depth"

    if step == "connect":
        print(f"Connecting to {ip}")
        for kind, url in (("color", color_url), ("depth", depth_url)):
            cap, backend = open_stream(url, kind)
            print(f"{kind}: opened={cap.isOpened()} backend={backend}")
            if cap.isOpened():
                cap.release()
        return

    if step in ("color", "depth"):
        url = color_url if step == "color" else depth_url
        cap, backend = open_stream(url, step)
        print(f"{step} stream backend={backend} opened={cap.isOpened()}")
        if not cap.isOpened():
            return

        reader = StreamReader(cap, step)
        try:
            count = 0
            while count < frames:
                ok, frame = reader.get()
                if not ok or frame is None:
                    time.sleep(0.005)
                    continue
                count += 1
                print(f"[{step}] frame {count}: shape={frame.shape}, dtype={frame.dtype}, "
                      f"min={frame.min()}, max={frame.max()}")
                if show:
                    display_frame = normalize_depth_for_display(frame) if step == "depth" else frame
                    cv2.imshow(step, display_frame)
                    cv2.waitKey(1)
        finally:
            reader.stop()
            if show:
                cv2.destroyAllWindows()
        return

    if step == "both":
        color_cap, color_backend = open_stream(color_url, "color")
        depth_cap, depth_backend = open_stream(depth_url, "depth")
        print(f"color backend={color_backend} opened={color_cap.isOpened()}")
        print(f"depth backend={depth_backend} opened={depth_cap.isOpened()}")

        if not (color_cap.isOpened() and depth_cap.isOpened()):
            if color_cap.isOpened():
                color_cap.release()
            if depth_cap.isOpened():
                depth_cap.release()
            return

        color_reader = StreamReader(color_cap, "color")
        depth_reader = StreamReader(depth_cap, "depth")

        try:
            count = 0
            while count < frames:
                color_ok, color_frame = color_reader.get()
                depth_ok, depth_frame = depth_reader.get()

                if color_ok and color_frame is not None:
                    print(f"[color] frame {count + 1}: shape={color_frame.shape}, dtype={color_frame.dtype}")
                if depth_ok and depth_frame is not None:
                    print(f"[depth] frame {count + 1}: shape={depth_frame.shape}, dtype={depth_frame.dtype}")

                if show:
                    if color_ok and color_frame is not None:
                        cv2.imshow("color", color_frame)
                    if depth_ok and depth_frame is not None:
                        cv2.imshow("depth", normalize_depth_for_display(depth_frame))
                    cv2.waitKey(1)

                count += 1
                time.sleep(0.01)  # solo para no saturar la CPU en el bucle, no limita fps real
        finally:
            color_reader.stop()
            depth_reader.stop()
            if show:
                cv2.destroyAllWindows()
        return

    raise ValueError(f"unknown step: {step}")


def main():
    parser = argparse.ArgumentParser(description="Incremental RTSP camera test script")
    parser.add_argument("step", choices=["connect", "color", "depth", "both"], help="test stage to run")
    parser.add_argument("--ip", default="192.168.50.10", help="robot IP")
    parser.add_argument("--frames", type=int, default=5, help="number of frames to read")
    parser.add_argument("--show", action="store_true", help="show frames in OpenCV windows")
    args = parser.parse_args()

    run_step(args.step, args.ip, frames=args.frames, show=args.show)


if __name__ == "__main__":
    main()