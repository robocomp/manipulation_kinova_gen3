import cv2
import threading
import queue
import time
import numpy as np

def video_depth_thread(cap, inqueue):
    print("inicio bucle")
    while cap.isOpened():
        print("esta abierto")
        ret, frame = cap.read()
        if ret:
            inqueue.put(frame)
    print("hilo finish")
    cap.release()


depth_queue = queue.Queue(maxsize=1)

depth_stream = cv2.VideoCapture(
    "gst-launch-1.0 rtspsrc location=rtspt://192.168.1.10/depth latency=30 ! rtpgstdepay ! videoconvert ! appsink",
    cv2.CAP_GSTREAMER)

print(depth_stream.isOpened())

# time.sleep(3)

depth_thread = threading.Thread(target=video_depth_thread, args=(depth_stream, depth_queue))
depth_thread.start()

while True:
    if not depth_queue.empty():
        frame = depth_queue.get()
        #frame = frame.astype(np.uint16)
        print(frame.shape)
        cv2.imshow("Depth", frame)
        cv2.waitKey(1)
        print(type(frame[0][10]))
    else:
        print("No frame")
    time.sleep(0.05)
    print("hilo principal")

