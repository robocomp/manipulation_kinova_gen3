import cv2

pipeline = (
    "rtspsrc location=rtsp://192.168.50.10/color protocols=tcp latency=50 ! "
    "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
    "video/x-raw,format=BGR ! appsink drop=true sync=false"
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
ret, frame = cap.read()
print("ret:", ret)
print("shape:", frame.shape if frame is not None else None)
print("ndim:", frame.ndim if frame is not None else None)