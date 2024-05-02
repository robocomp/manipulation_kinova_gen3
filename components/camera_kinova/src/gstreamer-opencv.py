import cv2

gstreamer_str = ("gst-launch-1.0 rtspsrc location=rtsp://192.168.1.10/color latency=30 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink")
cap = cv2.VideoCapture("gst-launch-1.0 rtspsrc location=rtsp://192.168.1.10/color latency=30 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret:
        cv2.imshow("Input via Gstreamer", frame)
        cv2.waitKey(1);
cap.release()
cv2.destroyAllWindows()