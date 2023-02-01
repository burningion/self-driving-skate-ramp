import cv2

import numpy as np

capture_device = 0
capture_fps = 30
capture_width = 640
capture_height = 480

capture_device = f"nvarguscamerasrc sensor-id={capture_device} ! video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, format=(string)NV12, framerate=(fraction){capture_fps}/1 ! nvvidconv ! video/x-raw, width=(int){capture_width}, height=(int){capture_height}, format=(string)BGRx ! videoconvert ! appsink"

video = cv2.VideoCapture(capture_device, cv2.CAP_GSTREAMER)

while True:
    re, frame = video.read()
    cv2.imshow('cam', frame)

    if cv2.waitKey(1) == 27:
        break
cv2.destroyAllWindows()
