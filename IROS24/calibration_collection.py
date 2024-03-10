import numpy as np
import cv2
import glob


gst_string = 'v4l2src ! video/x-raw, format=GRAY8, width=640, height=400, framerate=30/1 ! videoconvert ! videoflip method=2 ! appsink sync=true'
# gst_string = 'v4l2src ! video/x-raw, format=GRAY8, width=1280, height=800, framerate=30/1 ! videoconvert ! videoflip method=2 ! appsink sync=true'
cap = cv2.VideoCapture(gst_string, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

i = 0
while True:
    ret, img = cap.read()
    cv2.imshow('frame', img)
    if cv2.waitKey() == ord('s'):
        cv2.imwrite("img_"+str(i)+".jpg", img)
        i += 1
    if cv2.waitKey(1) == ord('q'):
        break
cv2.destroyAllWindows()