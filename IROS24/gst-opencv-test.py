import numpy as np
import cv2
from multiprocessing import Process
# import matplotlib.pyplot as plt
import time

gst_string = 'v4l2src ! video/x-raw, format=GRAY8, width=640, height=400, framerate=30/1 ! videoconvert ! videoflip method=2 ! appsink sync=false'
cap = cv2.VideoCapture(gst_string, cv2.CAP_GSTREAMER)
# cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
    
calib_data = np.load("calibration_data.npz")
mtx = calib_data["matrix"]
dist = calib_data["coefficients"]


ret, frame = cap.read()
h, w = frame.shape
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
x_roi, y_roi, w_roi, h_roi = roi

right_side_off = 20
left_side_off = 20
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    # frame_undistort = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    frame_undistort = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
    frame_undistort = frame_undistort[y_roi:y_roi+h_roi, x_roi:x_roi+w_roi]
    frame_undistort = cv2.resize(frame_undistort, (200, 125), # 1.6 aspect ratio (same as camera (640x400))
               interpolation=cv2.INTER_LINEAR)
    frame_undistort = frame_undistort[:, left_side_off:-right_side_off] # final image size is 200-left_side_off-right_side_off x 125-top_off-bottom_off (currently 160x125)
    # print(frame_undistort.shape)
    
    # Our operations on the frame come here
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Display the resulting frame
    ret, thresh_image = cv2.threshold(frame_undistort, 35, 255, cv2.THRESH_BINARY)
    frame_for_visualization = cv2.resize(thresh_image, (640-right_side_off-left_side_off, 400), interpolation=cv2.INTER_CUBIC)
    cv2.imshow('frame', frame_for_visualization)
    # cv2.imshow('frame', frame_undistort)
    if cv2.waitKey(1) == ord('q'):
        break