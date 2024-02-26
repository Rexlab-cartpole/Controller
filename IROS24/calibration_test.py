import numpy as np
import cv2

calib_data = np.load("calibration_data.npz")

mtx = calib_data["matrix"]
dist = calib_data["coefficients"]

print(mtx)
print(dist)

img = cv2.imread('img_0.jpg')
h, w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
# print(newcameramtx)
# print(roi)

# undistort
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
cv2.imshow("undistorted", dst)
cv2.waitKey()