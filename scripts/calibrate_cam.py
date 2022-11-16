# https://www.geeksforgeeks.org/camera-calibration-with-python-opencv/

# Import required modules
import sys
import cv2
import numpy as np
import os
import yaml

# Define the dimensions of checkerboard
CHECKERBOARD = (6, 9)

# stop the iteration when specified
# accuracy, epsilon, is reached or
# specified number of iterations are completed.
criteria = (cv2.TERM_CRITERIA_EPS +
            cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Vector for 3D points
threedpoints = []

# Vector for 2D points
twodpoints = []

#  3D points real world coordinates
objectp3d = np.zeros((1, CHECKERBOARD[0]
                      * CHECKERBOARD[1],
                      3), np.float32)
objectp3d[0,:,:2] = np.mgrid[0:CHECKERBOARD[0],
                               0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

print("opencv errors 1")
# aparently my webcam "showmewebcam on a pi" bugs on opencv 4.6
capture = cv2.VideoCapture("/dev/video1")
print("wierd errors 2")

print("Capture 12 frames or press q")
print("large amount of frames can tale long to calculate...")

for i in range(32):
    if capture.isOpened():
        result, frame = capture.read()

        if result is True:
            grayColor = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            # If desired number of corners are
            # found in the image then ret = true
            ret, corners = cv2.findChessboardCorners(
                            grayColor, CHECKERBOARD,
                            cv2.CALIB_CB_ADAPTIVE_THRESH
                            +cv2.CALIB_CB_FAST_CHECK +
                            cv2.CALIB_CB_NORMALIZE_IMAGE)

            # If desired number of corners can be detected then,
            # refine the pixel coordinates and display
            # them on the images of checker board
            if ret == True:
                threedpoints.append(objectp3d)

                # Refining pixel coordinates
                # for given 2d points.
                corners2 = cv2.cornerSubPix(
                    grayColor, corners, (11, 11), (-1, -1), criteria)

                twodpoints.append(corners2)

                # Draw and display the corners
                frame = cv2.drawChessboardCorners(frame,
                                                  CHECKERBOARD,
                                                  corners2, ret)

            cv2.imshow('img', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()

h, w = frame.shape[:2]

print("Runing calibration")

# Perform camera calibration by
# passing the value of above found out 3D points (threedpoints)
# and its corresponding pixel coordinates of the
# detected corners (twodpoints)

ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
    threedpoints, twodpoints, grayColor.shape[::-1], None, None)

# Displaying required output
print(" Camera matrix:")
print(matrix)

print("\n Distortion coefficient:")
print(distortion)

print("\n Rotation Vectors:")
print(r_vecs)

print("\n Translation Vectors:")
print(t_vecs)

print("Saving distortion results calibration.yml")

with open("calibration.yml", "w") as yaml_file:
    yaml_file.write(yaml.dump({"matrix": matrix.tolist(),
                               "distortion": distortion.tolist()}))
