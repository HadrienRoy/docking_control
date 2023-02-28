#!/usr/bin/env python3

# camera_calibration.py
#
# Author: Hadrien Roy
# 
# Inputs: Images of a checkerboad
# Outputs: Intrisnic camera parameters
#
# Purpose: To obtain the intrinsic camera parameters

### Imports ###
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 
import glob


def main(args=None):
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob('*.jpg')

    nX = 9  # Number of interior corners, x-axis
    nY = 6  # Number of interior corners, y-axis

    square_size = 0.01  # Length of square in meters

    for filename in images:
        # Load image
        img = cv2.imread(filename)

        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (nY,nX), None)

        # If corners found, add/draw object points, image points (after refining them)
        if ret == True:
            # Append object points
            objpoints.append(objp)

            # Find more exact corner pixels
            corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)

            # Append image points
            imgpoints.append(corners)
            
            # Draw and display the corners
            cv2.drawChessboardCorners(img, (nY,nX), corners2, ret)
            
            # Use for Testing, shows image and waits a period
            cv2.imshow('img', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    # Calibrate the camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Undistort camera
    #dst = cv2.undistort (img, mtx, dist, None, newcameramtx)

    # Print intrinsic camera matrix
    fx = mtx[0,0]
    fy = mtx[1,1]
    cx = mtx[0,2]
    cy = mtx[1,2]

    params = (fx, fy, cx, cy)

    print()
    print('all units below measured in pixels:')
    print('  fx = {}'.format(mtx[0,0]))
    print('  fy = {}'.format(mtx[1,1]))
    print('  cx = {}'.format(mtx[0,2]))
    print('  cy = {}'.format(mtx[1,2]))
    print()
    print('pastable into Python:')
    print('  fx, fy, cx, cy = {}'.format(repr(params)))
    print()


if __name__ == "__main__":
    main()