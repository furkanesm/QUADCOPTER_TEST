#!/usr/bin/env python3
import cv2
import numpy as np
import os
import glob
import yaml

"""
Camera Calibration Stub for AR0234 Sensor

WARNING: The datasheet claims a 90-degree horizontal FOV, but a standard pinhole calculation
with focal_length=3.6mm and sensor_width=5.76mm yields ~77 degrees.
This divergence strongly implies severe optical distortion (e.g. barrel distortion).

Pinhole GSD formulas used in the system are STRICTLY APPROXIMATE and are only valid for 
DISARMED/SITL testing. Before real autonomous flight, you MUST:
1. Capture 15-20 images of a checkerboard pattern at various angles.
2. Run this script to generate true camera intrinsics (fx, fy, cx, cy) and distortion parameters.
3. Update `camera_config.yaml` and set `is_calibrated: true`.
"""

def perform_calibration(images_dir, board_size=(9, 6), square_size_m=0.025):
    # Termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2) * square_size_m

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob(os.path.join(images_dir, '*.jpg')) # Match your format
    
    if len(images) == 0:
        print(f"Error: No images found in {images_dir}. Calibration cannot proceed.")
        return False
        
    print(f"Found {len(images)} images for calibration.")
    # TODO: Implement the actual image finding loop
    # for fname in images:
    #     img = cv2.imread(fname)
    #     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #     
    #     # Find the chess board corners
    #     ret, corners = cv2.findChessboardCorners(gray, board_size, None)
    #     # ... complete the pipeline ...
    #
    # ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    print("TODO: Checkerboard pipeline is currently a STUB. Please implement and run with real images.")
    # Return mock/empty to indicate not finished
    return False

if __name__ == '__main__':
    print("=== AR0234 Camera Calibration Script ===")
    print("Executing this script is MANDATORY for real hardware flight!")
    perform_calibration("./calibration_images")
