import pytest
import sys
sys.path.append('c:/Users/Murat Ege/OneDrive/Masaüstü/DroneTest/QUADCOPTER_TEST/ros2_ws/src/vision_processing')
from vision_processing.config_validation import VisionConfigValidator

def test_flags_false():
    c = VisionConfigValidator(has_calib=False)
    assert c.validate(640, 480) == (False, "NO_CALIBRATION")
    
    c = VisionConfigValidator(has_calib=True, has_mount=False)
    assert c.validate(640, 480) == (False, "NO_MOUNT")
    
    c = VisionConfigValidator(has_calib=True, has_mount=True, has_ground_ref=False)
    assert c.validate(640, 480) == (False, "NO_GROUND_REF")

def test_resolution_mismatch():
    c = VisionConfigValidator(has_calib=True, has_mount=True, has_ground_ref=True,
                              cam_model='plumb_bob', cam_fx=10, cam_fy=10, cam_cx=5, cam_cy=5,
                              cam_dists=[0,0,0,0], calib_width=640, calib_height=480)
    assert c.validate(800, 600) == (False, "CALIB_RESOLUTION_MISMATCH")

def test_invalid_calibration_values():
    # fx=0, cx=0
    c1 = VisionConfigValidator(has_calib=True, has_mount=True, has_ground_ref=True,
                              cam_model='plumb_bob', cam_fx=0, cam_fy=10, cam_cx=0, cam_cy=5,
                              cam_dists=[0,0,0,0], calib_width=640, calib_height=480)
    assert c1.validate(640, 480) == (False, "NO_CALIBRATION")
    
    # short dists
    c2 = VisionConfigValidator(has_calib=True, has_mount=True, has_ground_ref=True,
                              cam_model='plumb_bob', cam_fx=10, cam_fy=10, cam_cx=5, cam_cy=5,
                              cam_dists=[0,0], calib_width=640, calib_height=480)
    assert c2.validate(640, 480) == (False, "NO_CALIBRATION")
    
    # wrong model
    c3 = VisionConfigValidator(has_calib=True, has_mount=True, has_ground_ref=True,
                              cam_model='fisheye', cam_fx=10, cam_fy=10, cam_cx=5, cam_cy=5,
                              cam_dists=[0,0,0,0], calib_width=640, calib_height=480)
    assert c3.validate(640, 480) == (False, "NO_CALIBRATION")

def test_valid():
    c = VisionConfigValidator(has_calib=True, has_mount=True, has_ground_ref=True,
                              cam_model='plumb_bob', cam_fx=10, cam_fy=10, cam_cx=5, cam_cy=5,
                              cam_dists=[0,0,0,0], calib_width=640, calib_height=480)
    assert c.validate(640, 480) == (True, "OK")
