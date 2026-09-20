class VisionConfigValidator:
    def __init__(self, 
                 has_calib=False, has_mount=False, has_ground_ref=False,
                 cam_model='plumb_bob', calib_width=0, calib_height=0,
                 cam_fx=0.0, cam_fy=0.0, cam_cx=0.0, cam_cy=0.0,
                 cam_dists=[0.0, 0.0, 0.0, 0.0, 0.0]):
        self.has_calib = has_calib
        self.has_mount = has_mount
        self.has_ground_ref = has_ground_ref
        
        self.cam_model = cam_model
        self.calib_width = calib_width
        self.calib_height = calib_height
        
        self.cam_fx = cam_fx
        self.cam_fy = cam_fy
        self.cam_cx = cam_cx
        self.cam_cy = cam_cy
        self.cam_dists = cam_dists

    def validate(self, img_w, img_h):
        if not self.has_calib:
            return False, "NO_CALIBRATION"
        if not self.has_mount:
            return False, "NO_MOUNT"
        if not self.has_ground_ref:
            return False, "NO_GROUND_REF"
            
        if self.cam_model != 'plumb_bob':
            return False, "NO_CALIBRATION"
        if self.cam_fx <= 0 or self.cam_fy <= 0 or self.cam_cx <= 0 or self.cam_cy <= 0:
            return False, "NO_CALIBRATION"
        if len(self.cam_dists) < 4:
            return False, "NO_CALIBRATION"
            
        if self.calib_width != img_w or self.calib_height != img_h:
            return False, "CALIB_RESOLUTION_MISMATCH"
            
        return True, "OK"
