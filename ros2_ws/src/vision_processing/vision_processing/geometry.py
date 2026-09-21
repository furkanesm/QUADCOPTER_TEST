import numpy as np
import math

class RaycastProjector:
    def __init__(self, fx: float, fy: float, cx: float, cy: float, dist_coeffs: list = None,
                 mount_xyz: list = [0,0,0], mount_rpy: list = [0,0,0]):
        """
        :param fx, fy, cx, cy: Camera intrinsics matched to incoming image shape
        :param dist_coeffs: Distortion coefficients [k1, k2, p1, p2, k3] or None
        :param mount_xyz: Body-to-Camera offset in meters (Forward-Left-Up or ENU)
        :param mount_rpy: Body-to-Camera rotation in radians (roll, pitch, yaw)
        """
        self.intrinsics = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        self.dists = np.array(dist_coeffs, dtype=np.float32) if dist_coeffs else np.zeros((5,), dtype=np.float32)
        
        self.mount_offset = np.array(mount_xyz).reshape(3, 1)
        self.mount_rot = self._euler_to_mat(*mount_rpy)
        
    def _euler_to_mat(self, roll, pitch, yaw):
        si, c, sj, cj, sk, ck = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch), math.sin(yaw), math.cos(yaw)
        R_x = np.array([[1, 0, 0], [0, c, -si], [0, si, c]])
        R_y = np.array([[cj, 0, sj], [0, 1, 0], [-sj, 0, cj]])
        R_z = np.array([[ck, -sk, 0], [sk, ck, 0], [0, 0, 1]])
        return R_z @ R_y @ R_x

    def _quat_to_mat(self, qx, qy, qz, qw):
        # normalize
        n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
        return np.array([
            [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
            [    2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
            [    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])

    def unproject_pixel(self, u: float, v: float, 
                        vehicle_xyz: list, vehicle_quat: list,
                        ground_plane_z: float = 0.0,
                        max_distance_m: float = 100.0) -> dict:
        """
        Projecks a pixel onto a defined ground altitude.
        :param u,v: Bbox center or contact point. Make sure it isn't doubly distorted/resized.
        :param vehicle_xyz: [x,y,z] ENU or local frame
        :param vehicle_quat: [x,y,z,w] orientation
        :param ground_plane_z: altitude of ground in the SAME frame as vehicle_xyz
        :return: {"valid": Bool, "x": float, "y": float, "reason": str}
        """
        import cv2
        
        # 1. Undistort pixel if dist_coeffs are non-zero
        pt = np.array([[[u, v]]], dtype=np.float32)
        if np.any(self.dists):
            pt = cv2.undistortPoints(pt, self.intrinsics, self.dists, P=self.intrinsics)
        
        u_u = pt[0,0,0]
        v_u = pt[0,0,1]
        
        # 2. Camera Frame Ray (Optical frame: Z forward, X right, Y down)
        cx, cy = self.intrinsics[0,2], self.intrinsics[1,2]
        fx, fy = self.intrinsics[0,0], self.intrinsics[1,1]
        
        ray_c = np.array([
            (u_u - cx) / fx,
            (v_u - cy) / fy,
            1.0
        ])
        ray_c = ray_c / np.linalg.norm(ray_c)
        
        # Optical to standard (Z up, X forward, Y left) - assuming typical ROS optical standard
        # Standard: X_cam = Z_opt, Y_cam = -X_opt, Z_cam = -Y_opt
        R_cam_to_ros = np.array([
            [ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0]
        ])
        ray_ros = R_cam_to_ros @ ray_c
        
        # 3. Apply Mount Transform (from Mount to Body)
        ray_body = self.mount_rot @ ray_ros
        p_cam_body = self.mount_offset
        
        # 4. Apply Vehicle Pose
        R_body_to_world = self._quat_to_mat(*vehicle_quat)
        ray_world = R_body_to_world @ ray_body
        
        p_cam_world = np.array(vehicle_xyz).reshape(3,1) + R_body_to_world @ p_cam_body
        
        # 5. Intersect with Ground Plane (Z = ground_plane_z)
        if abs(ray_world[2]) < 1e-4:
            return {"valid": False, "x": 0.0, "y": 0.0, "reason": "RAY_PARALLEL_TO_GROUND"}
            
        t = (ground_plane_z - p_cam_world[2]) / ray_world[2]
        
        if t < 0:
            return {"valid": False, "x": 0.0, "y": 0.0, "reason": "INTERSECTION_BEHIND_CAMERA_OR_UPWARD_RAY"}
            
        if t > max_distance_m:
            return {"valid": False, "x": 0.0, "y": 0.0, "reason": "INTERSECTION_TOO_FAR"}
            
        p_ground = p_cam_world + t * ray_world.reshape(3,1)
        
        if not np.isfinite(p_ground).all():
            return {"valid": False, "x": 0.0, "y": 0.0, "reason": "NOT_FINITE"}

        p_flat = np.asarray(p_ground, dtype=float).ravel()
        return {"valid": True, "x": float(p_flat[0]), "y": float(p_flat[1]), "reason": "OK"}
