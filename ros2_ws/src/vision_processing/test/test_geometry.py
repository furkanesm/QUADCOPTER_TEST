import sys
import math
sys.path.append('c:/Users/Murat Ege/OneDrive/Masaüstü/DroneTest/QUADCOPTER_TEST/ros2_ws/src/vision_processing')
from vision_processing.geometry import RaycastProjector

def kontrol(ad, sonuc, x, y):
    ok = (sonuc.get('valid') is True and abs(sonuc['x'] - x) < 1e-6 and abs(sonuc['y'] - y) < 1e-6)
    print(f"{'GECTI' if ok else 'KALDI'} | {ad} | {sonuc}")
    assert ok, f"Hata: beklenen {x},{y} ama gelen {sonuc}"

def test_geometry_math():
    p = RaycastProjector(fx=1200, fy=1200, cx=960, cy=600, dist_coeffs=[0,0,0,0,0], mount_xyz=[0,0,0], mount_rpy=[0.0, math.pi/2, 0.0])
    veh_xyz = [50.0, 30.0, 10.0]
    q0 = [0.0, 0.0, 0.0, 1.0]
    kontrol("asal nokta (+pi/2)", p.unproject_pixel(960, 600, veh_xyz, q0, ground_plane_z=0.0), 50.0, 30.0)
    kontrol("120px sag, yaw=0", p.unproject_pixel(1080, 600, veh_xyz, q0, ground_plane_z=0.0), 50.0, 29.0)
    s = math.sqrt(0.5)
    q90 = [0.0, 0.0, s, s]
    kontrol("120px sag, yaw=+90", p.unproject_pixel(1080, 600, veh_xyz, q90, ground_plane_z=0.0), 51.0, 30.0)

    p2 = RaycastProjector(fx=1200, fy=1200, cx=960, cy=600, dist_coeffs=[0,0,0,0,0], mount_xyz=[0,0,0], mount_rpy=[0.0, -math.pi/2, 0.0])
    res = p2.unproject_pixel(960, 600, veh_xyz, q0, ground_plane_z=0.0)
    assert not res['valid'] and res['reason'] == 'INTERSECTION_BEHIND_CAMERA_OR_UPWARD_RAY'

    # (D5): piksel (960, 480) (asal noktanın 120 px ÜSTÜ) → beklenen x=51.0, y=30.0
    res_up = p.unproject_pixel(960, 480, veh_xyz, q0, ground_plane_z=0.0)
    kontrol("120px yukari (pitch=+pi/2, yaw=0)", res_up, 51.0, 30.0)

if __name__ == "__main__":
    test_geometry_math()
