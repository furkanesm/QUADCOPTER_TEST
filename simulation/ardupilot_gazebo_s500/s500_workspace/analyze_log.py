from pymavlink import mavutil
import sys
import numpy as np

log_file = '/home/furkan/s500_sim/s500_test_backups/backup_20260919_1953/logs/00000001.BIN'
print(f"Reading log: {log_file}")
mlog = mavutil.mavlink_connection(log_file)

events = []
hover_start = None
hover_end = None
arm_time = None
disarm_time = None

alt_data = []
att_data = []
vel_data = []
rc_data = []

# First pass: find events
while True:
    m = mlog.recv_match(type=['MSG', 'MODE', 'EV'], blocking=False)
    if m is None:
        break
    t = m.TimeUS / 1e6
    if m.get_type() == 'MSG':
        events.append((t, f"MSG: {m.Message}"))
        if "Arming" in m.Message: arm_time = t
        if "Disarming" in m.Message: disarm_time = t
    elif m.get_type() == 'MODE':
        events.append((t, f"MODE: {m.Mode}"))
    elif m.get_type() == 'EV':
        events.append((t, f"EV: {m.Id}"))

print(f"Arm time: {arm_time} s, Disarm time: {disarm_time} s")

# Second pass: read data
mlog.rewind()
while True:
    m = mlog.recv_match(type=['POS', 'ATT', 'RCIN', 'RCOU', 'XKY0'], blocking=False) # XKY0 or similar for EKF? We'll use POS for Alt/Vel
    if m is None:
        break
    t = m.TimeUS / 1e6
    
    # We only care about data between Arming and Disarming
    if arm_time and disarm_time and (t < arm_time or t > disarm_time):
        continue

    msg_type = m.get_type()
    if msg_type == 'POS':
        alt_data.append((t, m.RelHomeAlt, m.Lat, m.Lng))
    elif msg_type == 'ATT':
        att_data.append((t, m.Roll, m.Pitch, m.Yaw)) # degrees
    elif msg_type == 'RCOU':
        rc_data.append((t, m.C1, m.C2, m.C3, m.C4))

print(f"Collected POS: {len(alt_data)}, ATT: {len(att_data)}, RCOU: {len(rc_data)}")

# Find hover segment: Alt ~2.0 (+- 0.5), and relatively stable.
# We'll just define hover as: > 1.5m and < 2.5m, in the middle of the flight.
hover_segment_alt = []
hover_segment_att = []
hover_segment_rc = []
hover_segment_vel = []

for t, alt, lat, lng in alt_data:
    if 1.5 < alt < 2.5:
        if hover_start is None: hover_start = t
        hover_end = t
        hover_segment_alt.append(alt)

print(f"Hover window: {hover_start} to {hover_end}")

if len(hover_segment_alt) > 0:
    for t, r, p, y in att_data:
        if hover_start <= t <= hover_end:
            hover_segment_att.append((r, p, y))
    
    for t, c1, c2, c3, c4 in rc_data:
        if hover_start <= t <= hover_end:
            hover_segment_rc.append((c1, c2, c3, c4))

    print("\n--- HOVER SUMMARY ---")
    alt_arr = np.array(hover_segment_alt)
    print(f"Alt (m, RelHomeAlt): Mean={alt_arr.mean():.2f}, Min={alt_arr.min():.2f}, Max={alt_arr.max():.2f}")
    
    if len(hover_segment_att) > 0:
        att_arr = np.array(hover_segment_att)
        print(f"Roll (deg): Mean={att_arr[:,0].mean():.2f}, MaxAbs={np.max(np.abs(att_arr[:,0])):.2f}")
        print(f"Pitch (deg): Mean={att_arr[:,1].mean():.2f}, MaxAbs={np.max(np.abs(att_arr[:,1])):.2f}")
        yaw_unwrapped = np.unwrap(np.radians(att_arr[:,2]))
        print(f"Yaw (deg, unwrapped drift): {(np.degrees(yaw_unwrapped[-1]) - np.degrees(yaw_unwrapped[0])):.2f}")
    
    if len(hover_segment_rc) > 0:
        rc_arr = np.array(hover_segment_rc)
        print(f"RCOU (PWM):")
        print(f"  C1: Mean={rc_arr[:,0].mean():.1f} Min={rc_arr[:,0].min()} Max={rc_arr[:,0].max()}")
        print(f"  C2: Mean={rc_arr[:,1].mean():.1f} Min={rc_arr[:,1].min()} Max={rc_arr[:,1].max()}")
        print(f"  C3: Mean={rc_arr[:,2].mean():.1f} Min={rc_arr[:,2].min()} Max={rc_arr[:,2].max()}")
        print(f"  C4: Mean={rc_arr[:,3].mean():.1f} Min={rc_arr[:,3].min()} Max={rc_arr[:,3].max()}")

vel_n = []
vel_e = []
vel_d = []
mlog.rewind()
while True:
    m = mlog.recv_match(type=['XKF1'], blocking=False)
    if m is None: break
    t = m.TimeUS / 1e6
    if hover_start <= t <= hover_end:
        vel_n.append(m.VN)
        vel_e.append(m.VE)
        vel_d.append(m.VD)
if len(vel_n) > 0:
    vn, ve, vd = np.array(vel_n), np.array(vel_e), np.array(vel_d)
    print(f"Velocity (m/s):")
    print(f"  North: Mean={vn.mean():.3f} MaxAbs={np.max(np.abs(vn)):.3f}")
    print(f"  East:  Mean={ve.mean():.3f} MaxAbs={np.max(np.abs(ve)):.3f}")
    print(f"  Down:  Mean={vd.mean():.3f} MaxAbs={np.max(np.abs(vd)):.3f}")
alt_disarm = [alt for t, alt, lat, lng in alt_data if 1418 < t < 1420]
if len(alt_disarm) > 0:
    print(f"Altitude just before Disarm: {alt_disarm[-1]:.2f} m")
