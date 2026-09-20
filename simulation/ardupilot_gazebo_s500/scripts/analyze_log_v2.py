from pymavlink import mavutil
import numpy as np
import math

log_file = '/home/furkan/s500_sim/s500_test_backups/backup_20260919_1953/logs/00000001.BIN'
mlog = mavutil.mavlink_connection(log_file)

time_offset = None
pos_data, att_data, rcou_data, xkf1_data = [], [], [], []

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of earth in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

print("Parsing log...")
while True:
    m = mlog.recv_match(type=['POS', 'ATT', 'RCOU', 'XKF1'], blocking=False)
    if m is None: break
    t = m.TimeUS / 1e6
    if time_offset is None: time_offset = t
    t_rel = t - time_offset
    
    m_type = m.get_type()
    if m_type == 'POS':
        pos_data.append((t_rel, m.RelHomeAlt, m.Lat, m.Lng))
    elif m_type == 'ATT':
        att_data.append((t_rel, m.Roll, m.Pitch, m.Yaw))
    elif m_type == 'RCOU':
        rcou_data.append((t_rel, m.C1, m.C2, m.C3, m.C4))
    elif m_type == 'XKF1':
        # EKF3 core/instance is usually in m.C (Core)
        core = getattr(m, 'C', 0)
        xkf1_data.append((t_rel, core, m.VN, m.VE, m.VD))

# Convert to numpy arrays for easier slicing
pos = np.array(pos_data)
att = np.array(att_data)
rc = np.array(rcou_data)
xkf1 = np.array(xkf1_data)

# Compute message frequencies
def print_freq(name, data):
    if len(data) > 1:
        dt = np.diff(data[:, 0])
        freq = 1.0 / np.mean(dt)
        print(f"  {name} freq: ~{freq:.1f} Hz")

print("Message Frequencies:")
print_freq("POS", pos)
print_freq("ATT", att)
print_freq("RCOU", rc)
print_freq("XKF1", xkf1)

# Find takeoff, hover, landing segments based on XKF1 velocity and POS alt
# Let's align time arrays (we'll just use conditions)
# Hover criteria: after initial climb (VD goes from negative to near zero), 
# and before descent (VD becomes consistently positive).
# Let's smooth VD to find macroscopic phases.
xkf1_t = xkf1[:, 0]
vd = xkf1[:, 4]
alt_interp = np.interp(xkf1_t, pos[:, 0], pos[:, 1])

# Detect Takeoff: Alt > 0.1 and VD < -0.1
# Detect Hover: Alt > 1.5 and abs(VD) < 0.2 continuously for some seconds.
# Let's just find the first time VD > -0.2 after reaching 1.5m, and ends when VD > 0.3 for landing.
hover_start = None
hover_end = None

for i in range(len(xkf1_t)):
    if alt_interp[i] > 1.8 and abs(vd[i]) < 0.2:
        hover_start = xkf1_t[i]
        break

for i in range(len(xkf1_t)-1, -1, -1):
    if alt_interp[i] > 1.8 and abs(vd[i]) < 0.2:
        hover_end = xkf1_t[i]
        break

print(f"\nSegments (relative time):")
print(f"  Takeoff: start -> {hover_start:.1f} s")
print(f"  Hover:   {hover_start:.1f} s -> {hover_end:.1f} s (Duration: {hover_end - hover_start:.1f} s)")
print(f"  Landing: {hover_end:.1f} s -> end")

# HOVER ANALYSIS
idx_pos = (pos[:,0] >= hover_start) & (pos[:,0] <= hover_end)
idx_att = (att[:,0] >= hover_start) & (att[:,0] <= hover_end)
idx_rc  = (rc[:,0] >= hover_start) & (rc[:,0] <= hover_end)
idx_xkf = (xkf1[:,0] >= hover_start) & (xkf1[:,0] <= hover_end)

h_pos = pos[idx_pos]
h_att = att[idx_att]
h_rc  = rc[idx_rc]
h_xkf = xkf1[idx_xkf]

alt = h_pos[:, 1]
alt_target = 2.0
err = alt - alt_target
print("\n--- HOVER SUMMARY ---")
print(f"Altitude (POS.RelHomeAlt): Min={alt.min():.3f}, Mean={alt.mean():.3f}, Max={alt.max():.3f} m")
print(f"Alt Error (vs 2.0m Cmd): MaxAbs={np.max(np.abs(err)):.3f} m, RMS={np.sqrt(np.mean(err**2)):.3f} m")

# Speed and Position
# Horizontal speed = sqrt(VN^2 + VE^2)
speed_h = np.sqrt(h_xkf[:, 2]**2 + h_xkf[:, 3]**2)
vd_h = h_xkf[:, 4]
print(f"XKF1 EKF Core used: {int(h_xkf[0, 1])}")
print(f"Horiz Speed (sqrt(VN^2+VE^2)): Mean={np.mean(speed_h):.3f}, Max={np.max(speed_h):.3f} m/s")
print(f"Vert Speed (VD): MaxAbs={np.max(np.abs(vd_h)):.3f} m/s")

lat0, lng0 = h_pos[0, 2], h_pos[0, 3]
drift = np.array([haversine(lat0, lng0, lat, lng) for lat, lng in zip(h_pos[:, 2], h_pos[:, 3])])
print(f"Max Horiz Position Drift (vs hover start): {np.max(drift):.3f} m")

# Roll/Pitch/Yaw
roll = h_att[:, 1]
pitch = h_att[:, 2]
yaw = h_att[:, 3]
yaw_unwrapped = np.degrees(np.unwrap(np.radians(yaw)))

print(f"Roll: Mean={roll.mean():.3f}°, MaxAbs={np.max(np.abs(roll)):.3f}°, RMS={np.sqrt(np.mean(roll**2)):.3f}°")
print(f"Pitch: Mean={pitch.mean():.3f}°, MaxAbs={np.max(np.abs(pitch)):.3f}°, RMS={np.sqrt(np.mean(pitch**2)):.3f}°")
print(f"Yaw (unwrapped): Start-End Diff={yaw_unwrapped[-1]-yaw_unwrapped[0]:.3f}°, Range(Min-Max)={yaw_unwrapped.max()-yaw_unwrapped.min():.3f}°")

# Motors
rc_c1, rc_c2, rc_c3, rc_c4 = h_rc[:, 1], h_rc[:, 2], h_rc[:, 3], h_rc[:, 4]
def motor_stats(name, data):
    near_limit = np.sum((data >= 1890) | (data <= 1110)) / len(data) * 100
    print(f"  {name}: Mean={data.mean():.1f}, Min={data.min():.0f}, Max={data.max():.0f} | Sınırda Geçen Süre: {near_limit:.1f}%")

print(f"Motors (RCOU, 1100-1900 limits):")
motor_stats("C1 (FR/CCW)", rc_c1)
motor_stats("C2 (BL/CCW)", rc_c2)
motor_stats("C3 (FL/CW)", rc_c3)
motor_stats("C4 (BR/CW)", rc_c4)
