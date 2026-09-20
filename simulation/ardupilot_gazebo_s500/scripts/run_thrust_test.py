#!/usr/bin/env python3
import os
import sys
import subprocess
import time
import datetime
import math
import csv
import re
import signal
import json
import threading

# --- CONFIG ---
WORLD_PATH = "/home/furkan/s500_sim/ardupilot_gazebo/worlds/rotor_test.world"
RESULTS_BASE = "/home/furkan/s500_sim/thrust_test_results"
FT_TOPIC = "/world/rotor_test/model/rotor_stand/joint/ft_joint/sensor/ft_sensor/forcetorque"
JS_TOPIC = "/world/rotor_test/model/rotor_stand/joint_state"
CMD_TOPIC = "/model/rotor_stand/joint/rotor_0_joint/cmd_vel"

K_VAL = 2.6269644483e-5
GRAVITY = 9.80665
TEST_POINTS = [
    (2900, 270),
    (4450, 580),
    (5400, 860),
    (6874, 1370)
]

IGN_ENV = os.environ.copy()
IGN_ENV["IGN_PARTITION"] = "thrust_test_partition"

gz_proc = None

def cleanup():
    global gz_proc
    if gz_proc is not None and gz_proc.poll() is None:
        print("\nTerminating Gazebo...")
        gz_proc.terminate()
        try:
            gz_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print("Gazebo did not terminate in 5s, killing...")
            gz_proc.kill()
            gz_proc.wait()

def signal_handler(sig, frame):
    print("\nCtrl+C detected. Shutting down cleanly.")
    cleanup()
    sys.exit(1)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

class TopicReader(threading.Thread):
    def __init__(self, topic, is_ft):
        super().__init__()
        self.topic = topic
        self.is_ft = is_ft
        self.samples = []
        self.running = True
        self.parse_errors = 0
        self.proc = subprocess.Popen(
            ["stdbuf", "-oL", "ign", "topic", "-e", "-t", topic],
            env=IGN_ENV, stdout=subprocess.PIPE, text=True
        )

    def run(self):
        buffer = []
        while self.running:
            line = self.proc.stdout.readline()
            if not line:
                if self.proc.poll() is not None:
                    break
                continue
            
            if "header {" in line:
                if buffer:
                    self.parse_msg("".join(buffer))
                buffer = [line]
            elif buffer:
                buffer.append(line)
                
        if buffer:
            self.parse_msg("".join(buffer))
            
    def parse_msg(self, msg):
        sec_m = re.search(r'sec:\s*(\d+)', msg)
        nsec_m = re.search(r'nsec:\s*(\d+)', msg)
        
        if not sec_m and not nsec_m and "stamp" not in msg:
            self.parse_errors += 1
            return
            
        sec = int(sec_m.group(1)) if sec_m else 0
        nsec = int(nsec_m.group(1)) if nsec_m else 0
        t = sec + nsec * 1e-9
        
        if self.is_ft:
            if "force {" in msg or "torque {" in msg:
                fz = 0.0
                force_block = re.search(r'force\s*{([^}]*)}', msg)
                if force_block:
                    z_m = re.search(r'z:\s*([-e0-9.]+)', force_block.group(1))
                    if z_m:
                        fz = float(z_m.group(1))
                self.samples.append((t, fz))
            else:
                self.parse_errors += 1
        else:
            joint_blocks = msg.split("joint {")[1:]
            for jb in joint_blocks:
                if 'name: "rotor_0_joint"' in jb:
                    vel = 0.0
                    vel_m = re.search(r'velocity:\s*([-e0-9.]+)', jb)
                    if vel_m:
                        vel = float(vel_m.group(1))
                    self.samples.append((t, vel))

    def stop(self):
        self.running = False
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except:
                self.proc.kill()
                self.proc.wait()

def set_vel(rad_s):
    try:
        subprocess.run(["ign", "topic", "-t", CMD_TOPIC, "-m", "ignition.msgs.Double", "-p", f"data: {rad_s}"], 
                       env=IGN_ENV, capture_output=True, timeout=5)
    except Exception as e:
        print(f"Error setting velocity: {e}")

def measure_steady_state(target_rpm, timeout=20.0):
    reader_ft = TopicReader(FT_TOPIC, True)
    reader_js = TopicReader(JS_TOPIC, False)
    reader_ft.start()
    reader_js.start()
    
    start_time = time.time()
    last_err = "Veri bekleniyor"
    best_res = None
    
    try:
        while time.time() - start_time < timeout:
            time.sleep(0.5)
            
            ft_samples = list(reader_ft.samples)
            js_samples = list(reader_js.samples)
            
            if not ft_samples or not js_samples:
                last_err = f"Veri yok (FT: {len(ft_samples)}, JS: {len(js_samples)})"
                continue
                
            t_min = max(ft_samples[0][0], js_samples[0][0])
            t_max = min(ft_samples[-1][0], js_samples[-1][0])
            t_window = t_max - t_min
            
            if t_window < 1.0:
                last_err = f"Ortak zaman < 1.0s (Pencere: {t_window:.3f} s)"
                continue
                
            # Filter samples within the strict overlap window
            ft_filtered = [s[1] for s in ft_samples if t_min <= s[0] <= t_max]
            js_filtered = [s[1] for s in js_samples if t_min <= s[0] <= t_max]
            
            n_ft = len(ft_filtered)
            n_js = len(js_filtered)
            
            if n_ft < 30 or n_js < 30:
                last_err = f"Yetersiz ornek (N_FT: {n_ft}, N_JS: {n_js})"
                continue
                
            ft_mean = sum(ft_filtered) / n_ft
            js_mean = sum(js_filtered) / n_js
            
            ft_std = math.sqrt(sum((x - ft_mean)**2 for x in ft_filtered) / n_ft)
            js_std = math.sqrt(sum((x - js_mean)**2 for x in js_filtered) / n_js)
            
            rate_ft = n_ft / t_window if t_window > 0 else 0
            rate_js = n_js / t_window if t_window > 0 else 0
            
            if js_std < 5.0 and ft_std < 0.5:
                best_res = {
                    'success': True,
                    'ft_mean': ft_mean,
                    'ft_std': ft_std,
                    'js_mean': js_mean,
                    'js_std': js_std,
                    'n_ft': n_ft,
                    'n_js': n_js,
                    'rate_ft': rate_ft,
                    'rate_js': rate_js,
                    't_min': t_min,
                    't_max': t_max,
                    't_window': t_window,
                    'raw_ft': ft_samples,
                    'raw_js': js_samples
                }
                break
            else:
                last_err = f"Kararsiz olcum (JS_Std: {js_std:.2f}, FT_Std: {ft_std:.2f})"
                
    finally:
        reader_ft.stop()
        reader_js.stop()
        reader_ft.join(timeout=2)
        reader_js.join(timeout=2)
        
    if best_res:
        return best_res
        
    # Timeout occurred
    t_win = 0.0
    if reader_ft.samples and reader_js.samples:
        t_m = max(reader_ft.samples[0][0], reader_js.samples[0][0])
        t_M = min(reader_ft.samples[-1][0], reader_js.samples[-1][0])
        t_win = max(0.0, t_M - t_m)
        
    return {
        'success': False, 
        'reason': f"Timeout: {last_err} | Pencere={t_win:.3f}s | N_FT={len(reader_ft.samples)} N_JS={len(reader_js.samples)} | Hata_FT={reader_ft.parse_errors} Hata_JS={reader_js.parse_errors}"
    }

def main():
    global gz_proc
    
    os.makedirs(RESULTS_BASE, exist_ok=True)
    run_id = datetime.datetime.now().strftime("run_%Y%m%d_%H%M%S")
    run_dir = os.path.join(RESULTS_BASE, run_id)
    os.makedirs(run_dir, exist_ok=True)
    
    gz_log = open(os.path.join(run_dir, "gazebo.log"), "w")
    
    try:
        print(f"Starting Gazebo in partition: {IGN_ENV['IGN_PARTITION']}")
        gz_proc = subprocess.Popen(["ign", "gazebo", "-s", "-r", WORLD_PATH], 
                                   env=IGN_ENV, stdout=gz_log, stderr=subprocess.STDOUT)
        time.sleep(5)
        
        print("Measuring Tare (0 RPM) with steady-state criteria...")
        set_vel(0.0)
        tare_res = measure_steady_state(0.0, timeout=20.0)
        if not tare_res['success']:
            print(f"Error: Could not achieve steady state for Tare measurement: {tare_res.get('reason', 'Unknown')}")
            sys.exit(1)
            
        tare_z = tare_res['ft_mean']
        print(f"Tare Z Force: {tare_z:.4f} N (StdDev: {tare_res['ft_std']:.4f})")
        
        # --- KUVVET YÖNÜ AÇIKLAMASI ---
        # Sensör child_to_parent yönünde ölçüm yapmaktadır (child=base_link, parent=dummy_link).
        # Gravity (yerçekimi) child link'i (base_link) ve rotoru -Z (aşağı) yönde çeker, bu da parent üzerinde aşağı yönlü eksi (-) kuvvet oluşturur.
        # İtki (Thrust) oluştuğunda child yukarı itilir, parent üzerindeki aşağı yönlü çekme kuvveti azalır (yani kuvvet + yönde değişir).
        # Sonuç olarak: İşaretli İtki = (Ham Fz) - (Dara Fz).
        # Beklenen Dara: ~(1.0 + 0.010625) kg * -9.8 m/s^2 (Gazebo varsayilan gravity) = ~-9.904125 N.
        # Not: İtkinin gf cinsine çevrilmesinde ise uluslararası standart g=9.80665 m/s^2 kullanılmaktadır.
        
        csv_path = os.path.join(run_dir, "summary.csv")
        raw_path = os.path.join(run_dir, "raw_data.json")
        raw_data_log = {"tare": tare_res, "tests": []}
        all_passed = True
        
        with open(csv_path, "w", newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Target_RPM", "Meas_RPM", "RPM_StdDev", "T_Min_s", "T_Max_s", "T_Window_s", 
                             "N_FT", "N_JS", "Rate_FT_Hz", "Rate_JS_Hz",
                             "Raw_Fz_N", "Tare_Fz_N", "Signed_Thrust_N", "Thrust_StdDev_N", 
                             "Signed_Thrust_gf", "Formula_N", "Ref_gf", "Diff_Pct", "Status"])
                             
            print(f"\n{'Target RPM':>10} | {'Meas RPM':>10} | {'Signed N':>10} | {'Signed gf':>10} | {'Formula N':>10} | {'Ref gf':>10} | {'Diff %':>8}")
            print("-" * 87)
            
            for rpm_target, ref_gf in TEST_POINTS:
                omega_target = rpm_target * 2 * math.pi / 60.0
                set_vel(omega_target)
                
                res = measure_steady_state(rpm_target, timeout=20.0)
                
                if not res['success']:
                    print(f"{rpm_target:10.0f} | FAILED: {res.get('reason', 'Unknown')}")
                    writer.writerow([rpm_target, "", "", "", "", "", "", "", "", "", "", tare_z, "", "", "", "", ref_gf, "", f"FAILED: {res.get('reason', 'Unknown')}"])
                    all_passed = False
                    continue
                
                meas_rpm = res['js_mean'] * 60.0 / (2 * math.pi)
                rpm_std = res['js_std'] * 60.0 / (2 * math.pi)
                
                raw_fz = res['ft_mean']
                signed_thrust_n = raw_fz - tare_z
                signed_thrust_gf = signed_thrust_n * 1000.0 / GRAVITY
                
                formula_n = K_VAL * (res['js_mean'] ** 2)
                diff_pct = (signed_thrust_gf - ref_gf) / ref_gf * 100.0
                
                print(f"{rpm_target:10.0f} | {meas_rpm:10.0f} | {signed_thrust_n:10.4f} | {signed_thrust_gf:10.1f} | {formula_n:10.4f} | {ref_gf:10.1f} | {diff_pct:7.2f}%")
                
                writer.writerow([
                    rpm_target, f"{meas_rpm:.2f}", f"{rpm_std:.2f}", f"{res['t_min']:.3f}", f"{res['t_max']:.3f}", f"{res['t_window']:.3f}",
                    res['n_ft'], res['n_js'], f"{res['rate_ft']:.1f}", f"{res['rate_js']:.1f}",
                    f"{raw_fz:.4f}", f"{tare_z:.4f}", f"{signed_thrust_n:.4f}", f"{res['ft_std']:.4f}",
                    f"{signed_thrust_gf:.1f}", f"{formula_n:.4f}", ref_gf, f"{diff_pct:.2f}", "SUCCESS"
                ])
                
                raw_data_log["tests"].append({
                    "target_rpm": rpm_target,
                    "result": res
                })
        
        with open(raw_path, "w") as f:
            json.dump(raw_data_log, f, indent=2)
            
        print(f"\nTest complete. Results saved in: {run_dir}")
        if not all_passed:
            sys.exit(1)
            
    except Exception as e:
        print(f"Execution failed: {e}")
        sys.exit(1)
    finally:
        set_vel(0.0)
        cleanup()
        gz_log.close()

if __name__ == "__main__":
    main()
