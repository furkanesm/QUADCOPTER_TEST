#!/usr/bin/env bash
# ==============================================================================
# simulation/run_phase_ab.sh
# Gazebo Fortress + ArduPilot SITL + MAVROS + Companion Otonom Uçuş Test Koşucusu
# ==============================================================================
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

chmod +x "$REPO_DIR/simulation/run_phase_ab.sh" 2>/dev/null || true

GZ_CONTAINER="s500_gz_sim"
GZ_GUI_CONTAINER="s500_gz_gui"
SITL_CONTAINER="s500_sitl_stack"

cleanup() {
    echo "[RUNNER] Temizlik yapılıyor..."
    docker stop "$GZ_CONTAINER" 2>/dev/null || true
    docker rm -f "$GZ_CONTAINER" 2>/dev/null || true
    docker stop "$GZ_GUI_CONTAINER" 2>/dev/null || true
    docker rm -f "$GZ_GUI_CONTAINER" 2>/dev/null || true
    docker stop "$SITL_CONTAINER" 2>/dev/null || true
    docker rm -f "$SITL_CONTAINER" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

MODE="${1:-measure}"
echo "=============================================================================="
echo "S500 Gazebo Tam Akış Doğrulama Betiği (run_phase_ab.sh)"
echo "Çalışma Modu: $MODE"
echo "=============================================================================="

# Log dizinini oluştur
mkdir -p "$REPO_DIR/simulation/run_logs"

# 1. Temizlik
cleanup

# 2. Gazebo Başlat (Container 1: autonomous-systems-onboard-otonomi_sistemi:latest)
echo "[RUNNER] Gazebo Fortress headless başlatılıyor..."
docker run -d --name "$GZ_CONTAINER" --network host --ipc host \
    -v "$REPO_DIR":/workspace \
    -w /workspace/simulation/ardupilot_gazebo_s500 \
    autonomous-systems-onboard-otonomi_sistemi:latest \
    bash -c '
        export IGN_GAZEBO_RESOURCE_PATH=/workspace/simulation/ardupilot_gazebo_s500/models:/workspace/simulation/ardupilot_gazebo_s500/worlds:$IGN_GAZEBO_RESOURCE_PATH
        export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
        ign gazebo -s -r worlds/hasmetlimiss_s500.world
    '

# Gazebo port 9002'nin açılmasını bekle
echo "[RUNNER] Gazebo port 9002 bekleniyor..."
for i in {1..30}; do
    if ss -tuln | grep -q ":9002 "; then
        echo "[RUNNER] Gazebo port 9002 hazır! ($i saniye)"
        break
    fi
    sleep 1
done

if ! ss -tuln | grep -q ":9002 "; then
    echo "[HATA] Gazebo port 9002 açılamadı!"
    docker logs "$GZ_CONTAINER" | tail -n 30
    exit 1
fi

if [ "${GUI:-0}" = "1" ]; then
    echo "[RUNNER] GUI=1 tespit edildi. Gazebo GUI başlatılıyor ($GZ_GUI_CONTAINER)..."
    docker run -d --name "$GZ_GUI_CONTAINER" --network host --ipc host \
        -e DISPLAY="${DISPLAY:-:0}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$REPO_DIR":/workspace \
        -w /workspace/simulation/ardupilot_gazebo_s500 \
        autonomous-systems-onboard-otonomi_sistemi:latest \
        bash -c '
            export IGN_GAZEBO_RESOURCE_PATH=/workspace/simulation/ardupilot_gazebo_s500/models:/workspace/simulation/ardupilot_gazebo_s500/worlds:$IGN_GAZEBO_RESOURCE_PATH
            export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
            ign gazebo -g
        ' || echo "[UYARI] Gazebo GUI başlatılamadı!"
fi

# 3. SITL ve Görev Yürütücüsünü Başlat (Container 2: s500-sitl-humble:latest)
echo "[RUNNER] SITL ve ROS 2 yığını başlatılıyor (Mod: $MODE)..."

docker run --name "$SITL_CONTAINER" --network host \
    -v "$REPO_DIR":/workspace \
    -w /workspace \
    s500-sitl-humble:latest \
    bash -c "
        source /opt/ros/humble/setup.bash
        source /workspace/ros2_ws/install/setup.bash 2>/dev/null || true
        export PYTHONUNBUFFERED=1
        export PYTHONDONTWRITEBYTECODE=1

        # 1. cv2 kontrolü ve gerekirse kurulumu
        if ! python3 -c 'import cv2' &>/dev/null; then
            echo '[RUNNER] cv2 modülü bulunamadı, pip ile yükleniyor...'
            pip install --no-input --quiet 'numpy<2' 'opencv-python-headless==4.9.0.80' > /workspace/simulation/run_logs/pip.log 2>&1 || {
                echo '[HATA] pip install başarısız oldu!'
                cat /workspace/simulation/run_logs/pip.log
                exit 1
            }
        fi

        # 2. Bağımlılıkların doğrulanması
        if ! python3 -c 'import cv2, numpy, vision_interfaces' &>/dev/null; then
            echo '[HATA] Python bağımlılıkları doğrulanamadı (cv2, numpy, vision_interfaces)!'
            python3 -c 'import cv2, numpy, vision_interfaces'
            exit 1
        fi

        python3 - << 'PYEOF'
import os
import sys
import time
import math
import subprocess
import threading
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Path, Odometry
from mavros_msgs.msg import State

MODE = '$MODE'
print(f'[PY-RUNNER] Python görev orkestratörü başladı. Hedef Mod: {MODE}', flush=True)

# Log dizini
LOG_DIR = '/workspace/simulation/run_logs'
os.makedirs(LOG_DIR, exist_ok=True)
planner_log_path = os.path.join(LOG_DIR, 'planner.log')
adapter_log_path = os.path.join(LOG_DIR, 'adapter.log')
sitl_lua_log_path = os.path.join(LOG_DIR, 'sitl_lua.log')
mavros_log_path = os.path.join(LOG_DIR, 'mavros.log')

planner_log_file = open(planner_log_path, 'w')
adapter_log_file = open(adapter_log_path, 'w')
sitl_lua_log_file = open(sitl_lua_log_path, 'w')
mavros_log_file = open(mavros_log_path, 'w')

# 1. /tmp/sitl_run çalışma dizini hazırla
os.makedirs('/tmp/sitl_run/scripts', exist_ok=True)
subprocess.run(['cp', '/workspace/s500_lua_mission/s500_mission.lua', '/tmp/sitl_run/scripts/'], check=True)
with open('/tmp/sitl_run/extra.parm', 'w') as f:
    f.write('SCR_ENABLE 1\nSCR_VM_I_RST 0\nSYSID_MYGCS 255\n')

# 2. ArduPilot SITL binary başlat
print('[PY-RUNNER] ArduCopter SITL başlatılıyor...', flush=True)
sitl_proc = subprocess.Popen([
    '/opt/ardupilot/build/sitl/bin/arducopter',
    '--model', 'JSON',
    '--speedup', '1',
    '--defaults', '/opt/ardupilot/Tools/autotest/default_params/copter.parm,/opt/ardupilot/Tools/autotest/default_params/gazebo-iris.parm,/workspace/simulation/ardupilot_gazebo_s500/s500_workspace/parameters/s500_2550kg_sitl_snapshot.parm,/tmp/sitl_run/extra.parm'
], cwd='/tmp/sitl_run', stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# 3. MAVProxy router başlat (5760 -> 14550, 14551, 14552)
time.sleep(2)
print('[PY-RUNNER] MAVProxy yönlendirici başlatılıyor...', flush=True)
mavproxy_proc = subprocess.Popen([
    'mavproxy.py',
    '--master', 'tcp:127.0.0.1:5760',
    '--out', 'udp:127.0.0.1:14550',
    '--out', 'udp:127.0.0.1:14551',
    '--out', 'udp:127.0.0.1:14552',
    '--daemon',
    '--default-modules', ''
], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# 4. MAVROS başlat
time.sleep(2)
print(f'[PY-RUNNER] MAVROS başlatılıyor (Log: {mavros_log_path})...', flush=True)
mavros_proc = subprocess.Popen([
    'ros2', 'run', 'mavros', 'mavros_node',
    '--ros-args',
    '-p', 'fcu_url:=udp://127.0.0.1:14550@14555',
    '-p', 'tgt_system:=1',
    '-p', 'tgt_component:=1'
], stdout=mavros_log_file, stderr=subprocess.STDOUT, text=True)

# 5. MAVLink Companion Adapter başlat (çıktı adapter.log dosyasına)
time.sleep(2)
print(f'[PY-RUNNER] mavlink_mission_commander başlatılıyor (Log: {adapter_log_path})...', flush=True)
adapter_proc = subprocess.Popen([
    'python3', '/workspace/companion/mavlink_mission_commander.py',
    '--ros-args',
    '-p', 'mavlink_connection:=udp:127.0.0.1:14551'
], stdout=adapter_log_file, stderr=subprocess.STDOUT, text=True)

# 6. ROS2 İzleyici Düğümü Kur
rclpy.init()
class MissionWatcher(Node):
    def __init__(self):
        super().__init__('mission_watcher')
        self.ground_pose = None
        self.hedef_bekle_pose = None
        self.current_pose = None
        self.landing_pose = None
        self.odom_origin = None
        self.takeoff_ref = None
        self.fsm_state = 'UNKNOWN'
        self.session_id = None
        self.statustexts = []
        self.events = []
        self.return_path_msg = None
        self.is_armed = None
        self.was_armed_seen = False

        self.sub_state = self.create_subscription(
            State, '/mavros/state', self.state_cb, 10)
        self.sub_pose = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(
            Odometry, '/mavros/local_position/odom', self.odom_cb, qos_profile_sensor_data)
        self.sub_event = self.create_subscription(
            String, '/adapter/event_status', self.event_cb, 10)
        self.sub_takeoff = self.create_subscription(
            PoseStamped, '/mission/takeoff_return_point', self.takeoff_cb, 10)
        self.sub_return_path = self.create_subscription(
            Path, '/planner/return_path', self.return_path_cb, 10)

    def state_cb(self, msg):
        self.is_armed = msg.armed
        if msg.armed:
            self.was_armed_seen = True

    def pose_cb(self, msg):
        self.current_pose = msg
        if self.ground_pose is None:
            self.ground_pose = msg

    def odom_cb(self, msg):
        if self.odom_origin is None:
            self.odom_origin = msg

    def event_cb(self, msg):
        txt = msg.data
        self.events.append((time.time(), txt))
        if txt.startswith('SESSION_ACTIVE:'):
            try:
                self.session_id = int(txt.split(':')[1])
            except:
                pass
        elif txt.startswith('OBSERVED_STATE:'):
            self.fsm_state = txt.split(':')[1].strip()
        elif txt.startswith('CORRELATED_TRANSITION:'):
            parts = txt.split(':')
            if len(parts) >= 2:
                self.fsm_state = parts[1].strip()

    def takeoff_cb(self, msg):
        self.takeoff_ref = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def return_path_cb(self, msg):
        self.return_path_msg = msg

watcher = MissionWatcher()

# Servis istemcileri
cli_start_session = watcher.create_client(Trigger, '/adapter/start_session')
print('[PY-RUNNER] Servis /adapter/start_session bekleniyor...', flush=True)
cli_start_session.wait_for_service(timeout_sec=15.0)

from mavros_msgs.srv import SetMode
cli_set_mode = watcher.create_client(SetMode, '/mavros/set_mode')
cli_set_mode.wait_for_service(timeout_sec=10.0)

# Path planner node başlat (çıktı planner.log dosyasına)
planner_proc = None
if MODE == 'measure':
    print(f'[PY-RUNNER] PathPlannerNode başlatılıyor (Varsayılan, Log: {planner_log_path})...', flush=True)
    planner_proc = subprocess.Popen([
        'python3', '/workspace/companion/path_planner_node.py'
    ], stdout=planner_log_file, stderr=subprocess.STDOUT, text=True)
elif MODE in ['full', 'failsafe']:
    # Hesaplanan genişletilmiş arena parametreleri:
    print(f'[PY-RUNNER] PathPlannerNode başlatılıyor (Genişletilmiş Arena, Log: {planner_log_path})...', flush=True)
    planner_proc = subprocess.Popen([
        'python3', '/workspace/companion/path_planner_node.py',
        '--ros-args',
        '-p', 'arena_width:=65.0',
        '-p', 'arena_height:=50.0',
        '-p', 'origin_offset_x:=20.0',
        '-p', 'origin_offset_y:=25.0',
        '-p', 'strict_boundary_walls:=false'
    ], stdout=planner_log_file, stderr=subprocess.STDOUT, text=True)

# Oturumu başlat
print('[PY-RUNNER] Oturum başlatma isteği gönderiliyor (/adapter/start_session)...', flush=True)
req = Trigger.Request()
fut = cli_start_session.call_async(req)
rclpy.spin_until_future_complete(watcher, fut, timeout_sec=5.0)
print(f'[PY-RUNNER] Start session cevabı: {fut.result()}', flush=True)

# SITL STATUSTEXT izleme ve sitl_lua.log yazma iş parçacığı
def watch_sitl_logs():
    time.sleep(1.0)
    try:
        m = mavutil.mavlink_connection('udp:127.0.0.1:14552')
        while rclpy.ok():
            msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)
            if msg and msg.text:
                t = msg.text
                line = f'{time.strftime(\"%Y-%m-%d %H:%M:%S\")} [STATUSTEXT] {t}\n'
                sitl_lua_log_file.write(line)
                sitl_lua_log_file.flush()
                if '[S500 LUA]' in t or 'AP:' in t:
                    print(f'  [SITL-LOG] {t}', flush=True)
    except Exception as e:
        print(f'[PY-RUNNER] watch_sitl_logs hatası: {e}', flush=True)

log_thread = threading.Thread(target=watch_sitl_logs, daemon=True)
log_thread.start()

# Yerdeki konumu bekle ve kaydet
print('[PY-RUNNER] Yerdeki pose (/mavros/local_position/pose) bekleniyor (Maks 90s)...', flush=True)
t_start = time.time()
while time.time() - t_start < 90:
    rclpy.spin_once(watcher, timeout_sec=0.2)
    if watcher.ground_pose is not None:
        break

if watcher.ground_pose is None:
    print('[HATA] 90 saniye içinde /mavros/local_position/pose verisi alınamadı!', flush=True)
    print('--- ros2 topic list | grep mavros ---', flush=True)
    subprocess.run('ros2 topic list | grep mavros || true', shell=True)
    print('--- ros2 topic echo /mavros/state --once ---', flush=True)
    subprocess.run('timeout 5 ros2 topic echo /mavros/state --once || true', shell=True)
    print('--- ros2 topic info /mavros/local_position/pose -v ---', flush=True)
    subprocess.run('ros2 topic info /mavros/local_position/pose -v || true', shell=True)
    if planner_proc:
        planner_proc.terminate()
    adapter_proc.terminate()
    mavros_proc.terminate()
    mavproxy_proc.terminate()
    sitl_proc.terminate()
    planner_log_file.close()
    adapter_log_file.close()
    sitl_lua_log_file.close()
    mavros_log_file.close()
    rclpy.shutdown()
    sys.exit(1)

gp = watcher.ground_pose.pose.position
print(f'[ÖLÇÜM] Yerdeki Konum (Kalkış Öncesi ENU): x={gp.x:.4f}, y={gp.y:.4f}, z={gp.z:.4f}', flush=True)

# GUIDED moda geç (Lua durum makinesini tetikle!)
# Not: Arm ve Takeoff komutunu s500_mission.lua kendisi yürütmektedir (HAZIRLIK -> arming:arm(), DIKEY_TIRMANIS -> vehicle:start_takeoff(33.0))
print('[PY-RUNNER] GUIDED moda geçiş komutu gönderiliyor...', flush=True)
mode_req = SetMode.Request()
mode_req.custom_mode = 'GUIDED'
fut_mode = cli_set_mode.call_async(mode_req)
rclpy.spin_until_future_complete(watcher, fut_mode, timeout_sec=5.0)
print(f'[PY-RUNNER] GUIDED mod geçiş sonucu: {fut_mode.result()}', flush=True)

# MAX_FLIGHT_WAIT yapılandırması
wait_limits = {
    'measure': 200,
    'full': 400,
    'failsafe': 500
}
MAX_FLIGHT_WAIT = wait_limits.get(MODE, 300)
print(f'[PY-RUNNER] Görev izleme döngüsü başladı (Maksimum Süre: {MAX_FLIGHT_WAIT}s, Mod: {MODE})...', flush=True)

hedef_bekle_reached = False
t_flight_start = time.time()
fake_published = False

while (time.time() - t_flight_start) < MAX_FLIGHT_WAIT:
    rclpy.spin_once(watcher, timeout_sec=0.2)
    
    # HEDEF_BEKLE durumuna ulaşıldığında ölçüm yap
    if watcher.fsm_state == 'HEDEF_BEKLE' and not hedef_bekle_reached:
        hedef_bekle_reached = True
        t_hedef_bekle = time.time()
        watcher.hedef_bekle_pose = watcher.current_pose
        hp = watcher.hedef_bekle_pose.pose.position
        print('='*70, flush=True)
        print(f'[ÖLÇÜM] HEDEF_BEKLE Durumuna Ulaşıldı! (Geçen Süre: {time.time()-t_flight_start:.1f}s)', flush=True)
        print(f'[ÖLÇÜM] HEDEF_BEKLE Konumu (ENU): x={hp.x:.4f}, y={hp.y:.4f}, z={hp.z:.4f}', flush=True)
        if watcher.takeoff_ref:
            print(f'[ÖLÇÜM] Kilitli Kalkış Referansı (takeoff_return_pos_ned 3D): {watcher.takeoff_ref}', flush=True)
        print('='*70, flush=True)

        if MODE == 'measure':
            print('[PY-RUNNER] Ölçüm modu tamamlandı. Döngü sonlandırılıyor.', flush=True)
            break
            
        elif MODE == 'full' and not fake_published:
            print('[PY-RUNNER] FULL MOD: Sahte tespit yayınlayıcı çalıştırılıyor...', flush=True)
            subprocess.run([
                'python3', '/workspace/simulation/publish_fake_detections.py',
                '--session-id', str(watcher.session_id or 1)
            ], check=True)
            fake_published = True

    # Full veya Failsafe modlarında iniş / tamamlanma kontrolü
    if hedef_bekle_reached and MODE in ['full', 'failsafe']:
        cur_z = watcher.current_pose.pose.position.z if watcher.current_pose else 999.0
        
        # Döngüyü bitirme koşulu:
        # (1) FSM durumu INIS ve irtifa yere yakın (z < 0.5 m)
        # (2) FSM durumu TAMAMLANDI (State 9)
        # (3) Kalkıştan sonra disarm teyidi alınması
        if (watcher.fsm_state == 'INIS' and cur_z < 0.5) or \
           (watcher.fsm_state == 'TAMAMLANDI') or \
           (watcher.was_armed_seen and watcher.is_armed is False):
            watcher.landing_pose = watcher.current_pose
            print('='*70, flush=True)
            print(f'[PY-RUNNER] İNİŞ / GÖREV TAMAMLANMASI TESPİT EDİLDİ!', flush=True)
            print(f'[PY-RUNNER] Son Durum: {watcher.fsm_state}, İrtifa: {cur_z:.2f}m, Armed: {watcher.is_armed}', flush=True)
            print('='*70, flush=True)
            break

    time.sleep(0.1)

# Rapor Özeti Yazdır
print('\n' + '='*75, flush=True)
print('ÖLÇÜM VE TEST SONUÇLARI:', flush=True)
if watcher.ground_pose:
    gp = watcher.ground_pose.pose.position
    print(f'1. Yerdeki Konum (ENU)    : x={gp.x:.4f}, y={gp.y:.4f}, z={gp.z:.4f}')
if watcher.hedef_bekle_pose:
    hp = watcher.hedef_bekle_pose.pose.position
    print(f'2. HEDEF_BEKLE Konum (ENU): x={hp.x:.4f}, y={hp.y:.4f}, z={hp.z:.4f}')
    print(f'   NED Karşılığı (Tahmin) : North={hp.y:.4f}, East={hp.x:.4f}, Down={-hp.z:.4f}')
if watcher.takeoff_ref:
    print(f'3. Kalkış Ref (NED 3D)    : North={watcher.takeoff_ref[0]:.4f}, East={watcher.takeoff_ref[1]:.4f}, Down={watcher.takeoff_ref[2]:.4f}')
if watcher.landing_pose and watcher.takeoff_ref:
    lp = watcher.landing_pose.pose.position
    # ENU -> NED: North=lp.y, East=lp.x
    d_north = lp.y - watcher.takeoff_ref[0]
    d_east = lp.x - watcher.takeoff_ref[1]
    horiz_drift = math.hypot(d_north, d_east)
    print(f'4. İnişteki Yatay Sapma   : {horiz_drift:.3f} metre (Kalkış referansına göre)')
print(f'5. Son FSM Durumu         : {watcher.fsm_state}')
print('='*75, flush=True)

# Süreçleri sonlandır
if planner_proc:
    planner_proc.terminate()
adapter_proc.terminate()
mavros_proc.terminate()
mavproxy_proc.terminate()
sitl_proc.terminate()

planner_log_file.close()
adapter_log_file.close()
sitl_lua_log_file.close()
mavros_log_file.close()
rclpy.shutdown()
PYEOF
    "

echo "[RUNNER] run_phase_ab.sh tamamlandı."
