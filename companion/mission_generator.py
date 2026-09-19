"""
mission_generator.py — Lawnmower / Tek-Geçiş Mission Üretici
==============================================================
20x20m'lik alanı kaplayan waypoint listesi oluşturur ve SITL'e yükler.

NEDEN TEK GEÇİŞ:
  30m irtifada 62.2° HFOV ile yerdeki kapsama genişliği ~36.2m.
  %10 örtüşme sonrası net strip width ~32.6m.
  20m genişliğindeki alan tek şeritte tamamen kaplandığından,
  sadece alanın bir kenarından diğerine düz bir hat yeterli.
  Daha az waypoint = daha hızlı test, daha az sorun riski.

NEDEN pysmavlink wp formatı:
  pymavlink'in mavwp modülü, MAVLink MISSION_ITEM mesajlarını
  oluşturmak ve upload etmek için hazır araçlar sağlar.
  Manuel struct packing'e gerek kalmaz.
"""

import math
from pymavlink import mavutil

# ─── config'den import ───────────────────────────────────
from config import ALTITUDE, AREA_SIZE, STRIP_WIDTH


def gps_offset(lat: float, lon: float, east_m: float, north_m: float):
    """
    Bir GPS noktasından metre cinsinden offset hesaplar.
    Küçük mesafeler için (<1km) düz dünya yaklaşımı yeterli.

    NEDEN BU FONKSİYON:
      mavextra.gps_offset() pymavlink'te mevcut ama import yolu
      karmaşık olabiliyor. 20m mesafeler için basit aritmetik
      yeterli doğruluğu sağlar (~mm hassasiyet).

    Args:
        lat, lon: Başlangıç noktası (derece)
        east_m:   Doğu yönünde offset (metre, + = doğu)
        north_m:  Kuzey yönünde offset (metre, + = kuzey)

    Returns:
        (new_lat, new_lon) tuple
    """
    # 1 derece enlem ≈ 111320m (sabit)
    # 1 derece boylam ≈ 111320m * cos(lat) (enleme bağlı)
    dlat = north_m / 111320.0
    dlon = east_m / (111320.0 * math.cos(math.radians(lat)))
    return (lat + dlat, lon + dlon)


def generate_single_pass_waypoints(
    home_lat: float,
    home_lon: float,
    altitude: float = ALTITUDE,
    area_size: float = AREA_SIZE,
) -> list:
    """
    20x20m alanı tek geçişle tarayan waypoint listesi üretir.

    Plan (kuşbakışı — kuzey yukarı):

        HOME (0,0)
          |
          v  (takeoff)
          |
        WP1 ────────────────► WP2
     (-10, -10)             (-10, +10)
          ▲                     |
          |                     v
        WP4 ◄──────────────── WP3
     (+10, -10)             (+10, +10)
          |
          v  (alan sonunda otomatik RTL beklenebilir
               veya detection loop RTL tetikler)

    Offsetler home noktasını merkez alarak ±10m (area_size/2).
    """
    half = area_size / 2.0
    waypoints = []

    # ── Sıra 0: HOME ──
    # MAVLink convention: seq=0 = home noktası
    waypoints.append({
        "seq": 0,
        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        "param1": 0, "param2": 0, "param3": 0, "param4": 0,
        "lat": home_lat,
        "lon": home_lon,
        "alt": 0,
    })

    # ── Sıra 1: TAKEOFF ──
    waypoints.append({
        "seq": 1,
        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        "command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        "param1": 15,    # minimum pitch (derece) — copter'da ignored
        "param2": 0, "param3": 0, "param4": 0,
        "lat": home_lat,
        "lon": home_lon,
        "alt": altitude,
    })

    # ── Tarama waypoint'leri: alan köşelerini gez ──
    # 4 köşe → Güney-batı'dan başla, saat yönünde dön
    corners = [
        (-half, -half),   # WP2: güney-batı
        (-half, +half),   # WP3: güney-doğu
        (+half, +half),   # WP4: kuzey-doğu
        (+half, -half),   # WP5: kuzey-batı
    ]

    for i, (north_offset, east_offset) in enumerate(corners):
        wp_lat, wp_lon = gps_offset(home_lat, home_lon, east_offset, north_offset)
        waypoints.append({
            "seq": i + 2,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "param1": 0,    # hold time (s)
            "param2": 2,    # acceptance radius (m)
            "param3": 0,    # pass through (0=stop)
            "param4": 0,    # yaw (0=auto)
            "lat": wp_lat,
            "lon": wp_lon,
            "alt": altitude,
        })

    # ── Son: RTL komutu ──
    # Mission bittiğinde otomatik RTL. Detection loop zaten
    # daha önce RTL tetikleyebilir, bu sadece güvenlik ağı.
    waypoints.append({
        "seq": len(waypoints),
        "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        "command": mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        "param1": 0, "param2": 0, "param3": 0, "param4": 0,
        "lat": 0, "lon": 0, "alt": 0,
    })

    return waypoints


def upload_mission(master, waypoints: list):
    """
    Waypoint listesini MAVLink üzerinden SITL'e gönderir.

    NEDEN ELLE UPLOAD:
      pymavlink'in mavwp.MAVWPLoader sınıfı var ama
      basit kullanımlarda doğrudan mission_item göndermek
      daha az sürprizli. Akış:

      1. MISSION_COUNT gönder (toplam wp sayısı)
      2. SITL her MISSION_REQUEST'e karşılık ilgili wp'yi gönder
      3. MISSION_ACK bekle

    Args:
        master: pymavlink bağlantı nesnesi
        waypoints: generate_single_pass_waypoints() çıktısı
    """
    # ── Adım 1: Kaç tane waypoint olduğunu bildir ──
    master.waypoint_count_send(len(waypoints))
    print(f"[MISSION] {len(waypoints)} waypoint yükleniyor...")

    # ── Adım 2: Her MISSION_REQUEST'e cevap ver ──
    for i in range(len(waypoints)):
        # SITL sırayla MISSION_REQUEST gönderir
        msg = master.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
                                blocking=True, timeout=10)
        if msg is None:
            print(f"[MISSION] HATA: WP {i} için MISSION_REQUEST timeout!")
            return False

        wp = waypoints[msg.seq]

        # MISSION_ITEM gönder
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            wp["seq"],                     # seq
            wp["frame"],                   # frame
            wp["command"],                 # command
            1 if wp["seq"] == 1 else 0,    # current (ilk navigasyon wp=1)
            1,                             # autocontinue
            wp["param1"],
            wp["param2"],
            wp["param3"],
            wp["param4"],
            int(wp["lat"] * 1e7),          # lat (degE7)
            int(wp["lon"] * 1e7),          # lon (degE7)
            wp["alt"],                     # alt
        )
        print(f"  WP[{wp['seq']}] cmd={wp['command']} "
              f"lat={wp['lat']:.7f} lon={wp['lon']:.7f} alt={wp['alt']}m")

    # ── Adım 3: ACK bekle ──
    ack = master.recv_match(type="MISSION_ACK", blocking=True, timeout=10)
    if ack is None:
        print("[MISSION] HATA: MISSION_ACK alınamadı!")
        return False

    if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("[MISSION] Mission başarıyla yüklendi ✓")
        return True
    else:
        print(f"[MISSION] HATA: Mission reddedildi, tip={ack.type}")
        return False


if __name__ == "__main__":
    # Bağımsız test: waypoint'leri sadece konsola yazdır
    # Varsayılan CMAC lokasyonu (ArduPilot SITL default home)
    DEFAULT_LAT = -35.363262
    DEFAULT_LON = 149.165237

    wps = generate_single_pass_waypoints(DEFAULT_LAT, DEFAULT_LON)
    print(f"\n{'='*60}")
    print(f"  Tek-geçiş mission planı ({len(wps)} waypoint)")
    print(f"  Home: ({DEFAULT_LAT}, {DEFAULT_LON})")
    print(f"  Alan: {AREA_SIZE}x{AREA_SIZE}m, İrtifa: {ALTITUDE}m")
    print(f"  Strip width: {STRIP_WIDTH:.1f}m (tek şerit yeterli)")
    print(f"{'='*60}\n")

    for wp in wps:
        cmd_name = {
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT: "WAYPOINT",
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "TAKEOFF",
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH: "RTL",
        }.get(wp["command"], f"CMD_{wp['command']}")

        print(f"  [{wp['seq']}] {cmd_name:10s} "
              f"lat={wp['lat']:12.7f}  lon={wp['lon']:12.7f}  alt={wp['alt']}m")
