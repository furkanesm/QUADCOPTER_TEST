"""
search_and_rtl.py — Ana Orkestrasyon Script'i
===============================================
20x20m alanda İHA ile hedef arama ve otomatik RTL.

İKİ MOD:
  A) --test-image: Sadece YOLO inference testi. MAVLink/SITL gerekmez.
     Görüntüyü yükler, best.pt ile inference çalıştırır, sonuçları yazdırır, çıkar.
  B) Tam uçuş akışı (--test-image YOK):
     1. SITL'e pymavlink ile bağlan
     2. Mission yükle → Arm → Takeoff → AUTO → Detection loop → RTL

KULLANIM:
  # Sadece inference testi (SITL gerekmez):
  python search_and_rtl.py --test-image hedef.jpg

  # Tam uçuş + detection (SITL çalışmalı):
  python search_and_rtl.py --connect tcp:127.0.0.1:5762

  # Dry-run (mission yükle, arm etme):
  python search_and_rtl.py --connect tcp:127.0.0.1:5762 --dry-run

  # Test frame enjeksiyonu (AUTO'dan 8sn sonra):
  python search_and_rtl.py --connect tcp:127.0.0.1:5762 --inject-image Hedef_0002.jpg --inject-at 8
"""

import argparse
import sys
import time
import os

import cv2

# ── Proje modülleri ──
from config import (
    MAVLINK_CONNECTION,
    ALTITUDE,
    TAKEOFF_ALTITUDE,
    CONFIDENCE_THRESHOLD,
    MODEL_PATH,
    TARGET_CLASS_NAME,
    TARGET_CLASS_ID,
    DETECTION_INTERVAL,
    HEARTBEAT_TIMEOUT,
)
from detector import TargetDetector


# ─────────────────────────────────────────────────────────
#  YARDIMCI FONKSİYONLAR (uçuş modu)
# ─────────────────────────────────────────────────────────

def wait_for_heartbeat(master):
    """
    İlk heartbeat mesajını bekler. Bu olmadan hiçbir komut gönderilemez.
    """
    from pymavlink import mavutil
    print("[BAĞLANTI] Heartbeat bekleniyor...")
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=HEARTBEAT_TIMEOUT)
    if msg is None:
        print("[BAĞLANTI] HATA: Heartbeat alınamadı! SITL çalışıyor mu?")
        sys.exit(1)
    print(f"[BAĞLANTI] Heartbeat alındı ✓ (system={master.target_system}, "
          f"component={master.target_component})")


def set_mode(master, mode_name: str):
    """
    Uçuş modunu değiştirir.
    """
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"[MOD] HATA: '{mode_name}' modu bulunamadı!")
        print(f"[MOD] Mevcut modlar: {list(master.mode_mapping().keys())}")
        return False

    master.set_mode(mode_id)

    # Mod değişikliğinin gerçekleştiğini heartbeat'ten doğrula
    start = time.time()
    while time.time() - start < 10:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.custom_mode == mode_id:
            print(f"[MOD] Mod değiştirildi: {mode_name} ✓")
            return True
    print(f"[MOD] UYARI: Mod değişikliği doğrulanamadı (timeout)")
    return False


def arm_vehicle(master):
    """
    Aracı arm eder (motorları devreye alır).
    """
    from pymavlink import mavutil
    print("[ARM] Arm komutu gönderiliyor...")
    master.arducopter_arm()

    # Arm durumunu doğrula
    start = time.time()
    while time.time() - start < 10:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[ARM] Araç arm edildi ✓")
            return True

    print("[ARM] HATA: Arm başarısız! Pre-arm check'leri kontrol edin.")
    return False


def takeoff(master, altitude: float):
    """
    GUIDED modda hedef irtifaya kalkış komutu gönderir.
    """
    from pymavlink import mavutil
    print(f"[TAKEOFF] Hedef irtifa: {altitude}m")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,           # confirmation
        0, 0, 0, 0,  # param1-4 (kullanılmaz)
        0, 0,        # lat, lon (kullanılmaz, mevcut pozisyon)
        altitude     # param7: hedef irtifa (metre)
    )

    # İrtifaya ulaşılmasını bekle
    print("[TAKEOFF] İrtifaya ulaşılması bekleniyor...")
    start = time.time()
    while time.time() - start < 60:  # max 60s bekle
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # mm → m
            if current_alt >= altitude * 0.90:  # %90 hedefe ulaşınca devam
                print(f"[TAKEOFF] İrtifa ulaşıldı: {current_alt:.1f}m ✓")
                return True
    print("[TAKEOFF] UYARI: İrtifa timeout (60s)")
    return False


def get_gps_position(master):
    """
    Güncel GPS pozisyonunu döndürür (tek seferlik okuma).
    """
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    if msg:
        return {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.relative_alt / 1000.0,
        }
    return None


def wait_for_gps_fix(master, timeout: float = 30.0) -> dict:
    """
    GPS 3D fix elde edilene kadar bekler, sonra pozisyonu döndürür.
    """
    GPS_FIX_3D = 3
    start = time.time()
    last_fix_type = 0
    got_3d_fix = False

    print(f"[GPS] 3D fix bekleniyor (max {timeout:.0f}s)...")

    while time.time() - start < timeout:
        gps_msg = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1)
        if gps_msg:
            last_fix_type = gps_msg.fix_type
            sat_count = gps_msg.satellites_visible if hasattr(gps_msg, 'satellites_visible') else '?'
            elapsed = time.time() - start
            print(f"  [{elapsed:5.1f}s] fix_type={last_fix_type} "
                  f"uydu={sat_count}", end="")

            if last_fix_type >= GPS_FIX_3D:
                print(" → 3D FIX ✓")
                got_3d_fix = True
                break
            else:
                fix_names = {0: 'GPS yok', 1: 'Fix yok', 2: '2D fix'}
                print(f" → {fix_names.get(last_fix_type, f'tip={last_fix_type}')}")
        else:
            elapsed = time.time() - start
            print(f"  [{elapsed:5.1f}s] GPS mesajı bekleniyor...")

    if not got_3d_fix:
        print(f"[GPS] HATA: {timeout:.0f}s içinde 3D fix alınamadı! "
              f"(son fix_type={last_fix_type})")
        return None

    print("[GPS] Pozisyon okunuyor...")
    for _ in range(5):
        pos_msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if pos_msg and pos_msg.lat != 0:
            pos = {
                "lat": pos_msg.lat / 1e7,
                "lon": pos_msg.lon / 1e7,
                "alt": pos_msg.relative_alt / 1000.0,
            }
            print(f"[GPS] Pozisyon alındı: lat={pos['lat']:.7f}, "
                  f"lon={pos['lon']:.7f}, alt={pos['alt']:.1f}m ✓")
            return pos

    print("[GPS] HATA: 3D fix var ama GLOBAL_POSITION_INT okunamadı!")
    return None


def resolve_mode_name(master, custom_mode):
    """
    custom_mode numarasını insan-okunur mod adına çevir.
    mavutil mode mapping kullanır.
    """
    mode_map = master.mode_mapping()
    # Ters lookup: {mode_id: mode_name}
    reverse_map = {v: k for k, v in mode_map.items()}
    return reverse_map.get(custom_mode, f"UNKNOWN({custom_mode})")


# ─────────────────────────────────────────────────────────
#  TEST-IMAGE MODU (izole inference, MAVLink yok)
# ─────────────────────────────────────────────────────────

def run_test_image(image_path: str, model_path: str):
    """
    Sadece YOLO inference testi çalıştırır.
    """
    print(f"\n{'='*60}")
    print("  🧪 İZOLE İNFERENCE TESTİ")
    print(f"{'='*60}")
    print(f"  Görüntü: {image_path}")
    print(f"  Model:   {model_path}")
    print(f"  Eşik:    {CONFIDENCE_THRESHOLD}")
    print(f"{'='*60}\n")

    # ── Görüntü yükle ──
    if not os.path.isfile(image_path):
        print(f"[HATA] Test görüntüsü bulunamadı: {image_path}")
        sys.exit(1)
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"[HATA] Test görüntüsü okunamadı: {image_path}")
        sys.exit(1)
    print(f"[INFERENCE] Görüntü yüklendi: {frame.shape[1]}x{frame.shape[0]} px\n")

    # ── Detector yükle ──
    abs_model_path = os.path.abspath(model_path)
    if not os.path.isfile(abs_model_path):
        print(f"[HATA] Model dosyası bulunamadı: {abs_model_path}")
        sys.exit(1)

    detector = TargetDetector(
        model_path=model_path,
        confidence_threshold=CONFIDENCE_THRESHOLD,
        target_class_name=TARGET_CLASS_NAME,
        target_class_id=TARGET_CLASS_ID,
    )

    # ── Inference çalıştır ──
    print(f"\n{'─'*40}")
    print("  INFERENCE SONUÇLARI")
    print(f"{'─'*40}")

    targets, obstacles = detector.detect(frame)

    print(f"\n  --- Hedefler ({len(targets)}) ---")
    if not targets:
        print("  Hiçbir hedef tespit edilmedi (eşik üstü).")
    else:
        for i, d in enumerate(targets, 1):
            print(f"  [{i}] Sınıf: {d.class_name} (id={d.class_id})")
            print(f"      Güven: {d.confidence:.1%}")
            print(f"      Bbox merkez: ({d.center_x:.0f}, {d.center_y:.0f}) px")
            print(f"      Bbox: [{d.x1:.0f},{d.y1:.0f},{d.x2:.0f},{d.y2:.0f}]")
            print()

    print(f"  --- Engeller ({len(obstacles)}) ---")
    if not obstacles:
        print("  Hiçbir engel tespit edilmedi.")
    else:
        for i, d in enumerate(obstacles, 1):
            print(f"  [{i}] Sınıf: {d.class_name} (id={d.class_id})")
            print(f"      Güven: {d.confidence:.1%}")
            print(f"      Bbox merkez: ({d.center_x:.0f}, {d.center_y:.0f}) px")
            print(f"      Bbox: [{d.x1:.0f},{d.y1:.0f},{d.x2:.0f},{d.y2:.0f}]")
            print()

    print(f"{'='*60}")
    print(f"  Toplam hedef: {len(targets)}, Toplam engel: {len(obstacles)} (eşik: {CONFIDENCE_THRESHOLD})")
    if targets:
        best = max(targets, key=lambda d: d.confidence)
        print(f"  En yüksek hedef güven: {best.class_name} @ {best.confidence:.1%}")
        print(f"\n  ✅ Bu görüntüyle uçuşta RTL TETİKLENİRDİ.")
    else:
        print(f"\n  ⚠️  Bu görüntüyle uçuşta RTL tetiklenmezdi.")
    print(f"{'='*60}")


# ─────────────────────────────────────────────────────────
#  TAM UÇUŞ AKIŞI (MAVLink + mission + detection loop)
# ─────────────────────────────────────────────────────────

def run_full_flight(args):
    """
    Tam uçuş akışı: SITL bağlantısı → mission → arm → takeoff → tarama → RTL.
    pymavlink ve mission_generator burada import edilir (lazy).
    """
    from pymavlink import mavutil
    from mission_generator import generate_single_pass_waypoints, upload_mission

    # ── Detector yükle (opsiyonel) ──
    detector = None
    abs_model_path = os.path.abspath(args.model)
    if os.path.isfile(abs_model_path):
        detector = TargetDetector(
            model_path=args.model,
            confidence_threshold=CONFIDENCE_THRESHOLD,
            target_class_name=TARGET_CLASS_NAME,
            target_class_id=TARGET_CLASS_ID,
        )
    else:
        print(f"[UYARI] Model dosyası bulunamadı: {abs_model_path}")
        print("[UYARI] Detection devre dışı, sadece uçuş mantığı çalışacak.")

    # ── 1. MAVLink Bağlantısı ──
    print(f"\n[BAĞLANTI] Bağlanılıyor: {args.connect}")
    master = mavutil.mavlink_connection(args.connect)
    wait_for_heartbeat(master)

    # ── Telemetri stream'lerini başlat ──
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
    )
    time.sleep(0.5)

    # ── 2. Parametreleri ayarla ──
    print("[PARAM] ARMING_CHECK=0 ayarlanıyor (SITL)...")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'ARMING_CHECK',
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    time.sleep(1)

    # ── 3. GPS fix bekle & home pozisyonunu al ──
    home_pos = wait_for_gps_fix(master, timeout=30)
    if home_pos is None:
        print("[HOME] HATA: GPS fix alınamadı, çıkılıyor!")
        sys.exit(1)
    print(f"[HOME] Home: lat={home_pos['lat']:.7f}, lon={home_pos['lon']:.7f}, "
          f"alt={home_pos['alt']:.1f}m")

    # ── 4. Mission Oluştur ve Yükle ──
    print(f"\n{'─'*40}")
    print("  MISSION YÜKLEME")
    print(f"{'─'*40}")
    waypoints = generate_single_pass_waypoints(home_pos["lat"], home_pos["lon"])
    success = upload_mission(master, waypoints)
    if not success:
        print("[MISSION] Mission yüklenemedi, çıkılıyor.")
        sys.exit(1)

    if args.dry_run:
        print("\n[DRY-RUN] Mission yüklendi, arm atlanıyor. Çıkılıyor.")
        return

    # ── 5. GUIDED Mod → Arm → Takeoff ──
    print(f"\n{'─'*40}")
    print("  KALKIŞ")
    print(f"{'─'*40}")
    if not set_mode(master, "GUIDED"):
        sys.exit(1)
    time.sleep(1)

    if not arm_vehicle(master):
        sys.exit(1)
    time.sleep(1)

    if not takeoff(master, TAKEOFF_ALTITUDE):
        print("[TAKEOFF] Takeoff başarısız, RTL'ye geçiliyor...")
        set_mode(master, "RTL")
        sys.exit(1)
    time.sleep(2)

    # ── 6. AUTO Moda Geç → Tarama Başlasın ──
    print(f"\n{'─'*40}")
    print("  TARAMA BAŞLIYOR")
    print(f"{'─'*40}")
    if not set_mode(master, "AUTO"):
        print("[MOD] AUTO moda geçilemedi, RTL'ye geçiliyor...")
        set_mode(master, "RTL")
        sys.exit(1)

    # ── 7. Detection Loop ──
    print(f"[DETECTION] Tarama döngüsü başlıyor "
          f"(aralık: {DETECTION_INTERVAL}s, eşik: {CONFIDENCE_THRESHOLD})")
    if args.inject_image:
        print(f"[DETECTION] Frame enjeksiyonu: {args.inject_image} @ T+{args.inject_at}s (AUTO'dan)")
    print("[DETECTION] Durdurmak için Ctrl+C\n")

    target_found = False

    # ── AUTO başlangıç zamanı (enjeksiyon için referans) ──
    auto_start_time = time.time()
    loop_start_time = auto_start_time

    # ── Heartbeat loglama: ilk 5 HB detayı ──
    hb_log_count = 0
    HB_LOG_LIMIT = 5

    # ── Disarm streak (3 ardışık HB ile armed=False → disarm kararı) ──
    disarm_streak = 0

    # ── "Frame kaynağı yok" bir kez yazılsın ──
    frame_source_warned = False

    # ── Enjeksiyon yapıldı mı? (bir kez enjekte et) ──
    injection_done = False

    # ── Son geçerli HB zamanı (5s uyarı için) ──
    last_valid_hb_time = time.time()
    hb_timeout_warned = False

    try:
        while True:
            # ── Heartbeat al (autopilot filtresi) ──
            hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)

            if hb is not None:
                # İlk 5 HB'yi detaylı logla
                if hb_log_count < HB_LOG_LIMIT:
                    hb_log_count += 1
                    print(f"[HB #{hb_log_count}] type={hb.type}, "
                          f"srcSystem={hb.get_srcSystem()}, "
                          f"srcComponent={hb.get_srcComponent()}")

                # GCS heartbeat'lerini filtrele (type == MAV_TYPE_GCS = 6)
                if hb.type == mavutil.mavlink.MAV_TYPE_GCS:
                    continue

                # srcSystem filtresi: sadece bağlı olduğumuz autopilot
                if hb.get_srcSystem() != master.target_system:
                    continue

                # Geçerli autopilot heartbeat
                last_valid_hb_time = time.time()
                hb_timeout_warned = False

                armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                mode_name = resolve_mode_name(master, hb.custom_mode)

                # İrtifa: master.messages cache'ten oku (non-blocking)
                alt = 0.0
                gpi = master.messages.get('GLOBAL_POSITION_INT', None)
                if gpi is not None:
                    alt = gpi.relative_alt / 1000.0

                # MISSION_CURRENT seq bilgisi
                mc = master.messages.get('MISSION_CURRENT', None)
                wp_seq = mc.seq if mc is not None else -1

                elapsed = time.time() - loop_start_time
                print(f"[SCAN T+{elapsed:.1f}s] armed={armed}, mode={mode_name}, "
                      f"alt={alt:.1f}m, wp_seq={wp_seq}")

                # ── Disarm streak kontrolü ──
                if not armed:
                    disarm_streak += 1
                    if disarm_streak >= 3:
                        print(f"[DETECTION] Araç disarm oldu "
                              f"(3 ardışık HB'de armed=False), çıkılıyor.")
                        break
                else:
                    disarm_streak = 0
            else:
                # HB timeout
                if (time.time() - last_valid_hb_time) > 5.0 and not hb_timeout_warned:
                    print("[UYARI] 5 saniyedir geçerli heartbeat alınamıyor!")
                    hb_timeout_warned = True
                continue

            # ── Frame al ──
            frame = None

            # --- INJECT MODE ---
            if args.inject_image and not injection_done:
                inject_elapsed = time.time() - auto_start_time
                if inject_elapsed >= args.inject_at:
                    print(f"[INJECT] {args.inject_at}s doldu (T+{inject_elapsed:.1f}s), "
                          f"test görüntüsü enjekte ediliyor: {args.inject_image}")
                    frame = cv2.imread(args.inject_image)
                    if frame is None:
                        print(f"[INJECT HATA] Görüntü dosyası okunamadı: {args.inject_image}")
                    injection_done = True
            # -------------------

            # Frame yok uyarısı (bir kez)
            if frame is None and not frame_source_warned:
                print("[DETECTION] Frame kaynağı yok")
                frame_source_warned = True

            # ── Inference ──
            if frame is not None and detector is not None:
                targets, obstacles = detector.detect(frame)

                # Ham detector çıktısını logla (enjekte edilen frame için)
                if targets or obstacles:
                    print(f"[DETECTOR HAM] Hedef: {len(targets)}, Engel: {len(obstacles)}")
                    for d in targets:
                        print(f"  → Hedef: sınıf={d.class_name}({d.class_id}), "
                              f"conf={d.confidence:.2f}, "
                              f"bbox=[{d.x1:.0f},{d.y1:.0f},{d.x2:.0f},{d.y2:.0f}]")
                    for d in obstacles:
                        print(f"  → Engel: sınıf={d.class_name}({d.class_id}), "
                              f"conf={d.confidence:.2f}, "
                              f"bbox=[{d.x1:.0f},{d.y1:.0f},{d.x2:.0f},{d.y2:.0f}]")

                # Sadece hedef tespiti → RTL tetikle
                if targets:
                    best_det = max(targets, key=lambda d: d.confidence)
                    pos = get_gps_position(master)

                    print(f"\n{'!'*60}")
                    print(f"  🎯 HEDEF TESPİT EDİLDİ!")
                    print(f"  Sınıf:    {best_det.class_name} (id={best_det.class_id})")
                    print(f"  Güven:    {best_det.confidence:.1%}")
                    print(f"  Piksel:   ({best_det.center_x:.0f}, {best_det.center_y:.0f})")
                    print(f"  Bbox:     [{best_det.x1:.0f},{best_det.y1:.0f},"
                          f"{best_det.x2:.0f},{best_det.y2:.0f}]")
                    if pos:
                        print(f"  GPS:      lat={pos['lat']:.7f}, "
                              f"lon={pos['lon']:.7f}, alt={pos['alt']:.1f}m")
                    print(f"  WP Seq:   {wp_seq}")
                    print(f"{'!'*60}")
                    print(f"\n[RTL] Hedefe en yüksek güvenle tespit edildi, "
                          f"RTL moduna geçiliyor...")

                    # ── RTL Tetikle ──
                    set_mode(master, "RTL")
                    target_found = True
                    break

            time.sleep(DETECTION_INTERVAL)

    except KeyboardInterrupt:
        print("\n[KULLANICI] Ctrl+C algılandı, RTL'ye geçiliyor...")
        set_mode(master, "RTL")

    # ── 8. RTL Bekleme ──
    print(f"\n{'─'*40}")
    print("  RTL — EVE DÖNÜŞ")
    print(f"{'─'*40}")
    print("[RTL] Home noktasına dönüş bekleniyor...")

    disarm_streak_rtl = 0
    start = time.time()
    while time.time() - start < 120:  # max 2 dakika bekle
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb is not None:
            if hb.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            if hb.get_srcSystem() != master.target_system:
                continue
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not armed:
                disarm_streak_rtl += 1
                if disarm_streak_rtl >= 3:
                    print("[RTL] Araç iniş yaptı ve disarm oldu ✓")
                    break
            else:
                disarm_streak_rtl = 0
        pos_msg = master.messages.get('GLOBAL_POSITION_INT', None)
        if pos_msg:
            print(f"  Pozisyon: lat={pos_msg.lat/1e7:.7f}, "
                  f"lon={pos_msg.lon/1e7:.7f}, "
                  f"alt={pos_msg.relative_alt/1000.0:.1f}m",
                  end="\r")
        time.sleep(2)
    else:
        print("\n[RTL] UYARI: RTL timeout (120s)")

    # ── SONUÇ ──
    print(f"\n{'='*60}")
    if target_found:
        print("  ✅ GÖREV TAMAMLANDI: Hedef bulundu, RTL yapıldı.")
    else:
        print("  ⚠️  GÖREV TAMAMLANDI: Hedef bulunamadı, mission bitti.")
    print(f"{'='*60}")


# ─────────────────────────────────────────────────────────
#  ANA GİRİŞ NOKTASI
# ─────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="İHA ile 20x20m alanda hedef arama + otomatik RTL"
    )
    parser.add_argument(
        "--connect", default=MAVLINK_CONNECTION,
        help=f"MAVLink bağlantı string'i (default: {MAVLINK_CONNECTION})"
    )
    parser.add_argument(
        "--test-image", default=None,
        help="İzole inference testi: görüntüyü yükle, best.pt çalıştır, "
             "sonuçları yazdır ve çık. MAVLink/SITL gerekmez."
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Mission'ı yükle ama arm etme (debug/kontrol için)"
    )
    parser.add_argument(
        "--model", default=MODEL_PATH,
        help=f"YOLO model dosyası yolu (default: {MODEL_PATH})"
    )
    parser.add_argument(
        "--inject-image", default=None,
        help="Test frame enjeksiyonu: AUTO moduna geçişten --inject-at saniye "
             "sonra bu görüntüyü kamera yerine enjekte eder."
    )
    parser.add_argument(
        "--inject-at", type=float, default=8,
        help="Enjeksiyon zamanı: AUTO moduna geçişten kaç saniye sonra "
             "(default: 8)"
    )
    args = parser.parse_args()

    print("=" * 60)
    print("  İHA HEDEF ARAMA + OTOMATİK RTL")
    print("=" * 60)
    print(f"  Model:      {args.model}")
    print(f"  İrtifa:     {ALTITUDE}m")

    # ── KARAR: Test-image modu mu, tam uçuş mu? ──
    if args.test_image:
        print(f"  Mod:        İZOLE INFERENCE TESTİ")
        print(f"  Görüntü:    {args.test_image}")
        print("=" * 60)
        run_test_image(args.test_image, args.model)
    else:
        print(f"  Mod:        TAM UÇUŞ")
        print(f"  Bağlantı:   {args.connect}")
        print(f"  Dry-run:    {args.dry_run}")
        if args.inject_image:
            print(f"  Inject:     {args.inject_image} @ T+{args.inject_at}s")
        print("=" * 60)
        run_full_flight(args)


if __name__ == "__main__":
    main()
