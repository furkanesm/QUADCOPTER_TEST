"""
config.py — Merkezi yapılandırma dosyası
========================================
Tüm ayarlanabilir parametreler burada.
NEDEN: Parametreleri tek bir yerde toplamak, farklı test senaryolarında hızlıca
değişiklik yapmayı kolaylaştırır. Diğer modüller sadece bu dosyayı import eder.
"""

import math

# ─────────────────────────────────────────────
#  BAĞLANTI
# ─────────────────────────────────────────────
# SITL varsayılan olarak 14550 portunda UDP dinler.
# Gerçek donanımda serial port string'i (ör: '/dev/ttyUSB0') olacak.
MAVLINK_CONNECTION = "udp:127.0.0.1:14550"

# ─────────────────────────────────────────────
#  UÇUŞ PARAMETRELERİ
# ─────────────────────────────────────────────
ALTITUDE = 30          # metre — eğitim datasına uygun (30-45m aralığı)
TAKEOFF_ALTITUDE = 30  # metre — kalkış hedef irtifası (= tarama irtifası)
AREA_SIZE = 20         # metre — 20x20m kare test alanı

# ─────────────────────────────────────────────
#  KAMERA / FOV
# ─────────────────────────────────────────────
# Varsayılan: RPi Camera Module v2 (Sony IMX219)
# Gerçek donanımda kullanılan kameraya göre güncellenecek.
CAMERA_HFOV_DEG = 62.2   # derece — yatay görüş açısı
CAMERA_VFOV_DEG = 48.8   # derece — dikey görüş açısı
STRIP_OVERLAP = 0.10      # %10 örtüşme (güvenlik payı)


def calculate_strip_width(altitude: float = ALTITUDE,
                          hfov_deg: float = CAMERA_HFOV_DEG,
                          overlap: float = STRIP_OVERLAP) -> float:
    """
    Yerdeki tarama şerit genişliğini hesaplar.

    Formül:  ground_width = 2 * h * tan(HFOV/2)
             strip_width  = ground_width * (1 - overlap)

    30m irtifa, 62.2° HFOV, %10 örtüşme ile:
        ground_width = 2 * 30 * tan(31.1°) ≈ 36.2m
        strip_width  = 36.2 * 0.9 ≈ 32.6m
    Bu 20x20m alanı TEK şeritte kaplar.
    """
    hfov_rad = math.radians(hfov_deg)
    ground_width = 2.0 * altitude * math.tan(hfov_rad / 2.0)
    strip_width = ground_width * (1.0 - overlap)
    return strip_width


# Hesaplanmış değer — diğer modüller bunu import eder
STRIP_WIDTH = calculate_strip_width()

# ─────────────────────────────────────────────
#  YOLO / TESPİT
# ─────────────────────────────────────────────
# best.pt QUADCOPTER_TEST kök dizininde duruyor.
# Script companion/ altından çalıştığında bir üst dizine bakar.
MODEL_PATH = "../best.pt"

# Aranan sınıf adı ve ID'si.
# class id 0 = 'Hedef'. Sadece bu sınıf erken RTL tetikler.
TARGET_CLASS_NAME = "Hedef"
TARGET_CLASS_ID = 0

# Minimum güven eşiği — bunun altındaki tespitler dikkate alınmaz.
CONFIDENCE_THRESHOLD = 0.55

# ─────────────────────────────────────────────
#  ZAMANLAMA
# ─────────────────────────────────────────────
# Detection loop'unun ne sıklıkla çalışacağı (saniye).
# SITL'de kamera olmadığından test modunda bu süre aralığıyla
# test görüntüsü decision'a sokulur.
DETECTION_INTERVAL = 1.0

# Heartbeat timeout (saniye)
HEARTBEAT_TIMEOUT = 30


if __name__ == "__main__":
    # Hızlı kontrol: python config.py çalıştırınca değerleri göster
    print(f"Altitude:           {ALTITUDE}m")
    print(f"Area:               {AREA_SIZE}x{AREA_SIZE}m")
    print(f"Camera HFOV:        {CAMERA_HFOV_DEG}°")
    print(f"Ground width:       {2 * ALTITUDE * math.tan(math.radians(CAMERA_HFOV_DEG) / 2):.1f}m")
    print(f"Strip width (net):  {STRIP_WIDTH:.1f}m")
    print(f"Strips needed:      {math.ceil(AREA_SIZE / STRIP_WIDTH)}")
    print(f"Confidence thresh:  {CONFIDENCE_THRESHOLD}")
    print(f"Model path:         {MODEL_PATH}")
    print(f"Target class:       {TARGET_CLASS_NAME} (id={TARGET_CLASS_ID})")
