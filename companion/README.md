# Companion Script — İHA Hedef Arama + Otomatik RTL

20×20m alanda İHA ile havadan YOLO-tabanlı hedef arama ve otomatik RTL sistemi.  
SITL (Software-In-The-Loop) test ortamı için tasarlanmıştır.

## Mimari

```
companion/
├── config.py              # Merkezi parametreler (irtifa, FOV, eşikler)
├── detector.py            # YOLO inference (best.pt) — vision_node.py'den adapte
├── mission_generator.py   # Lawnmower waypoint üretimi + MAVLink upload
├── search_and_rtl.py      # Ana script: bağlan → arm → tara → tespit → RTL
├── requirements.txt       # Python bağımlılıkları
└── README.md              # Bu dosya
```

## Gereksinimler

- **Python 3.8+**
- **SITL** (ArduCopter) — WSL2/Linux üzerinde çalışır
- Python paketleri:
  ```bash
  pip install -r requirements.txt
  ```

## Hızlı Başlangıç

### 1. SITL'i Başlat (Terminal 1 — WSL/Linux)

```bash
cd QUADCOPTER_TEST
Tools/autotest/sim_vehicle.py -v ArduCopter --map --console
```

### 2. Companion Script'i Çalıştır (Terminal 2)

```bash
cd QUADCOPTER_TEST/companion
python search_and_rtl.py --connect udp:127.0.0.1:14550
```

### Opsiyonel Modlar

```bash
# Test görüntüsüyle inference testi
python search_and_rtl.py --test-image hedef.jpg

# Sadece mission yükle, arm etme (debug)
python search_and_rtl.py --dry-run

# Farklı model dosyası
python search_and_rtl.py --model /path/to/model.pt
```

## Uçuş Akışı

1. **Bağlantı** → SITL'e UDP ile bağlan, heartbeat bekle
2. **Mission yükle** → 4 köşe waypoint (20x20m alanı kaplayan tek geçiş)
3. **GUIDED → Arm → Takeoff** → 30m irtifaya çık
4. **AUTO moda geç** → Waypoint taramasını başlat
5. **Detection loop** → Her saniye best.pt inference, confidence > %55 ise:
   - GPS koordinatlarını logla
   - RTL moduna geç
6. **RTL** → Home noktasına dön, iniş yap, disarm

## Bağımsız Modül Testleri

```bash
# Config hesaplamalarını göster
python config.py

# Tek görüntüde inference testi
python detector.py test_image.jpg

# Mission planını konsola yazdır (upload yok)
python mission_generator.py
```

## FOV Hesabı

| Parametre | Değer |
|---|---|
| Yükseklik | 30m |
| Yatay FOV | 62.2° |
| Yer kapsamı | ~36.2m |
| Net şerit genişliği (%10 örtüşme) | ~32.6m |
| 20m alan → gerekli şerit | **1 (tek geçiş)** |

## Notlar

- **SITL Windows'ta doğrudan çalışmaz**, WSL2 (Ubuntu) gerektirir
- **Script** (pymavlink kısmı) hem Windows hem Linux'ta çalışır
- `best.pt` dosyası `QUADCOPTER_TEST/` kök dizininde olmalı
- Gerçek kamera yokken `--test-image` ile inference test edilir
- `ARMING_CHECK=0` sadece SITL'de kullanılır, **gerçek donanımda KULLANMAYIN**
