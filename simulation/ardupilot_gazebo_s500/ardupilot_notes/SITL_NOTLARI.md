# ArduPilot SITL Çalışma Notları (18 Eylül)

## 1. Çalışılan Dizin
- Ana ArduPilot reposu: `/home/furkan/ardupilot`

## 2. Ortam Kurulumu (Kalıcı Çözüm)
- Başlangıçta MAVProxy eksik olduğu için aşağıdaki betik ile Ubuntu için gerekli tüm bağımlılıklar kalıcı olarak kuruldu:
  ```bash
  ./Tools/environment_install/install-prereqs-ubuntu.sh -y
  ```
- WAF build sistemi sayesinde yarınki simülasyon başlatmaları çok daha hızlı (saniyeler içinde) gerçekleşecek.

## 3. Parametre Dosyası
- **Dosya Yolu:** `/home/furkan/İndirilenler/s500_quad_baseline.param`
- **İşlevi:** S500 gövde tipi (quadcopter) için gerekli PID ayarlarını, filtreleme ve motor yapılandırmalarını içerir.

## 4. SITL'i Özel Parametrelerle Başlatma
- Parametreleri SITL'e dahil etmek için kullandığımız ana komut:
  ```bash
  Tools/autotest/sim_vehicle.py -v Copter --console --add-param-file="/home/furkan/İndirilenler/s500_quad_baseline.param"
  ```
- **Sonuç:** Varsayılan Copter parametreleri ile harmanlanarak toplam **1412 parametre** başarıyla sisteme yüklendi.

## 5. Sistemin Son Durumu
- Hiçbir kalkış (takeoff) veya manuel parametre doğrulaması (param show) henüz **yapılmadı**.
- Sistem başarıyla ayağa kalktı ve MAVProxy `STABILIZE` modunda komut bekleyecek şekilde test edildi.

## 6. Yarın İçin Hızlı Başlangıç Rehberi
Bilgisayarı açtığınızda doğrudan şu komutları sırasıyla terminale (ardupilot klasörü içinde) yapıştırıp çalışmanız yeterlidir:

```bash
# Sadece terminal MAVProxy'yi bulamazsa diye yol tanımlaması:
export PATH="$HOME/.local/bin:$PATH"

# Simülasyonu S500 parametreleriyle başlatma:
Tools/autotest/sim_vehicle.py -v Copter --console --add-param-file="/home/furkan/İndirilenler/s500_quad_baseline.param"
```
