# S500 Simulation Workspace

Bu klasör S500 projesinin SITL simülasyonu için yerel ortamda üretilen parametre, analiz, dönüştürme ve referans dosyalarını içerir.

## parameters/
- `s500_2400kg_sitl_snapshot.parm`: İlk CAD verilerine dayalı 2.400 kg ağırlığındaki aracın simülasyon testi sırasındaki tam MAVProxy parametre kaydıdır.
- `s500_2550kg_sitl_snapshot.parm`: Jetson ve ekstra yükler dahil edilip 2.550 kg'a güncellenmiş modelin son yatay hareket ve yaw itki testi sırasındaki parametre kaydıdır.
**Önemli:** Bu dosyalar sadece SITL ortamı içindir, gerçek uçuş kontrolcüsüne doğrudan YÜKLENMEMELİDİR. ArduPilot'un yerel SITL ortamındaki `gazebo-iris` varsayılanlarıyla oluşturulmuştur.

## scripts/
- `calculate_inertia.py`, `compute_liftdrag.py`: Temel fizik hesaplamaları.
- `scale_inertia.py`, `patch_sdf.py`: (MUTATOR) Model.sdf dosyalarını dinamik güncelleyen yardımcı yazılımlar.
- `analyze_log.py`, `generate_report.py`: Uçuş kayıtlarını (`.BIN`) analiz etmek için kullanılır.

## data/
Referans CSV verileri (ör. SunnySky X3108S KV720 itki tablosu vb.).

## Eklenen İlk Dosyalar (İndirilenler'den)
- `parameters/s500_quad_baseline.param`: Projenin başında kullanılan, S500 quadcopter (Frame Class 1, Frame Type 1) yapılandırmasını içeren baz SITL parametre dosyası.
- `S500_Gazebo_Entegrasyon_Gorevi.md`: Sistemin ArduPilot Gazebo'ya entegre edilmesini başlatan ana görev ve planlama dokümanı.
