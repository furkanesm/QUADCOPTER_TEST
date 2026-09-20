# S500 Simülasyon Çalışması - Güncel Notlar

- **Başlatma Dizini:** Simülasyon `s500_force_test` dizininde çalıştırılmaktadır.
- **Gazebo ve Eklenti:** Fortress 6.18.0 ve derlenen ArduPilot eklentisi (libArduPilotPlugin.so) kullanılmaktadır.
- **Başlatma Komutları:**
  *Gazebo (1. Terminal):*
  `export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/home/furkan/s500_sim/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}`
  `export IGN_GAZEBO_RESOURCE_PATH=/home/furkan/s500_sim/ardupilot_gazebo/models:/home/furkan/s500_sim/ardupilot_gazebo/worlds:${IGN_GAZEBO_RESOURCE_PATH}`
  `ign gazebo -v 4 -r s500_custom_runway.world`
  *SITL (2. Terminal):*
  `cd /home/furkan/s500_sim/s500_force_test`
  `/home/furkan/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map`

### Ara Model ve Fiziksel Yapılandırılma
- **Kütle ve Atalet:** CAD raporundan alınan 2.400 kg kütle, CG ve atalet değerleri, "CAD +X = İleri, CAD +Y = Yukarı, CAD +Z = Sağ" varsayımı ile model eksenlerine (başlangıç: pervane geometrik merkezi) aktarıldı.
  - *Asal Atalet Değerleri (kg·m²):* 0.0048870305, 0.0091492191, 0.0106929470 (Triangle Inequality sağlanır).
  - *Dağılım:* Gerçekte ölçülmüş bir bileşen kütlesi olmadığı için, simülasyon geçerliliğini korumak adına rotor kütleleri 0.010 kg alınmış, kalan büyük kütle (2.060 kg) ve atalet `base_link`'e toplanmıştır. Görsel geometri hala Iris'tir, CAD eksen dönüşümleri modelleme kabulüdür.
- **Motor ve Aerodinamik:** APC1147 pervanesi (14.8 V) için kuvvet eğrisi matematiksel olarak (LiftDrag `cla` = 5.153257 ve motor çarpanı `719.8436`) uyarlanmıştır.
- **Jiroskop Düzelmesi:** "PreArm: Gyros inconsistent" hatasını çözmek için, model.sdf içerisindeki sensör eklemleri (`iris/imu_joint` ve `iris/ground_truth/odometry_sensorgt_joint`) `fixed` yapılmıştır. Bu müdahale sonrası Gazebo içi fiziksel sensör sürüklenmesi durmuş ve IMU verisi tam kararlı hale gelmiştir.

### Uçuş Performansı ve Log Analizi
- **Tarihli Uçuş:** Başarılı bir GUIDED kalkış, kararlı hover ve iniş yapılmıştır.
- **Zamanlama (BIN TimeUS):** 
  - t=0 (Log Başlangıcı): Aracın gücünün açıldığı (boot) an.
  - Arming: 1020.45 sn, Disarming: 1419.82 sn.
  - Kalkış: 1025.78 sn'de 1.5m irtifa geçildi. 
  - Kararlı Hover (387 saniye): 1031.78 sn (tahmini yerleşme) - 1414.58 sn (iniş düşey hızının başlaması) arası ölçütlere uygun kesintisiz süre.
- **Hover Değerlendirmesi:**
  - Yükseklik: Ortalama 2.02 m (2.0m hedefine maks hata 0.072m, RMS hata 0.021m).
  - Yatay konum kayması: Maksimim 0.162 m.
  - Yatay Sürat (sqrt(VN²+VE²)): Ortalama 0.094 m/s. Düşey Hız (VD): Max Abs 0.222 m/s.
  - Salınım: Roll RMS 0.158°, Pitch RMS 0.716°. Unwrapped Yaw farkı: 0.484°.
- **Motor Çıkışları (RCOU 1100-1900 PWM):** Max tespit edilen 1864'tür (Hiçbir örnek 1890 sınırına ulaşmamıştır). `SERVOx_MIN`/`MAX` ile modelin `<servo_min>`/`<max>` parametreleri birebir eşleşmektedir. *Not: ArduCopter loglarındaki limit bayrakları kesin doyum (saturation) göstermediğinden, salt yüksek PWM değerinden dolayı düşük kaldırma rezervi veya ağır araç dinamiği olduğu yönünde kesin bir sonuca varılamamıştır.*

### Kalibre Edilmeyen / Belirsiz Bırakılan Konular
- İzole rotor kuvveti, yaw torku/sürüklenme miktarı ve motor geçici tepkisi henüz doğrulanmamıştır.
- Batarya ve akım modeli kalibre edilmemiştir; konsolda görülen voltajın, itki referansına otomatik bağlanacağı varsayılamaz.
- Motor kontrolcüsü donanım doyum kanıtları ve `MOT_PWM_MIN`/`MAX` gibi FSM ilişkili etkin kontrol sınırları doğrudan değerlendirilememiştir.

### Analiz Betikleri ve Yedekler
- Kaynak Log: `/home/furkan/s500_sim/s500_force_test/logs/00000001.BIN` (ve mav.tlog)
- Mevcut Yedek (Geçici): `/home/furkan/s500_sim/s500_test_backups/backup_20260919_1953`
- Doğrulama Betikleri: `/home/furkan/s500_sim/ardupilot_gazebo/scripts/` altına taşınmıştır. (`analyze_log_v2.py`, `calc_principal_inertia.py`). Modeli yamalayan eski betikler devredışı bırakılmıştır.

### Güncelleme: Jetson Eklenmesi ve 2.550 kg Modeli
- **Kütle Artışı:** S500 modelinin toplam kütlesi, Jetson bilgisayar eklenmesi varsayımıyla **2.400 kg'dan 2.550 kg'a** güncellenmiştir (Çarpan: 1.0625).
- **Dağılım Varsayımı:** Jetson'un tam fiziksel konumu bilinmediğinden, CG (ağırlık merkezi) konumu ve geometri aynı kabul edilmiş; mevcut eşdeğer kütle dağılımı ve atalet (rotor ataletleri dahil) orantılı olarak artırılmıştır.
- **Uçuş Doğrulaması:** Yukarıda belirtilen başarılı uçuş günlüğü ve log analizi **2.400 kg**'lık eski modele aittir. Yeni oluşturulan **2.550 kg** model henüz uçuşta doğrulanmamıştır.
- **Belirsizlikler:** Rotor ataletleri ölçeklenmiş olmasına rağmen, motor geçici (transient) tepkisi ve artan ataletin motor ivmelenmesine etkisi henüz kalibre edilmemiştir.

### 2.550 kg Modeli - İlk Uçuş Denemesi Sonuçları
- **Log Kaynağı:** `/home/furkan/s500_sim/s500_force_test/logs/00000002.BIN`
- **Uçuş Olayları:**
  - Motorların çalıştırılması (ARM): 118.23 s
  - İniş moduna geçiş (LAND): 246.68 s
  - Motorların kapatılması (DISARMED): 249.26 s
- **Kesintisiz Hover (Havada Asılı Kalma) Analizi:** Bu analiz, tüm havada kalış süresini değil, sadece kalkış ve iniş geçişlerinden arındırılmış yaklaşık **48 saniyelik** kararlı uçuş kesitini kapsamaktadır.
  - Bağıl İrtifa (2m hedefi): Minimum 1.83 m, Ortalama 2.01 m, Maksimum 2.07 m.
  - Açılar (Maksimum Mutlak): Roll 1.77°, Pitch 2.35°.
  - Hiçbir uyarı veya kırım (FAIL/WARN) mesajı alınmamıştır.
- **Geçerlilik ve Varsayımlar:** Ağırlık 2.550 kg olarak güncellenmiş ve orantılı kütle/atalet varsayımı korunarak uçulmuştur. Motor geçici (transient) tepkisi ve yeni ataletle motor doyum kalibrasyon sınırları hâlâ tam doğrulanmamış (belirsiz) durumdadır.

### 2.550 kg Modeli - Yatay Hareket ve Yaw Testi (00000004.BIN)
- **Yedek Yolu:** `/home/furkan/s500_sim/s500_test_backups/backup_20260920_1740`
- **Olay Zamanları (BIN):**
  - ARM (Motorların Çalıştırılması): 84.37 sn
  - LAND Moduna Geçiş: 287.55 sn
  - DISARMED (Motorların Kapatılması): 294.19 sn
- **Ölçülen Hareket ve Yönelim Sonuçları:**
  - **Yatay Hareket:** MAV_FRAME_LOCAL_OFFSET_NED kullanılarak verilen X=1 (Kuzey) hareketinin uçuş verilerinde iki kez işletildiği gözlendi. Araç, başlangıç noktasına göre (N=-0.00m) toplamda **~2.08 metre Kuzeye** ulaştı. Güney yönlü geri çekilme komutu hiç alınmamış/işletilmemiş; araç bu kuzey konumunda inişe geçene kadar (N=2.03m) sabit beklemiştir.
  - **Yaw Değişimi:** İstenen saat yönü (CW) dönüşü başarıyla doğrulanmıştır. Yatay hareketlerin tamamlanmasından sonra başlangıç yaw açısı ~1.4° iken, manevra sonrasında ~30.3° olarak ölçülmüş ve **net 28.9° CW** dönüş tespit edilmiştir.
- **Sonuç:** Yeni uçuş oturumunun düzgün kapatılması sayesinde tüm LAND ve DISARM olayları diskte kalıcı hale gelmiştir. Önceki oturumun (00000003.BIN) belirsizliği giderilmiş ve araç komutlara kararlı bir otonomiyle yanıt vermiştir.

## İzole Rotor İtki Testi (2.550 kg Modeli için)
Gazebo Fortress ortamında motor geçici tepkisinden, ArduPilot PID döngülerinden, yaw torkundan, batarya modelinden ve tüm araç aerodinamiğinden bağımsız olarak, yalnızca rotor itkisini doğrulamak amacıyla hız kontrollü (JointController) tek rotorlu izole bir test ortamı kurulmuştur.

**Kullanılan Dosyalar:**
- **World Dosyası:** `/home/furkan/s500_sim/ardupilot_gazebo/worlds/rotor_test.world`
- **Test Betiği:** `/home/furkan/s500_sim/ardupilot_gazebo/scripts/run_thrust_test.py`
- **Sonuç Klasörü:** `/home/furkan/s500_sim/thrust_test_results/run_20260920_182601`

**Testin Kapsamı ve Kısıtlamaları:**
- Bu test sadece doğrudan hız komutu verilen (`cmd_vel`) tek bir rotorun kararlı durum (steady-state) ürettiği düşey itkiyi (`LiftDrag`) Gazebo `ForceTorque` sensörü ile ölçer.
- Geçici motor tepkisi (transient response), gerçek donanım PWM-RPM eğrisi, ArduPilot motor mikseri veya batarya kısıtları bu testin kapsamında **değildir**.
- Beklenen dara: 1.010625 kg × -9.8 m/s² (Gazebo yerçekimi) = -9.904125 N olarak ölçülmüş ve işaretli net itki (Ham Fz - Dara) üzerinden sonuçlar hesaplanmıştır. N -> gf dönüşümlerinde ise uluslararası g=9.80665 m/s² sabiti kullanılmıştır.

**Sayısal Sonuçlar (Ortak Ölçüm Aralığı > 1.1s):**
- **2900 RPM:** Ölçülen: 247.1 gf | Teorik Formül: 2.4227 N | Referans: 270 gf | Fark: -%8.50
- **4450 RPM:** Ölçülen: 581.7 gf | Teorik Formül: 5.7047 N | Referans: 580 gf | Fark: +%0.30
- **5400 RPM:** Ölçülen: 856.6 gf | Teorik Formül: 8.4004 N | Referans: 860 gf | Fark: -%0.40
- **6874 RPM:** Ölçülen: 1388.1 gf | Teorik Formül: 13.6123 N | Referans: 1370 gf | Fark: +%1.32

*(Sonuçlar F=k·ω² formülüyle ve LiftDrag katsayılarıyla birebir uyuşmaktadır.)*
