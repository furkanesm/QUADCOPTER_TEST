# S500 Mission FSM — Gazebo Fortress Entegrasyonu Görev Dokümanı

## Amaç

Şu ana kadar `s500_mission_fsm` state machine'i, ArduPilot SITL'in **fiziksiz JSON backend'i** üzerinde test edildi (gerçek uçuş fiziği yok, sadece FSM mantığı ve MAVROS haberleşmesi doğrulandı). Bir sonraki adım, aynı FSM'i **Gazebo Fortress içinde gerçek fizik motoruyla** (yerçekimi, motor dinamiği, PID tepkisi, çarpışma) test etmek — gerçek S500 donanımını uçurmadan önceki son simülasyon aşaması.

Bu iş iki aşamaya bölünmüştür. **Aşama 2'ye geçmeden önce Aşama 1'in tamamlanıp onaylanması gerekir.**

---

## Arka Plan — Araştırmanın Sonucu

- ArduPilot'un resmi Gazebo model deposunda (`ArduPilot/ardupilot_gazebo`) **"S500" adında hazır bir model yoktur.** Depodaki tek quadcopter modeli `iris_with_ardupilot` (3DR Iris).
- Ancak ArduPilot'un resmi parametre setinde **S500'e özel, hazır bir parametre dosyası vardır: `Holybro-S500.param`.** Yani S500 uçuş kontrolcüsü tarafında zaten tanınan bilinen bir frame, sadece 3B görsel/fiziksel model dosyası (SDF) eksik.
- Iris ve S500 fiziksel olarak yakın sınıftadır (ikisi de 4 motorlu X-frame, ~500mm çap sınıfı), ama itki/ağırlık oranları belirgin farklıdır:
  - Iris hover gazı: ~%35-40
  - S500 hover gazı (Holybro-S500.param → `MOT_THST_HOVER`): **%25**
- Bu fark nedeniyle Iris parametreleriyle alınan fiziksel sonuçlar (PID tepkisi, tırmanma hızı, batarya tüketimi vb.) gerçek S500 davranışını yansıtmaz.

---

## AŞAMA 1 — Hızlı Doğrulama (Iris modeliyle)

**Amaç:** S500'ün fiziksel doğruluğunu değil, FSM + MAVROS + Gazebo entegrasyonunun genel olarak çalışıp çalışmadığını doğrulamak.

### Adımlar

1. **Ortamı hazırla:**
   ```bash
   export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/ardupilot_gazebo/models
   ```

2. **Gazebo dünyasını başlat** (ArduPilot Gazebo deposundaki hazır bir dünya dosyasıyla, örn. `runway.sdf`):
   ```bash
   gz sim -v4 -r runway.sdf
   ```

3. **ArduPilot SITL'i Gazebo arayüzüyle başlat** (Iris modeli, JSON backend yerine gerçek Gazebo fizik köprüsü kullanılarak):
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map
   ```

4. **MAVROS'u her zamanki gibi başlat** (önceki SITL testlerinde kullandığımız `apm.launch` ile aynı, sadece FCU artık Gazebo'daki simüle motorları kontrol ediyor).

5. **Test A'yı (basit kalkış → 5s bekleme → iniş) bu ortamda çalıştır.** Amaç: FSM'in state geçişlerinin (HAZIRLIK → GUIDED_VE_ARM → KALKIS → HAVADA_BEKLEME → INIS → TAMAMLANDI) Gazebo'nun gerçek zamanlı fizik simülasyonunda da önceki SITL testleriyle tutarlı şekilde çalıştığını doğrulamak.

### Beklenen Sonuç

- Gazebo penceresinde Iris modelinin görsel olarak kalktığını, 5 saniye beklediğini ve indiğini görmelisin.
- `sitl_test_results.json` benzeri bir sonuç (veya konsol logu) üzerinden state sırasının doğru olduğunu teyit et.
- Bu aşamada **fiziksel doğruluk önemli değil** — Iris'in ağırlığı/itkisi S500'den farklı olsa da, amaç sadece "kod Gazebo'da da çalışıyor mu" sorusuna cevap vermek.

**Aşama 1 başarılı olursa Aşama 2'ye geç. Başarısız olursa (örn. MAVROS bağlantı sorunu, Gazebo plugin hatası) önce bunu çöz.**

---

## AŞAMA 2 — Gerçek S500 Fiziksel Modeli

**Amaç:** Gerçek S500 donanımının kütle, geometri ve itki özelliklerini yansıtan özel bir Gazebo modeli oluşturmak, böylece simülasyon sonuçları gerçek uçuşu güvenilir şekilde temsil etsin.

### Adım 1: Klasör yapısını oluştur

ArduPilot Gazebo entegrasyonunun beklediği standart format:
```
s500_with_ardupilot/
├── model.config
└── model.sdf
```

### Adım 2: `iris_with_ardupilot` modelini kopyala, temel al

Sıfırdan yazmak yerine, `ArduPilot/ardupilot_gazebo` deposundaki `iris_with_ardupilot` klasörünü kopyalayıp üzerinde değişiklik yap — motor eklentisi (`ArduPilotPlugin`), sensör tanımları (IMU) ve kontrol kanalı yapısı (0-3 arası 4 motor, VELOCITY tipi) zaten doğru şablonu içeriyor.

### Adım 3: Geometriyi S500'e uyarla

- **Iris kolları hafif asimetrik X düzenindedir** (x=±0.13m, y=±0.21-0.22m).
- **S500 tam simetrik 45° X geometrisindedir** (x≈±0.177m, y≈±0.177m).
- Her `<link>` ve `<joint>` tanımındaki motor konumlarını (`<pose>`) bu simetrik 500mm ölçülerine göre güncelle.

### Adım 4: Kütleyi güncelle

- Iris taban kütlesi: ~1.5 kg (toplam ~1.65 kg).
- Tipik S500 (4S 4500-5000mAh pil, 2216 motorlar, otopilot dahil): **1.4 - 1.8 kg** arası. Kendi konfigürasyonuna (payload, kamera vb. varsa) göre bu aralıkta net bir değer seç ve `<inertial>` bloğundaki kütle ve atalet momentlerini buna göre güncelle.

### Adım 5: Motor/pervane itkisini güncelle

- S500 genellikle **2216 920KV motor + 1045 (10 inç) veya 9450 pervane, 4S LiPo** kullanır.
- `model.sdf` içindeki her `<control channel="N">` bloğundaki `<multiplier>` değeri motor tepe açısal hızını (rad/s) ifade eder. Iris'te bu değer 838; S500'de 4S voltajında 920KV motor için bu değer **~950-1050 rad/s** bandına çekilmeli.
- Pervanenin aerodinamik alan değerleri (`gz-sim-lift-drag-system` eklentisi varsa) 10 inç pal boyutuna göre güncellenmeli.

### Adım 6: Görsel mesh (basitleştirilmiş, yeterli)

Gerçek bir STL/DAE dosyasına şu aşamada ihtiyaç yok. Basit bir silindir (gövde) + 4 ince silindir/kutu (kollar) geometrisiyle idare edilebilir — fiziksel doğruluk (kütle, itki) görsel detaydan daha önemli.

### Adım 7: SITL'i doğru parametre dosyasıyla başlat

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map \
    --add-param-file=Holybro-S500.param
```

`Holybro-S500.param` dosyası şu kritik değerleri zaten içeriyor, bunları elle tekrar girmene gerek yok:
- `MOT_THST_HOVER = 0.25` (hover gazı oranı, Iris'in %35-40'ından farklı)
- `MOT_SPIN_ARM = 0.07`, `MOT_SPIN_MIN = 0.09` (motor dönüş limitleri)
- `MOT_BAT_VOLT_MAX = 16.8`, `MOT_BAT_VOLT_MIN = 13.2` (4S batarya limitleri)
- `ATC_RAT_PIT_*`, `ATC_RAT_RLL_*` (S500'ün kol yapısı ve ataletine göre ayarlanmış PID değerleri)

### Adım 8: Doğrulama

Aşama 1'de kullandığın Test A'yı bu sefer `s500_with_ardupilot` modeliyle ve `Holybro-S500.param` ile tekrar çalıştır. Bu sefer:
- Hover sırasında motor gaz seviyesinin gerçekten ~%25 civarında dengelendiğini gözlemleyebilirsin (Gazebo konsolunda veya MAVROS `/mavros/rc/out` üzerinden).
- Tırmanma/alçalma dinamiğinin Iris'e göre daha "güçlü/çevik" hissettirmesi beklenir (S500'ün daha yüksek itki/ağırlık oranı nedeniyle).

---

## Özet Tablo

| | Aşama 1 (Iris) | Aşama 2 (S500) |
|---|---|---|
| Model | `iris_with_ardupilot` (olduğu gibi) | `s500_with_ardupilot` (yeni, uyarlanmış) |
| Parametre dosyası | Varsayılan Iris parametreleri | `Holybro-S500.param` |
| Amaç | Kod/entegrasyon doğrulaması | Fiziksel gerçekçilik |
| Fiziksel doğruluk | Önemli değil | Kritik |
| Ne zaman | Hemen | Aşama 1 onaylandıktan sonra |

---

## Önemli Not

Eğer üzerinde çalışılan gerçek S500 donanımında standart konfigürasyondan (farklı batarya kapasitesi, ekstra payload/kamera ağırlığı, farklı motor/pervane) bir sapma varsa, bu bilgi Aşama 2'nin kütle ve itki hesaplarına mutlaka yansıtılmalı — aksi halde simülasyon sonuçları yine gerçek drone'dan sapabilir.
