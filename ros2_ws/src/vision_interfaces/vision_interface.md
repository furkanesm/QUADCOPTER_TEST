# Vision Interface / FSM ROTA_TESPIT Kılavuzu

Bu paket (`vision_interfaces` ve `vision_processing`), görüntü işleme (YOLO) döngüsünü bağımsız bir ROS 2 Node'u olarak yalıtıp, sonuçlarını FSM'nin tüketmesini sağlayacak bir yapıdadır.

## 1. Topic ve Mesaj Yapısı

**Topic Adı:** `/vision/detections`

**Mesaj Tipi:** `vision_interfaces/msg/DetectionArray`
```yaml
std_msgs/Header header
vision_interfaces/Detection[] detections
float32 altitude_used
bool pose_valid
```

**`vision_interfaces/msg/Detection` İçeriği:**
```yaml
string class_name        # Örneğin: 'Hedef', 'Bariyer' vb.
int32 class_id           # Model.names içerisindeki ID
float32 confidence       # Tespit güveni (0.0 - 1.0)
float32 bbox_cx          # Orijinal piksel ölçeğinde orta nokta X
float32 bbox_cy          # Orijinal piksel ölçeğinde orta nokta Y
float32 bbox_w           # Bounding Box genişliği (Piksel)
float32 bbox_h           # Bounding Box yüksekliği (Piksel)
bool position_valid      # TRUE ise aşağıdaki koordinatlara güvenebilirsin. FALSE ise NaN.
float32 local_x          # ENU sistemindeki Local X (Takeoff noktasına göre) metre
float32 local_y          # ENU sistemindeki Local Y (Takeoff noktasına göre) metre
```

## 2. Mimari Kurallar ve Garantiler

1. **Mesaj Frekansı:** Node, kendisine tanımlanan `inference_rate_hz` (örn. 10 Hz) değeri ile devamlı çalışır. 
2. **"Boş Liste" vs "Ölü Düğüm" Ayırımı:**
   - Eğer Düğüm (Node) ayakta ise **HER DÖNGÜDE (10 HZ)** mesaj yayınlar. 
   - Kamerada tespit yoksa bile, boş bir `detections = []` barındıran `DetectionArray` mesajı akar.
   - **FSM Tarafı:** Eğer `/vision/detections` nesnesi 1-2 saniye boyunca *HİÇ* okunmuyorsa (Timeout), vizyon düğümü çökmüş/ölmüştür veya kameradan frame gelmiyordur!
3. **Koordinat Çerçevesi:** Faz 2 aktif olduğunda, hesaplanan `local_x` / `local_y` tıpkı MAVROS'un `/mavros/local_position/pose`'dan veya otopilottan aldığınız kalkış referanslı ENU sisteminde olur. FSM içerisinde hedef Waypoint belirlerken bu metrik veriler direkt otopilota (örneğin Local Target olarak) basılabilir.

## 3. FSM `ROTA_TESPIT` Durumu İçin Önerilen Pseudo-Kod

`mission_node.py` dosyasına entegre etmek üzere, `TARAMA_SONRASI_BEKLEME` yerine kurgulanacak durumu (State) şu yaklaşımla inşaa edebilirsiniz:

```python
# FSM Sınıfı içindeki _fsm_loop içerisinde eklenecek yeni durum mantığı:

def _handle_rota_tespit(self, elapsed: float):
    # 1. Vision düğümü hayatta mı?
    if (time.time() - self.last_vision_msg_time) > 2.0:
         self._abort_mission("Vision düğümü yanıt vermiyor! /vision/detections zaman aşımı.")
         return

    # 2. Timeout: Hedef bulunamama durumu (örneğin Max 15 sn aradık)
    if elapsed > 15.0:
        self.get_logger().warn("[ROTA_TESPIT] Hedef 15s icinde bulunamadi! RTL'ye donuluyor.")
        self._transition_to(MissionState.KALKIS_NOKTASINA_DONUS, "Hedef bulunamadi timeout.")
        return

    # 3. Son alınan tespit dizisi (Kuyruktaki son DetectionArray mesaji) 
    # Not: vision_callback içinden `self.latest_detections` değişkenine depolandı varsayıyoruz.
    if self.latest_detections and len(self.latest_detections) > 0:
        
        # Filtreleme: "Hedef" isimli, konumu geçerli ve en yüksek configdence olanı al.
        targets = [d for d in self.latest_detections if d.class_name == "Hedef" and d.position_valid]
        
        if targets:
            best_target = max(targets, key=lambda t: t.confidence)
            
            # Opsiyonel: 3 ardışık frame'de hedefin teyit edilmesini beklemek gürültüyü azaltır.
            self.target_hit_count += 1
            if self.target_hit_count >= 3:
                self.get_logger().info(f"[ROTA_TESPIT] HEDEF ONAYLANDI! (X:{best_target.local_x:.2f}, Y:{best_target.local_y:.2f})")
                
                # Uçuş planına/Navigasyona hedefi gönder (Veya hedefe doğru yeni WP türet!)
                # ...
                
                self._transition_to(MissionState.NEXT_STATE, "Hedef tespit edildi ve konum doğrulandı.")
        else:
            # Gelen dizide "Hedef" yok veya pozisyon geçersiz (Henüz havada stabil değiliz)
            self.target_hit_count = max(0, self.target_hit_count - 1)
```
