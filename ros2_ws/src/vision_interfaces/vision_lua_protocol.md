TASLAK: Lua ile ortak konfigürasyonla doğrulanmadı

# Jetson - Lua İletişim Protokolü

**Taşıyıcı:** `COMMAND_LONG` (`MAV_CMD_USER_1` = 31010)

## Olay Parametreleri (Giden)

| Param | İçerik | Detay |
|---|---|---|
| p1 | NED north VEYA Sağlık | EKF origin'e göre, metre (veya Liveliness durumu) |
| p2 | NED east | EKF origin'e göre, metre |
| p3 | Confidence | |
| p4 | SEQ | Unique Sequence Number |
| p5 | MSG_TYPE | |
| p6 | SESSION_ID | |
| p7 | 1 | Protokol Sürümü |

## Olay Türleri (MSG_TYPE)
*   `1`: START_FOUND (bilgi)
*   `2`: GOAL_FOUND (bilgi)
*   `3`: GOTO_OBSERVATION
*   `4`: ROUTE_READY
*   `5`: GCS_DELIVERED
*   `0xAA`: LIVELINESS (p1 sağlık, p2/p3/p4=0, ACK beklemez)
*   `0xAB`: APP_ACK (Lua'dan Jetson'a geri teyit)

## APP_ACK Geri Bildirimi (`0xAB`)
APP_ACK'nin kendisi tekrar ACK beklemez.
| Param | İçerik |
|---|---|
| p1 | orijinal MSG_TYPE |
| p2 | 0 = ACCEPTED / 1 = REJECTED |
| p3 | 0 |
| p4 | orijinal SEQ |
| p5 | 0xAB |
| p6 | orijinal SESSION_ID |
| p7 | 1 |

* Tekrar iletilen olaylar aynı oturum, sıra, tür ve yükü olduğu gibi korur. Lua aynı işlemi yinelemez, en sonki sonucunu tekrar teyit eder.
* ACK kabulünde kaynak ve hedef kimlikleri de (System/Component ID) denetlenir.
* Standart COMMAND_ACK veya SEQ-only V_ACK olay kuyruğunu tamamlamaz. Özel APP_ACK şarttır.
* ACCEPTED dönmesi, robotun hareketinin bittiği anlamına gelmez.
* Yeni veya bilinmeyen oturumlu paketler olay mekanizması üzerinden kabul edilmez.

## Mevcut Durum
* **GOTO_OBSERVATION, ROUTE_READY ve GCS_DELIVERED** üreticileri şu an koda BAĞLI DEĞİLDİR.
* `session_active=False` olduğu için adaptör uçuş kontrolcüsüne olay VE canlılık mesajı göndermemektedir. Bu nedenle Jetson→Lua trafiği şu an tamamen kapalıdır.

## Örnek Yapılandırma
*(Ortak konfigürasyonla doğrulanmalı)*
*   **FCU (Cube/Lua):** System 1, Component 1
*   **Vision (Jetson):** System 1, Component 191

## Açık Kararlar
*   **Hedef Firmware Desteği:** Hedef firmware'de Lua üzerinden doğrudan MAVLink alma/gönderme desteği henüz doğrulanmamıştır. C++ çekirdeğinin `UNSUPPORTED` diyerek ACK basması, paketin Lua'ya ulaşmadığının tek başına kanıtı DEĞİLDİR. Sınanmalıdır.
*   **Zaman Uyumu:** Clock_type seçimi tek başına zaman senkronizasyonunu başaramaz, asıl görüntü oluşturma zamanıyla uyum incelenmelidir.
*   **Oturum Kabul Mekanizması:** Jetson'un oturum açması (Handshake/Session Start) için komut akışı.
*   **Origin Eşleşmesi:** Lua içerisindeki tespit edilen konumların EKF origin'e göre mi yoksa Home noktasına göre mi tahsis edileceği.
*   **Arama Sınırları:** Hedef arama bölgesi dış çiti sınırlandırması.
*   **Dönüş Koşulu:** UAV'nin eve dönme mantığı ROUTE_READY yayınından hemen sonra mı tetiklenecek yoksa GCS_DELIVERED verisinden sonra mı.
