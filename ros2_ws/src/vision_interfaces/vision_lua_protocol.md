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
* Protokolün temel handshake (onaylaşma) kısmı eklenmiştir.

## Örnek Yapılandırma
*(Ortak konfigürasyonla doğrulanmalı)*
*   **FCU (Cube/Lua):** System 1, Component 1
*   **Vision (Jetson):** System 1, Component 191

## Handshake İşlemi ve Lua Sözleşmesi
Jetson tarafı oturum işlemlerini (IDLE -> STARTING -> ACTIVE) şeklinde yerel olarak kurala bağlamış durumdadır. Aşağıdaki sözleşme bildirimleri hedef Jetson-Lua iletişim yapısını özetler. *(Not: Bu sözleşme henüz gerçek bir Lua uçağı ile donanımsal olarak doğrulanmamıştır.)*

### 1. Yeni Oturum İsteği (SESSION_START = 0xAC)
Jetson'dan Lua'ya gelir.

| Param | İçerik | Detay |
|---|---|---|
| p1 | 0 | - |
| p2 | 0 | - |
| p3 | 0 | - |
| p4 | Görev Sırası (SEQ) | 1..16777215 arası atanmış işlem numarası |
| p5 | 0xAC | Özel mesaj türü (SESSION_START) |
| p6 | SESSION_ID | Jetson'un rastgele atadığı kimlik numarası (1..16777215) |
| p7 | 1 | Versiyon |

- Lua, işlem durumunun izin verdiği aşamada bu oturumu açıkça kabul eder.
- Aynı isteğin tekrar gönderilmesi halinde Lua'nın yine aynı ACK'yi üretmesi beklenir; ancak Lua bu onayı yeniden değerlendirip uçuş sistemini başa **sarmamalıdır**.
- Kabul işlemi, uçuş FSM'ini veya EKF origin noktasına dayalı ortak referansları kendi başına sıfırlamaz. Sadece Jetson veri akışına yetki verildiğini temsil eder.
- Jetson farklı bir `SESSION_ID` ile gelirse (Jetson yeniden başlaması vs.), bu tek başına Lua'daki mevcut aktif uçuş oturumunu/durumunu sıfırlamaz. Sadece yeni haberleşme kimliğidir.

### 2. İsteğe Yanıt Geri Bildirimi (APP_ACK)
Lua'dan Jetson'a özel yanıt:

| Param | İçerik | Detay |
|---|---|---|
| p1 | 0xAC | Orijinal istek türü |
| p2 | 0 veya 1 | 0 = ACCEPTED (Oturum Açıldı) / 1 = REJECTED (Oturum Reddedildi) |
| p3 | 0 | - |
| p4 | Orijinal SEQ | Jetson'un yolladığı SEQ değeri |
| p5 | 0xAB | APP_ACK belirticiği |
| p6 | Orijinal SESSION_ID | Jetson'un yolladığı SESSION_ID |
| p7 | 1 | Versiyon |

- Geçerli fakat ortam elverişsizliği vb. nedenden dolayı kabul edilmeyen isteğe `REJECTED (1)` dönülür. Ret vermek ile hiç cevap vermeyip (sessiz) Timeout'a düşürmek eş tutulmaz (Reject gelmesi Jetson'u hızla rahatlatır).
- Lua tarafında oturumu bilinçli bırakma, kopma ya da yeniden kabul etme politikasının Lua kendi uçuş betiği içerisinde ayrıca açıkça idare edilmesi gerekir.
