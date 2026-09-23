# Hasmetlimiss Arazi - S500 Simülasyon Ortamı

Bu branch (`simulation/hasmetlimiss-s500-2550g`), Hasmetlimiss arazisini (obj) ve 2.550 kg (2550 g) S500 drone modelimizi içeren yeni Gazebo ortamını barındırmaktadır.

## Dikkat Edilmesi Gerekenler & Doğrulanmamış Durumlar
- **Uçuş ve Temas Testi:** Bu yeni arazide uçuş ve zemin temas testleri henüz tamamlanmamış/doğrulanmamıştır.
- **Kaplamalar (Textures):** Elimizde `hasmetlimiss.mtl` ve ilgili doku (texture) resimleri bulunmadığı için model geçici olarak gri bir önizleme ile yüklenmektedir. Renkler ve kaplamalar eksiktir.
- **Model Yönü & Konumu:** Arazi objesi +90 derece X ekseninde döndürülerek yatay hale getirilmiştir. S500 modeli ise `0 8 0.5` koordinatlarında başlatılmaktadır.
- **Arka Plan:** Gökyüzü (`<sky>`) etiketi kaldırılarak arka plan mavimsi düz bir renge (`0.72 0.76 0.80 1`) ayarlanmıştır.

## Kullanılan Drone Modeli
Standart Iris modeli *kullanılmamıştır*. Repomuzdaki 2550 gram kütleye, doğru atalet, CG, motor sırası, sensör eklemleri ve LiftDrag ayarına sahip olan orijinal `s500_custom_with_ardupilot` modelimiz korunmuştur. Model klasöründe değişiklik yapılmamıştır, sadece yeni dünya dosyasından doğru model (`model://s500_custom_with_ardupilot`) çağrılmaktadır.

## Kurulum ve Bağımlılıklar (Arkadaşım İçin)
Gazebo Fortress ve ArduPilot SITL eklentisinin (`ardupilot_gazebo`) sisteminizde kurulu olması gerekmektedir. Derlenmiş `.so` dosyaları repoda yer almadığı için, bilgisayarınızda eklentilerin kaynak koddan derlenmiş ve sistem yollarına eklenmiş olduğundan emin olun. (Iris mesh bağımlılıkları da aynı eklenti kaynaklarından sağlanmaktadır).

## Başlatma Talimatı
Aşağıdaki bash betiği, bulunduğunuz dizini bularak Gazebo yol değişkenlerini (`IGN_GAZEBO_RESOURCE_PATH` ve `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`) ayarlar ve ortamı başlatır:

```bash
cd simulation/ardupilot_gazebo_s500
./launch_hasmetlimiss.sh
```
