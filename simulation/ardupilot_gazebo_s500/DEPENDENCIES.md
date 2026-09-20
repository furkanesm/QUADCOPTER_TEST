# S500 Simülasyon Bağımlılıkları ve Yol Tanımlamaları

Dosyalar `QUADCOPTER_TEST/simulation/ardupilot_gazebo_s500` klasörüne taşındığı için, simülasyonu başlatırken kaynak ve model yollarının ortam değişkenlerine (Environment Variables) eklenmesi zorunludur.

## 1. ArduPilot Gazebo Eklentileri (Plugin) Bağımlılığı
Bu klasörde kaynak C++ plugin kodları (ör. `ArduPilotPlugin`) yoktur. Sistemi çalıştıran (özellikle Fortress uyumlu) eklentiler için [ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) deposunun derlenmiş olması gerekir.

- **Uyumlu ArduPilot Gazebo Sürümü:** Geliştirmeler sırasında test edilen ve kararlı çalışan sürüm `fortress` dalına ait `5f1a88511a60fd6149654ca1ea677173bfd38259` commit'idir.
- **Derleme (Fortress İçin):**
  ```bash
  git clone https://github.com/ArduPilot/ardupilot_gazebo.git
  cd ardupilot_gazebo
  git checkout fortress
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j4
  ```
Derlenmiş eklentilerin yolunu Gazebo'ya göstermek için (`QUADCOPTER_TEST` dizininde çalışırken):
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
```

## 2. Model, Mesh ve Dünya Yolları (Resource Path)
Gazebo'nun `s500_custom_with_ardupilot` modelini ve içindeki mesh'leri (`meshes/iris.dae`, `meshes/iris_prop_cw.dae` vb.) bulabilmesi için, model dizininin dışarıdan tanıtılması gerekir. Bu mesh'ler standart Gazebo Iris modelinden alınıp `models/s500_custom_with_ardupilot/meshes/` içine dahil edilmiştir (dış bir bağımlılık kalmamıştır).
Ancak `runway` gibi default modeller için klasör yolları şarttır:
```bash
export GZ_SIM_RESOURCE_PATH=$(pwd)/simulation/ardupilot_gazebo_s500/models:${GZ_SIM_RESOURCE_PATH}
export GZ_SIM_RESOURCE_PATH=$(pwd)/simulation/ardupilot_gazebo_s500/worlds:${GZ_SIM_RESOURCE_PATH}
```

## 3. Python Analiz Betikleri Bağımlılıkları
`s500_workspace/scripts` ve `verify_inertia.py` içindeki analiz betiklerinin çalışması için şu kütüphaneler gereklidir:
```bash
pip3 install pandas pymavlink matplotlib
```

## 4. Temiz Bir Kurulumda S500 Testini Başlatma Adımları
Öncelikle 1. ve 2. adımdaki `export` komutlarını çalıştırın (veya `~/.bashrc` dosyanıza ekleyin).

**A. İzole İtki (Rotor) Testi:**
```bash
cd QUADCOPTER_TEST/simulation/ardupilot_gazebo_s500
python3 scripts/run_thrust_test.py
```

**B. ArduPilot SITL Simülasyonu Uçuşu:**
Simülasyonun önceki başarılı uçuş başlatma akışı şudur:
```bash
cd QUADCOPTER_TEST
# SITL başlat (Parametreleri otonom yüklemek veya varsayılanla başlamak için)
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map
```
*Not Parametreler Hakkında:* Eğer sanal EEPROM'unuz temizse ve gerçek donanım bazlı S500 pin/sensör ayarlarını SITL içine çekmek istiyorsanız, MAVProxy terminalinde veya başlatma komutuna `--add-param-file="simulation/ardupilot_gazebo_s500/s500_workspace/parameters/s500_quad_baseline.param"` ekleyerek baseline'ı **bir kereye mahsus** yükleyebilirsiniz. Bu dosyanın sürekli yüklenmesi **zorunlu değildir**, çünkü SITL uçuş sırasında öğrenilmiş hover gazı (`MOT_THST_HOVER`) gibi değerleri sanal EEPROM'unda tutar (örneğin 2.55kg uçuşunda bu değer 0.48'e çıkmıştı).
