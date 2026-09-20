# S500 Simülasyon Bağımlılıkları ve Yol Tanımlamaları

Dosyalar ana `ardupilot_gazebo` deposundan alınıp `QUADCOPTER_TEST/simulation/ardupilot_gazebo_s500` klasörüne taşındığı için, simülasyonu başlatırken kaynak ve model yollarının ortam değişkenlerine (Environment Variables) eklenmesi zorunludur. Aksi takdirde Gazebo, `libArduPilotPlugin.so` eklentilerini veya `model://s500_custom_with_ardupilot` mesh'lerini bulamayacaktır.

## 1. ArduPilot Gazebo Eklentileri (Plugin) Bağımlılığı
Bu klasörde kaynak C++ plugin kodları (ör. `ArduPilotPlugin`) yoktur. Doğrudan resmi repoya bağımlıdır.
Sistemin çalışması için resmi [ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) kütüphanesinin bilgisayarınızda derlenmiş olması gerekir.
Derlenmiş eklentilerin yolunu Gazebo'ya göstermek için:
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
```

## 2. Model ve Dünya Yolları (Resource Path)
Gazebo'nun `s500_custom_with_ardupilot` modelini ve `iris.dae` gibi mesh dosyalarını bulabilmesi için, bu klasörün bulunduğu konumun sisteme tanıtılması gerekir.
Eğer deponuz (QUADCOPTER_TEST) `/home/kullanici/QUADCOPTER_TEST` konumundaysa:
```bash
export GZ_SIM_RESOURCE_PATH=/home/kullanici/QUADCOPTER_TEST/simulation/ardupilot_gazebo_s500/models:${GZ_SIM_RESOURCE_PATH}
export GZ_SIM_RESOURCE_PATH=/home/kullanici/QUADCOPTER_TEST/simulation/ardupilot_gazebo_s500/worlds:${GZ_SIM_RESOURCE_PATH}
```
*(Not: Model `iris.dae` dosyasını `models/s500_custom_with_ardupilot/meshes/` içinden yükler. Mesh URI'leri model.sdf içerisinde görelidir (`meshes/iris.dae`)*

## 3. Python Analiz Betikleri Bağımlılıkları
`s500_workspace/scripts` ve `verify_inertia.py` içindeki analiz betiklerinin çalışması için şu kütüphaneler gereklidir:
```bash
pip3 install pandas pymavlink matplotlib
```

## 4. Temiz Bir Kurulumda S500 Testini Başlatma Adımları
1. Yukarıdaki ortam değişkenlerini `~/.bashrc` dosyanıza ekleyin veya çalışmadan önce terminalde export edin.
2. İzole itki testini çalıştırmak için:
   ```bash
   cd QUADCOPTER_TEST/simulation/ardupilot_gazebo_s500
   python3 scripts/run_thrust_test.py
   ```
3. ArduPilot SITL simülasyonu için:
   ```bash
   cd QUADCOPTER_TEST
   Tools/autotest/sim_vehicle.py -v Copter -f gazebo-iris --console --add-param-file="simulation/ardupilot_gazebo_s500/s500_workspace/parameters/s500_quad_baseline.param"
   # Diğer sekmede:
   ign gazebo simulation/ardupilot_gazebo_s500/worlds/s500_custom_runway.world
   ```
