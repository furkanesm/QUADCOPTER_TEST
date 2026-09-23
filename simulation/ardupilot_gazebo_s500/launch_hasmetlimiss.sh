#!/bin/bash
# Bu betik bulunduğu konumu baz alarak Gazebo yollarını ayarlar.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Gazebo Resource Path (Modeller ve dünyalar için)
export IGN_GAZEBO_RESOURCE_PATH="${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${IGN_GAZEBO_RESOURCE_PATH}"

# Plugin yolu (Eğer ardupilot_gazebo build klasörünüz farklıysa burayı güncelleyin)
# Varsayılan olarak sistemde kurulu kabul ediyoruz, gerekirse ekleyebilirsiniz:
# export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/path/to/ardupilot_gazebo/build:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"

echo "Gazebo kaynak yolları ayarlandı:"
echo "IGN_GAZEBO_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}"
echo "S500 ve Hasmetlimiss arazisi başlatılıyor..."

ign gazebo worlds/hasmetlimiss_s500.world
