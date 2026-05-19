#!/bin/bash

set -e

cd ~/ros2_ws

# Sprawdzenie czy podano argument czyszczący
if [[ "$1" == "clean" || "$1" == "--clean" || "$1" == "-c" ]]; then
    echo "0/3 Czyszczenie obszaru roboczego (usunięcie build, install, log)..."
    rm -rf build/ install/ log/
fi

echo "1/3 Budowanie projektu"
colcon build --symlink-install

echo "2/3 Dodanie do sciezki"
source install/setup.bash

echo "3/3 Odpalenie projektu"
ros2 launch hexarover_bringup hexarover.launch.py

echo "Trzymaj sie moze i nie dziala narazie ale dasz rade"