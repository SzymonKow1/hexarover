#!/bin/bash

set -e

cd ~/ros2_ws


echo "1/3 Budowanie projektu"
colcon build --symlink-install

echo "2/3 Dodanie do sciezki"
source install/setup.bash

echo "3/3 Odpalenie projektu"
ros2 launch hexarover_bringup hexarover.launch.py

echo "Gotowe :)"
