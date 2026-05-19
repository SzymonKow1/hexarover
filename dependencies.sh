#!/bin/bash

# Zatrzymuje skrypt, jeśli któreś polecenie zwróci błąd
set -e

echo "1/4 Aktualizacja listy pakietów..."
sudo apt update

echo "2/4 Instalacja paczek systemowych i ROS..."
sudo apt install -y libgsl-dev ros-jazzy-tf2-geometry-msgs


echo "3/4 Poprawa plików nagłówkowych dla ROS 2 Jazzy..."
sed -i 's/tf2_geometry_msgs.h/tf2_geometry_msgs.hpp/g' ~/ros2_ws/src/ros2_laser_scan_matcher/include/ros2_laser_scan_matcher/laser_scan_matcher.h

echo "4/4 Instalacja zależności przez rosdep..."
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "================================================="
echo "Gotowe! Wszystkie zależności zostały zainstalowane."
echo "Możesz teraz przejść do katalogu ~/ros2_ws i odpalić colcon build."
