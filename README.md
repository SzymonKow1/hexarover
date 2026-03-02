## 🚀 Szybki Start 

Wykonaj poniższe kroki w terminalu, aby pobrać projekt i uruchomić symulację.

### 1. Pobranie repozytorium
Upewnij się, że masz utworzony workspace (katalog roboczy). Wejdź do folderu `src`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
### 2. Sklonuj repozytorium
```
git clone [https://github.com/SzymonKow1/hexarover.git](https://github.com/SzymonKow1/hexarover.git)
```
### 3. Zainstaluj niezbędne pakiety ROS 2 Jazzy
```
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-rf2o-laser-odometry
```
### 4. Instalacja i Budowanie
```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
### 5. Uruchomienie Symulacji 
```
source install/setup.bash
ros2 launch hexarover_bringup hexarover.launch.py
```
### 6. Testowe sterowanie 
(w nowym oknie terminala)
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
```
## 📊 Wizualizacja (RViz2)

Projekt posiada skonfigurowane środowisko RViz2, które uruchamia się automatycznie razem z symulacją.

* **Lokalizacja konfiguracji:** `src/hexarover_bringup/rviz/rviz.rviz`

### Jak korzystać?
Po uruchomieniu komendy startowej (`ros2 launch...`) otworzy się okno RViz.

* **Czerwone kropki/linie** – odczyty z lasera (ściany i przeszkody).
* **Model robota** – aktualna pozycja i stan kół.

### 🛠 Rozwiązywanie problemów z RViz
Jeśli po pobraniu zmian (`git pull`) RViz zgłasza błąd o braku pliku konfiguracyjnego, **musisz przebudować projekt**, aby nowy plik został zainstalowany:

```bash
colcon build --symlink-install
source install/setup.bash
```
