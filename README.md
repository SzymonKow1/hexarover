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
### 3. Instalacja i Budowanie
```
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```
### 4. Uruchomienie Symulacji 
```
source install/setup.bash
ros2 launch hexarover_bringup hexarover.launch.py
```
### 5. Testowe sterowanie 
(w nowym oknie terminala)
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
```
