## 🚀 Szybki Start 

Wykonaj poniższe kroki w terminalu, aby pobrać projekt i uruchomić symulację.

### 1. Sklonuj repozytorium
upewnij sie ze znajdujesz sie w katalogu domowym:
```bash
cd
```
repozytorium lokalnie bedzie nazywac sie ros2_ws
```bash
git clone --recurse-submodules https://github.com/SzymonKow1/hexarover.git ros2_ws
```
### 3. Nadaniem uprawnien skryptom sh
```
chmod +x dependencies.sh
chmod +x build_and_run.sh
```
### 4. Instalacja zależności
do instalacji wykorzystujemy skrypt, nalezy wpisac haslo (to samo o co prosi komputer kiedy wpisujesz komende sudo)
```
./dependencies.sh
```
### 5. Budowa i uruchomienie projektu
Wykorzystujemy skrypt sh:
```
./build_and_run.sh
```

## 📊 Wizualizacja (RViz2)

Projekt posiada skonfigurowane środowisko RViz2, które uruchamia się automatycznie razem z symulacją.

* **Lokalizacja konfiguracji:** `src/hexarover_bringup/rviz/rviz.rviz`

## Przydatne komendy
kręcenie w kólko:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
```
sterowaniem robotem z klawiatury 
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 🛠 Rozwiązywanie problemów
#### Rviz
Jeśli po pobraniu zmian (`git pull`) RViz zgłasza błąd o braku pliku konfiguracyjnego, **musisz przebudować projekt**, aby nowy plik został zainstalowany:

```bash
colcon build --symlink-install
source install/setup.bash
```
#### Dziwne Zachowania Projektu
Nalezy usunać folderu które cashują build
```bash
cd /ros2_ws
rm -rf build/ install/ log/
```
i zbudowac projekt na nowo 
```bash
./build_and_run.sh
```
W skajrnych sytacjach restart komputera.
## Przydatne linki 
https://eti.pg.edu.pl/node/64/podaniadruki-do-pobrania
https://ug.edu.pl/kandydaci
