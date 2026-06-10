## Lidar RPLidar A2M12

### Sprzęt

- RPLidar A2M12 (Slamtec)
- Podłączony przez USB do SOYO (przejściówka UART-USB, chipset CP210x)
- Port: `/dev/ttyUSB0`

### Praca z submodułem sllidar_ros2

Repozytorium `sllidar_ros2` jest dodane jako submoduł git w `src/sllidar_ros2`.
Oznacza to że w repo przechowywana jest tylko referencja do konkretnego commita
zewnętrznego repo — nie cały kod.

#### Pierwsze pobranie repo (git clone)

Zwykły `git clone` nie pobiera zawartości submodułów — trzeba to zrobić osobno:

```bash
git clone <adres_repo>
cd ros2_ws
git submodule update --init
```

Lub od razu przy klonowaniu:

```bash
git clone --recurse-submodules <adres_repo>
```

#### Po git pull

Jeśli ktoś zaktualizował submoduł i zrobił commit, po `git pull` trzeba zaktualizować submoduł:

```bash
git pull
git submodule update --init
```

#### Jak submoduł był dodawany

```bash
cd ~/ros2_ws
git submodule add https://github.com/Slamtec/sllidar_ros2.git src/sllidar_ros2
git commit -m "add sllidar_ros2 as submodule"
git push
```

#### Sprawdzenie czy submoduł jest poprawnie pobrany

```bash
ls ~/ros2_ws/src/sllidar_ros2/
```

Powinny być widoczne pliki źródłowe. Jeśli folder jest pusty — uruchom:

```bash
git submodule update --init
```

#### Po każdym pobraniu — build

```bash
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

### Instalacja sterownika

Pakiet `sllidar_ros2` jest dodany jako submoduł git:

```bash
git submodule update --init
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
```

Upewnij się że użytkownik jest w grupie `dialout`:

```bash
groups $USER
```

Jeśli nie ma `dialout`:

```bash
sudo usermod -aG dialout $USER
```

Potem zrestartuj system.

### Uruchomienie lidaru

```bash
ros2 launch sllidar_ros2 "sllidar_a2m12_launch .py"
```

Uwaga: spacja w nazwie pliku to błąd w oryginalnym repozytorium Slamtec — cudzysłowy są konieczne.

Lidar po uruchomieniu powinien zacząć się kręcić i wypisać:

```
SLLidar health status : OK.
current scan mode: Sensitivity, sample rate: 16 Khz, max_distance: 16.0 m, scan frequency:10.0 Hz
```

Dane publikowane są na temat `/scan` jako `sensor_msgs/LaserScan`.

### Wizualizacja w RViz2

W drugim terminalu:

```bash
rviz2
```

W RViz2:
1. Kliknij **Add** → wybierz **LaserScan** → OK
2. Rozwiń `LaserScan` w lewym panelu, zmień `Topic` na `/scan`
3. Zmień `Fixed Frame` z `map` na `laser`

Powinna pojawić się mapa punktów otoczenia.

Aby zapisać konfigurację: **File → Save Config As** → `~/ros2_ws/lidar.rviz`

Aby wczytać zapisaną konfigurację przy następnym uruchomieniu:

```bash
rviz2 -d ~/ros2_ws/lidar.rviz
```

### Parametry lidaru

| Parametr | Wartość |
|---|---|
| Tryb skanowania | Sensitivity |
| Częstotliwość próbkowania | 16 KHz |
| Zasięg | 16 m |
| Częstotliwość skanowania | 10 Hz |
| Firmware | 1.32 |
| Hardware Rev | 6 |

### Uwagi

- Lidar publikuje w ramce `laser` — przy integracji ze SLAMem trzeba ustawić odpowiedni `frame_id` w konfiguracji
- Temat `/scan` jest standardowym wejściem dla `slam_toolbox`