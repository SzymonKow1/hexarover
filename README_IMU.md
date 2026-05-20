## Konfiguracja IMU (SeeedStudio Grove MMA7660FC) z RPi 4B

### Sprzęt

- Raspberry Pi 4B (Debian 13)
- IMU SeeedStudio Grove 3-Axis Accelerometer MMA7660FC
- Kabel ethernet RPi ↔ SOYO

### Podłączenie IMU do GPIO RPi

| Kolor kabla | Pin GPIO RPi | Funkcja |
|---|---|---|
| Czarny | Pin 6 | GND |
| Czerwony | Pin 1 | VCC (3.3V) |
| Biały | Pin 3 | SDA (I2C) |
| Żółty | Pin 5 | SCL (I2C) |

### Pierwsza konfiguracja RPi (jednorazowo)

Połącz z WiFi:
```bash
sudo rfkill unblock wifi
sudo nmcli device wifi connect "NAZWA_SIECI" password "HASŁO" name "moje_wifi"
```

Włącz I2C przez raspi-config:
```bash
sudo raspi-config
```
Wejdź w `Interface Options` → `I2C` → `Yes`.

Zainstaluj narzędzia I2C:
```bash
sudo apt install i2c-tools python3-smbus -y
```

Sprawdź czy IMU jest wykryte (powinien pojawić się adres `4c` w wierszu `40:`):
```bash
sudo i2cdetect -y 1
```

### Konfiguracja sieci ethernet RPi ↔ SOYO

Adresy IP resetują się po restarcie — ustawiaj za każdym razem.

Na SOYO:
```bash
sudo ip addr add 192.168.10.1/24 dev enp2s0
sudo ip link set enp2s0 up
```

Na RPi:
```bash
sudo ip addr add 192.168.10.2/24 dev eth0
sudo ip link set eth0 up
```

Sprawdź połączenie:
```bash
ping 192.168.10.1 -c 3
```

### Architektura

RPi czyta dane z IMU przez I2C i wysyła przez UDP do SOYO.
SOYO odbiera dane i publikuje jako temat ROS2 `/imu`.

Skrypt na RPi: `~/imu_sender.py`
Węzeł ROS2 na SOYO: `cytron_driver/imu_node.py`

### Uruchomienie

Na SOYO:
```bash
ros2 run cytron_driver imu_node
```

Na RPi:
```bash
python3 ~/imu_sender.py
```

Sprawdzenie danych na SOYO (trzeci terminal):
```bash
ros2 topic echo /imu
```

# AKTUALIZACJA OD NOWEGO FOLDERU IMU_PUBLISHER 

## Uruchomienie systemu IMU od zera

### Wymagania
- RPi i SOYO połączone kablem ethernet
- IMU podłączone do GPIO RPi
- Skrypt `imu_sender.py` na RPi w `~/imu_sender.py`

### Krok 1 — SOYO: ustaw adres IP na ethernecie

```bash
sudo ip addr add 192.168.10.1/24 dev enp2s0
sudo ip link set enp2s0 up
```

### Krok 2 — RPi: ustaw adres IP na ethernecie

```bash
sudo ip addr add 192.168.10.2/24 dev eth0
sudo ip link set eth0 up
```

### Krok 3 — Sprawdź połączenie (z RPi)

```bash
ping 192.168.10.1 -c 3
```

### Krok 4 — SOYO: uruchom węzeł IMU

```bash
ros2 run imu_publisher imu_node
```

### Krok 5 — RPi: uruchom skrypt wysyłający dane

```bash
python3 ~/imu_sender.py
```

### Krok 6 — SOYO: sprawdź czy dane płyną (opcjonalnie)

```bash
ros2 topic echo /imu
```

### Uwagi
- Adresy IP resetują się po restarcie — trzeba ustawiać za każdym razem
- IMU publikuje dane na temat `/imu` jako wiadomość `sensor_msgs/Imu`
- Dane zawierają przyspieszenie liniowe w m/s² na osiach X, Y, Z

## Autostart IMU na RPi (systemd)

Skrypt `imu_sender.py` uruchamia się automatycznie przy starcie RPi
bez potrzeby logowania się ani wpisywania komend.

### Jak to działa

Serwis systemd `/etc/systemd/system/imu_sender.service` uruchamia skrypt
po tym jak sieć jest gotowa. Jeśli skrypt padnie — automatycznie się restartuje
po 5 sekundach.

### Przydatne komendy diagnostyczne

Sprawdź czy serwis działa:
```bash
sudo systemctl status imu_sender
```

Sprawdź logi jeśli coś nie działa:
```bash
sudo journalctl -u imu_sender.service -n 50
```

Ręczne zatrzymanie/uruchomienie:
```bash
sudo systemctl stop imu_sender
sudo systemctl start imu_sender
```

### Co sprawdzać gdy nie działa

- `Active: failed` → sprawdź logi przez `journalctl`
- `OSError: Network is unreachable` → sieć nie była gotowa przy starcie, sprawdź czy ethernet jest podłączony i czy RPi ma adres `192.168.10.2`
- `No such file or directory: '/dev/i2c-1'` → I2C nie jest włączone, wejdź w `sudo raspi-config` → `Interface Options` → `I2C`
- `FileNotFoundError: imu_sender.py` → skrypt nie istnieje w `/home/bobik/imu_sender.py`
- Na SOYO nie ma danych na `/imu` mimo że serwis działa → sprawdź czy SOYO ma adres `192.168.10.1` na `enp1s0`

