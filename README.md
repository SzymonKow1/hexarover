# Uruchomienie sterownika Cytron MDDS30 z Ubuntu 24.04 i testowanie działania sterownika

## Sprzęt

- Minikomputer SOYO M4 Mini (Ubuntu 24.04)
- Sterownik silników Cytron SmartDriveDuo-30 (MDDS30)
- Przejściówka USB-UART (chipset Silicon Labs CP210x)
- Akumulator LiPo 3S 11.1V
- Podwozie Dagu Wild Thumper 6WD (6 silników DC, 3 na kanał)

## Fizyczne podłączenie

Przejściówka USB-UART → Cytron MDDS30:
- TX przejściówki → IN1 na Cytronie
- GND przejściówki → GND na Cytronie
- IN2 zostawić niepodłączony (nieużywany w tym trybie)

UWAGA: Cytron nie wysyła danych zwrotnych, RX przejściówki zostaje wolny.

## Ustawienie DIP switchy na Cytronie

Pozycja: `11001100` 

- SW1=1, SW2=1 → tryb Serial Simplified (UART)
- SW3=0, SW4=0, SW5=1, SW6=1 → prędkość transmisji 9600 baud, (dokumentacja Cytron)
- SW7=0, SW8=0 → monitoring baterii LiPo

Po zmianie DIP switchy zawsze wcisnąć przycisk RESET na sterowniku.

## Protokół Serial Simplified

Jeden bajt to jedna komenda. Struktura bajtu Serial Simplified: 

- bit 7 - kanał(0 lewy/1 prawy) 
- bit 6 - kierunek (0 CW - Clockwise / 1 CCW - CounterClockwise) 
- bity 5 do 0 - prędkość (00000 -> stop / 11111 inaczej decymalnie 63 -> full speed) 

Bajt komendy trafia przez UART (TX) do pinu IN1 sterownika 

## Sprawdzenie połączenia na Ubuntu

Sprawdź czy system widzi przejściówkę USB-UART:
```bash
lsusb
```
Powinno pojawić się: `Silicon Labs CP210x UART Bridge`

Sprawdź czy port szeregowy jest dostępny:
```bash
ls /dev/ttyUSB*
```
Powinno pojawić się: `/dev/ttyUSB0`

Sprawdź grupy użytkownika:
```bash
groups $USER
```
Musi być `dialout` na liście. Jeśli nie ma:
```bash
sudo usermod -aG dialout $USER
```
Następnie zrestartować system. sudo reboot


## Testowanie silników z Pythona

Wysyłanie komend przez pyserial (zainstalowany domyślnie na Ubuntu 24.04):

**Lewy silnik CW (prędkość 20/63):**
```bash
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 9600); s.write(bytes([20]))"
```

**Lewy silnik CCW (prędkość 20/63):**
```bash
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 9600); s.write(bytes([84]))"
```

**Prawy silnik CW (prędkość 20/63):**
```bash
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 9600); s.write(bytes([148]))"
```

**Prawy silnik CCW (prędkość 20/63):**
```bash
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 9600); s.write(bytes([212]))"
```

**Stop lewy:**
```bash
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 9600); s.write(bytes([0]))"
```

**Stop prawy:**
```bash
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 9600); s.write(bytes([128]))"
```

## Kierunek jazdy robota

Po weryfikacji fizycznej:
- Jazda do przodu: lewy CCW + prawy CW
- Jazda do tyłu: lewy CW + prawy CCW




# Instalacja ROS2 Jazzy (Ubuntu 24.04)

Aktualizacja systemu:
```bash
sudo apt update && sudo apt upgrade -y
```

Instalacja narzędzi:
```bash
sudo apt install software-properties-common curl -y
```

Dodanie klucza GPG ROS2:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Dodanie repozytorium ROS2:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Odświeżenie listy pakietów i instalacja ROS2:
```bash
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

Aktywacja ROS2 (jednorazowo w sesji):
```bash
source /opt/ros/jazzy/setup.bash
```

Dodanie do bashrc żeby działało automatycznie przy każdym otwarciu terminala:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



# Sterownik Cytrona w ROS2

### Struktura paczki

Paczka `cytron_driver` znajduje się w `src/cytron_driver/`.
Główny plik to `src/cytron_driver/cytron_driver/cytron_node.py`.

Węzeł subskrybuje temat `/cmd_vel` (standardowy temat ROS2 do sterowania robotem)
i tłumaczy komendy prędkości na bajty wysyłane przez UART do Cytrona.

### Pierwsze uruchomienie

Zbuduj workspace:
```bash
cd ~/ros2_ws
colcon build
```

Zasourcuj ROS2 i workspace (jeśli nie masz w ~/.bashrc):
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Uruchom węzeł:
```bash
ros2 run cytron_driver cytron_node
```

Powinno pojawić się: [INFO] [cytron_driver]: Cytron driver uruchomiony

### Sterowanie

W drugim terminalu wysyłaj komendy na temat `/cmd_vel`.
Wartości `linear.x` i `angular.z` są w zakresie -1.0 do 1.0.

Jazda do przodu:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --once
```

Jazda do tyłu:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3}, angular: {z: 0.0}}" --once
```

Skręt w lewo:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.3}}" --once
```

Skręt w prawo:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.3}}" --once
```

Stop:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Uwagi

- Cytron musi być zasilony z akumulatora przed uruchomieniem węzła
- Port szeregowy to `/dev/ttyUSB0`, baud rate 9600
- Użytkownik musi być w grupie `dialout`
- Silniki Dagu wytrzymują max 7.5V — nie przekraczać wartości linear.x powyżej 0.5 dopóki robot nie jest przetestowany pod obciążeniem
