# Konfiguracja Raspberry Pi 5 z ROS2 Jazzy

## Sprzęt
- Raspberry Pi 5 (4GB RAM)
- Karta SD 64GB
- Zasilacz USB-C (**wymagany 5V/5A = 27W**, np. oficjalny zasilacz RPi 5)
- Czytnik kart SD (adapter USB→SD do laptopa)

> **UWAGA — zasilanie:** RPi 5 wymaga zasilacza 5V/5A (27W) z USB Power Delivery.
> Słabszy zasilacz (np. 5V/2A) spowoduje ostrzeżenie w systemie:
> `"This power supply is not capable of supplying 5A"` i throttling CPU przy
> większym obciążeniu (SLAM + YOLO). Na czas konfiguracji słabszy zasilacz wystarczy.

---

## 1. Przygotowanie karty SD na laptopie z Windows

### Potencjalny problem: Windows pyta o formatowanie karty SD
Gdy włożysz kartę SD z systemem Linux, Windows nie rozpoznaje systemu plików
i pyta o formatowanie — **kliknij Anuluj/Zamknij**, nie formatuj.
Jeśli karta jest Twoja i chcesz ją wyczyścić, Raspberry Pi Imager zrobi to sam.

### Potencjalny problem: karta SD z folderem DCIM
Jeśli na karcie jest tylko folder DCIM — możesz go zignorować.
Imager nadpisze całą kartę podczas wgrywania systemu.

### Potencjalny problem: blokada zapisu na karcie SD
Jeśli Imager zgłasza błąd zapisu (`Error writing to storage device`):
- Sprawdź czy adapter USB→SD ma fizyczny przełącznik blokady zapisu i przesuń go
- Jeśli przełącznik nie pomaga, spróbuj odblokować programowo przez Diskpart:
```
Win+R → cmd → Uruchom jako administrator
diskpart
list disk
select disk N        (N = numer Twojej karty SD)
attributes disk clear readonly
exit
```

---

## 2. Wgranie systemu — Raspberry Pi Imager

1. Pobierz i zainstaluj **Raspberry Pi Imager** na laptopie z Windows:
   👉 https://www.raspberrypi.com/software/

2. W Imagerze wybierz:
   - **Urządzenie:** Raspberry Pi 5
   - **System operacyjny:** Other general-purpose OS → Ubuntu → **Ubuntu Server 24.04 LTS (64-bit)**
   - **Pamięć:** Twoja karta SD

   > **Dlaczego Server a nie Desktop?**
   > Ubuntu Desktop jest za ciężki dla 4GB RAM przy jednoczesnym SLAM + YOLO.
   > Zamiast tego używamy SSH + X forwarding (MobaXterm) do wyświetlania okien
   > graficznych (RViz2, podgląd kamery) na laptopie.

3. W ustawieniach skonfiguruj:
   - **Hostname:** `hexarover`
   - **Użytkownik:** `bobik` (lub inny login)
   - **Hasło:** ustaw własne
   - **WiFi:** nazwa i hasło sieci (bez polskich znaków i spacji w nazwie)
   - **SSH:** włącz, uwierzytelnianie hasłem

4. Zapisz obraz — Imager sam wyczyści kartę i wgra system.

---

## 3. Pierwsze uruchomienie

1. Włóż kartę SD do RPi 5 i uruchom.
2. **Poczekaj ~2 minuty** — Ubuntu przy pierwszym starcie konfiguruje się automatycznie,
   pojawią się długie logi (SSH host keys, cloud-init itp.) — to normalne.
3. Gdy pojawi się prompt logowania, wpisz login i hasło.

Sprawdź temperaturę (wentylator nie kręci się przy niskiej temperaturze — normalne):
```bash
vcgencmd measure_temp
```

Sprawdź adres IP (potrzebny do SSH):
```bash
ip a
```
Szukaj adresu przy `wlan0`.

---

## 4. Aktualizacja systemu

```bash
sudo apt update && sudo apt upgrade -y
sudo reboot
```

Po restarcie zaloguj się ponownie.

---

## 5. Instalacja ROS2 Jazzy

### 5.1 Wymagania wstępne

```bash
sudo apt install software-properties-common curl ca-certificates -y
sudo update-ca-certificates
```

### 5.2 Klucz GPG ROS2

> **UWAGA:** Używamy flagi `--insecure` bo niektóre sieci (hotspoty telefoniczne)
> robią inspekcję SSL i powodują błąd certyfikatu. To jednorazowe pobranie klucza.

```bash
sudo curl -sSL --insecure https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### 5.3 Dodanie repozytorium ROS2

```bash
echo "deb [arch=arm64 trusted=yes signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Sprawdź czy plik powstał poprawnie:
```bash
cat /etc/apt/sources.list.d/ros2.list
```
Powinno wyświetlić linię zaczynającą się od `deb [arch=arm64 ...`.

### 5.4 Instalacja ROS2

> **ZNANY BUG na RPi 5 + Ubuntu 24.04:** ROS2 Jazzy ma konflikt zależności z nowszymi
> wersjami bibliotek systemowych Ubuntu. Zwykły `apt install` nie zadziała.
> Rozwiązanie: użyj `aptitude`, które automatycznie downgrade'uje konfliktujące biblioteki.
>
> Dodatkowo niektóre sieci blokują SSL dla packages.ros.org — używamy flagi
> `Verify-Peer=false`.

```bash
sudo apt install aptitude -y
sudo apt-get -o Acquire::https::packages.ros.org::Verify-Peer=false update
sudo aptitude -o Acquire::https::packages.ros.org::Verify-Peer=false install ros-jazzy-desktop
```

**Aptitude będzie pytać o rozwiązanie konfliktów — odpowiadaj tak:**

1. `Accept this solution? [Y/n/q/?]` → wpisz `n`
   *(pierwsza propozycja to "nie instaluj" — odrzucamy)*
2. `No solution found within the allotted time. Try harder? [Y/n]` → wpisz `Y`
3. `Accept this solution? [Y/n/q/?]` → wpisz `Y`
   *(druga propozycja zawiera downgrade bibliotek — akceptujemy)*
4. `Do you want to continue? [Y/n]` → wpisz `Y`

Instalacja trwa **kilkanaście minut**.

### 5.5 Konfiguracja środowiska

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5.6 Weryfikacja

```bash
ros2 topic list
```
Powinno wyświetlić `/parameter_events` i `/rosout`.

Pełny test działania:
```bash
ros2 run demo_nodes_py talker
```
Powinny pojawiać się linie `Publishing: "Hello World: 1"` itd. Zatrzymaj przez **Ctrl+C**.

---

## 6. Instalacja narzędzi deweloperskich

> Używamy flagi `Verify-Peer=false` bo colcon i rosdep są w repozytorium ROS.

```bash
sudo apt-get -o Acquire::https::packages.ros.org::Verify-Peer=false install python3-colcon-common-extensions python3-rosdep python3-pip git -y
```

### Inicjalizacja rosdep

```bash
sudo rosdep init
rosdep update
```

---

## 7. Utworzenie workspace ROS2

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 8. Konfiguracja WiFi

> **UWAGA:** Na Ubuntu Server `nmtui` domyślnie nie zarządza WiFi — sieć jest
> konfigurowana przez `netplan`. Nie używaj `nmtui` do zmiany WiFi.

Plik konfiguracyjny: `/etc/netplan/50-cloud-init.yaml`

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Przykładowa struktura z dwiema sieciami:
```yaml
network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: true
      dhcp6: true
  wifis:
    wlan0:
      optional: true
      dhcp4: true
      regulatory-domain: "PL"
      access-points:
        "NAZWA_SIECI_1":
          auth:
            key-management: "psk"
            password: "HASLO_1"
        "NAZWA_SIECI_2":
          auth:
            key-management: "psk"
            password: "HASLO_2"
```

> **Ważne:** wcięcia w pliku YAML muszą być dokładnie zachowane (spacje, nie taby).

Po edycji zastosuj zmiany:
```bash
sudo netplan apply
ip a
```

---

## Stan po konfiguracji

- Ubuntu Server 24.04 LTS na RPi 5
- ROS2 Jazzy zainstalowany i działający
- Workspace `~/ros2_ws` gotowy
- rosdep, colcon, pip, git zainstalowane
- Automatyczne sourcowanie ROS2 i workspace przy każdym logowaniu
