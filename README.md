Oto propozycja przejrzystego pliku `README.md`, który możesz bezpośrednio wkleić do swojego projektu. Opisuje on zarówno działanie skryptu `activate.sh`, jak i przeznaczenie skryptów dla Raspberry Pi 4 i 5.

***

# Sterowanie Robotem - Skrypty Uruchomieniowe

Repozytorium zawiera skrypty startowe przeznaczone dla dwóch minikomputerów sterujących pracą robota (**Raspberry Pi 4** oraz **Raspberry Pi 5**), a także narzędzie `activate.sh` ułatwiające ich globalne uruchamianie.

---

## 1. Jak dodać skrypty do systemu (Skrypt `activate.sh`)

Skrypt `activate.sh` służy do tego, aby automatycznie:
1. Nadać wybranemu plikowi uprawnienia do wykonywania (`chmod +x`).
2. Utworzyć dla niego skrót (dowiązanie symboliczne) w katalogu `/usr/local/bin`. 

Dzięki temu możesz wywołać dany skrypt z dowolnego folderu w terminalu, bez konieczności przechodzenia do katalogu projektu ani wpisywania ścieżek dostępu.

### Krok 1: Nadanie uprawnień dla `activate.sh` (jednorazowo)
Przed pierwszym użyciem nadaj uprawnienia samemu skryptowi aktywującemu:
```bash
chmod +x activate.sh
```

### Krok 2: Rejestracja skryptów startowych w systemie
Uruchom `activate.sh`, podając jako argument ścieżkę do skryptu, który chcesz dodać. 

*   **Jeśli konfigurujesz Raspberry Pi 4:**
    ```bash
    ./activate.sh rpi4-run.sh
    ```
*   **Jeśli konfigurujesz Raspberry Pi 5:**
    ```bash
    ./activate.sh rpi5-run.sh
    ```

*Uwaga: Podczas tego kroku skrypt poprosi o podanie hasła administratora (`sudo`), ponieważ tworzy globalne powiązanie w katalogach systemowych.*

---

## 2. Uruchamianie skryptów z dowolnego miejsca

Po pomyślnym wykonaniu kroku rejestracji, oryginalne rozszerzenie `.sh` zostaje pominięte dla ułatwienia zapisu. Możesz teraz otworzyć terminal w dowolnym katalogu na komputerze i uruchomić odpowiedni skrypt za pomocą prostej komendy:

*   **Dla Raspberry Pi 4:**
    ```bash
    rpi4-run
    ```
*   **Dla Raspberry Pi 5:**
    ```bash
    rpi5-run
    ```

Skrypty te automatycznie uruchamiają odpowiednie komponenty (np. węzły ROS 2, konfigurację czujników laserowych Lidar) dostosowane do zasobów i roli danego minikomputera w konstrukcji robota.

## 3. Rozwiązywanie problemów: Puste foldery w `src/` (Submoduły)

Jeśli po sklonowaniu repozytorium katalogi wewnątrz `src/` (takie jak `ros2_laser_scan_matcher`, `csm` czy `sllidar_ros2`) są puste, oznacza to, że Git nie pobrał jeszcze zawartości powiązanych bibliotek zewnętrznych.

Aby zainicjalizować submoduły i automatycznie pobrać cały brakujący kod, uruchom w głównym folderze projektu następujące polecenie:

```bash
git submodule update --init --recursive
```