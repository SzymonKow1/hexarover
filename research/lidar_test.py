from rplidar import RPLidar
import csv
import time

PORT_NAME = '/dev/ttyUSB0'
PREDKOSC_TRANSMISJI = 256000 

lidar = RPLidar(PORT_NAME, baudrate=PREDKOSC_TRANSMISJI)

try:
    print("Uruchamiam silnik...")
    lidar.start_motor()
    
    # Czekamy na stabilizację obrotów, żeby pominąć niestandardowe pakiety startowe
    print("Czekam 3 sekundy na stabilizację obrotów...")
    time.sleep(3)
    
    # Czyścimy bufor ze śmieci komunikacyjnych wygenerowanych przy rozruchu
    lidar._serial_port.reset_input_buffer()

    with open('dane_testowe.csv', mode='w', newline='') as plik_csv:
        writer = csv.writer(plik_csv)
        writer.writerow(['Kąt (stopnie)', 'Dystans (milimetry)'])

        print("Rozpoczynam zrzut danych pomiarowych...")
        
        # Pusta funkcja iter_scans() - bez problematycznego argumentu
        for numer_skanu, skan in enumerate(lidar.iter_scans()):
            print(f"Zapisuję pełny skan nr: {numer_skanu + 1}")
            
            for pomiar in skan:
                kat = pomiar[1]
                dystans = pomiar[2]
                writer.writerow([kat, dystans])
                
            if numer_skanu >= 9:
                break

except Exception as e:
    print(f"Wystąpił błąd: {e}")

finally:
    print("Zamykam sesję i wyłączam sprzęt.")
    # Zabezpieczenie bloku zamykającego, aby port zawsze został zwolniony
    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except:
        pass
