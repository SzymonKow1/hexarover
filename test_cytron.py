
import serial
import time
import sys

# PORT = '/dev/cytron'
PORT = '/dev/ttyUSB0'
BAUD = 9600
ADDRESS = 0
HEADER = 0x55

def make_packet(channel, command, address=0):
    channel_byte = (channel << 3) | (address & 0x07)
    checksum = (HEADER + channel_byte + command) % 256
    return bytes([HEADER, channel_byte, command, checksum])

print("=== TEST SPRZĘTOWY STEROWNIKA CYTRON ===")
print(f"Otwieram port {PORT} z prędkością {BAUD}...")

try:
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
except Exception as e:
    print(f"Błąd otwarcia portu: {e}")
    sys.exit(1)

# Wysyłamy dummy byte do auto-baudrate (0x80 / 128)
print("Wysyłam dummy byte do auto-baudrate (0x80)...")
ser.write(bytes([0x80]))
time.sleep(1.0)  # Czekamy sekundę na wykrycie

# Pętla testowa: kręć silnikami przez 3 sekundy
print("Wysyłam komendy ruchu do silników (lewy i prawy powoli do przodu)...")
# Prędkość: stop to 127, 160 to delikatny ruch do przodu
left_packet = make_packet(0, 160, ADDRESS)
right_packet = make_packet(1, 160, ADDRESS)

start_time = time.time()
while time.time() - start_time < 3.0:
    ser.write(left_packet)
    ser.write(right_packet)
    time.sleep(0.05)  # Wysyłamy co 50ms, aby uniknąć Timeoutu (zabezpieczenie 100ms)

# Zatrzymanie
print("Wysyłam komendę STOP (127)...")
stop_left = make_packet(0, 127, ADDRESS)
stop_right = make_packet(1, 127, ADDRESS)
ser.write(stop_left)
ser.write(stop_right)

ser.close()
print("Test zakończony.")
