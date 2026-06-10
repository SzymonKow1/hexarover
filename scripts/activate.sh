#!/bin/bash

# 1. Sprawdzenie, czy podano argument (ścieżkę do pliku)
if [ -z "$1" ]; then
    echo "Błąd: Nie podano pliku jako argument."
    echo "Użycie: $0 <sciezka_do_pliku>"
    exit 1
fi

TARGET_FILE="$1"

# 2. Sprawdzenie, czy wskazany plik istnieje
if [ ! -f "$TARGET_FILE" ]; then
    echo "Błąd: Plik '$TARGET_FILE' nie istnieje."
    exit 1
fi

# 3. Pobranie bezwzględnej (pełnej) ścieżki do pliku
ABS_PATH=$(cd "$(dirname "$TARGET_FILE")" && pwd)/$(basename "$TARGET_FILE")

# 4. Określenie nazwy komendy (usuwa rozszerzenie .sh, jeśli istnieje)
# Np. "skrypt.sh" zmieni się w komendę "skrypt"
COMMAND_NAME=$(basename "$TARGET_FILE" .sh)

echo "-> Nadawanie uprawnień do uruchamiania dla: $ABS_PATH"
chmod +x "$ABS_PATH"

echo "-> Tworzenie dowiązania symbolicznego w /usr/local/bin/$COMMAND_NAME"
echo "Może być wymagane podanie hasła administratora (sudo):"

# 5. Tworzenie dowiązania (opcja -f nadpisze istniejące, jeśli już tam było)
sudo ln -sf "$ABS_PATH" "/usr/local/bin/$COMMAND_NAME"

if [ $? -eq 0 ]; then
    echo "--------------------------------------------------"
    echo "Sukces! Plik jest gotowy."
    echo "Możesz go teraz uruchomić z dowolnego miejsca wpisując:"
    echo "  $COMMAND_NAME"
    echo "--------------------------------------------------"
else
    echo "Wystąpił błąd podczas tworzenia dowiązania symbolicznego."
    exit 1
fi