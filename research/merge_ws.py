import os

# Nazwa pliku wyjściowego
OUTPUT_FILE = "ros2_ws_context.txt"

# Foldery, które bezwzględnie pomijamy (bardzo ważne w ROS 2)
IGNORED_DIRS = {"build", "install", "log", ".git", "__pycache__", "build_isolated", "devel"}

# Rozszerzenia plików, które nas interesują
ALLOWED_EXTENSIONS = {".py", ".cpp", ".hpp", ".h", ".yaml", ".yml", ".md", ".json", ".msg", ".srv", ".action", ".md"}
# Specyficzne pliki konfiguracyjne ROS bez standardowych rozszerzeń
ALLOWED_NAMES = {"CMakeLists.txt", "package.xml"}

def generate_context():
    # Uruchamiamy w bieżącym katalogu (.)
    with open(OUTPUT_FILE, "w", encoding="utf-8") as outfile:
        for root, dirs, files in os.walk("."):
            # Modyfikacja dirs w miejscu pozwala os.walk pomijać te foldery całkowicie
            dirs[:] = [d for d in dirs if d not in IGNORED_DIRS]
            
            for file in files:
                _, ext = os.path.splitext(file)
                if ext.lower() in ALLOWED_EXTENSIONS or file in ALLOWED_NAMES:
                    file_path = os.path.join(root, file)
                    try:
                        # Ignorujemy ewentualne błędy kodowania, np. przy nietypowych znakach
                        with open(file_path, "r", encoding="utf-8", errors="ignore") as infile:
                            outfile.write(f"\n\n=========================================\n")
                            outfile.write(f"FILE: {file_path}\n")
                            outfile.write(f"=========================================\n\n")
                            outfile.write(infile.read())
                            outfile.write("\n")
                    except Exception as e:
                        print(f"Nie udało się odczytać pliku {file_path}: {e}")

if __name__ == "__main__":
    generate_context()
    print(f"Gotowe! Cały kontekst został zapisany w pliku: {OUTPUT_FILE}")
