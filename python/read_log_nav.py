import serial
import csv
import time
import sys

# =========================
# CONFIG
# =========================
PORT = "COM3"
BAUDRATE = 115200
OUTPUT_FILE = "log.csv"

SYNC_FRAME = bytes([0xAA, 0x55, 0xCC, 0x33])

# =========================
# MAIN
# =========================
def main():
    print(f"[INFO] Opening serial port {PORT} @ {BAUDRATE}")
    
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    except Exception as e:
        print(f"[ERROR] Cannot open serial port: {e}")
        sys.exit(1)

    # reset ESP32 / stabilisation UART
    time.sleep(2)

    print("[INFO] Sending SYNC frame...")
    ser.write(SYNC_FRAME)

    print("[INFO] Logging UART to CSV... (Ctrl+C to stop)")

    with open(OUTPUT_FILE, "w", newline="") as f:
        writer = csv.writer(f)

        try:
            while True:
                line = ser.readline().decode(errors="ignore").strip()

                if not line:
                    continue

                # Si format CSV déjà côté ESP32
                if "," in line:
                    writer.writerow(line.split(","))
                else:
                    writer.writerow([line])

        except KeyboardInterrupt:
            print("\n[INFO] Stopping logging...")

        finally:
            ser.close()

    print(f"[INFO] Saved to {OUTPUT_FILE}")


# =========================
# ENTRY POINT
# =========================
if __name__ == "__main__":
    main()