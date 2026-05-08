import serial
import struct
import numpy as np
import sys
import time

# ================= CONFIG =================
PORT = "COM3"
BAUD = 115200

SYNC = b'\xAA\x55\xCC\x33'
FRAME_SIZE = 54

BAR_WIDTH = 101
CENTER = BAR_WIDTH // 2
# ==========================================

# ================= SERIAL =================

class FrameReader:
    def __init__(self, ser):
        self.ser = ser
        self.buffer = bytearray()

    def read(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        self.buffer.extend(data)

        while True:
            idx = self.buffer.find(SYNC)

            if idx == -1:
                if len(self.buffer) > FRAME_SIZE:
                    self.buffer = self.buffer[-FRAME_SIZE:]
                return None

            if len(self.buffer) < idx + FRAME_SIZE:
                return None

            frame = self.buffer[idx:idx + FRAME_SIZE]
            self.buffer = self.buffer[idx + FRAME_SIZE:]

            checksum = 0
            for b in frame[:-1]:
                checksum ^= b

            if checksum == frame[-1]:
                return frame
            else:
                self.buffer = self.buffer[1:]

# ================= PARSE =================

def parse_logs(frame):
    return struct.unpack_from("<16h", frame, 21)

def q8_8_to_float(arr):
    return np.array(arr, dtype=float) / 256.0

# ================= DISPLAY =================

def make_bar_multi(values, symbols):
    """
    values : liste de valeurs [-1,1]
    symbols : liste de caractères ('v', '^', ...)
    """
    bar = ["-"] * BAR_WIDTH
    bar[CENTER] = "|"

    for val, sym in zip(values, symbols):
        val = max(-1, min(1, val))
        pos = int(CENTER + val * CENTER)

        # gestion collision
        if bar[pos] == "-":
            bar[pos] = sym
        else:
            bar[pos] = "*"  # collision visuelle

    return "".join(bar)

def display(frame_id, cmds1, cmds2):
    lines = []
    lines.append(f"frame {frame_id:05d} :")

    for i in range(3):
        line = make_bar_multi(
            [cmds1[i], cmds2[i]],
            ["v", "^"]
        )
        lines.append(line)

    output = "\n".join(lines)

    sys.stdout.write("\033[H\033[J")
    sys.stdout.write(output)
    sys.stdout.flush()

# ================= MAIN =================

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.01)
    reader = FrameReader(ser)

    frame_id = 0

    try:
        while True:
            frame = reader.read()
            if frame is None:
                continue

            logs = parse_logs(frame)

            cmds1 = q8_8_to_float(logs[9:12])   # v
            cmds2 = q8_8_to_float(logs[12:15])  # ^

            frame_id += 1

            display(frame_id, cmds1, cmds2)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nArrêt.")

    finally:
        ser.close()

if __name__ == "__main__":
    main()