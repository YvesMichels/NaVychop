import serial
import numpy as np
import matplotlib.pyplot as plt
import struct
import os
import msvcrt
import csv
import imageio

PORT = "COM6"
BAUD = 115200
SYNC = b'\xAA\x55\xCC\x33'

IMG_FORMAT = 0x01
POINT_FORMAT = 0x04
META_FORMAT = 0x04

IMG_WIDTH = 320
IMG_HEIGHT = 240
IMG_WIDTH_R = 40
IMG_HEIGHT_R = 30
MAX_POINTS = 300
MAX_POINTS_R = 104

# Crée dossiers de sauvegarde
os.makedirs("images", exist_ok=True)
os.makedirs("images_r", exist_ok=True)
os.makedirs("points", exist_ok=True)
os.makedirs("points_r", exist_ok=True)
os.makedirs("meta", exist_ok=True)

plt.ion()

fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(8, 4))

# Image pleine résolution
im_l = ax_l.imshow(
    np.zeros((IMG_HEIGHT, IMG_WIDTH), dtype=np.uint8),
    cmap='gray', vmin=0, vmax=255
)
ax_l.axis('off')
ax_l.set_title(f"{IMG_WIDTH}x{IMG_HEIGHT}")

# Image décimée
im_r = ax_r.imshow(
    np.zeros((IMG_HEIGHT_R, IMG_WIDTH_R), dtype=np.uint8),
    cmap='gray', vmin=0, vmax=255
)
ax_r.axis('off')
ax_r.set_title(f"{IMG_WIDTH_R}x{IMG_HEIGHT_R}")

plt.tight_layout()
plt.show()


def read_exact(ser, n):
    data = b''
    while len(data) < n:
        chunk = ser.read(n - len(data))
        if not chunk:
            raise TimeoutError("Timeout UART")
        data += chunk
    return data


def wait_for_sync(ser):
    buf = b''
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("Timeout waiting for SYNC")

        buf += b
        if len(buf) > len(SYNC):
            buf = buf[-len(SYNC):]

        if buf == SYNC:
            return


def enter_pressed():
    if msvcrt.kbhit():
        return msvcrt.getch() in (b'\r', b'\n')
    return False


ser = None
index = 0

try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
    print("Port série ouvert. Appuyez sur ENTER pour arrêter.")

    while True:

        if enter_pressed():
            break

        try:
            # --- Synchronisation ---
            wait_for_sync(ser)
            print("Nouvelle image synchronisée")

            checksum = 0

            # --- Image ---
            width  = int.from_bytes(read_exact(ser, 2), 'little')
            height = int.from_bytes(read_exact(ser, 2), 'little')
            fmt = read_exact(ser, 1)[0]

            if fmt != IMG_FORMAT:
                print("Format image invalide, resync")
                continue

            img_size = width * height
            raw_img = read_exact(ser, img_size)

            for b in raw_img:
                checksum ^= b

            img = np.frombuffer(raw_img, dtype=np.uint8)\
                    .reshape((height, width))
                    
            # --- Image Réduite ---
            width_r  = int.from_bytes(read_exact(ser, 2), 'little')
            height_r = int.from_bytes(read_exact(ser, 2), 'little')
            fmt = read_exact(ser, 1)[0]

            if fmt != IMG_FORMAT:
                print("Format image invalide, resync")
                continue

            img_size = width_r * height_r
            raw_img = read_exact(ser, img_size)

            for b in raw_img:
                checksum ^= b

            img_r = np.frombuffer(raw_img, dtype=np.uint8)\
                    .reshape((height_r, width_r))

            # --- Points ---
            nb_pts = int.from_bytes(read_exact(ser, 2), 'little')
            fmt = read_exact(ser, 1)[0]

            if fmt != POINT_FORMAT or nb_pts > 1000:
                print("Nombre de points invalide, resync")
                continue

            raw_pts = read_exact(ser, nb_pts * 4)
            for b in raw_pts:
                checksum ^= b

            points_32b = np.frombuffer(raw_pts, dtype=np.uint32)
            points = []

            for p in points_32b:
                grad =  p        & 0x3F
                pix  = (p >> 6)  & 0xFF
                x    = (p >> 14) & 0x1FF
                y    = (p >> 23) & 0x1FF
            
                points.append((y, x, pix, grad))

            # --- Points Réduit ---
            nb_pts_r = int.from_bytes(read_exact(ser, 2), 'little')
            fmt = read_exact(ser, 1)[0]

            if fmt != POINT_FORMAT or nb_pts_r > 1000:
                print("Nombre de points invalide, resync")
                continue

            raw_pts_r = read_exact(ser, nb_pts_r * 4)
            for b in raw_pts_r:
                checksum ^= b

            points_32b = np.frombuffer(raw_pts_r, dtype=np.uint32)
            points_r = []

            for p in points_32b:
                grad =  p        & 0x3F
                pix  = (p >> 6)  & 0xFF
                x    = (p >> 14) & 0x1FF
                y    = (p >> 23) & 0x1FF
            
                points_r.append((y, x, pix, grad))

            # --- Meta ---
            fmt = read_exact(ser, 1)[0]
            if fmt != META_FORMAT:
                print("Meta format invalide, resync")
                continue


            raw_meta = read_exact(ser, 256)
            # Premier mot (4 octets) = cam_param
            b0, b1, b2, b3 = raw_meta[:4]
            
            cam_param = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
            
            # Extraction des valeurs
            aec_value = cam_param & 0xFFFF        # bits 0..15
            ae_level  = (cam_param >> 16) & 0xFF  # bits 16..23
            agc_gain  = (cam_param >> 24) & 0xFF  # bits 24..31

            print(aec_value, ae_level, agc_gain)
            #print(raw_meta)
            
            for b in raw_meta:
                checksum ^= b
            

            # --- Checksum ---
            checksum_rx = read_exact(ser, 1)[0]
            if checksum != checksum_rx:
                print("Checksum incorrect, paquet ignoré")
                continue

            # --- Affichage ---
            im_l.set_data(img)
            ax_l.set_title(f"{width}x{height} | pts={nb_pts}")
            
            im_r.set_data(img_r)
            ax_r.set_title(f"{width_r}x{height_r}")
            # Affiche points
            for y, x, pix, grad in points[:MAX_POINTS]:
                ax_l.plot(x, y, marker='+', color='red', markersize=(1+grad))
                
            for y, x, pix, grad in points_r[:MAX_POINTS_R]:
                ax_r.plot(x, y, marker='+', color='red', markersize=(1+grad))

            fig.canvas.draw_idle()
            plt.pause(0.1)
            ax_l.lines.clear()  # supprime les croix pour le prochain paquet
            
            # --- Sauvegarde ---
            imageio.imwrite(f"images/{index:04d}.png", img)
            imageio.imwrite(f"images_r/{index:04d}.png", img_r)
            with open(f"points/{index:04d}.csv", 'w', newline='') as f_csv:
                writer = csv.writer(f_csv)
                writer.writerow(['y','x','pixel','grad'])
                writer.writerows(points[:MAX_POINTS])
                
            with open(f"points_r/{index:04d}.csv", 'w', newline='') as f_csv:
                writer = csv.writer(f_csv)
                writer.writerow(['y','x','pixel','grad'])
                writer.writerows(points_r[:MAX_POINTS_R])

            with open(f"meta/{index:04d}.csv", 'w', newline='') as f_csv:
                writer = csv.writer(f_csv)
                writer.writerow(["aec_value", "ae_level", "agc_gain"])
                writer.writerow([aec_value, ae_level, agc_gain])


            index += 1
        except TimeoutError as e:
            print("Timeout :", e)
            continue

finally:
    if ser and ser.is_open:
        ser.close()
        print("Port série fermé")