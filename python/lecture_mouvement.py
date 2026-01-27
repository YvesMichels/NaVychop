import serial
import struct
import time
import matplotlib.pyplot as plt
import numpy as np

# =======================
# Configuration
# =======================

PORT = "COM6"
BAUD = 115200
SYNC = b'\xAA\x55\xCC\x33'
TRANSFORM_FORMAT = 0x42   # à aligner avec le code C++
TIMEOUT = 5.0

# =======================
# Outils bas niveau
# =======================

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

# =======================
# Décodage mouvement
# =======================

def read_transform_packet(ser):
    checksum = 0

    # --- Format (NON inclus dans le checksum)
    fmt = read_exact(ser, 1)[0]
    if fmt != TRANSFORM_FORMAT:
        raise ValueError("Format TRANSFORM invalide")

    # --- Coût
    raw_cost = read_exact(ser, 4)
    for b in raw_cost:
        checksum ^= b
    cost_q16 = struct.unpack("<I", raw_cost)[0]

    # --- Paramètres transformation
    raw_params = read_exact(ser, 6 * 4)
    for b in raw_params:
        checksum ^= b
    params_q16 = struct.unpack("<6i", raw_params)
    
    # --- Logs 256 octets
    raw_params = read_exact(ser, 64 * 4)
    for b in raw_params:
        checksum ^= b
    logs_q16 = struct.unpack("<64i", raw_params)

    # --- Checksum reçu
    checksum_rx = read_exact(ser, 1)[0]
    if checksum != checksum_rx:
        raise ValueError(f"Checksum incorrect (calc={checksum:#02x}, rx={checksum_rx:#02x})")

    return cost_q16, params_q16, logs_q16

# =======================
# Visualisation
# =======================

def plot_cross(tx, ty, theta, ax):
    """Affiche une croix centrée en (tx, ty) et orientée selon theta (rad)."""
    L = 15.0  # longueur demi-bras de la croix en pixels
    # Rotation de la croix
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    
    # Coordonnées de la croix avant rotation
    coords = np.array([
        [0, 0], [L, -L],  # ligne horizontale
        [-L, L], [0, 0]   # ligne verticale
    ]).reshape(2, 4)
    
    coords_rot = R @ coords
    coords_rot[0, :] += tx
    coords_rot[1, :] += ty

    # Affichage
    ax.cla()
    ax.plot(coords_rot[0, 0:2], coords_rot[1, 0:2], 'r-', linewidth=2)  # horizontale
    ax.plot(coords_rot[0, 2:4], coords_rot[1, 2:4], 'r-', linewidth=2)  # verticale
    ax.set_xlim(-250, 250)
    ax.set_ylim(-250, 250)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title(f"tx={tx:+.2f}, ty={ty:+.2f}, theta={theta:.3f}")
    plt.pause(0.001)

# =======================
# Boucle principale
# =======================

def motion_reader():
    ser = None
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        print("Port série ouvert — lecture du mouvement en cours")
        print("Ctrl+C pour arrêter\n")

        fig, ax = plt.subplots()
        tx_cum = 0
        ty_cum = 0
        theta_cum = 0
        while True:
            wait_for_sync(ser)
            try:
                cost, (div, theta, tx, ty, vp_x, vp_y), logs_q16 = read_transform_packet(ser)
                # Conversion en float
                tx_f = tx / 65536.0
                ty_f = ty / 65536.0
                theta_f = theta / 65536.0  # radians approximatifs
                tx_cum = tx_cum + tx_f
                ty_cum = ty_cum + ty_f
                theta_cum = theta_cum + theta_f

                print(f"cost={cost/65536:8.4f} | "
                      f"zoom=({div:+.4f}) | "
                      f"Rot=({theta_f:+.4f}) | "
                      f"trans=({tx_f:+.4f},{ty_f:+.4f}) | "
                      f"centre=({vp_x/65536:+.4f},{vp_y/65536:+.4f})")
                
                for it in range(1, 11):
                    print(logs_q16[it]-logs_q16[it-1])

                # Affichage
                plot_cross(tx_cum, ty_cum, -theta_cum, ax)

            except ValueError as e:
                print("Paquet rejeté :", e)

    except KeyboardInterrupt:
        print("\nArrêt demandé par l'utilisateur")

    finally:
        if ser and ser.is_open:
            ser.close()
            print("Port série fermé")
            plt.close(fig)

# =======================
# Lancement
# =======================

if __name__ == "__main__":
    motion_reader()
