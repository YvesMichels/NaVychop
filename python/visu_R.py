#!/usr/bin/env python3
"""
visu_R.py - Visualisation temps réel de la matrice R_A_GRV envoyée par l'ESP32.

Trame attendue (une ligne par boucle) :
    R,r00,r01,r02,r10,r11,r12,r20,r21,r22\n

Usage :
    python visu_R.py --port COM5              # Windows
    python visu_R.py --port /dev/ttyUSB0      # Linux
    python visu_R.py --demo                   # sans matériel (matrice simulée)

Dépendances :
    pip install pyserial matplotlib numpy

Affichage :
    - à gauche : les 3 colonnes de R_A_GRV tracées dans le repère terre
      (colonne i = vecteur R[:, i]), avec le repère terre en pointillés gris ;
    - à droite : la matrice 3x3 (valeurs et code couleur) ;
    - en bas   : fréquence de réception, erreur d'orthonormalité, déterminant
      et élévation des colonnes 0 et 1 (R[2][0] et R[2][1], les composantes
      verticales lues par computeRollCommandSignal / computePitchCommandSignal).

Attention : ouvrir le port série réinitialise l'ESP32 (setup() est relancé).
Fermer tout autre moniteur série avant de lancer le script.
"""
import argparse
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ----------------------------------------------------------------------------
# Source de données (thread série ou démo) -> dernière matrice reçue
# ----------------------------------------------------------------------------
class MatrixSource:
    def __init__(self):
        self._lock = threading.Lock()
        self._R = np.eye(3)
        self._count = 0
        self.stop_event = threading.Event()
        self.status = "en attente de données"

    def set(self, R):
        with self._lock:
            self._R = R
            self._count += 1

    def get(self):
        with self._lock:
            return self._R.copy(), self._count


def parse_line(raw):
    """Retourne une matrice 3x3 si la ligne est une trame 'R,...' valide, sinon None."""
    txt = raw.decode("ascii", errors="ignore").strip()
    if not txt.startswith("R,"):
        return None                      # autres messages (INIT OK, etc.) ignorés
    parts = txt.split(",")
    if len(parts) != 10:
        return None
    try:
        vals = [float(p) for p in parts[1:]]
    except ValueError:
        return None
    return np.array(vals, dtype=float).reshape(3, 3)


def run_serial(src, port, baud):
    import serial
    while not src.stop_event.is_set():
        try:
            with serial.Serial(port, baud, timeout=0.2) as ser:
                src.status = f"connecté à {port} @ {baud}"
                ser.reset_input_buffer()
                while not src.stop_event.is_set():
                    R = parse_line(ser.readline())
                    if R is not None:
                        src.set(R)
        except (serial.SerialException, OSError) as e:
            src.status = f"port indisponible ({e}) - nouvelle tentative..."
            time.sleep(1.0)


def rot_xyz(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def run_demo(src):
    src.status = "mode démo (données simulées)"
    t0 = time.time()
    while not src.stop_event.is_set():
        t = time.time() - t0
        R = rot_xyz(0.5 * np.sin(0.7 * t), 0.4 * np.sin(0.4 * t), 0.3 * t)
        src.set(R)
        time.sleep(0.04)                 # 25 Hz


# ----------------------------------------------------------------------------
# Affichage
# ----------------------------------------------------------------------------
def build_figure(src):
    colors = ["tab:red", "tab:green", "tab:blue"]

    fig = plt.figure(figsize=(11, 5.5))
    try:
        fig.canvas.manager.set_window_title("R_A_GRV - temps réel")
    except Exception:
        pass

    # --- vue 3D ---
    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    ax3.set_xlim(-1, 1)
    ax3.set_ylim(-1, 1)
    ax3.set_zlim(-1, 1)
    if hasattr(ax3, "set_box_aspect"):          # absent des Matplotlib < 3.3
        ax3.set_box_aspect((1, 1, 1))
    ax3.set_xlabel("X terre")
    ax3.set_ylabel("Y terre")
    ax3.set_zlabel("Z terre")
    for v in np.eye(3):
        ax3.plot([0, v[0]], [0, v[1]], [0, v[2]], color="lightgray", lw=1, ls="--")
    lines = [
        ax3.plot([0, 1], [0, 0], [0, 0], color=c, lw=3, label=f"colonne {i}")[0]
        for i, c in enumerate(colors)
    ]
    ax3.legend(loc="upper left")

    # --- matrice ---
    axh = fig.add_subplot(1, 2, 2)
    im = axh.imshow(np.eye(3), vmin=-1, vmax=1, cmap="coolwarm")
    axh.set_xticks(range(3))
    axh.set_xticklabels(["col 0", "col 1", "col 2"])
    axh.set_yticks(range(3))
    axh.set_yticklabels(["lig 0", "lig 1", "lig 2"])
    axh.set_title("R_A_GRV")
    texts = [
        [axh.text(j, i, "", ha="center", va="center", fontsize=13) for j in range(3)]
        for i in range(3)
    ]

    info = fig.text(0.5, 0.015, "", ha="center", family="monospace", fontsize=9)
    fig.subplots_adjust(bottom=0.12)

    state = {"t": time.time(), "n": 0, "hz": 0.0}

    def update(_):
        R, n = src.get()

        for i, ln in enumerate(lines):
            v = R[:, i]
            ln.set_data([0, v[0]], [0, v[1]])           # compatible toutes versions
            ln.set_3d_properties([0, v[2]])

        im.set_data(R)
        for i in range(3):
            for j in range(3):
                texts[i][j].set_text(f"{R[i, j]:+.3f}")

        # fréquence de réception (lissée sur ~1 s)
        now = time.time()
        if now - state["t"] >= 1.0:
            state["hz"] = (n - state["n"]) / (now - state["t"])
            state["t"], state["n"] = now, n

        orth = np.linalg.norm(R.T @ R - np.eye(3))
        det = np.linalg.det(R)
        el0 = np.degrees(np.arcsin(np.clip(R[2, 0], -1, 1)))
        el1 = np.degrees(np.arcsin(np.clip(R[2, 1], -1, 1)))
        info.set_text(
            f"{src.status} | {state['hz']:4.1f} Hz | err.orth={orth:.1e} | det={det:+.4f} | "
            f"élév. col0={el0:+6.1f}°  col1={el1:+6.1f}°"
        )
        return []

    try:
        anim = FuncAnimation(fig, update, interval=40, blit=False, cache_frame_data=False)
    except TypeError:                           # Matplotlib ancien : pas de cache_frame_data
        anim = FuncAnimation(fig, update, interval=40, blit=False)
    return fig, anim


def main():
    ap = argparse.ArgumentParser(description="Visualisation temps réel de R_A_GRV")
    ap.add_argument("--port", help="port série (COM5, /dev/ttyUSB0, ...)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true", help="données simulées, sans matériel")
    args = ap.parse_args()

    if not args.demo and not args.port:
        ap.error("--port est obligatoire (ou utiliser --demo)")

    src = MatrixSource()
    target = (lambda: run_demo(src)) if args.demo else (lambda: run_serial(src, args.port, args.baud))
    th = threading.Thread(target=target, daemon=True)
    th.start()

    fig, anim = build_figure(src)
    try:
        plt.show(block=True)             # block=True : indispensable sous Spyder / IPython
    finally:
        src.stop_event.set()
        th.join(timeout=1.0)             # laisse le thread série se terminer proprement


if __name__ == "__main__":
    main()
