"""
param_tuner.py — Interface graphique de paramétrage temps réel
==============================================================
Dépendances : Python 3.8+, tkinter (standard), json, struct
Communication physique : pip install pyserial pygame
"""

from __future__ import annotations

import json
import math
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional

# ---------------------------------------------------------------------------
# Imports optionnels (pas bloquants si absents)
# ---------------------------------------------------------------------------
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False

# ---------------------------------------------------------------------------
# Constantes protocole paramétrage
# ---------------------------------------------------------------------------
SYNC_1        = 0xAA
SYNC_2        = 0x55
MSG_TYPE_BASE = 0x10   # offset MSB de l'id dans tx_type

# Constantes protocole commande (même SYNC)
TYPE_CMD  = 0x01
TYPE_SAVE = 0x02

INT16_MIN = -32768
INT16_MAX =  32767

BAUDRATE_DEFAULT  = 115200
LORA_RATE_HZ      = 5          # cadence max LoRa (paramétrage ET pilotage)
LORA_PERIOD_S     = 1.0 / LORA_RATE_HZ   # 200 ms

# Largeurs colonnes fixes partagées (header + lignes)
COL_WIDTHS = {
    "label":  200,
    "entry":  110,
    "quant":  160,
    "unit":    90,
    "status":  22,
    "send":    72,
    "reset":   28,
}
TOTAL_WIDTH = sum(COL_WIDTHS.values())

# ---------------------------------------------------------------------------
# Palette & polices
# ---------------------------------------------------------------------------
P = {
    "bg":         "#0F1117",
    "surface":    "#1A1D27",
    "surface2":   "#222535",
    "border":     "#2E3347",
    "accent":     "#4F8EF7",
    "accent_dim": "#2B4F8A",
    "warn":       "#F7A74F",
    "ok":         "#4FBF7F",
    "err":        "#F75F5F",
    "fg":         "#D8DCF0",
    "fg_dim":     "#6B7196",
    "fg_label":   "#9BA3CC",
    "mono":       ("Courier New", 10),
    "ui":         ("Segoe UI", 10),
    "head":       ("Segoe UI Semibold", 11),
    "cat":        ("Segoe UI Semibold", 12),
    "title":      ("Segoe UI", 16, "bold"),
    "big":        ("Segoe UI", 13, "bold"),
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

def find_first_serial_port() -> Optional[str]:
    """Retourne le premier port série détecté, ou None."""
    if not HAS_SERIAL:
        return None
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else None

def make_tooltip(widget, text: str):
    if not text:
        return
    tip = None
    def show(e):
        nonlocal tip
        tip = tk.Toplevel(widget)
        tip.wm_overrideredirect(True)
        tip.wm_geometry(f"+{e.x_root+12}+{e.y_root+6}")
        tk.Label(tip, text=text, justify="left",
                 bg="#2A2D40", fg=P["fg"], relief="flat",
                 padx=8, pady=4, font=("Segoe UI", 9),
                 wraplength=340).pack()
    def hide(_):
        nonlocal tip
        if tip:
            tip.destroy(); tip = None
    widget.bind("<Enter>", show)
    widget.bind("<Leave>", hide)

# ---------------------------------------------------------------------------
# Modèle de donnée
# ---------------------------------------------------------------------------
@dataclass
class ParamDef:
    id:          int
    label:       str
    description: str
    ptype:       str
    binpt:       int
    default:     float
    unit:        str

    @property
    def scale(self) -> float:
        return float(1 << self.binpt)

    @property
    def phys_min(self) -> float:
        return 0.0 if self.ptype == "uint16" else INT16_MIN / self.scale

    @property
    def phys_max(self) -> float:
        return 65535.0 / self.scale if self.ptype == "uint16" else INT16_MAX / self.scale

    def to_int16(self, value: float) -> int:
        return max(INT16_MIN, min(INT16_MAX, round(value * self.scale)))

    def from_int16(self, raw: int) -> float:
        return raw / self.scale

    def quantized(self, value: float) -> float:
        return self.from_int16(self.to_int16(value))


@dataclass
class Category:
    id:          str
    label:       str
    description: str
    params:      List[ParamDef] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Chargement JSON
# ---------------------------------------------------------------------------
def load_parameters(path: Path) -> List[Category]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    categories = []
    for cat_data in data.get("categories", []):
        params = [
            ParamDef(
                id          = int(p["id"]),
                label       = p["label"],
                description = p.get("description", ""),
                ptype       = p.get("type", "float"),
                binpt       = int(p.get("binpt", 0)),
                default     = float(p.get("default", 0)),
                unit        = p.get("unit", ""),
            )
            for p in cat_data.get("parameters", [])
        ]
        categories.append(Category(
            id          = cat_data["id"],
            label       = cat_data["label"],
            description = cat_data.get("description", ""),
            params      = params,
        ))
    return categories


# ---------------------------------------------------------------------------
# Couche série partagée (paramétrage + commande)
# ---------------------------------------------------------------------------
class SerialLink:
    """
    Gère un seul port série partagé entre le module de paramétrage
    et le module de commande manette.
    Thread-safe via un lock.
    """
    def __init__(self):
        self._port: Optional["serial.Serial"] = None
        self._lock = threading.Lock()
        self.port_name: str = ""
        self.connected: bool = False

    # ── Connexion ────────────────────────────────────────────────────────
    def connect(self, port: str, baudrate: int = BAUDRATE_DEFAULT) -> bool:
        if not HAS_SERIAL:
            return False
        try:
            with self._lock:
                if self._port and self._port.is_open:
                    self._port.close()
                self._port = serial.Serial(port, baudrate, timeout=0.1)
                self.connected = self._port.is_open
                self.port_name = port if self.connected else ""
            return self.connected
        except Exception as e:
            print(f"[SERIAL] Erreur connexion : {e}")
            self.connected = False
            return False

    def disconnect(self):
        with self._lock:
            if self._port and self._port.is_open:
                self._port.close()
            self.connected = False
            self.port_name = ""

    # ── Envoi brut ───────────────────────────────────────────────────────
    def write(self, data: bytes) -> bool:
        with self._lock:
            if not self.connected or not self._port:
                return False
            try:
                self._port.write(data)
                return True
            except Exception as e:
                print(f"[SERIAL] Erreur écriture : {e}")
                self.connected = False
                return False

    # ── Trame paramètre ──────────────────────────────────────────────────
    def send_param(self, param: ParamDef, value: float) -> bool:
        raw     = param.to_int16(value)
        id_lsb  =  param.id        & 0xFF
        id_msb  = (param.id >> 8)  & 0xFF
        tx_type = MSG_TYPE_BASE + id_msb
        dat0    = id_lsb
        dat1    =  raw & 0xFF
        dat2    = (raw >> 8) & 0xFF
        payload = bytes([tx_type, dat0, dat1, dat2])
        frame   = bytes([SYNC_1, SYNC_2]) + payload + bytes([crc8(payload)])

        ok = self.write(frame)
        if not ok:
            print(f"[SIM-PARAM] id=0x{param.id:04X} ({param.label}) "
                  f"val={value:.6g} raw={raw} frame={frame.hex(' ').upper()}")
        return True   # simulation locale toujours "OK" pour l'UI

    # ── Trame commande vol ────────────────────────────────────────────────
    def send_cmd(self, tx_type: int, dat0: int, dat1: int, dat2: int) -> bool:
        payload = [tx_type, dat0, dat1, dat2]
        c = crc8(bytes(payload))
        frame = bytes([SYNC_1, SYNC_2, tx_type, dat0, dat1, dat2, c])
        ok = self.write(frame)
        if not ok:
            print(f"[SIM-CMD] {frame.hex(' ').upper()}")
        return ok


# ---------------------------------------------------------------------------
# Moteur de commande manette (thread dédié)
# ---------------------------------------------------------------------------
SAFE   = 0
MANUAL = 1
HOLD   = 2

class JoystickController:
    """
    Réplique exacte de la logique de commandes_sol_ps4.py,
    tournant dans un thread séparé pour ne pas bloquer l'UI.
    """
    DEADZONE_AXIS0 = 0.05
    DEADZONE_AXIS3 = 0.08
    LOOP_HZ        = 5        # 200 ms — watchdog avion + limite LoRa

    def __init__(self, link: SerialLink):
        self._link       = link
        self._thread: Optional[threading.Thread] = None
        self._running    = False

        # État observable par l'UI
        self.joystick_ok = False
        self.mode_name   = "SAFE"
        self.motor_val   = 0
        self.yaw_val     = 0.0
        self.pitch_val   = 0.0
        self.status_cb   = None   # callback(str) appelé à chaque tick

    # ── Démarrage / arrêt ────────────────────────────────────────────────
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    # ── Boucle principale ────────────────────────────────────────────────
    def _loop(self):
        if not HAS_PYGAME:
            print("[JOY] pygame non disponible")
            return

        pygame.init()
        pygame.joystick.init()

        joystick       = None
        flight_mode    = SAFE
        yaw            = 0.0
        pitch_offset   = 0.0
        motor_hold     = 0.0
        hold_time      = 0.0
        prev_triangle  = False
        prev_x         = False
        save_request   = 0

        while self._running:
            t0 = time.time()

            # ── Détection / reconnexion manette ───────────────────────────
            pygame.event.pump()
            n = pygame.joystick.get_count()
            if n == 0:
                joystick = None
                self.joystick_ok = False
            elif joystick is None:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                self.joystick_ok = True
                print(f"[JOY] Détecté : {joystick.get_name()}")

            if joystick is None:
                time.sleep(0.5)
                continue

            try:
                # ── Boutons mode ──────────────────────────────────────────
                triangle = joystick.get_button(3)
                x_btn    = joystick.get_button(0)

                if triangle and not prev_triangle:
                    flight_mode = MANUAL if flight_mode == SAFE else SAFE
                prev_triangle = triangle

                if x_btn and not prev_x and flight_mode == MANUAL:
                    motor_hold = (joystick.get_axis(5) + 1) / 2
                    hold_time  = time.time()
                    flight_mode = HOLD
                prev_x = x_btn

                # ── Gaz R2 ────────────────────────────────────────────────
                r2 = max(0.0, min(1.0, (joystick.get_axis(5) + 1) / 2))

                # ── Mode HOLD ─────────────────────────────────────────────
                if flight_mode == HOLD:
                    if joystick.get_button(10): motor_hold = min(1.0, motor_hold + 0.02)
                    if joystick.get_button(9):  motor_hold = max(0.0, motor_hold - 0.02)
                    if time.time() - hold_time < 2.0:
                        r2_eff = 0.0
                    else:
                        r2_eff = r2
                        if r2_eff > 0.1:
                            flight_mode = MANUAL
                else:
                    r2_eff = r2

                # ── Yaw ───────────────────────────────────────────────────
                yaw_in = joystick.get_axis(0)
                if abs(yaw_in) < self.DEADZONE_AXIS0:
                    yaw_in = 0.0
                if joystick.get_button(14): yaw += 0.01
                if joystick.get_button(13): yaw -= 0.01
                yaw_byte = max(0, min(255, int(255 * (yaw + yaw_in + 1) / 2)))

                # ── Pitch ─────────────────────────────────────────────────
                pitch_in = joystick.get_axis(3)
                if abs(pitch_in) < self.DEADZONE_AXIS3:
                    pitch_in = 0.0
                pitch = max(-1.0, min(1.0, pitch_in + pitch_offset))
                pitch_byte = int(((pitch + 1) / 2) * 255)
                if joystick.get_button(12): pitch_offset += 0.01
                if joystick.get_button(11): pitch_offset -= 0.01

                # ── Moteur ────────────────────────────────────────────────
                if   flight_mode == SAFE:   motor = 0
                elif flight_mode == MANUAL: motor = int((0.25 + 0.75 * r2_eff) * 255)
                elif flight_mode == HOLD:   motor = int((0.25 + 0.75 * motor_hold) * 255)
                else:                        motor = 0

                # ── Save ──────────────────────────────────────────────────
                square = joystick.get_button(2)
                circle = joystick.get_button(1)
                if square:  save_request = 1
                elif circle: save_request = 2

                # ── Envoi trame ───────────────────────────────────────────
                if save_request != 0:
                    self._link.send_cmd(TYPE_SAVE, save_request, yaw_byte, pitch_byte)
                    save_request = 0
                else:
                    self._link.send_cmd(TYPE_CMD, motor, yaw_byte, pitch_byte)

                # ── État observable ───────────────────────────────────────
                self.mode_name  = ["SAFE", "MANUAL", "HOLD"][flight_mode]
                self.motor_val  = motor
                self.yaw_val    = yaw + yaw_in
                self.pitch_val  = pitch

                if self.status_cb:
                    self.status_cb()

            except Exception as e:
                print(f"[JOY] Erreur boucle : {e}")

            # ── Cadence 20 Hz ─────────────────────────────────────────────
            elapsed = time.time() - t0
            sleep_t = max(0.0, 1.0 / self.LOOP_HZ - elapsed)
            time.sleep(sleep_t)

        pygame.quit()


# ---------------------------------------------------------------------------
# Popup Pilotage
# ---------------------------------------------------------------------------
class PilotageWindow(tk.Toplevel):

    POLL_MS = 100   # rafraîchissement UI

    def __init__(self, parent, link: SerialLink):
        super().__init__(parent)
        self.title("Pilotage — Commande manette")
        self.geometry("560x420")
        self.configure(bg=P["bg"])
        self.resizable(False, False)

        self._link  = link
        self._ctrl  = JoystickController(link)
        self._ctrl.status_cb = self._schedule_refresh

        self._build_ui()
        self._poll()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Construction UI ──────────────────────────────────────────────────
    def _build_ui(self):
        # Titre
        tk.Label(self, text="🎮  Pilotage",
                 bg=P["bg"], fg=P["fg"],
                 font=P["title"]).pack(anchor="w", padx=16, pady=(12, 6))

        # ── Carte statut connexions ──────────────────────────────────────
        card_conn = tk.Frame(self, bg=P["surface"])
        card_conn.pack(fill="x", padx=12, pady=(0, 8))

        tk.Label(card_conn, text="État des connexions",
                 bg=P["surface"], fg=P["fg_dim"],
                 font=("Segoe UI", 8)).pack(anchor="w", padx=10, pady=(6, 2))

        row_conn = tk.Frame(card_conn, bg=P["surface"])
        row_conn.pack(fill="x", padx=10, pady=(0, 8))

        # Antenne
        ant_frame = tk.Frame(row_conn, bg=P["surface2"],
                             padx=12, pady=8)
        ant_frame.pack(side="left", expand=True, fill="x",
                       padx=(0, 6), pady=0)
        tk.Label(ant_frame, text="Antenne série",
                 bg=P["surface2"], fg=P["fg_label"],
                 font=P["ui"]).pack(anchor="w")
        self._dot_ant = tk.Label(ant_frame, text="●",
                                  bg=P["surface2"], fg=P["err"],
                                  font=("Segoe UI", 18))
        self._dot_ant.pack(side="left")
        self._lbl_ant = tk.Label(ant_frame, text="Non connectée",
                                  bg=P["surface2"], fg=P["err"],
                                  font=P["ui"])
        self._lbl_ant.pack(side="left", padx=6)

        # Manette
        joy_frame = tk.Frame(row_conn, bg=P["surface2"],
                              padx=12, pady=8)
        joy_frame.pack(side="left", expand=True, fill="x",
                       padx=(6, 0))
        tk.Label(joy_frame, text="Manette",
                 bg=P["surface2"], fg=P["fg_label"],
                 font=P["ui"]).pack(anchor="w")
        self._dot_joy = tk.Label(joy_frame, text="●",
                                  bg=P["surface2"], fg=P["err"],
                                  font=("Segoe UI", 18))
        self._dot_joy.pack(side="left")
        self._lbl_joy = tk.Label(joy_frame, text="Non connectée",
                                  bg=P["surface2"], fg=P["err"],
                                  font=P["ui"])
        self._lbl_joy.pack(side="left", padx=6)

        # ── Boutons contrôle ─────────────────────────────────────────────
        btn_row = tk.Frame(self, bg=P["bg"])
        btn_row.pack(fill="x", padx=12, pady=(0, 8))

        self._btn_start = tk.Button(
            btn_row, text="▶  Démarrer commande",
            bg=P["ok"], fg="white",
            activebackground="#3DA060", activeforeground="white",
            font=P["head"], relief="flat", bd=0,
            padx=14, pady=6, cursor="hand2",
            command=self._toggle_control,
        )
        self._btn_start.pack(side="left", padx=(0, 8))

        # Bouton connexion auto antenne
        tk.Button(
            btn_row, text="🔌  Connecter antenne (auto)",
            bg=P["accent_dim"], fg=P["fg"],
            activebackground=P["accent"], activeforeground="white",
            font=P["ui"], relief="flat", bd=0,
            padx=10, pady=6, cursor="hand2",
            command=self._auto_connect,
        ).pack(side="left")

        # ── Carte télémétrie ─────────────────────────────────────────────
        tk.Frame(self, bg=P["border"], height=1).pack(fill="x", padx=12)
        card_telem = tk.Frame(self, bg=P["surface"])
        card_telem.pack(fill="x", padx=12, pady=8)

        tk.Label(card_telem, text="Données temps réel",
                 bg=P["surface"], fg=P["fg_dim"],
                 font=("Segoe UI", 8)).pack(anchor="w", padx=10, pady=(6, 2))

        telem_grid = tk.Frame(card_telem, bg=P["surface"])
        telem_grid.pack(fill="x", padx=10, pady=(0, 8))

        self._telem_vars: dict[str, tk.StringVar] = {}
        fields = [
            ("Mode",   "SAFE"),
            ("Moteur", "0"),
            ("Yaw",    "0.00 rad"),
            ("Pitch",  "0.00 rad"),
        ]
        for col, (lbl, init) in enumerate(fields):
            cell = tk.Frame(telem_grid, bg=P["surface2"], padx=10, pady=6)
            cell.grid(row=0, column=col, padx=4, sticky="nsew")
            telem_grid.columnconfigure(col, weight=1)
            tk.Label(cell, text=lbl,
                     bg=P["surface2"], fg=P["fg_dim"],
                     font=("Segoe UI", 8)).pack(anchor="w")
            sv = tk.StringVar(value=init)
            self._telem_vars[lbl] = sv
            tk.Label(cell, textvariable=sv,
                     bg=P["surface2"], fg=P["fg"],
                     font=P["big"]).pack(anchor="w")

        # ── Carte aide ───────────────────────────────────────────────────
        tk.Frame(self, bg=P["border"], height=1).pack(fill="x", padx=12)
        aide = tk.Frame(self, bg=P["bg"])
        aide.pack(fill="x", padx=12, pady=8)
        aide_txt = (
            "△ = SAFE ↔ MANUAL    ✕ = activer HOLD    □ = log ON    ○ = log SAVE\n"
            "R2 = Gaz    LS gauche/droite = Yaw    RS haut/bas = Pitch\n"
            "←/→ D-pad = Yaw trim    ↑/↓ D-pad = Pitch trim    L3/R3 = Hold ±"
        )
        tk.Label(aide, text=aide_txt,
                 bg=P["bg"], fg=P["fg_dim"],
                 font=("Segoe UI", 8), justify="left").pack(anchor="w")

        self._running = False

    # ── Connexion automatique antenne ────────────────────────────────────
    def _auto_connect(self):
        port = find_first_serial_port()
        if port is None:
            messagebox.showwarning(
                "Aucun port détecté",
                "Aucun port série disponible.\n"
                + ("(pyserial non installé)" if not HAS_SERIAL else
                   "Vérifiez le branchement de l'antenne."))
            return
        ok = self._link.connect(port)
        if ok:
            messagebox.showinfo("Connecté", f"Antenne connectée sur {port}")
        else:
            messagebox.showerror("Échec", f"Impossible d'ouvrir {port}")

    # ── Démarrage / arrêt commande ───────────────────────────────────────
    def _toggle_control(self):
        if not self._running:
            if not HAS_PYGAME:
                messagebox.showerror(
                    "pygame manquant",
                    "Installez pygame pour activer la commande :\n"
                    "  pip install pygame")
                return
            self._running = True
            self._ctrl.start()
            self._btn_start.config(text="■  Arrêter commande",
                                   bg=P["err"],
                                   activebackground="#C04040")
        else:
            self._running = False
            self._ctrl.stop()
            self._btn_start.config(text="▶  Démarrer commande",
                                   bg=P["ok"],
                                   activebackground="#3DA060")

    # ── Rafraîchissement UI ──────────────────────────────────────────────
    def _schedule_refresh(self):
        """Appelé depuis le thread joystick — replanifie dans le thread Tk."""
        try:
            self.after(0, self._refresh_ui)
        except Exception:
            pass

    def _refresh_ui(self):
        # Antenne
        ant_ok = self._link.connected
        self._dot_ant.config(fg=P["ok"] if ant_ok else P["err"])
        self._lbl_ant.config(
            text=f"Connectée ({self._link.port_name})" if ant_ok else "Non connectée",
            fg=P["ok"] if ant_ok else P["err"])

        # Manette
        joy_ok = self._ctrl.joystick_ok
        self._dot_joy.config(fg=P["ok"] if joy_ok else P["err"])
        self._lbl_joy.config(
            text="Connectée" if joy_ok else "Non connectée",
            fg=P["ok"] if joy_ok else P["err"])

        # Télémétrie
        if self._running:
            mode_colors = {"SAFE": P["fg_dim"], "MANUAL": P["ok"], "HOLD": P["warn"]}
            self._telem_vars["Mode"].set(self._ctrl.mode_name)
            self._telem_vars["Moteur"].set(str(self._ctrl.motor_val))
            self._telem_vars["Yaw"].set(f"{self._ctrl.yaw_val:.2f} rad")
            self._telem_vars["Pitch"].set(f"{self._ctrl.pitch_val:.2f} rad")

    def _poll(self):
        """Rafraîchissement périodique même sans trame joystick (statut connexion)."""
        self._refresh_ui()
        if not self.winfo_exists():
            return
        self.after(self.POLL_MS, self._poll)

    def _on_close(self):
        self._ctrl.stop()
        self.destroy()


# ---------------------------------------------------------------------------
# Widget : ligne de paramètre
# ---------------------------------------------------------------------------
class ParamRow(tk.Frame):

    def __init__(self, parent, param: ParamDef, link: SerialLink,
                 bg: str, **kwargs):
        super().__init__(parent, bg=bg, **kwargs)
        self.param    = param
        self._link    = link
        self._pending = False

        self._var = tk.StringVar(value=self._fmt(param.default))
        self._var.trace_add("write", self._on_edit)

        x = 0

        lbl = tk.Label(self, text=param.label,
                       bg=bg, fg=P["fg_label"],
                       font=P["ui"], anchor="w", width=1)
        lbl.place(x=x, y=0, width=COL_WIDTHS["label"], height=28)
        make_tooltip(lbl, f"[id={param.id}]  {param.description}")
        x += COL_WIDTHS["label"]

        ef = tk.Frame(self, bg=P["border"])
        ef.place(x=x+2, y=4, width=COL_WIDTHS["entry"]-4, height=20)
        self._entry = tk.Entry(ef, textvariable=self._var,
                               font=P["mono"],
                               bg=P["surface"], fg=P["fg"],
                               insertbackground=P["fg"],
                               relief="flat", bd=0)
        self._entry.place(x=1, y=1, width=COL_WIDTHS["entry"]-6, height=18)
        self._entry.bind("<Return>",   self._on_send)
        self._entry.bind("<KP_Enter>", self._on_send)
        self._entry.bind("<FocusOut>", self._on_focusout)
        x += COL_WIDTHS["entry"]

        self._lbl_q = tk.Label(self, text="",
                               bg=bg, fg=P["fg_dim"],
                               font=P["mono"], anchor="w")
        self._lbl_q.place(x=x+4, y=0, width=COL_WIDTHS["quant"]-4, height=28)
        x += COL_WIDTHS["quant"]

        tk.Label(self, text=param.unit,
                 bg=bg, fg=P["fg_dim"],
                 font=P["ui"], anchor="w"
                 ).place(x=x+4, y=0, width=COL_WIDTHS["unit"]-4, height=28)
        x += COL_WIDTHS["unit"]

        self._status = tk.Label(self, text="●", bg=bg, fg=P["fg_dim"],
                                font=("Segoe UI", 11))
        self._status.place(x=x, y=0, width=COL_WIDTHS["status"], height=28)
        x += COL_WIDTHS["status"]

        tk.Button(self, text="Envoyer",
                  bg=P["accent_dim"], fg=P["fg"],
                  activebackground=P["accent"], activeforeground="white",
                  font=P["ui"], relief="flat", bd=0,
                  cursor="hand2", command=self._on_send
                  ).place(x=x+2, y=3, width=COL_WIDTHS["send"]-4, height=22)
        x += COL_WIDTHS["send"]

        tk.Button(self, text="↺",
                  bg=bg, fg=P["fg_dim"],
                  activebackground=P["surface2"], activeforeground=P["fg"],
                  font=P["ui"], relief="flat", bd=0,
                  cursor="hand2", command=self._reset
                  ).place(x=x+2, y=3, width=COL_WIDTHS["reset"]-2, height=22)

        self.config(height=28)
        self.pack_propagate(False)
        self._refresh_quant()

    @staticmethod
    def _fmt(v: float) -> str:
        return f"{v:.6g}"

    def _refresh_quant(self):
        try:
            v = float(self._var.get())
            q = self.param.quantized(v)
            r = self.param.to_int16(v)
            self._lbl_q.config(text=f"≈ {q:.6g}  [{r}]", fg=P["fg_dim"])
        except ValueError:
            self._lbl_q.config(text="— invalide —", fg=P["err"])

    def _on_edit(self, *_):
        self._pending = True
        self._status.config(fg=P["warn"])
        self._refresh_quant()

    def _on_focusout(self, *_):
        try:
            v = max(self.param.phys_min,
                    min(self.param.phys_max, float(self._var.get())))
            self._var.set(self._fmt(v))
        except ValueError:
            pass

    def _on_send(self, *_):
        try:
            v = float(self._var.get())
        except ValueError:
            self._status.config(fg=P["err"])
            messagebox.showerror("Valeur invalide",
                                 f"'{self._var.get()}' n'est pas un nombre.")
            return
        if v < self.param.phys_min or v > self.param.phys_max:
            messagebox.showwarning(
                "Hors plage",
                f"Plage : [{self.param.phys_min:.4g} ; {self.param.phys_max:.4g}]\n"
                "La valeur sera ramenée au bord.")
            v = max(self.param.phys_min, min(self.param.phys_max, v))
            self._var.set(self._fmt(v))
        ok = self._link.send_param(self.param, v)
        self._status.config(fg=P["ok"] if ok else P["err"])
        if ok:
            self._pending = False

    def _reset(self):
        self._var.set(self._fmt(self.param.default))
        self._pending = False
        self._status.config(fg=P["fg_dim"])
        self._refresh_quant()

    @property
    def has_pending(self) -> bool:
        return self._pending


# ---------------------------------------------------------------------------
# En-tête de colonnes (largeurs fixes, partagé)
# ---------------------------------------------------------------------------
class ColumnHeader(tk.Frame):
    COLS = [
        ("Paramètre",        "label"),
        ("Valeur physique",  "entry"),
        ("Quantifiée [raw]", "quant"),
        ("Unité",            "unit"),
        ("",                 "status"),
        ("",                 "send"),
        ("",                 "reset"),
    ]
    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=P["surface"], **kwargs)
        x = 0
        for txt, key in self.COLS:
            tk.Label(self, text=txt,
                     bg=P["surface"], fg=P["fg_dim"],
                     font=("Segoe UI", 8), anchor="w"
                     ).place(x=x+4, y=0, width=COL_WIDTHS[key]-4, height=20)
            x += COL_WIDTHS[key]
        self.config(height=20)
        self.pack_propagate(False)


# ---------------------------------------------------------------------------
# Panneau catégorie repliable
# ---------------------------------------------------------------------------
class CategoryPanel(tk.Frame):

    def __init__(self, parent, cat: Category, link: SerialLink,
                 collapsed: bool = True, **kwargs):
        super().__init__(parent, bg=P["surface"], **kwargs)
        self.rows: List[ParamRow] = []
        self._collapsed = collapsed

        hdr = tk.Frame(self, bg=P["accent_dim"], cursor="hand2")
        hdr.pack(fill="x")

        self._arrow = tk.Label(hdr, text="▶" if collapsed else "▼",
                               bg=P["accent_dim"], fg="white",
                               font=("Segoe UI", 10), padx=6, pady=6)
        self._arrow.pack(side="left")

        tk.Label(hdr, text=cat.label,
                 bg=P["accent_dim"], fg="white",
                 font=P["cat"], anchor="w", padx=4, pady=6
                 ).pack(side="left")

        if cat.description:
            tk.Label(hdr, text=cat.description,
                     bg=P["accent_dim"], fg="#A8C4FF",
                     font=("Segoe UI", 9), padx=6, pady=6
                     ).pack(side="left")

        tk.Label(hdr, text=f"{len(cat.params)} paramètres",
                 bg=P["accent_dim"], fg="#6A88C0",
                 font=("Segoe UI", 9), padx=12
                 ).pack(side="right")

        for w in hdr.winfo_children() + [hdr]:
            w.bind("<Button-1>", self._toggle)

        self._body = tk.Frame(self, bg=P["surface"])
        tk.Frame(self._body, bg=P["border"], height=1).pack(fill="x")
        for idx, p in enumerate(cat.params):
            bg = P["surface"] if idx % 2 == 0 else P["surface2"]
            row = ParamRow(self._body, p, link, bg=bg)
            row.pack(fill="x")
            self.rows.append(row)
        tk.Frame(self._body, bg=P["border"], height=1).pack(fill="x")

        if not collapsed:
            self._body.pack(fill="x")

    def _toggle(self, _=None):
        self._collapsed = not self._collapsed
        if self._collapsed:
            self._body.pack_forget()
            self._arrow.config(text="▶")
        else:
            self._body.pack(fill="x")
            self._arrow.config(text="▼")

    def expand(self):
        if self._collapsed: self._toggle()

    def collapse(self):
        if not self._collapsed: self._toggle()


# ---------------------------------------------------------------------------
# Fenêtre principale
# ---------------------------------------------------------------------------
class ParamTunerApp(tk.Tk):

    def __init__(self, json_path: Path):
        super().__init__()
        self.title("Param Tuner — Interface de paramétrage embarqué")
        self.geometry("1100x720")
        self.minsize(TOTAL_WIDTH + 40, 500)
        self.configure(bg=P["bg"])

        self._link      = SerialLink()
        self.categories = load_parameters(json_path)
        self.all_rows:  List[ParamRow]      = []
        self.panels:    List[CategoryPanel] = []
        self._pilot_win: Optional[PilotageWindow] = None

        self._build_ui()

    # ── Construction UI ──────────────────────────────────────────────────
    def _build_ui(self):
        # Titre
        tb = tk.Frame(self, bg=P["bg"])
        tb.pack(fill="x", padx=16, pady=(12, 4))

        tk.Label(tb, text="⚙  Param Tuner",
                 bg=P["bg"], fg=P["fg"], font=P["title"]).pack(side="left")

        self._lbl_conn = tk.Label(tb, text="● Hors-ligne (simulation)",
                                   bg=P["bg"], fg=P["warn"],
                                   font=("Segoe UI", 10))
        self._lbl_conn.pack(side="right", padx=8)

        tk.Button(tb, text="Connecter…",
                  bg=P["accent_dim"], fg=P["fg"],
                  activebackground=P["accent"], activeforeground="white",
                  font=P["ui"], relief="flat", bd=0,
                  padx=10, pady=4, cursor="hand2",
                  command=self._connect_dialog
                  ).pack(side="right", padx=4)

        # Barre d'actions
        ab = tk.Frame(self, bg=P["surface"], pady=4)
        ab.pack(fill="x")

        for txt, cmd, fg, bg in [
            ("Envoyer tout",       self._send_all,     "white",       P["accent"]),
            ("Envoyer modifiés",   self._send_pending, P["fg"],       P["accent_dim"]),
            ("Réinitialiser tout", self._reset_all,    P["fg_dim"],   P["surface2"]),
            ("Tout déplier",       self._expand_all,   P["fg_dim"],   P["surface2"]),
            ("Tout replier",       self._collapse_all, P["fg_dim"],   P["surface2"]),
        ]:
            tk.Button(ab, text=txt, command=cmd,
                      bg=bg, fg=fg,
                      activebackground=P["accent"], activeforeground="white",
                      font=P["ui"], relief="flat", bd=0,
                      padx=10, pady=5, cursor="hand2"
                      ).pack(side="left", padx=(8, 0), pady=4)

        # Bouton Pilotage (droit)
        tk.Button(ab, text="🎮  Pilotage",
                  bg="#3A2A6A", fg="#C8B4FF",
                  activebackground="#5A3A9A", activeforeground="white",
                  font=P["head"], relief="flat", bd=0,
                  padx=12, pady=5, cursor="hand2",
                  command=self._open_pilotage
                  ).pack(side="right", padx=8, pady=4)

        tk.Button(ab, text="Exporter JSON…",
                  command=self._export_json,
                  bg=P["surface2"], fg=P["fg_dim"],
                  activebackground=P["surface"], activeforeground=P["fg"],
                  font=P["ui"], relief="flat", bd=0,
                  padx=10, pady=5, cursor="hand2"
                  ).pack(side="right", padx=4, pady=4)

        # En-tête colonnes fixe
        chf = tk.Frame(self, bg=P["surface"])
        chf.pack(fill="x", padx=8)
        ColumnHeader(chf).pack(side="left")
        tk.Frame(self, bg=P["border"], height=1).pack(fill="x", padx=8)

        # Zone défilement
        sw = tk.Frame(self, bg=P["bg"])
        sw.pack(fill="both", expand=True, padx=8, pady=(2, 0))

        canvas = tk.Canvas(sw, bg=P["bg"], highlightthickness=0, bd=0)
        sb = ttk.Scrollbar(sw, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        inner = tk.Frame(canvas, bg=P["bg"])
        iid = canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfig(iid, width=e.width))
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

        def _scroll(e):
            d = -1*(e.delta//120) if e.delta else (1 if e.num == 5 else -1)
            canvas.yview_scroll(d, "units")
        canvas.bind_all("<MouseWheel>", _scroll)
        canvas.bind_all("<Button-4>",   _scroll)
        canvas.bind_all("<Button-5>",   _scroll)

        # Panneaux catégories
        for cat in self.categories:
            panel = CategoryPanel(inner, cat, self._link, collapsed=True)
            panel.pack(fill="x", pady=(4, 0))
            self.panels.append(panel)
            self.all_rows.extend(panel.rows)

        # Barre de statut
        stb = tk.Frame(self, bg=P["surface"], height=24)
        stb.pack(fill="x", side="bottom")
        self._lbl_status = tk.Label(
            stb,
            text=f"{len(self.all_rows)} paramètres — {len(self.categories)} catégories",
            bg=P["surface"], fg=P["fg_dim"],
            font=("Segoe UI", 9), anchor="w", padx=10)
        self._lbl_status.pack(side="left")

    # ── Dialogue connexion paramétrage ───────────────────────────────────
    def _connect_dialog(self):
        dlg = tk.Toplevel(self)
        dlg.title("Connexion série")
        dlg.geometry("380x200")
        dlg.configure(bg=P["bg"])
        dlg.transient(self); dlg.grab_set()

        # Détection auto
        auto_port = find_first_serial_port() or ""

        for r, (lbl, key, val) in enumerate([
            ("Port série", "port", auto_port or "COM3"),
            ("Baudrate",   "baud", str(BAUDRATE_DEFAULT)),
        ]):
            tk.Label(dlg, text=lbl, bg=P["bg"], fg=P["fg"],
                     font=P["ui"]).grid(row=r, column=0, padx=12, pady=10, sticky="w")
            sv = tk.StringVar(value=val)
            setattr(dlg, f"_{key}", sv)
            tk.Entry(dlg, textvariable=sv,
                     bg=P["surface"], fg=P["fg"],
                     insertbackground=P["fg"],
                     relief="flat", font=P["mono"], width=18
                     ).grid(row=r, column=1, padx=8)

        if auto_port:
            tk.Label(dlg, text=f"ℹ  Port détecté automatiquement : {auto_port}",
                     bg=P["bg"], fg=P["ok"],
                     font=("Segoe UI", 8)
                     ).grid(row=2, column=0, columnspan=2, padx=12, pady=0)

        def _do():
            ok = self._link.connect(dlg._port.get(), int(dlg._baud.get()))
            self._lbl_conn.config(
                text=f"● Connecté ({dlg._port.get()})" if ok
                     else "● Stub actif (simulation)",
                fg=P["ok"] if ok else P["warn"])
            if not ok and not HAS_SERIAL:
                messagebox.showinfo("pyserial manquant",
                                    "pip install pyserial\n"
                                    "Les envois s'affichent en console.")
            dlg.destroy()

        tk.Button(dlg, text="Connecter", command=_do,
                  bg=P["accent"], fg="white",
                  font=P["ui"], relief="flat", bd=0,
                  padx=14, pady=5, cursor="hand2"
                  ).grid(row=3, column=0, columnspan=2, pady=14)

    # ── Popup pilotage ───────────────────────────────────────────────────
    def _open_pilotage(self):
        if self._pilot_win and self._pilot_win.winfo_exists():
            self._pilot_win.lift()
            return
        self._pilot_win = PilotageWindow(self, self._link)

    # ── Envoi cadencé à 5 Hz (200 ms entre chaque trame) ───────────────
    PARAM_SEND_INTERVAL = 0.200   # s — limite LoRa SF9 BW125

    def _send_batch(self, rows: list, label: str):
        """Envoie une liste de ParamRow à 5 Hz dans un thread dédié."""
        if not rows:
            self._lbl_status.config(text="Aucun paramètre à envoyer",
                                     fg=P["fg_dim"])
            return

        def _worker():
            for i, r in enumerate(rows):
                r._on_send()
                self.after(0, lambda t=f"{label} … {i+1}/{len(rows)}": 
                           self._lbl_status.config(text=t, fg=P["warn"]))
                if i < len(rows) - 1:
                    time.sleep(self.PARAM_SEND_INTERVAL)
            self.after(0, lambda: self._lbl_status.config(
                text=f"{label} — {len(rows)} paramètre(s) envoyé(s)",
                fg=P["ok"]))

        threading.Thread(target=_worker, daemon=True).start()

    # ── Actions globales ─────────────────────────────────────────────────
    def _send_all(self):
        self._send_batch(self.all_rows, "Envoi total")

    def _send_pending(self):
        pending = [r for r in self.all_rows if r.has_pending]
        self._send_batch(pending, "Envoi modifiés")

    def _reset_all(self):
        if not messagebox.askyesno("Réinitialiser",
                                    "Remettre tous les paramètres à leurs valeurs par défaut ?"):
            return
        for r in self.all_rows: r._reset()
        self._lbl_status.config(text="Tous réinitialisés", fg=P["fg_dim"])

    def _expand_all(self):
        for p in self.panels: p.expand()

    def _collapse_all(self):
        for p in self.panels: p.collapse()

    def _export_json(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON", "*.json"), ("Tous", "*.*")],
            title="Exporter les valeurs courantes")
        if not path: return
        out = {"parameters": {}}
        for row in self.all_rows:
            try:   v = float(row._var.get())
            except ValueError: v = row.param.default
            out["parameters"][str(row.param.id)] = {
                "label":   row.param.label,
                "value":   v,
                "raw_i16": row.param.to_int16(v),
            }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
        self._lbl_status.config(text=f"Export : {path}", fg=P["ok"])


# ---------------------------------------------------------------------------
# Point d'entrée
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    json_path = (Path(sys.argv[1]) if len(sys.argv) > 1
                 else Path(__file__).parent / ".." / "parameters" / "parameters.json")
    if not json_path.exists():
        print(f"[ERREUR] Fichier introuvable : {json_path.resolve()}")
        sys.exit(1)
    ParamTunerApp(json_path).mainloop()