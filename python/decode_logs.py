#!/usr/bin/env python3
"""Décodage et affichage des logs navychop (format binaire logging.h, version 1 ;
l'ancien format logState de 48 octets est aussi reconnu automatiquement).

Spyder : renseigner FICHIER_BIN puis lancer le script (runfile). Aucune option requise.
CLI    : python decode_logs.py logs.bin [--no-plot] [--no-csv]

Produit :
  - des DataFrames pandas : fast, acc, mag, gps (temps t en secondes depuis le début de la boucle)
  - des CSV à côté du .bin : <nom>_fast.csv, _acc.csv, _mag.csv, _gps.csv et <nom>_merged.csv
    (une ligne par boucle rapide, acc/mag/gps = dernière valeur connue : proche de l'ancien log.csv)
  - les graphiques.
"""
import struct
import sys
from pathlib import Path

import numpy as np
import pandas as pd

FICHIER_BIN = "logs.bin"        # utilisé quand le script est lancé sans argument

# --- format (doit rester identique à logging.h) ---
REC_FAST, REC_ACC, REC_MAG, REC_GPS = 0xA1, 0xA2, 0xA3, 0xA4
FMT = {
    REC_FAST: "<BI4hh3h3h3B3bB",   # type, t_ms, q[4], altitude, eta[3], proj[3], cmd[3], ctrl[3], esc
    REC_ACC:  "<BH3h",             # type, t16, a[3]
    REC_MAG:  "<BH3h",             # type, t16, m[3]
    REC_GPS:  "<BHiihBB",          # type, t16, lat_e7, lon_e7, alt_dm, hdop_x10, nsat
}
SIZE = {k: struct.calcsize(v) for k, v in FMT.items()}
assert SIZE == {REC_FAST: 34, REC_ACC: 9, REC_MAG: 9, REC_GPS: 15}

Q4, Q6, Q14 = 16.0, 64.0, 16384.0


# --- ancien format (logState de communication.h, 48 octets, un enregistrement par boucle) ---
LEGACY_FMT = "<I3h3h4hh3h3h3b3bB3x"   # t_ms, a[3], mag[3], q[4], alt, eta[3], proj[3], cmd[3], ctrl[3], esc, pad
LEGACY_SIZE = struct.calcsize(LEGACY_FMT)
assert LEGACY_SIZE == 48


def parse_legacy(data: bytes):
    n = len(data) // LEGACY_SIZE
    rows = np.array([struct.unpack_from(LEGACY_FMT, data, i * LEGACY_SIZE) for i in range(n)], dtype=np.int64)
    fast = pd.DataFrame({
        "t": rows[:, 0] / 1000.0,
        "qw": rows[:, 7] / Q14, "qx": rows[:, 8] / Q14, "qy": rows[:, 9] / Q14, "qz": rows[:, 10] / Q14,
        "altitude": rows[:, 11] / Q6,
        "eta_yaw": rows[:, 12] / Q6, "eta_pitch": rows[:, 13] / Q6, "eta_roll": rows[:, 14] / Q6,
        "proj_yaw": rows[:, 15] / Q14, "proj_pitch": rows[:, 16] / Q14, "proj_roll": rows[:, 17] / Q14,
        "cmd_yaw": rows[:, 18], "cmd_pitch": rows[:, 19], "cmd_roll": rows[:, 20],
        "ctrl_yaw": rows[:, 21], "ctrl_pitch": rows[:, 22], "ctrl_roll": rows[:, 23],
        "esc": rows[:, 24],
    })
    acc = pd.DataFrame({"t": rows[:, 0] / 1000.0, "a_x": rows[:, 1] / Q4, "a_y": rows[:, 2] / Q4, "a_z": rows[:, 3] / Q4})
    mag = pd.DataFrame({"t": rows[:, 0] / 1000.0, "mag_x": rows[:, 4] / Q4, "mag_y": rows[:, 5] / Q4, "mag_z": rows[:, 6] / Q4})
    # l'ancien format répétait les valeurs lentes à chaque boucle : on ne garde que les changements
    for df, cols in ((acc, ["a_x", "a_y", "a_z"]), (mag, ["mag_x", "mag_y", "mag_z"])):
        keep = (df[cols].diff().abs().sum(axis=1) != 0)
        keep.iloc[0] = True
        df.drop(df.index[~keep], inplace=True)
        df.reset_index(drop=True, inplace=True)
    gps = pd.DataFrame(columns=["t", "lat", "lon", "gps_alt", "hdop", "nsat"])
    return fast, acc, mag, gps, n


def _t_from_t16(t16: int, t_ref_ms: int) -> int:
    """Déroule un temps 16 bits au plus proche du dernier temps rapide connu."""
    delta = ((t16 - (t_ref_ms & 0xFFFF) + 32768) & 0xFFFF) - 32768
    return t_ref_ms + delta


def parse(data: bytes):
    """Retourne (dict de listes de lignes, infos). Tolère une fin de fichier tronquée."""
    if data[:4] != b"NVCH":
        raise ValueError(
            f"en-tete invalide (magic NVCH absent) : fichier de {len(data)} octets, "
            f"16 premiers octets = {data[:16].hex(' ')}"
        )
    version, d_t = data[4], data[5]
    if version != 1:
        raise ValueError(f"version de format {version} non gérée")

    rows = {k: [] for k in FMT}
    pos, n = 8, len(data)
    t_ref = 0
    n_resync = 0
    while pos < n:
        t = data[pos]
        if t not in FMT:                       # octet inattendu : on cherche le prochain type valide
            n_resync += 1
            pos += 1
            continue
        if pos + SIZE[t] > n:                  # enregistrement final tronqué (coupure, crash)
            break
        f = struct.unpack_from(FMT[t], data, pos)
        pos += SIZE[t]
        if t == REC_FAST:
            t_ref = f[1]
            rows[t].append(f)
        else:
            rows[t].append((f[0], _t_from_t16(f[1], t_ref)) + f[2:])
    info = {"version": version, "d_t_ms": d_t, "octets": n, "resync": n_resync,
            "octets_ignores_fin": n - pos}
    return rows, info


def to_frames(rows):
    f = np.array(rows[REC_FAST], dtype=np.int64).reshape(-1, 20)
    fast = pd.DataFrame({
        "t": f[:, 1] / 1000.0,
        "qw": f[:, 2] / Q14, "qx": f[:, 3] / Q14, "qy": f[:, 4] / Q14, "qz": f[:, 5] / Q14,
        "altitude": f[:, 6] / Q6,
        "eta_yaw": f[:, 7] / Q6, "eta_pitch": f[:, 8] / Q6, "eta_roll": f[:, 9] / Q6,
        "proj_yaw": f[:, 10] / Q14, "proj_pitch": f[:, 11] / Q14, "proj_roll": f[:, 12] / Q14,
        "cmd_yaw": f[:, 13], "cmd_pitch": f[:, 14], "cmd_roll": f[:, 15],
        "ctrl_yaw": f[:, 16], "ctrl_pitch": f[:, 17], "ctrl_roll": f[:, 18],
        "esc": f[:, 19],
    })

    a = np.array(rows[REC_ACC], dtype=np.int64).reshape(-1, 5)
    acc = pd.DataFrame({"t": a[:, 1] / 1000.0, "a_x": a[:, 2] / Q4, "a_y": a[:, 3] / Q4, "a_z": a[:, 4] / Q4})

    m = np.array(rows[REC_MAG], dtype=np.int64).reshape(-1, 5)
    mag = pd.DataFrame({"t": m[:, 1] / 1000.0, "mag_x": m[:, 2] / Q4, "mag_y": m[:, 3] / Q4, "mag_z": m[:, 4] / Q4})

    g = np.array(rows[REC_GPS], dtype=np.int64).reshape(-1, 7)
    gps = pd.DataFrame({
        "t": g[:, 1] / 1000.0,
        "lat": g[:, 2] / 1e7, "lon": g[:, 3] / 1e7,
        "gps_alt": g[:, 4] / 10.0,
        "hdop": np.where(g[:, 5] == 255, np.nan, g[:, 5] / 10.0),   # 255 = pas de fix
        "nsat": g[:, 6],
    })
    for df in (acc, mag, gps):
        df.sort_values("t", inplace=True, ignore_index=True)       # le GPS est daté dans le passé
    return fast, acc, mag, gps


def merged(fast, acc, mag, gps):
    out = fast
    for df in (acc, mag, gps):
        if len(df):
            out = pd.merge_asof(out, df, on="t", direction="backward")
    return out


def euler_deg(q):
    """Angles d'Euler (roll, pitch, yaw) du quaternion GRV, mêmes formules que navigation.cpp.
    Repère IMU : à lire comme une aide visuelle ; l'asservissement utilise les projections."""
    w, x, y, z = q["qw"], q["qx"], q["qy"], q["qz"]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def plot(fast, acc, mag, gps):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(5, 2, figsize=(15, 14), sharex=False)
    ax = ax.ravel()

    r, p, y = euler_deg(fast)
    ax[0].plot(fast.t, r, label="roll"); ax[0].plot(fast.t, p, label="pitch"); ax[0].plot(fast.t, y, label="yaw")
    ax[0].set_title("Euler du quaternion GRV (°, repère IMU)"); ax[0].legend()

    ax[1].plot(fast.t, fast.proj_pitch, label="proj pitch"); ax[1].plot(fast.t, fast.proj_roll, label="proj roll")
    ax[1].plot(fast.t, fast.proj_yaw, label="proj yaw")
    ax[1].set_title("Erreurs de projection (entrées du PID)"); ax[1].legend()

    ax[2].plot(fast.t, fast.cmd_pitch, label="cmd pitch"); ax[2].plot(fast.t, fast.cmd_roll, label="cmd roll")
    ax[2].plot(fast.t, fast.cmd_yaw, label="cmd yaw")
    ax[2].set_title("Commandes sol (6 bits, neutre 31)"); ax[2].legend()

    ax[3].plot(fast.t, fast.eta_pitch, label="eta pitch"); ax[3].plot(fast.t, fast.eta_roll, label="eta roll")
    ax[3].plot(fast.t, fast.eta_yaw, label="eta yaw")
    ax[3].set_title("Intégrateurs"); ax[3].legend()

    ax[4].plot(fast.t, fast.ctrl_pitch, label="ctrl pitch"); ax[4].plot(fast.t, fast.ctrl_roll, label="ctrl roll")
    ax[4].plot(fast.t, fast.ctrl_yaw, label="ctrl yaw")
    ax[4].set_title("Contrôles gouvernes"); ax[4].legend()

    ax[5].plot(fast.t, fast.esc, color="k")
    ax[5].set_title("ESC (0-63)")

    if len(acc):
        for c in ("a_x", "a_y", "a_z"):
            ax[6].plot(acc.t, acc[c], label=c)
        ax[6].legend()
    ax[6].set_title("Accélération linéaire (m/s²)")

    if len(mag):
        for c in ("mag_x", "mag_y", "mag_z"):
            ax[7].plot(mag.t, mag[c], label=c)
        ax[7].legend()
    ax[7].set_title("Champ magnétique (µT)")

    ax[8].plot(fast.t, fast.altitude, label="baro")
    if len(gps):
        ax[8].plot(gps.t, gps.gps_alt - gps.gps_alt.iloc[0], ".", label="GPS (relatif au 1er point)")
    ax[8].set_title("Altitude (m)"); ax[8].legend()

    if len(gps) and gps.hdop.notna().any():
        g = gps.dropna(subset=["hdop"])
        lat0 = g.lat.iloc[0]
        east = (g.lon - g.lon.iloc[0]) * 111320.0 * np.cos(np.radians(lat0))
        north = (g.lat - lat0) * 110540.0
        sc = ax[9].scatter(east, north, c=g.t, s=8)
        ax[9].plot(east, north, lw=0.5, color="gray")
        ax[9].set_aspect("equal", adjustable="datalim")
        ax[9].set_title("Trace GPS (m, origine = 1er point valide ; couleur = t)")
        fig.colorbar(sc, ax=ax[9], label="t (s)")
    else:
        ax[9].set_title("GPS : aucun point valide")

    for a in ax[:9]:
        a.set_xlabel("t (s)"); a.grid(True, alpha=0.3)
    ax[9].grid(True, alpha=0.3)
    fig.tight_layout()
    plt.show()


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    pos = [a for a in argv if not a.startswith("--")]
    path = Path(pos[0] if pos else FICHIER_BIN)

    data = path.read_bytes()
    if data[:4] != b"NVCH" and len(data) % LEGACY_SIZE == 0 and len(data) > 0:
        fast, acc, mag, gps, n = parse_legacy(data)
        print(f"{path} : ANCIEN FORMAT logState ({len(data)} o = {n} x {LEGACY_SIZE}) ; "
              f"le firmware n'a pas enregistré avec le nouveau module logging")
        duree = fast.t.iloc[-1] - fast.t.iloc[0] if len(fast) else 0
        print(f"durée {duree:.1f} s | fast {len(fast)}, acc {len(acc)} (changements), mag {len(mag)} (changements), gps 0")
    else:
        rows, info = parse(data)
        fast, acc, mag, gps = to_frames(rows)
        duree = fast.t.iloc[-1] - fast.t.iloc[0] if len(fast) else 0
        print(f"{path} : {info['octets']} o, durée {duree:.1f} s | fast {len(fast)}, acc {len(acc)}, "
              f"mag {len(mag)}, gps {len(gps)} | octets ignorés (resync) {info['resync']}, "
              f"fin tronquée {info['octets_ignores_fin']} o")

    if "--no-csv" not in argv:
        stem = path.with_suffix("")
        for name, df in (("fast", fast), ("acc", acc), ("mag", mag), ("gps", gps)):
            df.to_csv(f"{stem}_{name}.csv", index=False)
        merged(fast, acc, mag, gps).to_csv(f"{stem}_merged.csv", index=False)
        print(f"CSV écrits à côté de {path.name}")
    if "--no-plot" not in argv:
        plot(fast, acc, mag, gps)
    return fast, acc, mag, gps


if __name__ == "__main__":
    fast, acc, mag, gps = main()
