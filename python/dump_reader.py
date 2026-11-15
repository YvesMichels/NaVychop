#!/usr/bin/env python3
"""Lecture fiable du dump de logs navychop (couche transport uniquement).

Usage en ligne de commande : python dump_reader.py COM5 logs.bin [--tentatives 5]
Usage depuis Spyder (runfile) : renseigner PORT_DEFAUT / FICHIER_DEFAUT ci-dessous,
puis lancer le script sans argument.

Envoie AA 55 CC 33, attend la trame d'en-tête, vérifie le CRC32 de CHAQUE trame
puis le CRC32 global du fichier. Si une trame est corrompue ou si la réception
s'interrompt, le dump est relancé entièrement (le fichier reste en flash).
Le décodage des enregistrements (0xA1..0xA4) sera fait à part.
"""
import struct
import sys
import time
import zlib

import serial  # pip install pyserial

# --- Valeurs utilisées quand le script est lancé sans argument (Spyder, runfile) ---
PORT_DEFAUT = "COM5"          # à adapter : port série de la carte (Gestionnaire de périphériques)
FICHIER_DEFAUT = "logs.bin"   # fichier de sortie (créé dans le dossier de travail)
TENTATIVES_DEFAUT = 5

SYNC = b"\xAA\x55\xCC"
T_HEADER, T_CHUNK, T_END = 0x33, 0x34, 0x35
TRIGGER = SYNC + bytes([T_HEADER])


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def read_exact(ser, n):
    data = ser.read(n)
    if len(data) != n:
        raise TimeoutError(f"timeout ({len(data)}/{n} octets)")
    return data


def wait_frame(ser, expected_type, timeout=10.0):
    """Cherche AA 55 CC <type> dans le flux (ignore le texte de debug)."""
    pat = SYNC + bytes([expected_type])
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout:
        b = ser.read(1)
        if not b:
            continue
        buf += b
        buf = buf[-4:]
        if bytes(buf) == pat:
            return
    raise TimeoutError("synchronisation non trouvée")


def one_attempt(ser):
    ser.reset_input_buffer()
    ser.write(TRIGGER)

    wait_frame(ser, T_HEADER, timeout=10.0)
    body = read_exact(ser, 11)                       # ver u8, chunk u16, size u32, n_chunks u32
    (crc_rx,) = struct.unpack("<I", read_exact(ser, 4))
    if crc32(body) != crc_rx:
        raise ValueError("CRC en-tête invalide")
    ver, chunk, size, n_chunks = struct.unpack("<BHII", body)
    print(f"dump v{ver} : {size} octets, {n_chunks} trames de {chunk} o")

    out = bytearray()
    for i in range(n_chunks):
        wait_frame(ser, T_CHUNK, timeout=5.0)
        head = read_exact(ser, 6)                    # idx u32, len u16
        idx, ln = struct.unpack("<IH", head)
        payload = read_exact(ser, ln)
        (crc_rx,) = struct.unpack("<I", read_exact(ser, 4))
        if crc32(head + payload) != crc_rx:
            raise ValueError(f"CRC invalide trame {i}")
        if idx != i:
            raise ValueError(f"trame {idx} reçue au lieu de {i}")
        out += payload
        if i % 200 == 0:
            print(f"  {100 * i // max(n_chunks, 1)} %", end="\r")

    wait_frame(ser, T_END, timeout=5.0)
    body = read_exact(ser, 9)                        # status u8, sent u32, crc_fichier u32
    (crc_rx,) = struct.unpack("<I", read_exact(ser, 4))
    if crc32(body) != crc_rx:
        raise ValueError("CRC trame de fin invalide")
    status, sent, file_crc = struct.unpack("<BII", body)
    if status != 0:
        raise ValueError("erreur de lecture flash côté carte")
    if sent != size or len(out) != size or crc32(bytes(out)) != file_crc:
        raise ValueError("taille ou CRC fichier incohérents")
    return bytes(out)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    port = PORT_DEFAUT
    dest = FICHIER_DEFAUT
    tentatives = TENTATIVES_DEFAUT

    pos = [a for a in argv if not a.startswith("--")]
    if "--tentatives" in argv:
        tentatives = int(argv[argv.index("--tentatives") + 1])
        pos = [a for a in pos if a != str(tentatives)]
    if len(pos) >= 1:
        port = pos[0]
    if len(pos) >= 2:
        dest = pos[1]
    print(f"port {port}, sortie {dest}, {tentatives} tentative(s) max")

    with serial.Serial(port, 115200, timeout=1.0) as ser:
        for k in range(1, tentatives + 1):
            try:
                data = one_attempt(ser)
                with open(dest, "wb") as f:
                    f.write(data)
                print(f"OK : {len(data)} octets -> {dest}")
                return
            except (TimeoutError, ValueError) as e:
                print(f"tentative {k}/{tentatives} échouée : {e}")
                time.sleep(1.0)
    raise RuntimeError("échec : dump non fiable")


if __name__ == "__main__":
    main()
