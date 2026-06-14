"""
gen_datalink.py — Génération automatique de datalink.cpp depuis parameters.json
================================================================================
Usage :
    python gen_datalink.py                        # lit ../parameters/parameters.json
    python gen_datalink.py chemin/parameters.json # chemin explicite
    python gen_datalink.py --output chemin/datalink.cpp

Règle de protocole (issue du JSON) :
    tx_type = 0x10 + (id >> 8)
    dat0    = id & 0xFF
    dat1    = LSB de la valeur int16
    dat2    = MSB de la valeur int16
    valeur physique = int16 / 2^binpt   (conversion écrite EN DUR dans le .cpp généré)

Le champ C++ cible est :  <cpp_struct>.<cpp_field>
    ex : system_param.control_param.Kp_pitch
"""

from __future__ import annotations

import json
import sys
import argparse
from pathlib import Path
from datetime import datetime

# ---------------------------------------------------------------------------
# Constantes protocole (doivent correspondre au firmware et à param_tuner.py)
# ---------------------------------------------------------------------------
MSG_TYPE_BASE  = 0x10   # offset appliqué au MSB de l'id dans tx_type
LORA_CMD_SET_PARAM = 0x10  # valeur de LORA_CMD_SET_PARAM dans config.h

# ---------------------------------------------------------------------------
# Chargement JSON
# ---------------------------------------------------------------------------
def load_json(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

# ---------------------------------------------------------------------------
# Calcul tx_type et dat0 à partir de l'id entier
# ---------------------------------------------------------------------------
def param_routing(param_id: int) -> tuple[int, int]:
    """Retourne (tx_type, dat0) pour un id donné."""
    id_lsb  =  param_id        & 0xFF
    id_msb  = (param_id >> 8)  & 0xFF
    tx_type =  MSG_TYPE_BASE   + id_msb
    return tx_type, id_lsb

# ---------------------------------------------------------------------------
# Génération du corps de la fonction datalink_set_param_lora()
# ---------------------------------------------------------------------------
def generate_cases(categories: list) -> list[str]:
    """
    Génère les blocs if/else if C++ pour chaque paramètre.
    Structure :
        if (tx_type == 0xXX && dat0 == 0xYY) {
            // param.cat_id.label  [binpt=N]
            system_param.control_param.Kp_pitch =
                (float)((int16_t)(dat1 | (dat2 << 8))) / 256.0f;  // 2^8
        }
    """
    lines = []
    first = True

    for cat in categories:
        cat_id     = cat["id"]
        cpp_struct = cat.get("cpp_struct", "system_param." + cat_id)

        for p in cat["parameters"]:
            pid      = int(p["id"])
            label    = p["label"]
            cpp_fld  = p.get("cpp_field", label)
            binpt    = int(p.get("binpt", 8))
            scale    = 1 << binpt          # 2^binpt  (entier exact)
            scalef   = f"{scale}.0f"       # représentation C float

            tx_type, dat0 = param_routing(pid)

            keyword = "if" if first else "else if"
            first   = False

            # Commentaire identifiant le paramètre
            lines.append(
                f"    {keyword} (tx_type == 0x{tx_type:02X} && dat0 == 0x{dat0:02X})"
                f"  /* param.{cat_id}.{label}  id={pid}  binpt={binpt} */"
            )
            lines.append("    {")

            # Conversion int16 → float en dur avec le binpt du JSON
            lines.append(
                f"        {cpp_struct}.{cpp_fld} ="
            )
            lines.append(
                f"            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8)))"
                f" / {scalef};  // 1 / 2^{binpt}"
            )
            lines.append("    }")

    return lines

# ---------------------------------------------------------------------------
# Génération du fichier complet
# ---------------------------------------------------------------------------
def generate_datalink_cpp(data: dict, out_path: Path):
    categories = data.get("categories", [])
    total      = sum(len(c["parameters"]) for c in categories)
    now        = datetime.now().strftime("%Y-%m-%d %H:%M")

    case_lines = generate_cases(categories)

    # ── Résumé de la table pour le commentaire d'en-tête ─────────────────
    table_lines = []
    table_lines.append(
        f"//  {'id':>4}  {'tx_type':>8}  {'dat0':>5}  {'binpt':>5}  "
        f"{'scale (2^binpt)':>16}  cible C++"
    )
    table_lines.append("//  " + "-" * 80)
    for cat in categories:
        cat_id     = cat["id"]
        cpp_struct = cat.get("cpp_struct", "system_param." + cat_id)
        for p in cat["parameters"]:
            pid     = int(p["id"])
            label   = p["label"]
            cpp_fld = p.get("cpp_field", label)
            binpt   = int(p.get("binpt", 8))
            tx_type, dat0 = param_routing(pid)
            table_lines.append(
                f"//  {pid:>4}    0x{tx_type:02X}     0x{dat0:02X}"
                f"  {binpt:>5}  {1<<binpt:>16}  "
                f"{cpp_struct}.{cpp_fld}"
            )

    # ── Assemblage du fichier ─────────────────────────────────────────────
    lines = []

    lines += [
        f"// *** FICHIER AUTO-GÉNÉRÉ — NE PAS MODIFIER À LA MAIN ***",
        f"// Généré par gen_datalink.py le {now}",
        f"// Source : parameters.json  ({total} paramètres, {len(categories)} catégorie(s))",
        f"//",
        f"// Règle de routage protocolaire :",
        f"//   tx_type = 0x{MSG_TYPE_BASE:02X} + (id >> 8)",
        f"//   dat0    = id & 0xFF",
        f"//   dat1    = octet bas  de la valeur int16",
        f"//   dat2    = octet haut de la valeur int16",
        f"//   valeur physique = int16 / 2^binpt  (échelle écrite en dur ci-dessous)",
        f"//",
        f"// Table des paramètres :",
    ] + table_lines + [
        f"//",
        f"",
        f'#include "datalink.h"',
        f"",
        f"",
        f"void datalink_set_param_lora(parameters &system_param, uint8_t lora_cmd[4])",
        f"{{",
        f"    // Décodage trame paramètre",
        f"    uint8_t tx_type = lora_cmd[0];",
        f"    uint8_t dat0    = lora_cmd[1];",
        f"    uint8_t dat1    = lora_cmd[2];  // LSB valeur",
        f"    uint8_t dat2    = lora_cmd[3];  // MSB valeur",
        f"",
        f"    // Seules les trames de type SET_PARAM sont traitées ici.",
        f"    // tx_type vaut 0x{MSG_TYPE_BASE:02X} pour les id 0x0000..0x00FF,",
        f"    //              0x{MSG_TYPE_BASE+1:02X} pour les id 0x0100..0x01FF, etc.",
        f"    if (tx_type < 0x{MSG_TYPE_BASE:02X})",
        f"        return;",
        f"",
    ]

    lines += case_lines

    lines += [
        f"",
        f"    // tx_type / dat0 inconnus : trame ignorée",
        f"}}",
        f"",
    ]

    out_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[OK] {out_path}  ({total} cas générés)")

# ---------------------------------------------------------------------------
# Génération du header datalink.h
# ---------------------------------------------------------------------------
def generate_datalink_h(out_path: Path):
    lines = [
        f"// *** FICHIER AUTO-GÉNÉRÉ — NE PAS MODIFIER À LA MAIN ***",
        f"#ifndef DATALINK_H",
        f"#define DATALINK_H",
        f"",
        f'#include <Arduino.h>',
        f'#include "project_types.h"',
        f'#include "navigation.h"',
        f'#include "control.h"',
        f'#include "config.h"',
        f"",
        f"void datalink_set_param_lora(parameters &system_param, uint8_t lora_cmd[4]);",
        f"",
        f"#endif",
        f"",
    ]
    out_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"[OK] {out_path}")

# ---------------------------------------------------------------------------
# Point d'entrée
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Génère datalink.cpp / datalink.h depuis parameters.json")
    parser.add_argument(
        "json", nargs="?",
        help="Chemin vers parameters.json (défaut : ../parameters/parameters.json)")
    parser.add_argument(
        "--output-cpp", default=None,
        help="Chemin de sortie pour datalink.cpp")
    parser.add_argument(
        "--output-h", default=None,
        help="Chemin de sortie pour datalink.h")
    parser.add_argument(
        "--output-dir", default=None,
        help="Dossier de sortie pour les deux fichiers (remplacé par --output-cpp/h)")
    args = parser.parse_args()

    # ── Résolution chemins ────────────────────────────────────────────────
    script_dir = Path(__file__).parent

    json_path = (Path(args.json) if args.json
                 else script_dir / ".." / "parameters" / "parameters.json")
    json_path = json_path.resolve()

    if not json_path.exists():
        print(f"[ERREUR] JSON introuvable : {json_path}")
        sys.exit(1)

    out_dir = (Path(args.output_dir).resolve() if args.output_dir
               else (script_dir / ".." / "nav" / "lib" / "datalink").resolve())

    cpp_path = Path(args.output_cpp).resolve() if args.output_cpp \
               else out_dir / "datalink.cpp"
    h_path   = Path(args.output_h).resolve()   if args.output_h   \
               else out_dir / "datalink.h"

    # ── Création du dossier de sortie si nécessaire ─────────────────────
    out_dir.mkdir(parents=True, exist_ok=True)

    # ── Chargement & génération ──────────────────────────────────────────
    data = load_json(json_path)
    print(f"[INFO] JSON      : {json_path}")
    print(f"[INFO] Sortie    : {out_dir}")

    generate_datalink_cpp(data, cpp_path)
    generate_datalink_h(h_path)

if __name__ == "__main__":
    main()