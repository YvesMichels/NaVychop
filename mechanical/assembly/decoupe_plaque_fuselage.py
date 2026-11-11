# -*- coding: utf-8 -*-

"""
Patron de découpe des pièces du fuselage dans une plaque de polystyrène.

La géométrie des pièces n'est plus définie ici : elle est lue dans le
fichier de référence pieces_fuselage.json, partagé avec structure_avion.py.

Convention (identique à l'ancienne version) :
    pièce      : points (a, b) = (largeur, longueur)
    affiche_obj: les colonnes sont [y, x] dans la plaque, coord = [y, x]
"""

import json
from collections import Counter
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# Fichier de référence des pièces
# ============================================================

# Par défaut : à côté de ce script (adapter si le JSON est ailleurs)
PIECES_FILE = Path(__file__).resolve().parent / "pieces_fuselage.json"


def charge_pieces(chemin):
    """Retourne {nom: np.array([[a, b], ...])} depuis le JSON de référence."""
    with open(chemin, encoding="utf-8") as f:
        data = json.load(f)
    return {
        nom: np.array(piece["points"], dtype=float)
        for nom, piece in data["pieces"].items()
    }


PIECES = charge_pieces(PIECES_FILE)


def dims(nom):
    """(largeur, longueur) de l'encombrement d'une pièce de référence."""
    p = PIECES[nom]
    return tuple(p.max(axis=0) - p.min(axis=0))


# ============================================================
# Dimensions de la plaque
# ============================================================

l_pl = 650
h_pl = 500


# ============================================================
# Opérations (traduction des opérations MATLAB)
# ============================================================

def neg(obj):
    """MATLAB : -obj (rotation de 180 degrés)."""
    return -obj


def fliplr(obj):
    """MATLAB : fliplr(obj) -> [y, x] devient [x, y]."""
    return np.fliplr(obj)


def mirror_c0(obj):
    """Miroir de la largeur (retournement de la plaque) : (a, b) -> (-a, b)."""
    return obj * np.array([-1.0, 1.0])


OPS = {
    "neg": neg,
    "fliplr": fliplr,
    "mirror_c0": mirror_c0,
}


def transforme(obj, ops=()):
    """Applique successivement les opérations de la liste ops."""
    for op in ops:
        obj = OPS[op](obj)
    return obj


# ============================================================
# Pièces découpées d'un seul tenant
# ============================================================
# Dans la plaque, avant et arrière d'une même plaque de fuselage sont
# découpés d'un seul tenant (650 mm), alors que le fichier de référence
# les décrit séparément (assemblage 3D).
# Format : nom : [(pièce de référence, ops, coord [y, x] dans l'ensemble)]

ENSEMBLES = {
    "fuselage_vertical_droit": [
        ("fuselage_vertical_droit_avant", [], (0, 0)),
        ("fuselage_vertical_droit_arriere", ["mirror_c0"], dims("fuselage_vertical_droit_avant")),
    ],
    "fuselage_vertical_gauche": [
        ("fuselage_vertical_gauche_avant", [], (0, 0)),
        ("fuselage_vertical_gauche_arriere", ["mirror_c0"], dims("fuselage_vertical_gauche_avant")),
    ],
    "fuselage_horizontal": [
        ("fuselage_horizontal_avant", [], (0, 0)),
        ("fuselage_horizontal_arriere", [], (0, dims("fuselage_horizontal_avant")[1])),
    ],
}


# ============================================================
# Implantation dans la plaque
# ============================================================
# (nom de pièce ou d'ensemble, ops, coord [y, x])

# Cotes utiles aux pièces imbriquées (déduites du fichier de référence)
L_BOX_V = dims("box_queue_vertical_droit")[0]     # largeur caisson vertical
L_BOX_H = dims("box_queue_horizontal")[0]         # largeur caisson horizontal
Y_BOX_V = h_pl - L_BOX_V                          # caissons verticaux calés en haut de plaque
Y_BOX_H = Y_BOX_V - L_BOX_H
Y_BORD = Y_BOX_H - dims("bord_40x40")[0]

DECOUPE = [

    # --- FUSELAGE ---
    ("fuselage_vertical_droit",       ["neg"], (60 + 2 * 65, 650)),
    ("fuselage_vertical_gauche",      [],      (60 + 2 * 65, 0)),
    ("fuselage_horizontal",           ["neg"], (2 * 65, 650)),
    ("fuselage_horizontal_arriere",   ["neg"], (65, 350)),             # fuselage_haut_queue
    ("fuselage_haut_ailes",           [],      (0, 350 + 25)),
    ("bord_65x25",                    [],      (0, 350)),
    ("bord_65x25",                    [],      (0, 350 + 25 + 250)),

    # --- QUEUE ---
    ("queue",                         [],      (2 * 60 + 2 * 65, 0)),
    ("gouverne_profondeur",           [],      (460, 250)),
    ("box_queue_vertical_droit",      [],      (Y_BOX_V, 0)),
    ("box_queue_vertical_gauche",     ["neg"], (Y_BOX_V + L_BOX_V, 150)),
    ("box_queue_horizontal",          [],      (Y_BOX_H, 0)),
    ("bord_40x40",                    [],      (Y_BORD, 0)),
    ("bord_60x60",                    [],      (400, 220)),

    # --- GOUVERNES DE LACET ---
    ("aileron_lacet_droit",           ["fliplr"],        (250, 400 + 40)),
    ("aileron_lacet_gauche",          ["neg", "fliplr"], (250 + 100, 400 + 100)),
    ("gouverne_lacet_droit",          [],      (245, 520)),
    ("gouverne_lacet_gauche",         [],      (245, 570)),

    # --- RENFORTS AILE ---
    ("renfort_aile",                  [],      (400, 280)),
    ("renfort_aile",                  [],      (400, 465)),
    ("piece_support_avant_aile",      [],      (350, 400)),
    ("piece_support_avant_aile",      [],      (350, 460)),
    ("piece_support_arriere_aile",    [],      (350, 520)),
    ("piece_support_arriere_aile",    [],      (350, 580)),

    # --- RENFORTS LATERAUX ---
    ("renfort_lateral",               [],      (400, 290)),
    ("renfort_lateral",               [],      (400, 344)),
    ("renfort_lateral",               [],      (410, 290)),
    ("renfort_lateral",               [],      (410, 344)),
]


# ============================================================
# Construction et affichage
# ============================================================

def construit(nom, ops, coord):
    """
    Retourne la liste des polygones [y, x] (dans la plaque) de la pièce
    ou de l'ensemble nom, après ops puis translation coord = [y, x].
    """
    if nom in ENSEMBLES:
        parties = [
            transforme(PIECES[p], o) + np.array(c, dtype=float)
            for p, o, c in ENSEMBLES[nom]
        ]
    else:
        parties = [PIECES[nom]]

    return [transforme(p, ops) + np.array(coord, dtype=float) for p in parties]


def aretes(poly):
    n = len(poly)
    return [
        frozenset((tuple(poly[i]), tuple(poly[(i + 1) % n])))
        for i in range(n)
    ]


def affiche_obj(polygones):
    """
    Trace les polygones d'une pièce. Les arêtes communes à deux parties
    d'un même ensemble (jonction avant/arrière) sont tracées en pointillé.
    """
    compte = Counter(a for p in polygones for a in aretes(p))

    for poly in polygones:
        n = len(poly)
        for i in range(n):
            p1, p2 = poly[i], poly[(i + 1) % n]
            commune = compte[frozenset((tuple(p1), tuple(p2)))] > 1
            plt.plot(
                [p1[1], p2[1]],
                [p1[0], p2[0]],
                color="0.6" if commune else "k",
                linestyle="--" if commune else "-",
                linewidth=0.8 if commune else 1.5,
            )


if __name__ == "__main__":

    plt.figure(figsize=(13, 10))

    # Plaque
    plt.gca().add_patch(
        plt.Rectangle((0, 0), l_pl, h_pl, fill=False, edgecolor="r")
    )

    for nom, ops, coord in DECOUPE:
        affiche_obj(construit(nom, ops, coord))

    plt.axis("equal")
    plt.xlim(0, l_pl)
    plt.ylim(0, h_pl)

    plt.xlabel("X [mm]")
    plt.ylabel("Y [mm]")
    plt.grid(True, alpha=0.2)

    plt.show()
