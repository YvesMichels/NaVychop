"""Balises (waypoints), conditions de passage et générateurs de trajectoires simples.

Convention : une balise est donnée en repère local NED : (north, east) en mètres par rapport
à l'origine, et `alt` = altitude (m, positive vers le haut) par rapport à l'origine.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np


@dataclass
class NavState:
    """Estimation de l'état de navigation, calculée par le contrôleur à partir des capteurs."""
    t: float
    north: float
    east: float
    alt: float        # m, positive vers le haut
    vn: float         # vitesse sol Nord (m/s)
    ve: float
    vd: float
    airspeed: float   # m/s


@dataclass
class Waypoint:
    """Balise + conditions de passage.

    Une balise est considérée comme franchie quand TOUTES les conditions suivantes sont vraies :
      1. condition horizontale : distance < `radius`, OU plan perpendiculaire à la branche franchi
         à moins de 3*radius (évite de tourner autour d'une balise manquée de peu) ;
      2. condition verticale (optionnelle) : |alt - alt_balise| <= `max_alt_error` ;
      3. condition personnalisée (optionnelle) : `condition(nav: NavState) -> bool`
         (ex. vitesse minimale, cap d'arrivée...).
    """
    north: float
    east: float
    alt: float = 80.0
    radius: float = 15.0
    speed: Optional[float] = None            # vitesse air demandée sur la branche menant à cette balise
    max_alt_error: Optional[float] = None
    condition: Optional[Callable[[NavState], bool]] = None
    name: str = ""


class Mission:
    """Liste ordonnée de balises avec gestion de la balise courante."""

    def __init__(self, waypoints, loop=True):
        self.waypoints = list(waypoints)
        self.loop = loop
        self.index = 0
        self.finished = False
        self.prev = None          # (north, east, alt) du début de la branche courante
        self.events = []          # (t, index de la balise franchie)

    def start(self, nav: NavState):
        self.index = 0
        self.finished = False
        self.prev = (nav.north, nav.east, nav.alt)

    @property
    def target(self) -> Waypoint:
        return self.waypoints[self.index]

    def check(self, nav: NavState) -> bool:
        """Teste le passage de la balise courante ; passe à la suivante si franchie."""
        if self.finished:
            return False
        wp, A = self.target, self.prev
        dist = math.hypot(wp.north - nav.north, wp.east - nav.east)
        leg_n, leg_e = wp.north - A[0], wp.east - A[1]
        leg = math.hypot(leg_n, leg_e)
        crossed = False
        if leg > 1e-6:
            past = ((nav.north - wp.north) * leg_n + (nav.east - wp.east) * leg_e) / leg  # >0 : au-delà de la balise
            crossed = past > 0.0 and dist < 3.0 * wp.radius
        horiz_ok = dist < wp.radius or crossed
        vert_ok = wp.max_alt_error is None or abs(nav.alt - wp.alt) <= wp.max_alt_error
        cond_ok = wp.condition is None or bool(wp.condition(nav))
        if not (horiz_ok and vert_ok and cond_ok):
            return False
        self.events.append((nav.t, self.index))
        self.prev = (wp.north, wp.east, wp.alt)
        if self.index + 1 >= len(self.waypoints):
            if self.loop:
                self.index = 0
            else:
                self.finished = True
        else:
            self.index += 1
        return True


# --------------------------------------------------------------------------- générateurs
def circle(center_north, center_east, radius, alt=80.0, n_points=16, wp_radius=15.0,
           clockwise=True, start_bearing_deg=-90.0, speed=None):
    """Cercle de balises. Par défaut, le point de départ est à l'Ouest du centre : un avion qui part
    de ce point cap au Nord tourne dans le sens horaire. La dernière balise est le point de départ."""
    sign = 1.0 if clockwise else -1.0
    wps = []
    for k in range(1, n_points + 1):
        b = math.radians(start_bearing_deg) + sign * 2 * math.pi * k / n_points
        wps.append(Waypoint(center_north + radius * math.cos(b), center_east + radius * math.sin(b),
                            alt, wp_radius, speed, name=f"C{k}"))
    return wps


def figure_eight(center_north, center_east, half_width, alt=80.0, n_points=24, wp_radius=15.0,
                 heading_deg=0.0, speed=None):
    """Huit (lemniscate de Bernoulli). Départ au croisement central avec le cap `heading_deg`.
    Le rayon de courbure minimal vaut 2/3 * half_width (aux extrémités des boucles) :
    half_width = 150 m -> rayon mini 100 m."""
    rot = math.radians(heading_deg) + math.radians(135.0)
    wps = []
    for k in range(1, n_points + 1):
        t = math.pi / 2 + 2 * math.pi * k / n_points
        d = 1.0 + math.sin(t) ** 2
        x = half_width * math.cos(t) / d
        y = half_width * math.sin(t) * math.cos(t) / d
        n = center_north + x * math.cos(rot) - y * math.sin(rot)
        e = center_east + x * math.sin(rot) + y * math.cos(rot)
        wps.append(Waypoint(n, e, alt, wp_radius, speed, name=f"E{k}"))
    return wps


def distance_to_path(waypoints, north, east, loop=True):
    """Distance (m) de points (tableaux north/east) à la ligne polygonale passant par les balises.
    Sert à évaluer la qualité du suivi (le polygone coupe légèrement l'intérieur d'un vrai cercle)."""
    pts = np.array([[w.north, w.east] for w in waypoints])
    if loop:
        pts = np.vstack([pts, pts[0]])
    P = np.column_stack([np.atleast_1d(north), np.atleast_1d(east)])
    best = np.full(len(P), np.inf)
    for a, b in zip(pts[:-1], pts[1:]):
        ab = b - a
        denom = float(ab @ ab)
        s = np.clip(((P - a) @ ab) / denom, 0.0, 1.0) if denom > 0 else np.zeros(len(P))
        best = np.minimum(best, np.linalg.norm(P - (a + s[:, None] * ab), axis=1))
    return best
