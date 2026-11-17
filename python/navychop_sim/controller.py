"""Contrôleurs de trajectoire.

Interface : un contrôleur reçoit à CHAQUE pas de simulation les dernières mesures capteurs
(dict, valeur None si pas de nouvelle mesure) et renvoie une `Command` haut niveau.
Il n'a jamais accès à l'état vrai. Pour tester un autre algorithme : hériter de `Controller`.

Premier algorithme fourni : `L1WaypointController`
  - estimation : position GPS extrapolée avec la vitesse GPS (compense la cadence 5 Hz et le retard),
    altitude = baromètre filtré, vitesse air = Pitot filtré ;
  - guidage latéral : loi L1 (Park et al.) sur la branche [balise précédente -> balise courante] ;
  - guidage vertical : PI + amortissement sur la vitesse verticale -> consigne de tangage ;
  - vitesse : PI sur la vitesse air -> poussée ;
  - virage coordonné : vitesse de lacet = g*tan(roll)/V.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from aircraft import G, Command
from geo import GeoOrigin, wrap_pi
from mission import Mission, NavState


class Controller:
    debug: dict

    def update(self, t: float, meas: dict) -> Command:
        raise NotImplementedError


@dataclass
class L1Params:
    cruise_speed: float = 18.0           # m/s (vitesse air)
    # guidage latéral
    l1_period: float = 14.0              # s
    l1_damping: float = 0.75
    l1_min: float = 25.0                 # m
    max_bank: float = math.radians(35.0)
    # guidage vertical
    kp_alt: float = 0.02                 # rad/m
    ki_alt: float = 0.002                # rad/(m·s)
    kd_alt: float = 0.05                 # rad/(m/s)
    max_pitch: float = math.radians(12.0)
    pitch_trim: float = 0.01             # rad
    # vitesse
    thr_trim: float = 0.46
    kp_speed: float = 0.12
    ki_speed: float = 0.03


class L1WaypointController(Controller):
    def __init__(self, mission: Mission, origin: GeoOrigin, params: L1Params | None = None):
        self.mission = mission
        self.origin = origin
        self.p = params or L1Params()
        self.debug = {}
        # mémoire des capteurs
        self._gps = None            # (t_mesure, nord, est, vn, ve, vd)
        self._alt = None            # altitude filtrée (m)
        self._airspeed = None
        self._course = 0.0
        self._started = False
        self._t_prev = None
        self._i_alt = 0.0
        self._i_speed = 0.0

    # ---------------------------------------------------------------- estimation
    def _ingest(self, meas):
        g = meas.get("gps")
        if g is not None:
            n, e = self.origin.to_local(g.lat, g.lon)
            self._gps = (g.t, n, e, g.vel_ned[0], g.vel_ned[1], g.vel_ned[2])
            if meas.get("baro") is None and self._alt is None:
                self._alt = g.alt - self.origin.alt0
        b = meas.get("baro")
        if b is not None:
            self._alt = b.altitude if self._alt is None else self._alt + 0.2 * (b.altitude - self._alt)
        a = meas.get("pitot")
        if a is not None:
            self._airspeed = a.airspeed if self._airspeed is None else self._airspeed + 0.2 * (a.airspeed - self._airspeed)

    def _estimate(self, t) -> NavState:
        tg, n, e, vn, ve, vd = self._gps
        dt = max(0.0, t - tg)                       # extrapolation depuis la dernière mesure GPS
        va = self._airspeed if self._airspeed is not None else math.hypot(vn, ve)
        return NavState(t, n + vn * dt, e + ve * dt, self._alt, vn, ve, vd, va)

    # ---------------------------------------------------------------- boucle de contrôle
    def update(self, t, meas) -> Command:
        P = self.p
        self._ingest(meas)
        if self._gps is None or self._alt is None:
            return Command(pitch=P.pitch_trim, throttle=P.thr_trim)     # attente du premier fix

        dt = 0.0 if self._t_prev is None else t - self._t_prev
        self._t_prev = t
        nav = self._estimate(t)
        if not self._started:
            self.mission.start(nav)
            self._started = True
        self.mission.check(nav)
        wp, A = self.mission.target, self.mission.prev

        # --- cap sol estimé
        vg = math.hypot(nav.vn, nav.ve)
        if vg > 1.0:
            self._course = math.atan2(nav.ve, nav.vn)

        # --- guidage latéral L1
        leg_n, leg_e = wp.north - A[0], wp.east - A[1]
        leg = math.hypot(leg_n, leg_e)
        if leg > 1.0:
            un, ue = leg_n / leg, leg_e / leg
        else:
            un, ue = math.cos(self._course), math.sin(self._course)
        rel_n, rel_e = nav.north - A[0], nav.east - A[1]
        along = rel_n * un + rel_e * ue
        xtrack = rel_e * un - rel_n * ue                       # >0 : à droite de la branche
        l1 = max(P.l1_min, P.l1_damping * P.l1_period * max(vg, 5.0) / math.pi)
        ref_dist = along + l1
        if not self.mission.finished:
            ref_dist = min(max(ref_dist, 0.0), leg) if leg > 1.0 else l1
        ref_n, ref_e = A[0] + un * ref_dist, A[1] + ue * ref_dist
        eta = wrap_pi(math.atan2(ref_e - nav.east, ref_n - nav.north) - self._course)
        a_cmd = 2.0 * vg * vg / l1 * math.sin(eta)
        roll_cmd = max(-P.max_bank, min(P.max_bank, math.atan2(a_cmd, G)))

        # --- guidage vertical (altitude interpolée le long de la branche)
        frac = min(max(along / leg, 0.0), 1.0) if leg > 1.0 else 1.0
        h_cmd = A[2] + (wp.alt - A[2]) * frac
        h_err = h_cmd - nav.alt
        if dt > 0.0:
            self._i_alt = max(-0.1, min(0.1, self._i_alt + P.ki_alt * h_err * dt)) if abs(h_err) < 10.0 else self._i_alt * 0.99
        pitch_cmd = P.pitch_trim + P.kp_alt * h_err + self._i_alt - P.kd_alt * (-nav.vd)
        pitch_cmd = max(-P.max_pitch, min(P.max_pitch, pitch_cmd))

        # --- vitesse -> poussée
        v_cmd = wp.speed if wp.speed is not None else P.cruise_speed
        v_err = v_cmd - nav.airspeed
        if dt > 0.0:
            self._i_speed = max(-0.3, min(0.3, self._i_speed + P.ki_speed * v_err * dt))
        throttle = max(0.0, min(1.0, P.thr_trim + P.kp_speed * v_err + self._i_speed))

        # --- virage coordonné
        yaw_rate = G * math.tan(roll_cmd) / max(nav.airspeed, 5.0)

        self.debug = {"target": self.mission.index, "xtrack_est": xtrack, "h_cmd": h_cmd,
                      "h_est": nav.alt, "eta": eta, "l1": l1, "roll_cmd": roll_cmd}
        return Command(roll=roll_cmd, pitch=pitch_cmd, yaw_rate=yaw_rate, throttle=throttle)
