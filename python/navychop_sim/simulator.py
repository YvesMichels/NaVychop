"""Boucle de simulation "dans la boucle" (SIL).

À chaque pas de temps dt :
    1. les capteurs échantillonnent l'état vrai de l'avion (GPS : position réelle + bruit...) ;
    2. le contrôleur reçoit ces mesures et calcule une commande haut niveau ;
    3. l'avion intègre sa dynamique avec cette commande.

Les coordonnées GPS sont donc générées au fil de l'avancement de la simulation.
Le contrôleur est appelé à la cadence de la simulation (100 Hz par défaut) : s'il doit tourner
plus lentement, il suffit qu'il se décime lui-même (ne calcule qu'un pas sur N).
Remplacer `controller` par un pont vers le vrai firmware (série, UDP...) permet de passer
en "hardware-in-the-loop" sans toucher au reste.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from aircraft import Aircraft, Command
from controller import Controller
from geo import GeoOrigin
from sensors import SensorSuite


@dataclass
class SimLog:
    t: np.ndarray
    pos: np.ndarray            # (N,3) NED vrai
    vel: np.ndarray            # (N,3)
    euler: np.ndarray          # (N,3) roll, pitch, yaw
    omega: np.ndarray          # (N,3)
    airspeed: np.ndarray
    cmd: np.ndarray            # (N,4) roll, pitch, yaw_rate, throttle
    gps: np.ndarray            # (M,4) t, nord, est, alt (mesures GPS reçues par le contrôleur)
    debug: dict = field(default_factory=dict)
    crashed: bool = False


class Simulator:
    def __init__(self, aircraft: Aircraft, sensors: SensorSuite, controller: Controller,
                 origin: GeoOrigin, dt: float = 0.01, log_every: int = 2):
        self.ac, self.sensors, self.controller, self.origin = aircraft, sensors, controller, origin
        self.dt, self.log_every = dt, log_every

    def step(self, override=None):
        """Avance d'UN pas (usage interactif, voir live_sim.py). Même ordre que run() :
        capteurs -> contrôleur -> dynamique. `override(cmd) -> Command` permet de modifier la commande
        calculée par le contrôleur (pilotage manuel) ; le contrôleur, lui, ne sait pas qu'on l'a surchargée.
        Retourne (t, meas, cmd_appliquée)."""
        ac = self.ac
        t = ac.t
        meas = self.sensors.read_all(t, ac)
        cmd = self.controller.update(t, meas)
        if override is not None:
            cmd = override(cmd)
        ac.step(cmd, self.dt)
        return t, meas, cmd

    def run(self, duration: float, stop_when_mission_finished: bool = False) -> SimLog:
        ac, dt = self.ac, self.dt
        rows, gps_rows, dbg = [], [], []
        mission = getattr(self.controller, "mission", None)
        for k in range(int(round(duration / dt))):
            t = k * dt
            meas = self.sensors.read_all(t, ac)               # 1) capteurs
            cmd = self.controller.update(t, meas)             # 2) contrôleur
            if meas.get("gps") is not None:
                n, e = self.origin.to_local(meas["gps"].lat, meas["gps"].lon)
                gps_rows.append((t, n, e, meas["gps"].alt - self.origin.alt0))
            if k % self.log_every == 0:
                rows.append((t, *ac.pos, *ac.vel, *ac.euler, *ac.omega, ac.airspeed,
                             cmd.roll, cmd.pitch, cmd.yaw_rate, cmd.throttle))
                dbg.append(dict(self.controller.debug))
            ac.step(cmd, dt)                                  # 3) dynamique
            if ac.crashed:
                break
            if stop_when_mission_finished and mission is not None and mission.finished:
                break
        a = np.array(rows)
        debug = {}
        keys = {key for d in dbg for key in d}          # union : le contrôleur peut ne rien publier avant son 1er fix
        for key in sorted(keys):
            debug[key] = np.array([d.get(key, np.nan) for d in dbg], dtype=float)
        return SimLog(t=a[:, 0], pos=a[:, 1:4], vel=a[:, 4:7], euler=a[:, 7:10], omega=a[:, 10:13],
                      airspeed=a[:, 13], cmd=a[:, 14:18], gps=np.array(gps_rows), debug=debug,
                      crashed=ac.crashed)
