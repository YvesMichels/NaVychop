"""Banc d'essai : exécute un scénario avec un contrôleur/estimateur donné et renvoie des métriques chiffrées.

    from benchmark import run_scenario
    res = run_scenario(lambda mission, origin: L1WaypointController(mission, origin), traj="eight", wind=5)
    print(res["xtrack_mean"], res["xtrack_max"], res["est_rms"])

`controller_factory(mission, origin)` doit renvoyer un `Controller`.
"""
from __future__ import annotations

import math

import numpy as np

from aircraft import Aircraft
from geo import GeoOrigin
from mission import Mission, circle, distance_to_path, figure_eight
from sensors import GPS, IMU, Barometer, Pitot, SensorSuite
from simulator import Simulator


def make_waypoints(traj="circle", size=None, alt=80.0, speed=18.0):
    if traj == "circle":
        r = size or 100.0
        return circle(0.0, r, r, alt=alt, n_points=16, wp_radius=15.0, speed=speed)
    if traj == "eight":
        return figure_eight(0.0, 0.0, size or 150.0, alt=alt, n_points=24, wp_radius=15.0, speed=speed)
    raise ValueError(traj)


def run_scenario(controller_factory, traj="circle", size=None, wind=0.0, wind_from=270.0, duration=150.0,
                 seed=0, alt=80.0, speed=18.0, gps_cls=GPS, gps_kwargs=None, settle=20.0, dt=0.01):
    """Retourne un dict de métriques + le journal complet (clé 'log')."""
    origin = GeoOrigin(45.0, 5.0, 200.0)
    waypoints = make_waypoints(traj, size, alt, speed)
    mission = Mission(waypoints, loop=True)
    wf = math.radians(wind_from)
    ac = Aircraft(wind_ned=(-wind * math.cos(wf), -wind * math.sin(wf), 0.0), altitude=alt, heading=0.0, speed=speed)
    rng = np.random.default_rng(seed)
    sensors = SensorSuite({"gps": gps_cls(origin, rng=rng, **(gps_kwargs or {})), "imu": IMU(rng=rng),
                           "baro": Barometer(rng=rng), "pitot": Pitot(rng=rng)})
    ctrl = controller_factory(mission, origin)
    log = Simulator(ac, sensors, ctrl, origin, dt=dt).run(duration)

    m = log.t > settle
    xt = distance_to_path(waypoints, log.pos[:, 0], log.pos[:, 1])
    res = {
        "crashed": log.crashed,
        "laps": len(mission.events) / len(waypoints),
        "xtrack_mean": float(xt[m].mean()), "xtrack_max": float(xt[m].max()),
        "alt_std": float((-log.pos[m, 2]).std()),
        # « nervosité » de la consigne de roulis : RMS de sa dérivée (deg/s)
        "roll_jitter": float(np.sqrt(np.mean((np.diff(np.degrees(log.cmd[m, 0])) / (log.t[1] - log.t[0])) ** 2))),
        "log": log,
    }
    # distance entre la position VRAIE et le centre de la balise, à l'instant où le contrôleur la déclare franchie
    miss = []
    for t_e, idx in mission.events:
        if t_e > settle:
            i = int(np.argmin(np.abs(log.t - t_e)))
            wp = waypoints[idx]
            miss.append(math.hypot(log.pos[i, 0] - wp.north, log.pos[i, 1] - wp.east))
    res["miss_mean"] = float(np.mean(miss)) if miss else float("nan")
    res["miss_max"] = float(np.max(miss)) if miss else float("nan")
    if "est_n" in log.debug:
        err = np.hypot(log.debug["est_n"] - log.pos[:, 0], log.debug["est_e"] - log.pos[:, 1])
        res["est_rms"] = float(np.sqrt(np.mean(err[m] ** 2)))
        res["est_max"] = float(err[m].max())
    return res


def print_table(rows):
    """rows : liste de (nom, résultat run_scenario)."""
    print(f"{'algorithme':<26}{'écart moy':>10}{'écart max':>10}{'passage moy':>13}{'passage max':>13}"
          f"{'err. est. RMS':>15}{'nervosité roll':>16}{'tours':>7}")
    for name, r in rows:
        est = f"{r['est_rms']:.2f} m" if "est_rms" in r else "-"
        flag = "  CRASH" if r["crashed"] else ""
        print(f"{name:<26}{r['xtrack_mean']:>8.1f} m{r['xtrack_max']:>8.1f} m{r['miss_mean']:>11.1f} m{r['miss_max']:>11.1f} m{est:>15}"
              f"{r['roll_jitter']:>11.0f} °/s{r['laps']:>7.2f}{flag}")
