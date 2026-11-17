"""Exemple complet : avion + capteurs simulés + suivi d'une trajectoire de balises.

Exemples :
    python run_sim.py                          # cercle de 100 m de rayon
    python run_sim.py --traj eight             # huit
    python run_sim.py --traj eight --wind 4 --wind-from 270 --duration 240
    python run_sim.py --no-show --out res.png  # sans fenêtre (enregistre seulement l'image)
"""
import argparse
import math

import numpy as np

from aircraft import Aircraft
from controller import L1Params, L1WaypointController
from geo import GeoOrigin
from mission import Mission, circle, distance_to_path, figure_eight
from sensors import SensorSuite
from simulator import Simulator


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--traj", choices=["circle", "eight"], default="circle")
    ap.add_argument("--size", type=float, default=None,
                    help="rayon du cercle (def. 100 m) ou demi-largeur du huit (def. 150 m)")
    ap.add_argument("--alt", type=float, default=80.0, help="altitude des balises (m)")
    ap.add_argument("--speed", type=float, default=18.0, help="vitesse air de croisière (m/s)")
    ap.add_argument("--duration", type=float, default=150.0, help="durée simulée (s)")
    ap.add_argument("--wind", type=float, default=0.0, help="vitesse du vent (m/s)")
    ap.add_argument("--wind-from", type=float, default=270.0, help="direction D'OÙ vient le vent (deg, 0 = Nord)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--out", default="sim_result.png")
    args = ap.parse_args()

    import matplotlib
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    origin = GeoOrigin(lat_deg=45.0, lon_deg=5.0, alt_m=200.0)

    # --- trajectoire : l'avion démarre en (0,0), cap Nord, 80 m d'altitude
    if args.traj == "circle":
        r = args.size or 100.0
        waypoints = circle(0.0, r, r, alt=args.alt, n_points=16, wp_radius=15.0, speed=args.speed)
    else:
        a = args.size or 150.0
        waypoints = figure_eight(0.0, 0.0, a, alt=args.alt, n_points=24, wp_radius=15.0,
                                 heading_deg=0.0, speed=args.speed)
    mission = Mission(waypoints, loop=True)

    # --- vent (vecteur NED de la masse d'air)
    wf = math.radians(args.wind_from)
    wind = (-args.wind * math.cos(wf), -args.wind * math.sin(wf), 0.0)

    # --- les trois briques : avion, capteurs, contrôleur
    ac = Aircraft(wind_ned=wind, altitude=args.alt, heading=0.0, speed=args.speed)
    sensors = SensorSuite.default(origin, seed=args.seed)
    ctrl = L1WaypointController(mission, origin, L1Params(cruise_speed=args.speed))
    sim = Simulator(ac, sensors, ctrl, origin, dt=0.01)

    log = sim.run(args.duration)

    # --- bilan
    xt = distance_to_path(waypoints, log.pos[:, 0], log.pos[:, 1], loop=True)
    settle = log.t > 20.0           # on ignore le transitoire initial
    laps = len(mission.events) / len(waypoints)
    print(f"Durée simulée        : {log.t[-1]:.0f} s" + ("   *** CRASH ***" if log.crashed else ""))
    print(f"Tours effectués      : {laps:.2f}  ({len(mission.events)} balises franchies)")
    print(f"Écart à la trajectoire (après 20 s) : moyen {xt[settle].mean():.1f} m, max {xt[settle].max():.1f} m")
    alt = -log.pos[:, 2]
    print(f"Altitude             : moyenne {alt[settle].mean():.1f} m, écart-type {alt[settle].std():.1f} m")
    print(f"Vitesse air          : moyenne {log.airspeed[settle].mean():.1f} m/s")

    # --- graphiques
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(4, 2, width_ratios=[1.2, 1])
    ax = fig.add_subplot(gs[:, 0])
    wn = np.array([[w.north, w.east] for w in waypoints])
    for w in waypoints:
        ax.add_patch(plt.Circle((w.east, w.north), w.radius, color="tab:orange", alpha=0.25))
    ax.plot(np.append(wn[:, 1], wn[0, 1]), np.append(wn[:, 0], wn[0, 0]), "--", color="tab:orange", lw=1,
            label="trajectoire des balises")
    ax.plot(wn[:, 1], wn[:, 0], "o", color="tab:orange", ms=4)
    ax.plot(log.gps[:, 2], log.gps[:, 1], ".", color="tab:gray", ms=2, label="mesures GPS (bruitées)")
    ax.plot(log.pos[:, 1], log.pos[:, 0], color="tab:blue", lw=1.5, label="trajectoire réelle")
    ax.plot(0, 0, "k*", ms=12, label="départ")
    ax.set_aspect("equal")
    ax.set_xlabel("Est (m)")
    ax.set_ylabel("Nord (m)")
    ax.set_title(f"Vue de dessus - {args.traj}" + (f" - vent {args.wind:.0f} m/s" if args.wind else ""))
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)

    a1 = fig.add_subplot(gs[0, 1])
    a1.plot(log.t, alt, label="altitude réelle")
    a1.plot(log.t, log.debug["h_cmd"], "--", label="consigne")
    a1.set_ylabel("Altitude (m)")
    a1.legend(fontsize=8)
    a2 = fig.add_subplot(gs[1, 1], sharex=a1)
    a2.plot(log.t, log.airspeed, label="vitesse air")
    a2.axhline(args.speed, ls="--", color="k", lw=0.8)
    a2.set_ylabel("m/s")
    a2b = a2.twinx()
    a2b.plot(log.t, log.cmd[:, 3], color="tab:red", lw=0.8)
    a2b.set_ylabel("poussée", color="tab:red")
    a3 = fig.add_subplot(gs[2, 1], sharex=a1)
    a3.plot(log.t, np.degrees(log.euler[:, 0]), label="roll")
    a3.plot(log.t, np.degrees(log.cmd[:, 0]), "--", lw=0.8, label="roll demandé")
    a3.plot(log.t, np.degrees(log.euler[:, 1]), label="pitch")
    a3.set_ylabel("deg")
    a3.legend(fontsize=8, ncol=3)
    a4 = fig.add_subplot(gs[3, 1], sharex=a1)
    a4.plot(log.t, xt)
    a4.set_ylabel("écart trajectoire (m)")
    a4.set_xlabel("t (s)")
    for a_ in (a1, a2, a3, a4):
        a_.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.out, dpi=130)
    print(f"Figure enregistrée : {args.out}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
