"""Simulation interactive : trajectoire en direct + prise de contrôle manuelle au clavier.

    python live_sim.py                       # cercle, guidage L1, GPS brut
    python live_sim.py --traj eight --estimator kalman --wind 4
    python live_sim.py --timescale 2         # simulation 2x plus rapide que le temps réel

Touches (la fenêtre doit avoir le focus) :
    ← / →      roulis gauche / droite   (virage coordonné)
    ↑ / ↓      nez en haut / nez en bas (tangage)
    espace     pause / reprise
    + / -      accélérer / ralentir la simulation (x0,25 à x16)
    r          recommencer
    q          quitter

PRINCIPE : l'autopilote (guidage + estimateur) tourne en permanence, il ne sait pas qu'on le surcharge.
Tant qu'une flèche est enfoncée, la commande correspondante est remplacée par la vôtre (roulis+lacet pour
← →, tangage pour ↑ ↓) ; dès qu'on relâche, l'autopilote reprend la main et doit revenir sur la trajectoire.
Le tracé devient rouge pendant le pilotage manuel.

Sans écran (test, capture pour un rapport) : un scénario de touches simulées peut être rejoué :
    python live_sim.py --snapshot res.png --seconds 90 --script "20-24:left,45-48:up"
"""
from __future__ import annotations

import argparse
import math
import time
from collections import deque
from dataclasses import replace

import numpy as np

from aircraft import G, Aircraft
from benchmark import make_waypoints
from controller import L1Params, L1WaypointController
from estimators import GpsImuBaroEstimator
from geo import GeoOrigin
from mission import Mission, distance_to_path
from sensors import SensorSuite
from simulator import Simulator

HELP = "←/→ roulis   ↑/↓ tangage   (relâcher : l'autopilote reprend)   espace pause   +/- vitesse   r reset   q quitter"


# =============================================================================== pilotage manuel
class ManualPilot:
    """Transforme l'état des flèches en surcharge de la commande de l'autopilote.

    Tant qu'une flèche est tenue, la consigne manuelle d'un axe (roulis, tangage) évolue en rampe à partir
    de l'attitude actuelle de l'avion (pas de saut) ; à la relâche, l'axe est rendu à l'autopilote.
    """
    GRACE = 0.08   # s : tolère les relâchements/répétitions intempestifs de la répétition clavier du système

    def __init__(self, sim: Simulator, max_roll=math.radians(40), max_pitch=math.radians(20),
                 roll_rate=math.radians(60), pitch_rate=math.radians(30)):
        self.sim = sim
        self.max_roll, self.max_pitch = max_roll, max_pitch
        self.roll_rate, self.pitch_rate = roll_rate, pitch_rate
        self.keys = {}                  # touche -> None (enfoncée) ou instant du relâchement
        self.dir_roll = 0               # -1, 0, +1
        self.dir_pitch = 0
        self.roll = None                # consigne manuelle courante (None : autopilote)
        self.pitch = None

    # -- état du clavier
    def press(self, key):
        self.keys[key] = None

    def release(self, key, now):
        if key in self.keys:
            self.keys[key] = now

    def _active(self, key, now):
        if key not in self.keys:
            return False
        released = self.keys[key]
        if released is None or now - released < self.GRACE:
            return True
        del self.keys[key]
        return False

    def read_keys(self, now):
        self.dir_roll = int(self._active("right", now)) - int(self._active("left", now))
        self.dir_pitch = int(self._active("up", now)) - int(self._active("down", now))

    @property
    def manual(self):
        return self.roll is not None or self.pitch is not None

    # -- surcharge appelée à chaque pas de simulation
    def override(self, cmd):
        ac, dt = self.sim.ac, self.sim.dt
        if self.dir_roll:
            if self.roll is None:
                self.roll = float(ac.euler[0])
            self.roll = float(np.clip(self.roll + self.dir_roll * self.roll_rate * dt, -self.max_roll, self.max_roll))
            cmd = replace(cmd, roll=self.roll, yaw_rate=G * math.tan(self.roll) / max(ac.airspeed, 5.0))
        else:
            self.roll = None
        if self.dir_pitch:
            if self.pitch is None:
                self.pitch = float(ac.euler[1])
            self.pitch = float(np.clip(self.pitch + self.dir_pitch * self.pitch_rate * dt, -self.max_pitch, self.max_pitch))
            cmd = replace(cmd, pitch=self.pitch)
        else:
            self.pitch = None
        return cmd


# =============================================================================== simulation + affichage
class LiveSim:
    def __init__(self, traj="circle", size=None, alt=80.0, cruise=18.0, wind=0.0, wind_from=270.0,
                 estimator="gps", seed=0, timescale=1.0, dt=0.01, log_every=5, clock=time.perf_counter):
        self.cfg = dict(traj=traj, size=size, alt=alt, cruise=cruise, wind=wind, wind_from=wind_from,
                        estimator=estimator, seed=seed)
        self.timescale, self.dt, self.log_every, self.clock = timescale, dt, log_every, clock
        self.max_steps = 100000000            # garde-fou : pas de simulation max par image affichée
        self.paused = False
        self.build()
        self._make_figure()

    # ------------------------------------------------------------------ construction / reset
    def build(self):
        c = self.cfg
        self.origin = GeoOrigin(45.0, 5.0, 200.0)
        self.waypoints = make_waypoints(c["traj"], c["size"], c["alt"], c["cruise"])
        self.mission = Mission(self.waypoints, loop=True)
        wf = math.radians(c["wind_from"])
        wind = (-c["wind"] * math.cos(wf), -c["wind"] * math.sin(wf), 0.0)
        self.ac = Aircraft(wind_ned=wind, altitude=c["alt"], heading=0.0, speed=c["cruise"])
        sensors = SensorSuite.default(self.origin, seed=c["seed"])
        est = GpsImuBaroEstimator(self.origin) if c["estimator"] == "kalman" else None
        self.ctrl = L1WaypointController(self.mission, self.origin, L1Params(cruise_speed=c["cruise"]), estimator=est)
        self.sim = Simulator(self.ac, sensors, self.ctrl, self.origin, dt=self.dt)
        self.pilot = ManualPilot(self.sim)
        self.crashed = False
        self.nstep = 0
        self._budget = 0.0
        self._last = self.clock()
        # historiques d'affichage
        self.trail = deque(maxlen=6000)        # (est, nord, manuel?)
        self.gps = deque(maxlen=300)
        self.hist = deque(maxlen=1200)         # (t, altitude, consigne altitude)
        self.xt_hist = []
        self._bounds = self._waypoint_bounds()

    def _waypoint_bounds(self):
        n = [w.north for w in self.waypoints] + [0.0]
        e = [w.east for w in self.waypoints] + [0.0]
        return [min(e), max(e), min(n), max(n)]

    # ------------------------------------------------------------------ avance de la simulation
    def advance(self, n_steps):
        for _ in range(n_steps):
            t, meas, _cmd = self.sim.step(self.pilot.override)
            self.nstep += 1
            g = meas.get("gps")
            if g is not None:
                n, e = self.origin.to_local(g.lat, g.lon)
                self.gps.append((e, n))
            if self.nstep % self.log_every == 0:
                n, e, down = self.ac.pos
                self.trail.append((e, n, 1.0 if self.pilot.manual else 0.0))
                dbg = self.ctrl.debug
                self.hist.append((self.ac.t, -down, dbg.get("h_cmd", np.nan)))
                b = self._bounds
                b[0], b[1], b[2], b[3] = min(b[0], e), max(b[1], e), min(b[2], n), max(b[3], n)
                self.xt_hist.append((self.ac.t, float(distance_to_path(self.waypoints, n, e)[0])))
            if self.ac.crashed:
                self.crashed = True
                break

    def update(self, _frame=None):
        """Une image : lit le clavier, avance la simulation du temps réel écoulé (x timescale), redessine."""
        now = self.clock()
        elapsed = min(now - self._last, 0.1)       # un à-coup de l'interface ne doit pas faire sauter la simulation
        self._last = now
        self.pilot.read_keys(now)
        if not self.paused and not self.crashed:
            self._budget += elapsed * self.timescale
            n = min(int(self._budget / self.dt), self.max_steps)
            self._budget -= n * self.dt
            if self._budget > self.dt:             # on n'arrive pas à suivre : on abandonne le retard
                self._budget = 0.0
            self.advance(n)
        self.draw()

    # ------------------------------------------------------------------ clavier
    @staticmethod
    def _key(event):
        k = event.key
        if k and len(k) > 1 and "+" in k:
            k = k.split("+")[-1]
        return k

    def on_press(self, event):
        k = self._key(event)
        if k in ("left", "right", "up", "down"):
            self.pilot.press(k)
        elif k in (" ", "space"):
            self.paused = not self.paused
        elif k in ("+", "="):
            self.timescale = min(16.0, self.timescale * 2)
        elif k in ("-", "_"):
            self.timescale = max(0.25, self.timescale / 2)
        elif k == "r":
            self.build()
            self.draw(full=True)

    def on_release(self, event):
        self.pilot.release(self._key(event), self.clock())

    # ------------------------------------------------------------------ figure
    def _make_figure(self):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon

        for k in ("keymap.back", "keymap.forward", "keymap.home", "keymap.pan", "keymap.zoom", "keymap.save",
                  "keymap.fullscreen", "keymap.grid", "keymap.grid_minor", "keymap.yscale", "keymap.xscale",
                  "keymap.copy"):
            try:
                plt.rcParams[k] = []               # libère les touches (flèches, r, p...) prises par matplotlib
            except KeyError:
                pass
        self.plt = plt
        self.fig = plt.figure(figsize=(13, 7.2))
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1.5, 1], height_ratios=[1, 1], wspace=0.15, hspace=0.25)
        ax = self.ax = self.fig.add_subplot(gs[:, 0])
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("Est (m)")
        ax.set_ylabel("Nord (m)")
        ax.grid(alpha=0.3)
        ax.set_title("Vue de dessus (bleu : autopilote, rouge : pilotage manuel)")

        self.artists_static = []
        for w in self.waypoints:
            ax.add_patch(Circle((w.east, w.north), w.radius, color="tab:orange", alpha=0.25, lw=0))
        pts = np.array([[w.east, w.north] for w in self.waypoints])
        ax.plot(np.append(pts[:, 0], pts[0, 0]), np.append(pts[:, 1], pts[0, 1]), "--", color="tab:orange", lw=1)
        ax.plot(pts[:, 0], pts[:, 1], "o", color="tab:orange", ms=3)
        self.l_leg, = ax.plot([], [], "-", color="tab:green", lw=2, alpha=0.6, label="branche courante")
        self.l_target, = ax.plot([], [], "o", mfc="none", mec="tab:red", ms=14, mew=2, label="balise visée")
        self.l_gps, = ax.plot([], [], ".", color="tab:gray", ms=3, label="mesures GPS")
        self.l_auto, = ax.plot([], [], "-", color="tab:blue", lw=1.6, label="trajectoire réelle")
        self.l_man, = ax.plot([], [], "-", color="tab:red", lw=2.2)
        self.l_est, = ax.plot([], [], "x", color="tab:green", ms=8, mew=2, label="position estimée")
        self.plane = Polygon(np.zeros((3, 2)), closed=True, fc="black", ec="white", zorder=10)
        ax.add_patch(self.plane)
        ax.legend(loc="upper right", fontsize=8)

        self.ax_alt = self.fig.add_subplot(gs[0, 1])
        self.l_alt, = self.ax_alt.plot([], [], color="tab:blue", label="altitude")
        self.l_hcmd, = self.ax_alt.plot([], [], "--", color="k", lw=0.9, label="consigne")
        self.ax_alt.set_xlabel("t (s)")
        self.ax_alt.set_ylabel("Altitude (m)")
        self.ax_alt.legend(fontsize=8, loc="upper left")
        self.ax_alt.grid(alpha=0.3)

        ax_info = self.fig.add_subplot(gs[1, 1])
        ax_info.axis("off")
        self.info = ax_info.text(0.0, 1.0, "", va="top", ha="left", family="monospace", fontsize=10)
        self.fig.text(0.5, 0.012, HELP, ha="center", fontsize=9, color="dimgray")

        self.fig.canvas.mpl_connect("key_press_event", self.on_press)
        self.fig.canvas.mpl_connect("key_release_event", self.on_release)
        self.draw(full=True)

    def _fit_view(self):
        e0, e1, n0, n1 = self._bounds
        half = max(e1 - e0, n1 - n0) / 2.0 + 60.0
        cx, cy = (e0 + e1) / 2.0, (n0 + n1) / 2.0
        self.ax.set_xlim(cx - half, cx + half)
        self.ax.set_ylim(cy - half, cy + half)
        return 2 * half

    def draw(self, full=False):
        ac, ctrl, mission = self.ac, self.ctrl, self.mission
        n, e, down = ac.pos
        span = self._fit_view()

        # avion (triangle orienté selon le cap)
        psi = ac.euler[2]
        fwd = np.array([math.sin(psi), math.cos(psi)])           # (Est, Nord)
        right = np.array([math.cos(psi), -math.sin(psi)])
        L = 0.03 * span
        c = np.array([e, n])
        self.plane.set_xy(np.array([c + fwd * L, c - fwd * 0.7 * L - right * 0.5 * L, c - fwd * 0.7 * L + right * 0.5 * L]))

        # traces
        if self.trail:
            arr = np.array(self.trail)
            auto, man = arr[:, :2].copy(), arr[:, :2].copy()
            auto[arr[:, 2] > 0.5] = np.nan
            man[arr[:, 2] < 0.5] = np.nan
            self.l_auto.set_data(auto[:, 0], auto[:, 1])
            self.l_man.set_data(man[:, 0], man[:, 1])
        else:
            self.l_auto.set_data([], [])
            self.l_man.set_data([], [])
        if self.gps:
            g = np.array(self.gps)
            self.l_gps.set_data(g[:, 0], g[:, 1])
        else:
            self.l_gps.set_data([], [])

        # balise visée, branche courante, position estimée
        dbg = ctrl.debug
        est_n, est_e = dbg.get("est_n"), dbg.get("est_e")
        if est_n is not None:
            self.l_est.set_data([est_e], [est_n])
            wp, A = mission.target, mission.prev
            self.l_target.set_data([wp.east], [wp.north])
            if A is not None:
                self.l_leg.set_data([A[1], wp.east], [A[0], wp.north])
        else:
            self.l_est.set_data([], [])

        # altitude sur les 60 dernières secondes
        t_now = ac.t
        if self.hist:
            h = np.array(self.hist)
            self.l_alt.set_data(h[:, 0], h[:, 1])
            self.l_hcmd.set_data(h[:, 0], h[:, 2])
            lo = np.nanmin(np.r_[h[:, 1], h[:, 2]]) - 5.0
            hi = np.nanmax(np.r_[h[:, 1], h[:, 2]]) + 5.0
            self.ax_alt.set_ylim(min(lo, self.cfg["alt"] - 10), max(hi, self.cfg["alt"] + 10))
        self.ax_alt.set_xlim(max(0.0, t_now - 60.0), max(60.0, t_now))

        # texte d'état
        if self.crashed:
            mode = "CRASH  (r : recommencer)"
        elif self.paused:
            mode = "PAUSE"
        elif self.pilot.manual:
            axes = ("roulis " if self.pilot.roll is not None else "") + ("tangage" if self.pilot.pitch is not None else "")
            mode = f"MANUEL ({axes.strip()})"
        else:
            mode = "AUTOPILOTE"
        xt = float(distance_to_path(self.waypoints, n, e)[0])
        est_err = math.hypot(est_n - n, est_e - e) if est_n is not None else float("nan")
        roll, pitch, _ = np.degrees(ac.euler)
        self.info.set_text(
            f"MODE            {mode}\n"
            f"temps simulé    {t_now:6.1f} s   (x{self.timescale:g})\n"
            f"altitude        {-down:6.1f} m   (consigne {dbg.get('h_cmd', float('nan')):.1f})\n"
            f"vitesse air     {ac.airspeed:6.1f} m/s\n"
            f"roulis/tangage  {roll:+6.1f} / {pitch:+5.1f} deg\n"
            f"écart au tracé  {xt:6.1f} m\n"
            f"balise visée    {mission.index + 1}/{len(self.waypoints)}   franchies : {len(mission.events)}\n"
            f"erreur estim.   {est_err:6.2f} m   ({self.cfg['estimator']})"
        )

    # ------------------------------------------------------------------ exécution
    def run_gui(self):
        from matplotlib.animation import FuncAnimation
        self.anim = FuncAnimation(self.fig, self.update, interval=40, cache_frame_data=False)   # ~25 images/s
        self.plt.show()

    def run_scripted(self, seconds, script):
        """Rejoue sans écran des appuis de touches simulés. script : [(t0, t1, {touches})] en secondes simulées."""
        frame = 0.05
        n_frames = int(round(seconds / frame))
        for _ in range(n_frames):
            t = self.ac.t
            keys = set()
            for t0, t1, ks in script:
                if t0 <= t < t1:
                    keys |= ks
            self.pilot.dir_roll = int("right" in keys) - int("left" in keys)
            self.pilot.dir_pitch = int("up" in keys) - int("down" in keys)
            self.advance(int(round(frame / self.dt)))
            if self.crashed:
                break
        self.draw()


def parse_script(text):
    """'20-24:left,45-48:up+right' -> [(20, 24, {'left'}), (45, 48, {'up', 'right'})]"""
    out = []
    for item in filter(None, (text or "").split(",")):
        span, keys = item.split(":")
        t0, t1 = (float(x) for x in span.split("-"))
        out.append((t0, t1, set(keys.split("+"))))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--traj", choices=["circle", "eight"], default="circle")
    ap.add_argument("--size", type=float, default=None, help="rayon du cercle (100 m) ou demi-largeur du huit (150 m)")
    ap.add_argument("--alt", type=float, default=80.0)
    ap.add_argument("--speed", type=float, default=18.0, help="vitesse air de croisière (m/s)")
    ap.add_argument("--wind", type=float, default=0.0)
    ap.add_argument("--wind-from", type=float, default=270.0)
    ap.add_argument("--estimator", choices=["gps", "kalman"], default="gps")
    ap.add_argument("--timescale", type=float, default=1.0, help="vitesse de la simulation (1 = temps réel)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--snapshot", help="sans écran : enregistre une image au lieu d'ouvrir la fenêtre")
    ap.add_argument("--seconds", type=float, default=60.0, help="durée simulée en mode --snapshot")
    ap.add_argument("--script", default="", help="touches simulées en mode --snapshot, ex. '20-24:left,45-48:up'")
    args = ap.parse_args()

    import matplotlib
    if args.snapshot:
        matplotlib.use("Agg")
    live = LiveSim(args.traj, args.size, args.alt, args.speed, args.wind, args.wind_from,
                   args.estimator, args.seed, args.timescale)
    if args.snapshot:
        live.run_scripted(args.seconds, parse_script(args.script))
        live.fig.savefig(args.snapshot, dpi=110)
        xt = np.array(live.xt_hist)
        print(f"Image enregistrée : {args.snapshot}  (t = {live.ac.t:.0f} s, écart au tracé final {xt[-1, 1]:.1f} m, "
              f"maximum {xt[:, 1].max():.1f} m)")
        return
    if matplotlib.get_backend().lower() in ("agg", "pdf", "svg", "ps", "cairo", "template"):
        raise SystemExit("Aucun backend graphique interactif disponible. Installez-en un "
                         "(pip install PyQt5, ou le paquet système python3-tk) ou utilisez --snapshot.")
    live.run_gui()


if __name__ == "__main__":
    main()
