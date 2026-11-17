"""Modèle de l'avion : solide rigide à 6 degrés de liberté + autopilote bas niveau.

CONVENTIONS (standard aéronautique)
-----------------------------------
- Repère monde NED : x = Nord, y = Est, z = Bas. L'altitude vaut donc  -z.
- Repère avion FRD : x = avant, y = droite, z = bas.
- Angles d'Euler (ZYX) : roll (phi) > 0 = aile droite vers le bas,
  pitch (theta) > 0 = nez vers le haut, yaw/cap (psi) > 0 = nez vers la droite
  (psi = 0 : Nord, psi = 90° : Est).
- Quaternion q = [w, x, y, z] : rotation repère avion -> repère monde.

CE QUE FAIT LE MODÈLE
---------------------
1. Un solide rigide (masse, matrice d'inertie, position, vitesse, quaternion, vitesse angulaire)
   intégré en RK4.
2. Des efforts aérodynamiques simples : portance (fonction de l'incidence alpha),
   traînée parabolique, force latérale (fonction du dérapage beta), poussée moteur.
3. Un "autopilote bas niveau" (boucle interne) : il transforme les commandes HAUT NIVEAU
   (roll, pitch, vitesse de lacet, poussée) en couples appliqués au solide, via un PD
   dimensionné avec l'inertie. C'est lui qui fait que l'avion "s'incline" ou "cabre" quand
   on le lui demande.

Le contrôleur de trajectoire (controller.py) ne voit QUE les commandes haut niveau
(classe `Command`) et les mesures capteurs : jamais l'état vrai.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

G = 9.81  # m/s²


# --------------------------------------------------------------------------- quaternions
def quat_mult(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_to_dcm(q):
    """Matrice de rotation repère avion -> repère monde."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_to_euler(q):
    """Retourne (roll, pitch, yaw) en radians."""
    w, x, y, z = q
    phi = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    theta = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    psi = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return phi, theta, psi


def quat_from_euler(phi, theta, psi):
    cr, sr = np.cos(phi / 2), np.sin(phi / 2)
    cp, sp = np.cos(theta / 2), np.sin(theta / 2)
    cy, sy = np.cos(psi / 2), np.sin(psi / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


# --------------------------------------------------------------------------- paramètres
@dataclass
class AircraftParams:
    """Paramètres physiques (ordre de grandeur d'un avion RC de ~2 kg, 1,6 m d'envergure).
    À remplacer par les valeurs réelles de navychop."""
    # inertie
    mass: float = 2.0                      # kg
    inertia: tuple = (0.08, 0.12, 0.18)    # Ixx, Iyy, Izz en kg·m² (matrice diagonale)
    # géométrie
    wing_area: float = 0.4                 # m²
    span: float = 1.6                      # m
    chord: float = 0.25                    # m
    rho: float = 1.225                     # kg/m³
    # aérodynamique
    CL0: float = 0.2
    CLa: float = 5.0                       # /rad
    CL_max: float = 1.2                    # décrochage simplifié : CL saturé
    CD0: float = 0.03
    k_induced: float = 0.06
    CY_beta: float = -0.3                  # force latérale due au dérapage
    Cm_alpha: float = -0.2                 # stabilité statique en tangage
    Cm_q: float = -3.0                     # amortissement en tangage
    Cl_p: float = -0.1                     # amortissement en roulis
    Cn_beta: float = 0.06                  # "girouette" (stabilité de route)
    Cn_r: float = -0.1                     # amortissement en lacet
    # propulsion : T = throttle * thrust_max * (1 - V / thrust_v0)
    thrust_max: float = 12.0               # N
    thrust_v0: float = 35.0                # m/s
    # autopilote bas niveau
    wn_roll: float = 12.0                  # rad/s
    wn_pitch: float = 8.0                  # rad/s
    zeta: float = 0.8
    k_yaw_rate: float = 5.0                # 1/s (bande passante de la boucle de vitesse de lacet)
    max_roll: float = np.radians(45.0)
    max_pitch: float = np.radians(25.0)
    max_yaw_rate: float = np.radians(60.0)
    max_torque: tuple = (4.0, 3.0, 1.5)    # N·m (saturation des gouvernes)


@dataclass
class Command:
    """Commandes HAUT NIVEAU envoyées à l'avion par le contrôleur de trajectoire."""
    roll: float = 0.0        # angle de roulis demandé (rad)
    pitch: float = 0.0       # angle de tangage demandé (rad)
    yaw_rate: float = 0.0    # vitesse de variation du cap demandée (rad/s)
    throttle: float = 0.46   # poussée moteur, 0..1


# --------------------------------------------------------------------------- avion
class Aircraft:
    """État : x = [pos(3), vel(3), q(4), omega(3)]  (pos/vel en NED, omega = p,q,r dans le repère avion)."""

    def __init__(self, params=None, wind_ned=(0.0, 0.0, 0.0), north=0.0, east=0.0,
                 altitude=80.0, heading=0.0, speed=18.0):
        self.p = params or AircraftParams()
        self.wind = np.array(wind_ned, dtype=float)   # vitesse de la masse d'air / sol (m/s)
        self.I = np.diag(self.p.inertia)
        self.Iinv = np.linalg.inv(self.I)
        self.tau_max = np.array(self.p.max_torque)
        self.t = 0.0
        self.x = self.trimmed_state(north, east, altitude, heading, speed)
        self.specific_force = np.zeros(3)             # force spécifique (mesurée par un accéléro), repère avion
        _, self.specific_force = self._deriv(self.x, Command())

    # -- état initial : vol en palier équilibré ---------------------------------
    def trimmed_state(self, north, east, altitude, heading, speed):
        P = self.p
        cl = 2 * P.mass * G / (P.rho * speed ** 2 * P.wing_area)
        alpha = (cl - P.CL0) / P.CLa
        q = quat_from_euler(0.0, alpha, heading)
        vel = speed * np.array([np.cos(heading), np.sin(heading), 0.0]) + self.wind
        return np.concatenate([[north, east, -altitude], vel, q, np.zeros(3)])

    # -- accès pratiques --------------------------------------------------------
    @property
    def pos(self):
        return self.x[0:3]

    @property
    def vel(self):
        return self.x[3:6]

    @property
    def quat(self):
        return self.x[6:10]

    @property
    def omega(self):
        return self.x[10:13]

    @property
    def euler(self):
        return quat_to_euler(self.quat)

    @property
    def altitude(self):
        return -self.x[2]

    @property
    def airspeed(self):
        return float(np.linalg.norm(quat_to_dcm(self.quat).T @ (self.vel - self.wind)))

    @property
    def crashed(self):
        return self.altitude < 0.0

    # -- dynamique --------------------------------------------------------------
    def _limit(self, cmd):
        P = self.p
        return Command(
            roll=float(np.clip(cmd.roll, -P.max_roll, P.max_roll)),
            pitch=float(np.clip(cmd.pitch, -P.max_pitch, P.max_pitch)),
            yaw_rate=float(np.clip(cmd.yaw_rate, -P.max_yaw_rate, P.max_yaw_rate)),
            throttle=float(np.clip(cmd.throttle, 0.0, 1.0)),
        )

    def _deriv(self, x, cmd):
        """Dérivée de l'état + force spécifique (N/kg) dans le repère avion."""
        P = self.p
        vel, q, om = x[3:6], x[6:10], x[10:13]
        R = quat_to_dcm(q)
        phi, theta, _ = quat_to_euler(q)
        p_, q_, r_ = om

        # --- vitesse air, incidence, dérapage
        va = R.T @ (vel - self.wind)
        V = max(float(np.linalg.norm(va)), 1.0)
        alpha = np.arctan2(va[2], max(va[0], 1e-3))
        beta = np.arcsin(np.clip(va[1] / V, -1.0, 1.0))
        qbar = 0.5 * P.rho * V * V
        S = P.wing_area

        # --- efforts (repère avion)
        CL = np.clip(P.CL0 + P.CLa * alpha, -P.CL_max, P.CL_max)
        CD = P.CD0 + P.k_induced * CL * CL
        lift = qbar * S * CL
        drag = qbar * S * CD
        side = qbar * S * P.CY_beta * beta
        thrust = cmd.throttle * P.thrust_max * max(0.0, 1.0 - V / P.thrust_v0)
        F = np.array([thrust + lift * np.sin(alpha), side, -lift * np.cos(alpha)]) - drag * va / V

        # --- couples : autopilote bas niveau (PD sur les angles) + aérodynamique
        I = P.inertia
        tau_ctrl = np.array([
            I[0] * (P.wn_roll ** 2 * (cmd.roll - phi) - 2 * P.zeta * P.wn_roll * p_),
            I[1] * (P.wn_pitch ** 2 * (cmd.pitch - theta) - 2 * P.zeta * P.wn_pitch * q_),
            I[2] * P.k_yaw_rate * (cmd.yaw_rate * np.cos(phi) * np.cos(theta) - r_),
        ])
        tau_ctrl = np.clip(tau_ctrl, -self.tau_max, self.tau_max)
        b, c = P.span, P.chord
        tau_aero = qbar * S * np.array([
            b * P.Cl_p * (b / (2 * V)) * p_,
            c * (P.Cm_alpha * alpha + P.Cm_q * (c / (2 * V)) * q_),
            b * (P.Cn_beta * beta + P.Cn_r * (b / (2 * V)) * r_),
        ])
        tau = tau_ctrl + tau_aero

        # --- équations du mouvement
        pos_dot = vel
        vel_dot = R @ F / P.mass + np.array([0.0, 0.0, G])
        q_dot = 0.5 * quat_mult(q, np.array([0.0, p_, q_, r_]))
        om_dot = self.Iinv @ (tau - np.cross(om, self.I @ om))
        return np.concatenate([pos_dot, vel_dot, q_dot, om_dot]), F / P.mass

    def step(self, cmd, dt):
        """Avance la simulation de dt secondes (RK4) avec la commande `cmd` maintenue constante."""
        cmd = self._limit(cmd)
        x = self.x
        k1, sf = self._deriv(x, cmd)
        k2, _ = self._deriv(x + 0.5 * dt * k1, cmd)
        k3, _ = self._deriv(x + 0.5 * dt * k2, cmd)
        k4, _ = self._deriv(x + dt * k3, cmd)
        x = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        x[6:10] /= np.linalg.norm(x[6:10])
        self.x = x
        self.specific_force = sf
        self.t += dt
