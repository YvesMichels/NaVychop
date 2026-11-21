"""Estimateurs de navigation (« hybridation » de capteurs).

Un estimateur reçoit à chaque pas les mesures capteurs (dict, None = pas de nouvelle mesure)
et renvoie un `NavState` (position, vitesse, altitude, vitesse air) ou None tant qu'il n'est pas prêt.
Le contrôleur n'utilise QUE ce NavState : on peut donc changer d'estimateur sans toucher au guidage.

Deux estimateurs sont fournis :

  ExtrapolationEstimator : GPS brut extrapolé avec la vitesse GPS (référence, très simple).
  GpsImuBaroEstimator    : filtre de Kalman faiblement couplé GPS + IMU + baromètre + Pitot,
                           avec une estimation d'attitude (AHRS) à partir du gyromètre/accéléromètre.
"""
from __future__ import annotations

import math

import numpy as np

from aircraft import G, quat_from_euler, quat_mult, quat_to_dcm
from geo import GeoOrigin, wrap_pi
from mission import NavState


class Estimator:
    def update(self, t: float, meas: dict) -> NavState | None:
        raise NotImplementedError


# --------------------------------------------------------------------------- référence
class ExtrapolationEstimator(Estimator):
    """Position GPS extrapolée avec la vitesse GPS ; altitude = baro filtré ; vitesse air = Pitot filtré."""

    def __init__(self, origin: GeoOrigin):
        self.origin = origin
        self._gps = None            # (t_mesure, nord, est, vn, ve, vd)
        self._alt = None
        self._airspeed = None

    def update(self, t, meas):
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
        if self._gps is None or self._alt is None:
            return None
        tg, n, e, vn, ve, vd = self._gps
        dt = max(0.0, t - tg)
        va = self._airspeed if self._airspeed is not None else math.hypot(vn, ve)
        return NavState(t, n + vn * dt, e + ve * dt, self._alt, vn, ve, vd, va)


# --------------------------------------------------------------------------- briques du filtre
class Kalman1D:
    """Filtre de Kalman [position, vitesse] sur UN axe, entrée = accélération mesurée.

    predict : p += v dt + a dt²/2 ; v += a dt          (bruit d'accélération `sigma_a`)
    update  : mesure de position et/ou de vitesse (scalaires, variances `r`)
    """

    def __init__(self, p0, v0, sigma_p0, sigma_v0, sigma_a):
        self.x = np.array([p0, v0], dtype=float)
        self.P = np.diag([sigma_p0 ** 2, sigma_v0 ** 2])
        self.sigma_a = sigma_a

    def predict(self, a, dt):
        F = np.array([[1.0, dt], [0.0, 1.0]])
        self.x = F @ self.x + np.array([0.5 * dt * dt, dt]) * a
        g = np.array([[0.5 * dt * dt], [dt]])
        self.P = F @ self.P @ F.T + (g @ g.T) * self.sigma_a ** 2

    def update(self, z, H, r):
        """z : vecteur de mesures, H : matrice d'observation (m×2), r : écarts-types (m,)."""
        H = np.atleast_2d(H).astype(float)
        z = np.atleast_1d(z).astype(float)
        R = np.diag(np.atleast_1d(r).astype(float) ** 2)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - H @ self.x)
        self.P = (np.eye(2) - K @ H) @ self.P


class AttitudeEstimator:
    """AHRS de type Mahony pour avion (gyro + accéléro + vitesse air + cap GPS).

    - Intègre le gyromètre (quaternion).
    - Recale roulis/tangage sur la direction de la gravité mesurée. En virage, l'accéléromètre ne mesure PAS
      la gravité seule : on retire l'accélération centripète cinématique  a_cin = ω × [V,0,0]  (V = vitesse air).
    - Recale le cap sur la route sol GPS (suppose peu de vent : le crabe dû au vent reste une erreur résiduelle).
    - Estime le biais du gyromètre (terme intégral).
    """

    def __init__(self, kp=2.0, ki=0.02, k_yaw=0.4):
        self.kp, self.ki, self.k_yaw = kp, ki, k_yaw
        self.q = None
        self.bias = np.zeros(3)

    def init(self, heading):
        self.q = quat_from_euler(0.0, 0.0, heading)

    @property
    def R(self):
        return quat_to_dcm(self.q)

    def update(self, dt, gyro, accel, airspeed, course=None):
        if self.q is None or dt <= 0:
            return
        R = self.R
        p_pred = R[2, :]                                   # direction « bas » dans le repère avion (prédite)
        a_cin = np.cross(gyro - self.bias, [airspeed, 0.0, 0.0])
        g_meas = a_cin - accel                             # gravité mesurée, repère avion
        n = np.linalg.norm(g_meas)
        e = np.zeros(3)
        if 0.5 * G < n < 1.5 * G:                          # rejette les mesures aberrantes
            e = np.cross(g_meas / n, p_pred)
        if course is not None:
            _, _, psi = _euler(self.q)
            e = e + self.k_yaw / max(self.kp, 1e-6) * wrap_pi(course - psi) * p_pred
        self.bias -= self.ki * e * dt
        w = gyro - self.bias + self.kp * e
        self.q = self.q + 0.5 * dt * quat_mult(self.q, np.array([0.0, *w]))
        self.q /= np.linalg.norm(self.q)


def _euler(q):
    from aircraft import quat_to_euler
    return quat_to_euler(q)


# --------------------------------------------------------------------------- estimateur hybride
class GpsImuBaroEstimator(Estimator):
    """Hybridation faiblement couplée GPS + IMU + baromètre + Pitot.

    - Horizontal : 2 filtres 1D (Nord, Est). Prédiction à la cadence de l'IMU avec l'accélération NED
      a = R·f + g (R fournie par l'AHRS) ; correction à chaque fix GPS (position ET vitesse).
    - Vertical : un filtre 1D, prédiction avec l'accélération verticale IMU, corrections baro + GPS.
    - Vitesse air : filtre passe-bas du Pitot.

    Réglages à connaître : `sigma_a` (confiance dans l'IMU : plus grand = on suit davantage le GPS),
    `sigma_gps_h` / `sigma_gps_v` / `sigma_baro` (doivent refléter les bruits réels des capteurs).
    """

    def __init__(self, origin: GeoOrigin, sigma_a=0.6, sigma_gps_h=1.5, sigma_gps_v=3.0, sigma_gps_vel=0.15,
                 sigma_baro=0.3, attitude: AttitudeEstimator | None = None):
        self.origin = origin
        self.sa = sigma_a
        self.sgh, self.sgv, self.sgvel, self.sb = sigma_gps_h, sigma_gps_v, sigma_gps_vel, sigma_baro
        self.ahrs = attitude or AttitudeEstimator()
        self.kn = self.ke = self.kh = None    # filtres Nord / Est / altitude
        self._t_imu = None
        self._airspeed = None
        self._course = None
        self._t_course = -1e9      # instant de la dernière route sol GPS valide
        self.course_max_age = 1.0  # s : au-delà, le cap GPS est jugé périmé (coupure GPS) et n'est plus utilisé

    def update(self, t, meas):
        # -- Pitot
        a = meas.get("pitot")
        if a is not None:
            self._airspeed = a.airspeed if self._airspeed is None else self._airspeed + 0.2 * (a.airspeed - self._airspeed)

        # -- initialisation au premier fix GPS
        g = meas.get("gps")
        if self.kn is None:
            if g is None:
                return None
            n, e = self.origin.to_local(g.lat, g.lon)
            vn, ve, vd = g.vel_ned
            self.kn = Kalman1D(n, vn, self.sgh, self.sgvel, self.sa)
            self.ke = Kalman1D(e, ve, self.sgh, self.sgvel, self.sa)
            self.kh = Kalman1D(g.alt - self.origin.alt0, -vd, self.sgv, self.sgvel, self.sa)
            self._course = math.atan2(ve, vn)
            self._t_course = t
            self.ahrs.init(self._course)

        # -- IMU : AHRS + prédiction des filtres
        imu = meas.get("imu")
        if imu is not None:
            dt = 0.0 if self._t_imu is None else imu.t - self._t_imu
            self._t_imu = imu.t
            if dt > 0.0:
                fresh = (t - self._t_course) < self.course_max_age
                self.ahrs.update(dt, imu.gyro, imu.accel, self._airspeed or 18.0, self._course if fresh else None)
                a_ned = self.ahrs.R @ imu.accel + np.array([0.0, 0.0, G])
                self.kn.predict(a_ned[0], dt)
                self.ke.predict(a_ned[1], dt)
                self.kh.predict(-a_ned[2], dt)

        # -- corrections
        if g is not None:
            n, e = self.origin.to_local(g.lat, g.lon)
            vn, ve, vd = g.vel_ned
            Hpv = np.eye(2)
            self.kn.update([n, vn], Hpv, [self.sgh, self.sgvel])
            self.ke.update([e, ve], Hpv, [self.sgh, self.sgvel])
            self.kh.update([g.alt - self.origin.alt0, -vd], Hpv, [self.sgv, self.sgvel])
            if math.hypot(vn, ve) > 5.0:
                self._course = math.atan2(ve, vn)
                self._t_course = t
        b = meas.get("baro")
        if b is not None:
            self.kh.update([b.altitude], [[1.0, 0.0]], [self.sb])

        va = self._airspeed if self._airspeed is not None else math.hypot(self.kn.x[1], self.ke.x[1])
        return NavState(t, self.kn.x[0], self.ke.x[0], self.kh.x[0], self.kn.x[1], self.ke.x[1], -self.kh.x[1], va)
