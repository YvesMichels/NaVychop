"""Capteurs simulés : GPS, IMU, baromètre, tube de Pitot.

Chaque capteur :
  - lit l'état VRAI de l'avion (jamais accessible au contrôleur),
  - ajoute du bruit (et éventuellement un biais / un retard),
  - ne délivre une mesure qu'à sa propre cadence (GPS 5 Hz, IMU 100 Hz...).

Pour ajouter un capteur (magnétomètre, télémètre...) : hériter de `Sensor`, implémenter
`measure(t, aircraft)` et l'ajouter au dictionnaire de `SensorSuite`.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from aircraft import Aircraft, quat_to_dcm
from geo import GeoOrigin


# --------------------------------------------------------------------------- mesures
@dataclass
class GPSMeasurement:
    t: float             # instant de la mesure (s)
    lat: float           # degrés
    lon: float           # degrés
    alt: float           # m (altitude absolue = alt de l'origine + altitude relative)
    vel_ned: np.ndarray  # vitesse sol Nord/Est/Bas (m/s)


@dataclass
class IMUMeasurement:
    t: float
    accel: np.ndarray    # force spécifique, repère avion (m/s²)
    gyro: np.ndarray     # vitesses angulaires p,q,r (rad/s)


@dataclass
class BaroMeasurement:
    t: float
    altitude: float      # altitude relative au point de décollage (m)


@dataclass
class AirspeedMeasurement:
    t: float
    airspeed: float      # m/s


# --------------------------------------------------------------------------- outils
class GaussMarkovNoise:
    """Bruit gaussien d'écart-type `sigma`.
    corr_time = 0  -> bruit blanc ; corr_time > 0 -> bruit corrélé (Gauss-Markov du 1er ordre),
    plus réaliste pour l'erreur de position d'un GPS (elle varie lentement)."""

    def __init__(self, sigma, rate_hz, corr_time, rng, size=1):
        self.sigma, self.rng, self.size = sigma, rng, size
        self.a = float(np.exp(-1.0 / (rate_hz * corr_time))) if corr_time > 0 else 0.0
        self.e = np.zeros(size)

    def sample(self):
        self.e = self.a * self.e + np.sqrt(1 - self.a ** 2) * self.sigma * self.rng.standard_normal(self.size)
        return self.e


class Sensor:
    """Classe de base : gère la cadence d'échantillonnage et le retard de livraison."""

    def __init__(self, rate_hz, delay_s=0.0, rng=None):
        self.period = 1.0 / rate_hz
        self.rate_hz = rate_hz
        self.delay = delay_s
        self.rng = rng or np.random.default_rng()
        self._next = 0.0
        self._pending = []   # (instant de disponibilité, mesure)

    def measure(self, t, ac: Aircraft):
        raise NotImplementedError

    def read(self, t, ac: Aircraft):
        """Retourne une nouvelle mesure si elle est disponible à l'instant t, sinon None."""
        if t >= self._next - 1e-9:
            self._pending.append((t + self.delay, self.measure(t, ac)))
            self._next += self.period
        if self._pending and self._pending[0][0] <= t + 1e-9:
            return self._pending.pop(0)[1]
        return None


# --------------------------------------------------------------------------- capteurs
class GPS(Sensor):
    """GPS simple : position réelle + bruit gaussien, vitesse sol réelle + bruit."""

    def __init__(self, origin: GeoOrigin, rate_hz=5.0, sigma_h=1.5, sigma_v=3.0, sigma_vel=0.15,
                 corr_time_s=0.0, delay_s=0.0, rng=None):
        super().__init__(rate_hz, delay_s, rng)
        self.origin = origin
        self.n_pos = GaussMarkovNoise(sigma_h, rate_hz, corr_time_s, self.rng, 2)
        self.n_alt = GaussMarkovNoise(sigma_v, rate_hz, corr_time_s, self.rng, 1)
        self.n_vel = GaussMarkovNoise(sigma_vel, rate_hz, 0.0, self.rng, 3)

    def measure(self, t, ac):
        north, east, down = ac.pos
        nn, ne = self.n_pos.sample()
        lat, lon = self.origin.to_latlon(north + nn, east + ne)
        alt = self.origin.alt0 - down + self.n_alt.sample()[0]
        return GPSMeasurement(t, lat, lon, alt, ac.vel + self.n_vel.sample())


class IMU(Sensor):
    """Accéléromètre + gyromètre avec bruit blanc et biais constant tiré au hasard."""

    def __init__(self, rate_hz=100.0, sigma_acc=0.05, sigma_gyro=0.005,
                 bias_acc=0.05, bias_gyro=0.002, rng=None):
        super().__init__(rate_hz, 0.0, rng)
        self.sigma_acc, self.sigma_gyro = sigma_acc, sigma_gyro
        self.b_acc = self.rng.normal(0, bias_acc, 3)
        self.b_gyro = self.rng.normal(0, bias_gyro, 3)

    def measure(self, t, ac):
        return IMUMeasurement(
            t,
            ac.specific_force + self.b_acc + self.sigma_acc * self.rng.standard_normal(3),
            ac.omega + self.b_gyro + self.sigma_gyro * self.rng.standard_normal(3),
        )


class Barometer(Sensor):
    """Altitude relative avec bruit blanc + dérive lente (marche aléatoire)."""

    def __init__(self, rate_hz=20.0, sigma=0.3, drift=0.02, rng=None):
        super().__init__(rate_hz, 0.0, rng)
        self.sigma, self.drift, self.bias = sigma, drift, 0.0

    def measure(self, t, ac):
        self.bias += self.drift * np.sqrt(self.period) * self.rng.standard_normal()
        return BaroMeasurement(t, ac.altitude + self.bias + self.sigma * self.rng.standard_normal())


class Pitot(Sensor):
    """Vitesse air (par rapport à la masse d'air) avec bruit blanc."""

    def __init__(self, rate_hz=20.0, sigma=0.3, rng=None):
        super().__init__(rate_hz, 0.0, rng)
        self.sigma = sigma

    def measure(self, t, ac):
        return AirspeedMeasurement(t, max(0.0, ac.airspeed + self.sigma * self.rng.standard_normal()))


# --------------------------------------------------------------------------- ensemble
class SensorSuite:
    """Regroupe les capteurs. `read_all` renvoie, à chaque pas, un dict
    {'gps': mesure|None, 'imu': ..., 'baro': ..., 'pitot': ...} (None = pas de nouvelle mesure)."""

    def __init__(self, sensors: dict):
        self.sensors = sensors

    @classmethod
    def default(cls, origin: GeoOrigin, seed=0):
        rng = np.random.default_rng(seed)
        return cls({
            "gps": GPS(origin, rng=rng),
            "imu": IMU(rng=rng),
            "baro": Barometer(rng=rng),
            "pitot": Pitot(rng=rng),
        })

    def read_all(self, t, ac):
        return {name: s.read(t, ac) for name, s in self.sensors.items()}
