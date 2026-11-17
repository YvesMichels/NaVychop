"""Conversions entre le repère local de la simulation (NED) et les coordonnées GPS.

Approximation "terre plate" : largement suffisante pour des vols de quelques km.
Repère local : x = Nord, y = Est, z = Bas (NED), origine = `GeoOrigin`.
"""
import math

R_EARTH = 6_378_137.0  # m


def wrap_pi(angle):
    """Ramène un angle (rad) dans [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class GeoOrigin:
    """Point de référence (latitude, longitude, altitude) de l'origine du repère local."""

    def __init__(self, lat_deg=45.0, lon_deg=5.0, alt_m=200.0):
        self.lat0 = lat_deg
        self.lon0 = lon_deg
        self.alt0 = alt_m
        self._cos_lat = math.cos(math.radians(lat_deg))

    def to_latlon(self, north, east):
        """(Nord, Est) en mètres -> (latitude, longitude) en degrés."""
        lat = self.lat0 + math.degrees(north / R_EARTH)
        lon = self.lon0 + math.degrees(east / (R_EARTH * self._cos_lat))
        return lat, lon

    def to_local(self, lat_deg, lon_deg):
        """(latitude, longitude) en degrés -> (Nord, Est) en mètres."""
        north = math.radians(lat_deg - self.lat0) * R_EARTH
        east = math.radians(lon_deg - self.lon0) * R_EARTH * self._cos_lat
        return north, east
