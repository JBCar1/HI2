"""Miscellaneous helper functions."""
import time
import math


class Timer:
    """Simple timer."""

    t0 = 0.0

    def tic(self):
        """Start timer."""
        self.t0 = time.time()

    def toc(self):
        """Stop timer."""
        return time.time() - self.t0


def latlong_distance(p1, p2):
    """Compute Haversine distance between points.

    LatLongDistance(p1, p2) returns distance in meters
    between points p1 and p2.

    A point p is a list/array p=[longitude, latitude]
    """
    radius = 6371  # km

    lat1 = p1[1] * math.pi / 180.0
    lat2 = p2[1] * math.pi / 180.0
    lon1 = p1[0] * math.pi / 180.0
    lon2 = p2[0] * math.pi / 180.0

    deltaLat = lat2 - lat1
    deltaLon = lon2 - lon1
    a = (math.sin(deltaLat / 2)**2
         + math.cos(lat1) * math.cos(lat2) * math.sin(deltaLon / 2)**2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = radius * c
    d = d * 1e3  # Return in m
    return d
