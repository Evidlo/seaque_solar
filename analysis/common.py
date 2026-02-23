#!/usr/bin/env python3
"""Common utilities for ISS solar analysis."""

import numpy as np
from datetime import datetime, timezone
from skyfield.api import load, EarthSatellite


def load_historical_tles(tle_file):
    """Load all TLEs from file, returning list of (epoch_jd, line1, line2)."""
    ts = load.timescale()
    tles = []
    with open(tle_file, 'r') as f:
        lines = f.readlines()
    # TLE file has line1, line2 pairs (no name line in this format)
    for i in range(0, len(lines) - 1, 2):
        line1 = lines[i].strip()
        line2 = lines[i + 1].strip()
        if line1.startswith('1 ') and line2.startswith('2 '):
            # Parse epoch from line1: columns 18-32 contain YYDDD.DDDDDDDD
            epoch_str = line1[18:32]
            year = int(epoch_str[:2])
            year = year + 2000 if year < 57 else year + 1900
            day_of_year = float(epoch_str[2:])
            # Convert to Julian date
            epoch_t = ts.utc(year, 1, day_of_year)
            tles.append((epoch_t.tt, line1, line2))
    return sorted(tles, key=lambda x: x[0])


def find_closest_tle(tles, timestamp):
    """Find TLE with epoch closest to (but not after) the given timestamp."""
    ts = load.timescale()
    t = ts.utc(datetime.fromtimestamp(timestamp, tz=timezone.utc))
    target_jd = t.tt

    # Find the TLE with epoch closest to but before the target time
    best = None
    for epoch_jd, line1, line2 in tles:
        if epoch_jd <= target_jd:
            if best is None or epoch_jd > best[0]:
                best = (epoch_jd, line1, line2)

    # If no TLE before target, use the earliest one
    if best is None:
        best = tles[0]

    return best[1], best[2]


def compute_sun_zenith_angle(timestamp, line1, line2, eph=None):
    """
    Compute angle between ISS zenith (radial outward) and sun direction.
    Returns angle in degrees.
    """
    ts = load.timescale()
    if eph is None:
        eph = load('de421.bsp')
    sun = eph['sun']
    earth = eph['earth']

    satellite = EarthSatellite(line1, line2, 'ISS', ts)
    t = ts.utc(datetime.fromtimestamp(timestamp, tz=timezone.utc))

    # Get ISS position relative to Earth center (geocentric)
    geocentric = satellite.at(t)
    iss_pos = geocentric.position.km

    # ISS zenith is the unit vector pointing radially outward (same as position direction)
    zenith = iss_pos / np.linalg.norm(iss_pos)

    # Get sun position relative to Earth center
    sun_geo = earth.at(t).observe(sun).apparent()
    sun_pos = sun_geo.position.km
    sun_dir = sun_pos / np.linalg.norm(sun_pos)

    # Compute angle between zenith and sun direction
    cos_angle = np.dot(zenith, sun_dir)
    angle_deg = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

    return angle_deg
