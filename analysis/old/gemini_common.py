#!/usr/bin/env python3

from skyfield.api import load, EarthSatellite
import numpy as np

def get_beta_angle(timestamp, tle_file):
    """
    Compute the solar beta angle of ISS zenith relative to the sun for a given timestamp.
    """
    ts = load.timescale()
    t = ts.utc(timestamp.year, timestamp.month, timestamp.day, timestamp.hour, timestamp.minute, timestamp.second)

    # Load TLE data
    satellites = load.tle_file(tle_file)
    # Find the most recent TLE for the given timestamp.
    # The tle file is a list of TLEs over time. We need to find the one that is closest and before the timestamp.
    sat = None
    for s in satellites:
        if s.epoch.tt > t.tt:
            break
        sat = s
    
    if sat is None:
        # if no tle is found before, take the first one.
        sat = satellites[0]

    # Get sun position
    eph = load('de421.bsp')
    sun = eph['sun']

    # Satellite position and velocity
    geocentric = sat.at(t)
    pos = geocentric.position.km
    vel = geocentric.velocity.km_per_s

    # Orbit plane normal vector
    normal = np.cross(pos, vel)
    normal /= np.linalg.norm(normal)

    # Sun vector
    sun_pos = sun.at(t).position.km
    sun_pos /= np.linalg.norm(sun_pos)
    
    # Beta angle
    beta_angle = np.arcsin(np.dot(sun_pos, normal))

    return np.rad2deg(beta_angle)
