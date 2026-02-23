#!/usr/bin/env python3
# Plot actual data downlinked 2024-12 + ISS solar beta angle

import pickle
import numpy as np
import matplotlib.pyplot as plt
import datetime
import numpy.linalg as LA

# --- Skyfield setup ---
from skyfield.api import load, EarthSatellite
from bisect import bisect_right


ts = load.timescale()
eph = load('de421.bsp')
sun = eph['sun']
earth = eph['earth']

# Load historical ISS TLEs

def load_iss_tles(path):
    sats = []
    with open(path) as f:
        lines = f.readlines()
    for i in range(0, len(lines), 2):
        l1 = lines[i].strip()
        l2 = lines[i+1].strip()
        sat = EarthSatellite(l1, l2, 'ISS', ts)
        sats.append((sat.epoch.tt, sat))
    sats.sort(key=lambda x: x[0])
    epochs = [e for e, _ in sats]
    return epochs, sats


tle_epochs, tle_sats = load_iss_tles('ISS_2024-01-01_to_2025-01-17.tle')


def get_sat_for_time(t):
    idx = bisect_right(tle_epochs, t.tt) - 1
    return tle_sats[max(idx, 0)][1]


# Solar beta angle

def beta_angle(t):
    sat = get_sat_for_time(t)
    geo = sat.at(t)
    r = geo.position.km
    v = geo.velocity.km_per_s

    h = np.cross(r, v)
    n_orbit = h / LA.norm(h)

    # Sun position is already Earth-centered in the ephemeris
    sun_vec = sun.at(t).position.km - r
    sun_hat = sun_vec / LA.norm(sun_vec)

    beta = 90.0 - np.degrees(np.arccos(np.dot(n_orbit, sun_hat)))
    return beta


# --- Packet processing ---
f = open('plsb_packets.pkl', 'rb')
gcomm_packets = pickle.load(f)

for packetnum, g in enumerate(gcomm_packets):
    print(packetnum, datetime.datetime.fromtimestamp(g.time), g if g.packet is None else str(g.packet)[:75])
    if g.packet is None:
        continue
    data = g.packet.payload
    if type(data) is not bytes:
        continue

    x = np.frombuffer(data, dtype='uint16').astype('float')
    assert len(x) == 1536, "Bad data length"

    v1, i1, v2, i2, v3, i3 = np.split(x, 6)

    adc_steps = 2**12

    v1 *= (3.3 / adc_steps)
    i1 *= (3.3 / adc_steps) / 9.2
    v2 *= (3.3 / adc_steps)
    i2 *= (3.3 / adc_steps) / 9.2
    v3 *= (3.3 / adc_steps)
    i3 *= (3.3 / adc_steps) / 9.2

    def sortpair(v, i):
        inds = v.argsort()
        return v[inds], i[inds]

    v1, i1 = sortpair(v1, i1)
    v2, i2 = sortpair(v2, i2)
    v3, i3 = sortpair(v3, i3)

    p1ind = np.argmax(v1 * i1)
    mv1, mi1 = v1[p1ind], i1[p1ind]
    p2ind = np.argmax(v2 * i2)
    mv2, mi2 = v2[p2ind], i2[p2ind]
    p3ind = np.argmax(v3 * i3)
    mv3, mi3 = v3[p3ind], i3[p3ind]

    plt.close('all')
    plt.figure(figsize=(12, 6))

    a = plt.subplot(3, 1, 1)
    plt.grid(True)
    plt.plot(v1, i1)
    plt.plot(mv1, mi1, 'r*')
    a.annotate(f'P Max = {mv1 * mi1:.2f}W', (mv1, mi1))
    a.set_ylabel('Current (amps)')

    a = plt.subplot(3, 1, 2)
    plt.grid(True)
    plt.plot(v2, i2)
    plt.plot(mv2, mi2, 'r*')
    a.annotate(f'P Max = {mv2 * mi2:.2f}W', (mv2, mi2))
    a.set_ylabel('Current (amps)')

    a = plt.subplot(3, 1, 3)
    plt.grid(True)
    plt.plot(v3, i3)
    plt.plot(mv3, mi3, 'r*')
    a.annotate(f'P Max = {mv3 * mi3:.2f}W', (mv3, mi3))
    a.set_ylabel('Current (amps)')
    a.set_xlabel('Voltage (volts)')

    t = ts.utc(datetime.datetime.fromtimestamp(g.time, datetime.UTC))
    beta = beta_angle(t)

    plt.suptitle(datetime.datetime.fromtimestamp(g.time).strftime('%Y-%m-%dT%H:%M:%S') + f"   |   Solar Beta = {beta:.2f}°")

    plt.grid(which='minor')
    plt.tight_layout()
    plt.savefig(f'/tmp/iv_{packetnum}_chat.png')
