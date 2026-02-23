#!/usr/bin/env python3

#!/usr/bin/env python3
# Plot actual data downlinked 2024-12

import pickle
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from math import pi, sin, cos, sqrt
from astropy.coordinates import get_sun
from astropy.time import Time

def jday(year, mon, day, hr, minute, sec):
    jd = (367.0 * year
         - 7 * (year + ((mon + 9) // 12.0)) * 0.25 // 1.0
         + 275 * mon / 9.0 // 1.0
         + day
         + 1721013.5)
    fr = (sec + minute * 60.0 + hr * 3600.0) / 86400.0;
    return jd, fr

def days2mdhms(year, days, round_to_microsecond=6):
    second = days * 86400.0
    if round_to_microsecond:
        second = round(second, round_to_microsecond)

    minute, second = divmod(second, 60.0)
    if round_to_microsecond:
        second = round(second, round_to_microsecond)

    minute = int(minute)
    hour, minute = divmod(minute, 60)
    day_of_year, hour = divmod(hour, 24)

    is_leap = year % 400 == 0 or (year % 4 == 0 and year % 100 != 0)
    month, day = _day_of_year_to_month_day(day_of_year, is_leap)
    if month == 13:  # behave like the original in case of overflow
        month = 12
        day += 31

    return month, day, int(hour), int(minute), second

def _day_of_year_to_month_day(day_of_year, is_leap):
    february_bump = (2 - is_leap) * (day_of_year >= 60 + is_leap)
    august = day_of_year >= 215
    month, day = divmod(2 * (day_of_year - 1 + 30 * august + february_bump), 61)
    month += 1 - august
    day //= 2
    day += 1
    return month, day

# Load TLE data
with open('ISS_2024-01-01_to_2025-01-17.tle', 'r') as f:
    lines = f.readlines()

tle_list = []
i = 0
while i < len(lines) - 1:
    line1 = lines[i].strip()
    line2 = lines[i + 1].strip()
    if line1.startswith('1 ') and line2.startswith('2 '):
        # Parse epoch
        two_digit_year = int(line1[18:20])
        year = 2000 + two_digit_year if two_digit_year < 57 else 1900 + two_digit_year
        epochdays = float(line1[20:32])
        mon, day, hr, mn, sc = days2mdhms(year, epochdays)
        jd, fr = jday(year, mon, day, hr, mn, sc)
        epoch_jd = jd + fr

        # Parse elements
        incl = float(line2[8:16])
        nodeo = float(line2[17:25])
        ecc_str = line2[26:33].replace(' ', '0')
        ecc = float('0.' + ecc_str)
        no = float(line2[52:63])  # mean motion rev/day

        tle_list.append({
            'jd': epoch_jd,
            'incl': incl,
            'nodeo': nodeo,
            'ecc': ecc,
            'no': no
        })
    i += 2

f = open('plsb_packets.pkl', 'rb')

gcomm_packets = pickle.load(f)

for packetnum, g in enumerate(gcomm_packets):
    print(packetnum, datetime.fromtimestamp(g.time), g if g.packet is None else str(g.packet)[:75])
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
        """Sort voltage and current by voltage"""
        inds = v.argsort()
        return v[inds], i[inds]

    v1, i1 = sortpair(v1, i1)
    v2, i2 = sortpair(v2, i2)
    v3, i3 = sortpair(v3, i3)

    # compute max powerpoint index
    p1ind = np.argmax(v1 * i1)
    mv1, mi1 = v1[p1ind], i1[p1ind]
    p2ind = np.argmax(v2 * i2)
    mv2, mi2 = v2[p2ind], i2[p2ind]
    p3ind = np.argmax(v3 * i3)
    mv3, mi3 = v3[p3ind], i3[p3ind]

    # Compute beta angle
    dt = datetime.fromtimestamp(g.time)
    t = Time(dt)
    target_jd = t.jd

    # Find closest TLE
    diffs = [abs(target_jd - tle['jd']) for tle in tle_list]
    min_idx = np.argmin(diffs)
    tle = tle_list[min_idx]

    t_days = target_jd - tle['jd']

    # Constants
    mu = 398600.4418  # km^3/s^2
    re = 6378.137  # km
    j2 = 0.00108262668

    # Compute nodal precession rate
    no = tle['no']  # rev/day
    n = no * 2 * pi / 86400  # rad/s
    a = (mu / n**2)**(1/3.0)  # km
    p = a * (1 - tle['ecc']**2)  # km
    cos_i = cos(tle['incl'] * pi / 180)
    dot_omega = -1.5 * j2 * (re / p)**2 * no * cos_i  # rev/day

    # Propagate node
    node = tle['nodeo'] + dot_omega * t_days  # degrees

    node_rad = node * pi / 180
    incl_rad = tle['incl'] * pi / 180

    h_unit = [
        sin(incl_rad) * sin(node_rad),
        -sin(incl_rad) * cos(node_rad),
        cos(incl_rad)
    ]
    norm = sqrt(sum(x**2 for x in h_unit))
    h_unit = [x / norm for x in h_unit]

    # Get sun position
    sun = get_sun(t)
    sun_xyz = sun.cartesian.xyz.value
    sun_norm = np.linalg.norm(sun_xyz)
    unit_s = sun_xyz / sun_norm

    # Compute dot product
    dot_prod = sum(unit_s[k] * h_unit[k] for k in range(3))

    # Beta angle in degrees
    beta = np.arcsin(dot_prod) * 180 / pi

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

    plt.suptitle(f"{dt.strftime('%Y-%m-%dT%H:%M:%S')}, Beta angle = {beta:.2f} deg")

    plt.grid(which='minor')
    plt.tight_layout()
    plt.savefig(f'/tmp/iv_{packetnum}_grok.png')