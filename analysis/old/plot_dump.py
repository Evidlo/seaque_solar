#!/usr/bin/env python3

import pickle
import numpy as np
import matplotlib.pyplot as plt

f = open('/tmp/plsb.pkl', 'rb')

gcomm_packets = pickle.load(f)

for g in gcomm_packets:
    if g.packet is not None and type(data := g.packet.payload) is bytes:
        break

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

plt.subplot(3, 1, 1)
plt.plot(v1, i1)
plt.subplot(3, 1, 2)
plt.plot(v2, i2)
plt.subplot(3, 1, 3)
plt.plot(v3, i3)

plt.show()