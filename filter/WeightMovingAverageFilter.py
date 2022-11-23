#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from scipy import io

input_mat = io.loadmat('./SonarAlt.mat')

def get_sonar(i):
    """Measure sonar."""
    z = input_mat['sonarAlt'][0][i]  # input_mat['sonaralt']: (1, 1501)
    return z

def Weight_Moving_Average_Filter(x_n, x_meas):
    """Calculate average sonar using a Weight_Moving_Average_Filter."""
    n = len(x_n)
    for i in range(n-1):
        x_n[i] = x_n[i+1]
    x_n[n-1] = x_meas

    for j in range(n-1):
		x_n[i] = x_n[i] * (i+1)
    x_avg = np.mean(x_n)
    return x_meas, x_n

# Input parameters.
n = 6
n_samples = 500
time_end = 10

dt = 0.02
time = np.arange(0, time_end, dt)
x_meas_save = np.zeros(n_samples)
x_esti_save = np.zeros(n_samples)

x_esti = None

for i in range(n_samples):
    x_meas = get_sonar(i)
    if i == 0:
        x_esti, x_n = x_meas, x_meas * np.ones(n)
    else:
        x_esti, w_n = Weight_Moving_Average_Filter(x_n, x_meas)

    x_meas_save[i] = x_meas
    x_esti_save[i] = x_esti

# plot
plt.plot(time, x_meas_save, 'r*', label='Measured')
plt.plot(time, x_esti_save, 'b-', label='Weight_Moving_Average_Filter')
plt.legend(loc='upper left')
plt.title('Measured Altitudes v.s. LPF Values')
plt.xlabel('Time [sec]')
plt.ylabel('Altitude [m]')
plt.show()
