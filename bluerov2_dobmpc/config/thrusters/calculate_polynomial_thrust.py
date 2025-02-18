import numpy as np


file = "T200-Public-Performance-Data-10-20V-September-2019.csv"

data = np.genfromtxt(file, dtype=float, delimiter=",", skip_header=1)
gain = np.linspace(-1000,1000, data.shape[0], dtype=float)
gainz = np.linspace(0, 1000, data.shape[0], dtype=float)
# map from kg-force to newton
N = data[:,5] * 9.80665

tau_x = 2*np.linspace(-N.max(),N.max(),data.shape[0])
tx_polys = np.polyfit(tau_x, gain, 4)
ty_polys = tx_polys.copy()
tz_polys = np.polyfit(tau_x, gainz, 4)
arm = np.sqrt(0.14**2+0.1**2)  # approximate arm length of thruster
Mz = 4 * np.linspace(-N.max(), N.max(),data.shape[0]) * arm
tr_polys = np.polyfit(Mz, gain, 4)
