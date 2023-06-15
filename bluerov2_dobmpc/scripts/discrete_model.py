import numpy as np
from scipy.linalg import expm

# Define the state variables
x = np.zeros((12, 1))
# Define the system parameters
M = np.zeros((6, 6))
C = np.zeros((6, 6))
D = np.zeros((6, 6))
K = np.zeros((6, 4))
g = np.zeros((6, 1))
# Define the control input
u = np.zeros((4, 1))
# Define the disturbance term
disturbance = np.zeros((12, 1))
# Define the operating point
x0 = np.zeros((12, 1))
u0 = np.zeros((4, 1))
# Define the sampling time
dt = 0.1

# Define the system model
f = np.dot(np.linalg.inv(M), (np.dot(K, u) + disturbance - np.dot(C, x[6:9]) - np.dot(D, np.concatenate((x[3:6], x[9:12]), axis=0)) - g))
# Linearize the system around the operating point
A = np.zeros((12, 12))
B = np.zeros((12, 4))
G = np.zeros((12, 1))
for i in range(12):
    x_temp = x.copy()
    x_temp[i] = x0[i]
    A[:, i] = (f - np.dot(np.linalg.inv(M), (np.dot(K, u) + disturbance - np.dot(C, x_temp[6:9]) - np.dot(D, np.concatenate((x_temp[3:6], x_temp[9:12]), axis=0)) - g))) / (x[i] - x0[i])
    B[:, i] = (f - np.dot(np.linalg.inv(M), (np.dot(K, u) + disturbance - np.dot(C, x[6:9]) - np.dot(D, np.concatenate((x[3:6], x[9:12]), axis=0)) - g))) / (u[i] - u0[i])
G = f - np.dot(A, x0) - np.dot(B, u0)
# Discretize the continuous-time model using the Euler integration method
Ad = expm(A*dt)
Bd = np.dot(np.linalg.inv(A), np.dot((expm(A*dt) - np.eye(12)), B))
Gd = np.dot(np.linalg.inv(A), np.dot((expm(A*dt) - np.eye(12)), G))
# Define the measurement equation
C = np.concatenate((np.eye(6), np.zeros((6, 6))), axis=1)
# Define the discrete-time state-space model
sys = (Ad, Bd, C, np.zeros((6, 4)), dt)