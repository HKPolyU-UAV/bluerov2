import numpy as np
import math

# Parameters
duration = 60  # seconds
sample_time = 0.05  # seconds
v = 1.5
r = 10
x0 = 0
y0 = 0
z0 = -95

# defint time t
t = np.arange(0,duration,sample_time)
print('t:',t)
print('len(t):',len(t))

traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4

# Set the initial conditions
traj[:, 0] = x0
traj[:, 1] = y0
traj[:, 2] = z0

# Set the initial orientation
traj[:, 3] = 0
traj[:, 4] = 0
traj[:, 5] = 0

# Set the initial velocity
traj[:, 6] = 0
traj[:, 7] = 0
traj[:, 8] = 0

# Set the initial angular velocity
traj[:, 9] = 0
traj[:, 10] = 0
traj[:, 11] = 0

# Set the initial control inputs
traj[:, 12] = 0
traj[:, 13] = 0
traj[:, 14] = 0
traj[:, 15] = 0

for i in range(1, len(t)):
    # Define the parametric equations for the "8" trajectory
    traj[i, 0] = x0 + r * np.sin(2 * np.pi * t[i] / duration)  # x-coordinate
    traj[i, 1] = y0 + r * np.sin(4 * np.pi * t[i] / duration)  # y-coordinate
    traj[i, 5] = np.arctan2(  # yaw angle (orientation)
        4 * np.pi * r * np.cos(4 * np.pi * t[i] / duration) / duration,
        2 * np.pi * r * np.cos(2 * np.pi * t[i] / duration) / duration
    )

# Save the trajectory to a txt file
np.savetxt('slam_pool_eight_v2.txt', traj, fmt='%f')

