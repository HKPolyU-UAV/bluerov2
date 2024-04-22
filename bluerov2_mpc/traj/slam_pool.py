import numpy as np
import math

# Parameters
duration = 1750  # seconds
sample_time = 0.05  # seconds
v = 1.5

x0 = 10
y0 = 20
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
traj[:, 5] = -np.pi/2

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

# Trajectory Definition
# 1) move from (10,20) to (10,-20)
# 2) move from (10,-20) to (5,-20)
# 3) move from (5,-20) to (5,20)
# 4) move from (5,20) to (0,20)
for i in range(1, len(t)):
    if t[i] < 40:
        traj[i, 0] = x0
        traj[i, 1] = y0 - 1*t[i]
        traj[i, 5] = -np.pi/2       # Face the negative y direction
    elif t[i] < 45:
        traj[i, 0] = x0 - 1*(t[i]-40)
        traj[i, 1] = y0 - 40
        traj[i, 5] = -np.pi              # Face the negative x direction
    elif t[i] < 85:
        traj[i, 0] = x0 - 5
        traj[i, 1] = y0 - 40 + 1*(t[i]-45)
        traj[i, 5] = -3*np.pi/2              # Face the positive y direction
    elif t[i] < 90:
        traj[i, 0] = x0 - 5 - 1*(t[i]-85)
        traj[i, 1] = y0
        traj[i, 5] = -np.pi              # Face the negative x direction
    elif t[i] < 130:
        traj[i,0] = x0 - 10
        traj[i,1] = y0 - 1*(t[i]-90)
        traj[i, 5] = -np.pi/2              # Face the negative y direction
    elif t[i] < 135:
        traj[i,0]=x0-10-1*(t[i]-130)
        traj[i,1]=y0-40
        traj[i,5]=-np.pi              # Face the negative x direction
    elif t[i] < 175:
        traj[i,0]=x0-15
        traj[i,1]=y0-40+1*(t[i]-135)
        traj[i,5]=-3*np.pi/2              # Face the positive y direction
    else:
        traj[i,0]=-5
        traj[i,1]=20
        traj[i,5]=-3*np.pi/2

# If the loop ended before t[i] >= 1750, set the remaining elements to (-5, 20)
if i <= len(t) - 1:
    traj[i+1:,0] = -5
    traj[i+1:,1] = 20
    traj[i,5]=-3*np.pi/2

# Save the trajectory to a txt file
np.savetxt('slam_pool.txt', traj, fmt='%f')

