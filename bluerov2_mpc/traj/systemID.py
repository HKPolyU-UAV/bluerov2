#-------------------------------------------------------
#Generate reference trajectory for system identification
#-------------------------------------------------------

import numpy as np
import math

# Parameters
sample_time = 0.05             # seconds
duration = 80                   # seconds

v = 1.5

# trajectory
traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)


traj[:,1] = 0                       # y
traj[:,2] = -20                     # z
traj[:,3] = 0                       # phi
traj[:,4] = 0                       # theta
traj[:,5] = 0                       # psi
traj[:,6] = 0                       # u
traj[:,7] = 0                       # v
traj[:,8] = 0                       # w
traj[:,9] = 0                       # p
traj[:,10] = 0                      # q
traj[:,11] = 0                      # r
traj[:,12] = 0                      # u1
traj[:,13] = 0                      # u2
traj[:,14] = 57.5                   # u3
traj[:,15] = 0                      # u4

for i in range(0,int(duration/sample_time+1)):
    if i <= 400:
        traj[i,0] = 0.075*i
    if i > 400 <= 800:
        traj[i,0] = 30 - 0.075*(i-400)
    if i > 800 <=1200:
        traj[i,0] = 0.05*(i-800)
    if i > 1200 <=1600:
        traj[i,0] = 20 - 0.05*(i-1200)

# write to txt
np.savetxt('systemID.txt',traj,fmt='%f')