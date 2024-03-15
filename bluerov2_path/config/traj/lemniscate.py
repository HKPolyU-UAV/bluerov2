#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np

# Parameters
sample_time = 0.05                 #seconds
duration = 60;                      #seconds
amp = 2
frq = 0.5

x0 = 0
y0 = 0
z0 = -20

# Trajectory
traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

traj[:,0] = amp*np.cos(t*frq)+x0    # x
traj[:,1] = amp*np.sin(t*frq)*np.cos(t*frq)+y0     # y
traj[:,2] = z0                      # z
traj[:,3] = 0                       # phi
traj[:,4] = 0                       # theta
traj[:,5] = 0                      # psi
traj[:,6] = -amp*frq*np.sin(t*frq)  # u
traj[:,7] = amp*frq*np.cos(t*2*frq) # v
traj[:,8] = 0                       # w
traj[:,9] = 0                       # p
traj[:,10] = 0                      # q
traj[:,11] = 0                      # r
traj[:,12] = 0                      # u1
traj[:,13] = 0                      # u2
traj[:,14] = 0                      # u2
traj[:,15] = 0                      # u2
# write to txt
np.savetxt('lemniscate.txt',traj,fmt='%f')