#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np

sample_time = 0.025             # seconds
duration = 120                   # seconds

# Trajectory
traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4

traj[:,0] = 0                       # x
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
traj[:,14] = 0                    # u3: set to overcome bouyant
traj[:,15] = 0                      # u4

# write to txt
np.savetxt('stationary.txt',traj,fmt='%f')