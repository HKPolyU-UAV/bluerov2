#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np
import math

sample_time = 0.025             # seconds
duration = 120                   # seconds
v = 0.8

# pier coordinate
pier = np.array([[17.74002,5.259887],[21.34002,5.259887],[21.34002,8.97006],[17.74002,8.97006]])
dis = 2
uuv_width = 0.2384

# Trajectory
traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4

traj[:,0] = pier[0][0]+uuv_width    # x
traj[:,1] = pier[0][1]-dis          # y
traj[:,2] = -34.5                   # z
traj[:,3] = 0                       # phi
traj[:,4] = 0                       # theta
traj[:,5] = 0.5*math.pi                       # psi
traj[:,6] = 0                       # u
traj[:,7] = 0                       # v
traj[:,8] = 0                       # w
traj[:,9] = 0                       # p
traj[:,10] = 0                      # q
traj[:,11] = 0                      # r
traj[:,12] = 0                      # u1
traj[:,13] = 0                      # u2
traj[:,14] = 0                      # u3: set to overcome bouyant
traj[:,15] = 0                      # u4

# for i in range(0,int(duration/sample_time+1)):
#     traj[i,0] = i*v*sample_time+traj[0,0]

# write to txt
np.savetxt('bridge_vision.txt',traj,fmt='%f')