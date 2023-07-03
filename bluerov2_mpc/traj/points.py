#--------------------------------------
#Generate reference trajectory for NMPC
#--------------------------------------

import numpy as np
import numpy.matlib
# Parameters
sample_time = 0.025                 #seconds
cycles = 5
step_interval = 5

points_matrix = np.array([[-0.5,-0.5,-20],[1.5,-0.5,-20],[1.5,1.5,-20],[-0.5,1.5,-20]])

# Trajectory
duration = cycles*np.size(points_matrix,0)*step_interval

traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

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
traj[:,14] = 0                      # u2
traj[:,15] = 0                      # u2
for i in range(1,cycles+1):
    for j in range(1,np.size(points_matrix,0)+1):
        traj_start = (i-1)*np.size(points_matrix,0)*step_interval+(j-1)*step_interval
        traj_end = (i-1)*np.size(points_matrix,0)*step_interval+j*step_interval
        traj[int(traj_start/sample_time):int(traj_end/sample_time),0:3] = np.tile(points_matrix[j-1,:],(int(step_interval/sample_time),1))

traj[-1,0:3] = traj[-2,0:3]

# write to txt
np.savetxt('points.txt',traj,fmt='%f')