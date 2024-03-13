#-------------------------------------------------------
#Generate reference trajectory for system identification
#-------------------------------------------------------

import numpy as np
import math

# Parameters
sample_time = 0.05             
duration = 80                   # seconds
max_forward_speed = 1.5

v = 1.5

r = 1.5
x0 = 0
y0 = 0
z0 = -20

# trajectory
traj = np.zeros((int(duration/sample_time+1),16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4
t = np.arange(0,duration,sample_time)
t = np.append(t, duration)

# # Define the number of samples for one complete sine wave
# num_samples_per_wave = int(2 * np.pi / (max_forward_speed * sample_time))

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
traj[:,14] = 57.5                   # u3
traj[:,15] = 0                      # u4

for i in range(0,int(duration/sample_time+1)):
    # X
    # if i <= 50:
    #     traj[i,0] = 0.075*i
    # if i > 50 <= 100:
    #     traj[i,0] = 3.75-0.075*(i-50)
    # if i > 100 <= 150:
    #     traj[i,0] = 0.05*(i-100)
    # if i > 150 <= 200:
    #     traj[i,0] = 2.5-0.05*(i-150)

    if i <= 200:
        traj[i, 0] = 2 * np.sin(2 * np.pi * i / 200)
    if i > 200:
        traj[i,0] = 0
    

for i in range(0,int(duration/sample_time+1)):
    # Y
    # if i > 200 <= 250:
    #     traj[i,1] = 0.075*(i-200)
    # if i > 250 <= 300:
    #     traj[i,1] = 3.75-0.075*(i-250)
    # if i > 300 <= 350:
    #     traj[i,1] = 0.05*(i-300)
    # if i > 350 <= 400:
    #     traj[i,1] = 2.5-0.05*(i-350)
    if i > 200 <= 400:
        traj[i, 1] = 2 * np.sin(2 * np.pi * i / 200)
    if i > 400:
        traj[i,1] = 0

for i in range(0,int(duration/sample_time+1)):
    # Z
    # if i > 400 <= 450:
    #     traj[i,2] = -20 + 0.075*(i-400)
    # if i > 450 <= 500:
    #     traj[i,2] = -16.25 - 0.075*(i-450)
    # if i > 500 <= 550:
    #     traj[i,2] = -20 + 0.05*(i-500)
    # if i > 550 <= 600:
    #     traj[i,2] = -17.5 - 0.05*(i-550)

    if i > 400 <= 600:
        traj[i, 2] = -20 + 2 * np.sin(2 * np.pi * i / 200)
    if i > 600:
        traj[i,2] = -20

for i in range(0,int(duration/sample_time+1)):
    # N
    # if i > 600 <= 650:
    #     traj[i,5] = 0.025*(i-600)
    # if i > 650 <= 700:
    #     traj[i,5] = 1.75-0.025*(i-650)
    # if i > 700 <= 750:
    #     traj[i,5] = 0.02*(i-700)
    # if i > 750 <= 800:
    #     traj[i,5] = 1-0.02*(i-750)

    if i > 600 <= 800:
        traj[i, 5] = 2 * np.sin(2 * np.pi * i / 200)
    if i > 800:
        traj[i, 5] = 0



# write to txt
np.savetxt('systemID.txt',traj,fmt='%f')