#--------------------------------------------------------
#Generate reference trajectory for bridge inspection task
#--------------------------------------------------------

import numpy as np
import math
import numpy.matlib
# Parameters
sample_time = 0.05                 #seconds
v = 1
angular_v = 0.5
cycles = 12
depth_interval = 2.85

#points_matrix = np.array([[0,0,-34.5],[16,3.5,-34.5],[23,3.5,-34.5],[23,10.7,-34.5],[16,10.7,-34.5],[16,3.5,-34.5]])
pier = np.array([[17.74002,5.259887],[21.34002,5.259887],[21.34002,8.97006],[17.74002,8.97006]])
dis = 1

isRounded = False

if isRounded == True:
    points = np.array([[pier[0][0],pier[0][1]-dis],[pier[1][0],pier[1][1]-dis],
                       [pier[1][0]+dis,pier[1][1]],[pier[2][0]+dis,pier[2][1]],
                       [pier[2][0],pier[2][1]+dis],[pier[3][0],pier[3][1]+dis],
                       [pier[3][0]-dis,pier[3][1]],[pier[0][0]-dis,pier[0][1]],
                       [pier[0][0],pier[0][1]-dis]])
else:
    points = np.array([[pier[0][0]-dis,pier[0][1]-dis],[pier[1][0]+dis,pier[1][1]-dis],
                       [pier[2][0]+dis,pier[2][1]+dis],[pier[3][0]-dis,pier[3][1]+dis],
                       [pier[0][0]-dis,pier[0][1]-dis]])
print(points)

# Initialize trajectory with vehicle initial position and heading
traj = np.zeros((1,16))
traj[0,0] = 0
traj[0,1] = 0
traj[0,2] = -34.5
traj[0,5] = 0


# original position to starting point
d = np.abs(np.sqrt((points[0][0]-traj[0,0])**2 + (points[0][1]-traj[0,1])**2))
t = int(d/v/sample_time)+1
local_path = np.zeros((t+1,16))
x_interval = (points[0][0] - traj[0,0])/t
x = np.arange(traj[0,0],points[0][0], x_interval)
x = np.append(x,points[0][0])
y_interval = (points[0][1] - traj[0,1])/t
y = np.arange(traj[0,1],points[0][1], y_interval)
y = np.append(y,points[0][1])
yaw = np.zeros(t+1)
angle = math.atan((points[0][1]-traj[0,1])/(points[0][0]-traj[0,0]))
yaw[:] = angle
yaw_turn = np.linspace(angle, 0.5*math.pi, 100)
yaw_turn = np.append(yaw_turn, 0.5*math.pi)
yaw[np.size(yaw)-np.size(yaw_turn):np.size(yaw)] = yaw_turn
local_path[:,0] = x
local_path[:,1] = y
local_path[:,5] = yaw
traj = np.append(traj,local_path,axis=0) 


# Generate trajectory based on velocity
for i in range(1,np.size(points)/2):
    if points[i][0]!=points[i-1][0] and points[i][1]!=points[i-1][1]:
        isStraight = False
    else:
        isStraight = True
    print(i)
    print(isStraight)

    # Generate straight path
    if isStraight==True:
        d = np.abs(np.sqrt((points[i][0]-points[i-1][0])**2 + (points[i][1]-points[i-1][1])**2))
        t = int(d/v/sample_time)+1
        local_path = np.zeros((t+1,16))
        # x coodinates
        if points[i][0] - points[i-1][0] != 0:
            x_interval = (points[i][0] - points[i-1][0])/t
            x = np.arange(points[i-1][0],points[i][0], x_interval)
            x = np.append(x,points[i][0])
        else:
            x = np.zeros(t+1)
            x[:] = points[i][0]

        # y coordinates
        if points[i][1] - points[i-1][1] != 0:
            y_interval = (points[i][1] - points[i-1][1])/t
            y = np.arange(points[i-1][1],points[i][1], y_interval)
            y = np.append(y,points[i][1])
        else:
            y = np.zeros(t+1)
            y[:] = points[i][1]
        # yaw angle
        yaw = np.zeros(t+1)
        yaw[:] = 0.5*math.pi + int(i/2)*0.5*math.pi

    # Generate rounded angle
    else:
        pier_no = (i-2)/2+1
        if pier_no > 3:
            pier_no = pier_no - 4
        print("pier no.")
        print(pier_no)
       
        d = 2*math.pi*dis/4
        time = d/angular_v
        timestep = np.linspace(0,time,d/angular_v/sample_time)
        angular_position = timestep*(angular_v/dis)
        # x, y coordinates
        if points[i][0]>points[i-1][0] and points[i][1]>points[i-1][1]:
            x = pier[pier_no][0] + dis*np.cos(angular_position)[::-1]
            y = pier[pier_no][1] - dis*np.sin(angular_position)[::-1]
            
        elif points[i][0]<points[i-1][0] and points[i][1]>points[i-1][1]:
            x = pier[pier_no][0] + dis*np.cos(angular_position)
            y = pier[pier_no][1] + dis*np.sin(angular_position)
            
        elif points[i][0]<points[i-1][0] and points[i][1]<points[i-1][1]:
            x = pier[pier_no][0] - dis*np.cos(angular_position)[::-1]
            y = pier[pier_no][1] + dis*np.sin(angular_position)[::-1]
            
        else:
            #print(pier[pier_no])
            x = pier[pier_no][0] - dis*np.cos(angular_position)
            y = pier[pier_no][1] - dis*np.sin(angular_position)
        # yaw angle
        yaw_pre = local_path[-1,5]
        yaw = np.zeros(np.size(timestep))
        yaw[:] = np.linspace(yaw_pre,yaw_pre+0.5*math.pi,np.size(timestep)) 


        local_path = np.zeros((np.size(timestep),16))
        
    local_path[:,0] = x
    local_path[:,1] = y
    local_path[:,5] = yaw
    traj = np.append(traj,local_path,axis=0) 

traj[:,2] = -34.5

if isRounded==False:
    for i in range(1,np.size(points)/2):
        pier_no = (i-2)/2+1
        if pier_no > 3:
            pier_no = pier_no - 4
        
'''
# Generate trajectory based on velocity
for i in range(1,np.size(points_matrix)/3):
    # calculate absolute distance between points
    d = np.abs(np.sqrt((points_matrix[i][0]-points_matrix[i-1][0])**2 + (points_matrix[i][1]-points_matrix[i-1][1])**2))
    t = int(d/v/sample_time)+1
    local_path = np.zeros((t+1,16))
    
    #local_path[:,2] = -34.5 + (i-1)*depth_interval
    # x trajectory
    if points_matrix[i][0] - points_matrix[i-1][0] != 0:
        x_interval = (points_matrix[i][0] - points_matrix[i-1][0])/t
        x = np.arange(points_matrix[i-1][0],points_matrix[i][0], x_interval)
        x = np.append(x,points_matrix[i][0])
    else:
        x = np.zeros(t+1)
        x[:] = points_matrix[i][0]

    # y trajectory
    if points_matrix[i][1] - points_matrix[i-1][1] != 0:
        y_interval = (points_matrix[i][1] - points_matrix[i-1][1])/t
        y = np.arange(points_matrix[i-1][1],points_matrix[i][1], y_interval)
        y = np.append(y,points_matrix[i][1])
    else:
        y = np.zeros(t+1)
        y[:] = points_matrix[i][1]
    
    # yaw degree
    yaw = np.zeros(t+1)
    if points_matrix[i][0]!=points_matrix[i-1][0] and points_matrix[i][1]!=points_matrix[i-1][1]:
        #initial_yaw = math.atan((points_matrix[1][1]-points_matrix[0][1])/(points_matrix[1][0]-points_matrix[0][0]))
        yaw[:] = math.atan((points_matrix[1][1]-points_matrix[0][1])/(points_matrix[1][0]-points_matrix[0][0]))
    else:
        yaw[:] = 0.5*math.pi*(i-1)
   
    local_path[:,0] = x
    local_path[:,1] = y
    local_path[:,2] = -34.5
    local_path[:,5] = yaw
    traj = np.append(traj,local_path,axis=0)
'''
# write to txt
np.savetxt('bridge_path.txt',traj,fmt='%f')

