#--------------------------------------------------------
#Generate reference trajectory for bridge inspection task
#--------------------------------------------------------

import numpy as np
import math
import numpy.matlib
# Parameters
sample_time = 0.05                 #seconds
v = 1.2
turn_v = 0.3

pier = np.array([[17.74002,5.259887],[21.34002,5.259887],[21.34002,8.97006],[17.74002,8.97006]])
dis = 1.5
camera_view = float(32*math.pi/180)
depth_interval = dis*math.tan(camera_view)*2
print(depth_interval)
cycles = int(35/(dis*math.tan(camera_view)*2))
print(cycles)
#cycles = 19
cycles = 1
depth_interval = 1.8
isRounded = True

if isRounded == True:
    points = np.array([[pier[0][0],pier[0][1]-dis],[pier[1][0],pier[1][1]-dis],
                       [pier[1][0]+dis,pier[1][1]],[pier[2][0]+dis,pier[2][1]],
                       [pier[2][0],pier[2][1]+dis],[pier[3][0],pier[3][1]+dis],
                       [pier[3][0]-dis,pier[3][1]],[pier[0][0]-dis,pier[0][1]],
                       [pier[0][0],pier[0][1]-dis]])
else:
    points = np.array([[pier[0][0]-dis,pier[0][1]-dis],[pier[1][0]+dis,pier[1][1]-dis],
                       [pier[2][0]+dis,pier[2][1]+dis],[pier[3][0]-dis,pier[3][1]+dis],
                       [pier[0][0]-dis,pier[0][1]-dis],[pier[0][0],pier[0][1]-dis]])
print(points)

# Initialize trajectory with vehicle initial position and heading
traj = np.zeros((1,16))
traj[0,0] = 10.5
traj[0,1] = 2.5
traj[0,2] = -34.5
traj[0,5] = 0
'''
traj0 = traj
for i in range(1,1200):
    traj = np.append(traj,traj0,axis=0) 
'''
# original position to starting point
d = np.abs(np.sqrt((points[0][0]-traj[0,0])**2 + (points[0][1]-traj[0,1])**2))
t = int(d/v/sample_time)+1
local_path = np.zeros((t+1,16))
x = np.linspace(traj[0,0],points[0][0],t+1)
y = np.linspace(traj[0,1],points[0][1],t+1)
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
#cycle_start = t+1+1199
cycle_start = t+1
print(cycle_start)

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
            x = np.linspace(points[i-1][0],points[i][0],t+1)
        else:
            x = np.zeros(t+1)
            x[:] = points[i][0]

        # y coordinates
        if points[i][1] - points[i-1][1] != 0:         
            y = np.linspace(points[i-1][1],points[i][1],t+1)
        else:
            y = np.zeros(t+1)
            y[:] = points[i][1]
        # yaw angle
        yaw = np.zeros(t+1)
        yaw[:] = 0.5*math.pi + int(i/2)*0.5*math.pi
        if isRounded==False:
            yaw[:] = 0.5*math.pi*i
            '''
            if i==1:
                for j in range(t+1):
                    if x[j]>pier[1][0]:  
                        index = j
                        print(index)
                        break
                yaw[index:t+1] = np.linspace(0.5*math.pi,0.75*math.pi,t+1-index)
            if i==2:
                for j in range(t+1):
                    if y[j]>pier[1][1]:
                        index = j
                        break
                yaw[0:index] = np.linspace(0.75*math.pi,math.pi,index)

                for j in range(t+1):
                    if y[j]>pier[2][1]:
                        index = j
                        break
                yaw[index:t+1] = np.linspace(math.pi,1.25*math.pi,t+1-index)
            if i==3:
                for j in range(t+1):
                    if x[j]<pier[2][0]:
                        index = j
                        break
                yaw[0:index] = np.linspace(1.25*math.pi,1.5*math.pi,index)
                for j in range(t+1):
                    if x[j]<pier[3][0]:
                        index = j
                        break
                yaw[index:t+1] = np.linspace(1.5*math.pi,1.75*math.pi,t+1-index)
            if i==4:
                for j in range(t+1):
                    if y[j]<pier[3][1]:
                        index = j
                        break
                yaw[0:index] = np.linspace(1.75*math.pi,2*math.pi,index)
                for j in range(t+1):
                    if y[j]<pier[0][1]:
                        index = j
                        break
                yaw[index:t+1] = np.linspace(2*math.pi,2.25*math.pi,t+1-index)
            if i==5:
                yaw[0:t+1] = np.linspace(2.25*math.pi,2.5*math.pi,t+1)
            '''
    # Generate rounded angle
    else:
        pier_no = (i-2)/2+1
        if pier_no > 3:
            pier_no = pier_no - 4
        print("pier no.")
        print(pier_no)
       
        d = 2*math.pi*dis/4
        time = d/turn_v
        timestep = np.linspace(0,time,d/turn_v/sample_time)
        angular_position = timestep*(turn_v/dis)
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

cycle_end = np.size(traj)/16-1
print(traj[cycle_end,:])
# repeat trajectory at difference depth
print(traj[cycle_start,:])

for i in range(1,cycles+1):
    depth = traj[0,2] + i*depth_interval
    repeat_matrix = np.zeros((cycle_end+1-cycle_start,16))
    repeat_matrix[:,0] = traj[cycle_start:cycle_end+1,0]
    repeat_matrix[:,1] = traj[cycle_start:cycle_end+1,1]
    repeat_matrix[:,2] = depth
    repeat_matrix[:,5] = traj[cycle_start:cycle_end+1,5] + i*math.pi*2

    t = int(depth_interval/v/sample_time)+1
    local_path = np.zeros((t+1,16))
    local_path[:,0] = traj[cycle_start,0]
    local_path[:,1] = traj[cycle_start,1]
    d = np.linspace(traj[-1,2],depth,t+1)
    local_path[:,2] = d
    local_path[:,5] = repeat_matrix[0,5]
    traj = np.append(traj,local_path,axis=0)
    traj = np.append(traj,repeat_matrix,axis=0)


# write to txt
np.savetxt('bridge_path.txt',traj,fmt='%f')

