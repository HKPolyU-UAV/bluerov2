import serial
import struct
import numpy as np
import time
ser = serial.Serial('/dev/ttyUSB0',115200)
state=0
buffer=b''
lasttime=0
accList=[]
angList=[]
angVelList=[]
# while(True):
for i in range(int(10*200)):
    inb=ser.read()
    # print(inb.hex(),end=' ')
    if inb.hex() == 'aa' and state==0:
        frame=ser.read(23)
        angRaw=frame[2:8]
        angVelRaw=frame[8:14]
        accRaw=frame[14:20]
        ang=np.array(struct.unpack('>3h',angRaw))*0.0055
        angVel=np.array(struct.unpack('>3h',angVelRaw))*0.0175
        acc=np.array(struct.unpack('>3h',accRaw))*0.004785
        accList.append(acc)
        angList.append(ang)
        angVelList.append(angVel)
        dt=(time.time_ns()-lasttime)*1e-9
        # print(1/dt)
        lasttime=time.time_ns()
        print(np.round(ang),np.round(angVel),np.round(acc))
        # print(round(accRaw_x),round(accRaw_y),round(accRaw_z))
        # input()
import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
accList=np.array(accList)
angList=np.array(angList)
angVelList=np.array(angVelList)
print(accList.shape)
plt.figure()
plt.plot(accList[:,0])
plt.plot(accList[:,1])
plt.plot(accList[:,2])
plt.title("acceleration") 
plt.figure()
plt.plot(angList[:,0])
plt.plot(angList[:,1])
plt.plot(angList[:,2])
plt.title("angle") 
plt.figure()
plt.plot(angVelList[:,0])
plt.plot(angVelList[:,1])
plt.plot(angVelList[:,2])
plt.title("angular velocity") 
# plt.figure()
plt.show()
# print(struct.unpack('>h',b'\x52\x09')[0])
# X  X  X  
# aa b5 15 01 da 00 45 00 00 ff fe ff fe 00 00 00 0c ff a3 f7 ff 0a ce 96