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
while(True):
# for i in range(int(10*200)):
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
