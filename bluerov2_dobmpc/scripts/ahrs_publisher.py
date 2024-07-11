import rospy
from std_msgs.msg import String
import serial
import struct
import numpy as np
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

def ahrs_publisher():
    # Configure the serial connection
    ser = serial.Serial('/dev/ttyUSB0',115200)
    state=0
    buffer=b''
    lasttime=0
    accList=[]
    angList=[]
    angVelList=[]

    # Initialize ROS node
    rospy.init_node('ahrs_publisher_node', anonymous=True)

    # Create a publisher for IMU data
    imu_pub = rospy.Publisher('/ahrs_data', Imu, queue_size=10)

    # Create an Imu message object
    imu_msg = Imu()

    # Read and publish IMU data
    rate = rospy.Rate(200)  # Publish at 200 Hz
    while not rospy.is_shutdown():
        inb=ser.read()
        if inb.hex() == 'aa' and state==0:
            frame=ser.read(23)
            angRaw=frame[2:8]
            angVelRaw=frame[8:14]
            accRaw=frame[14:20]
            ang=np.array(struct.unpack('>3h',angRaw))*0.0055*(3.14159/180.0)
            angVel=np.array(struct.unpack('>3h',angVelRaw))*0.0175
            acc=np.array(struct.unpack('>3h',accRaw))*0.004785
            accList.append(acc)
            angList.append(ang)
            angVelList.append(angVel)
            dt=(time.time_ns()-lasttime)*1e-9
            lasttime=time.time_ns()
            print(np.round(ang),np.round(angVel),np.round(acc))
        
            imu_msg.header.stamp = rospy.Time.now()
            quat = quaternion_from_euler(ang[0],ang[1],ang[2])
            imu_msg.orientation.w = quat[3]
            imu_msg.orientation.x = quat[0]
            imu_msg.orientation.y = quat[1]
            imu_msg.orientation.z = quat[2]
            imu_msg.linear_acceleration.x = acc[0]
            imu_msg.linear_acceleration.y = acc[1]
            imu_msg.linear_acceleration.z = acc[2]
            imu_msg.angular_velocity.x = angVel[0]
            imu_msg.angular_velocity.y = angVel[1]
            imu_msg.angular_velocity.z = angVel[2]

            imu_pub.publish(imu_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        ahrs_publisher()
    except rospy.ROSInterruptException:
        pass