import rospy
from nav_msgs.msg import Odometry  # replace with the actual message type of pose_gt and mpc_reference topics
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

real_x, real_y, real_z = [], [], []
ref_x, ref_y, ref_z = [], [], []
gt_start = False
ref_start = False
def gt_callback(data):
    global real_x, real_y, real_z
    global gt_start
    gt_start = True
    real_x.append(data.pose.pose.position.x)
    real_y.append(data.pose.pose.position.y)
    real_z.append(data.pose.pose.position.z)
    
def ref_callback(data):
    global ref_start
    ref_start = True
    global ref_x, ref_y, ref_z
    ref_x.append(data.pose.pose.position.x)
    ref_y.append(data.pose.pose.position.y)
    ref_z.append(data.pose.pose.position.z)
    
rospy.init_node('plotter', anonymous=True)
rospy.Subscriber('/bluerov2/pose_gt', Odometry, gt_callback)
rospy.Subscriber('/bluerov2/mpc/reference', Odometry, ref_callback)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim([10, 25])
ax.set_ylim([0, 15])
ax.set_zlim([-35, -25])
while not rospy.is_shutdown():
    if gt_start==True and ref_start==True:
        ax.plot(real_x, real_y, real_z, label='real', color='blue')
        ax.plot(ref_x, ref_y, ref_z, label='reference', color='yellow')
        ax.legend()
        plt.draw()
        plt.pause(0.05)

plt.show()