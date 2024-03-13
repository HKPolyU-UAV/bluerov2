import rospy
from nav_msgs.msg import Odometry  # replace with the actual message type of pose_gt and mpc_reference topics
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

real_x, real_y = [], []
ref_x, ref_y = [], []
gt_start = False
ref_start = False

def gt_callback(data):
    global real_x, real_y
    global gt_start
    gt_start = True
    real_x.append(data.pose.pose.position.x)
    real_y.append(data.pose.pose.position.y)
    
    
def ref_callback(data):
    global ref_start
    ref_start = True
    global ref_x, ref_y
    ref_x.append(data.pose.pose.position.x)
    ref_y.append(data.pose.pose.position.y)
    
    
rospy.init_node('plotter', anonymous=True)
rospy.Subscriber('/bluerov2/pose_gt', Odometry, gt_callback)
rospy.Subscriber('/bluerov2/mpc/reference', Odometry, ref_callback)

fig, ax = plt.subplots()

while not rospy.is_shutdown():
    if gt_start==True and ref_start==True:
        ax.clear()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_xlim([-2.2, 2.2])
        ax.set_ylim([-2.2, 2.2])
        ax.plot(real_x[-20:], real_y[-20:], label='Adaptive MPC', color='blue')
        ax.plot(ref_x[-20:], ref_y[-20:], label='Reference', color='orange')
        ax.scatter(real_x[-1], real_y[-1], color='blue', marker='o')
        ax.scatter(ref_x[-1], ref_y[-1], color='orange', marker='o')
        ax.legend()
        plt.draw()
        plt.pause(0.05)

plt.show()