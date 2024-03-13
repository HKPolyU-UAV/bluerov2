import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

am_X, am_Y, am_Z, am_N = [], [], [], []
da_X, da_Y, da_Z, da_N = [], [], [], []
am_start = False
da_start = False

def am_callback(data):
    global am_X, am_Y, am_Z, am_N
    global am_start
    am_start = True
    am_X.append(data.pose.pose.position.x)
    am_Y.append(data.pose.pose.position.y)
    am_Z.append(data.pose.pose.position.z)
    am_N.append(data.twist.twist.angular.z)

def da_callback(data):
    global da_X, da_Y, da_Z, da_N
    global da_start
    da_start = True
    da_X.append(data.pose.pose.position.x)
    da_Y.append(data.pose.pose.position.y)
    da_Z.append(data.pose.pose.position.z)
    da_N.append(data.twist.twist.angular.z)

rospy.init_node('plotter', anonymous=True)
rospy.Subscriber('/bluerov2/systemID/added_mass', Odometry, am_callback)
rospy.Subscriber('/bluerov2/systemID/linear_damping', Odometry, da_callback)

fig1, ax1 = plt.subplots(4, 1)
fig2, ax2 = plt.subplots(4, 1)

while not rospy.is_shutdown():
    if am_start == True:
        # Plot the added mass identification figure
        ax1[0].clear()
        ax1[0].set_ylabel('Xu\'')
        ax1[0].plot(am_X, label='Identified', color='blue')
        ax1[0].axhline(y=1.7182, label='Benchmark', color='orange')
        ax1[0].set_ylim([-5, 5])
        ax1[0].legend()
        fig1.suptitle('Added Mass Identification Results', fontsize=16)

        ax1[1].clear()
        ax1[1].set_ylabel('Yv\'')
        ax1[1].plot(am_Y, label='identified Yv', color='blue')
        ax1[1].axhline(y=0, label='true Yv', color='orange')
        ax1[1].set_ylim([-5, 5])

        ax1[2].clear()
        ax1[2].set_ylabel('Zw\'')
        ax1[2].plot(am_Z, label='identified Zw', color='blue')
        ax1[2].axhline(y=5.468, label='true Zw', color='orange')
        ax1[2].set_ylim([-5, 8])

        ax1[3].clear()
        ax1[3].set_ylabel('Nr\'')
        ax1[3].plot(am_N, label='identified Nr', color='blue')
        ax1[3].axhline(y=0.4006, label='true Nr', color='orange')
        ax1[3].set_ylim([-5, 5])

        # ax1[3].legend()
        # plt.title("Added Mass Identification Results")
        plt.draw()
        plt.pause(0.05)

    if da_start == True:
        # Plot the damping identification figure
        ax2[0].clear()
        ax2[0].set_ylabel('Xu')
        ax2[0].plot(da_X, label='Identified', color='blue')
        ax2[0].axhline(y=-11.7391, label='Benchmark', color='orange')
        ax2[0].set_ylim([-15, 5])
        ax2[0].legend()
        fig2.suptitle('Damping Coefficients Identification Results', fontsize=16)

        ax2[1].clear()
        ax2[1].set_ylabel('Yv')
        ax2[1].plot(da_Y, label='identified Yv', color='blue')
        ax2[1].axhline(y=-20, label='true Yv', color='orange')
        ax2[1].set_ylim([-25, 5])

        ax2[2].clear()
        ax2[2].set_ylabel('Zw')
        ax2[2].plot(da_Z, label='identified Zw', color='blue')
        ax2[2].axhline(y=-31.8678, label='true Zw', color='orange')
        ax2[2].set_ylim([-35, 5])

        ax2[3].clear()
        ax2[3].set_ylabel('Nr')
        ax2[3].plot(da_N, label='identified Nr', color='blue')
        ax2[3].axhline(y=-5, label='true Nr', color='orange')
        ax2[3].set_ylim([-10, 2])

        # ax2[3].legend()
        # plt.title("Damping Coefficients Identification Results")
        plt.draw()
        plt.pause(0.05)

plt.show()