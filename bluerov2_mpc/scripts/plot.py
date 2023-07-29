import rospy
from nav_msgs.msg import Odometry  # replace with the actual message type of pose_gt and mpc_reference topics
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

x_errors = []
y_errors = []
z_errors = []
    
def err_callback(data):
    global x_errors, y_errors, z_errors
    
    # Append the new error values to the global variables
    x_errors.append(data.pose.pose.position.x)
    y_errors.append(data.pose.pose.position.y)
    z_errors.append(data.pose.pose.position.z)

    
rospy.init_node('plotter_error', anonymous=True)
# rospy.Subscriber('/bluerov2/pose_gt', Odometry, gt_callback)
rospy.Subscriber('/bluerov2/mpc/error', Odometry, err_callback)

fig, ax = plt.subplots()

while not rospy.is_shutdown():
    plt.clf()
    plt.bar(['X', 'Y', 'Z'], [x_errors[-1], y_errors[-1], z_errors[-1]])
    plt.xlabel('Direction')
    plt.ylabel('Error')
    plt.ylim(-0.5, 0.5)
    plt.title('Control Errors (m)')
    plt.draw()
    plt.pause(0.001)

plt.show()