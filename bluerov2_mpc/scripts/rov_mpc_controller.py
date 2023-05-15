import rospy
import numpy as np
from uuv_control_interfaces import DPPIDControllerBase
from uuv_control_interfaces import DPControllerBase
from uuv_control_interfaces import DPControllerLocalPlanner as LocalPlanner
from uuv_control_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import WrenchStamped, PoseStamped, TwistStamped, \
    Vector3, Quaternion, Pose
from nav_msgs.msg import Odometry

class ROV_MPCController():
    """MPC controller for the dynamic positioning of ROVs. """

    _LABEL = 'MPC'

    def __init__(self):
        self._is_init = True
        
        # Reference with relation to the INERTIAL frame
        self._reference = dict(pos=np.zeros(3),
                               rot=np.zeros(4),
                               vel=np.zeros(6),
                               acc=np.zeros(6))
        
        self.planner_full_dof = False
        self._use_stamped_poses_only = False
        self.thrusters_only = True
        self._local_planner = LocalPlanner(
            full_dof=False,
            stamped_pose_only=False,
            thrusters_only=True)
        # Publisher
        self._reference_pub = rospy.Publisher('/bluerov2/mpc/reference',
                                              TrajectoryPoint,
                                              queue_size=1)
        #self._error_pub = rospy.Publisher('/bluerov2/mpc/error',
        #                                  TrajectoryPoint, queue_size=1)
        print("bluerov2 initialized!")
    
    
    def receive_reference(self):
        print("receive_reference starts")
        self._is_sub = False
        
        # Subscriber
        rospy.Subscriber("/bluerov2/pose_gt", Odometry, self.pos_callback)
        
        while self._is_sub == False:
            print("no subscriber")
            rospy.sleep(0.1)

        print("start subscribing")

        t = rospy.get_time()
        
        reference = self._local_planner.interpolate(t)
        print("call local_planner")
        if reference is not None:
            self._reference['pos'] = reference.p
            self._reference['rot'] = reference.q
            self._reference['vel'] = np.hstack((reference.v, reference.w))
            self._reference['acc'] = np.hstack((reference.a, reference.alpha))
            print("reference is not None")
            print(self._reference['pos'])
            print(self._reference['rot'])
            print(self._reference['vel'])
            print(self._reference['acc'])
        
        if reference is not None and self._reference_pub.get_num_connections() > 0:
            # Publish current reference
            msg = TrajectoryPoint()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self._local_planner.inertial_frame_id
            msg.pose.position = Vector3(*self._reference['pos'])
            msg.pose.orientation = Quaternion(*self._reference['rot'])
            msg.velocity.linear = Vector3(*self._reference['vel'][0:3])
            msg.velocity.angular = Vector3(*self._reference['vel'][3:6])
            msg.acceleration.linear = Vector3(*self._reference['acc'][0:3])
            msg.acceleration.angular = Vector3(*self._reference['acc'][3:6])
            self._reference_pub.publish(msg)
            print("reference sent!")
        
        
    def pos_callback(self, msg):
        self._is_sub = True
        current_pos = np.zeros(3)
        current_pos[0] = msg.pose.pose.position.x
        current_pos[1] = msg.pose.pose.position.y
        current_pos[2] = msg.pose.pose.position.z
        current_quat = np.zeros(4)
        current_quat[0] = msg.pose.pose.orientation.x
        current_quat[1] = msg.pose.pose.orientation.y
        current_quat[2] = msg.pose.pose.orientation.z
        current_quat[3] = msg.pose.pose.orientation.w
        self._local_planner.update_vehicle_pose(current_pos,current_quat)
        

if __name__ == '__main__':
    print('Starting MPC')
    rospy.init_node('rov_mpc_controller')

    try:
        node = ROV_MPCController()
        node.receive_reference()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')       