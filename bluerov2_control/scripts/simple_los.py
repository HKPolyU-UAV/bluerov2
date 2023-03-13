#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry  # We listen to this message for state estimation
from geometry_msgs.msg import AccelWithCovarianceStamped, WrenchStamped, AccelStamped, TwistStamped, PoseStamped, Vector3Stamped, Vector3, Twist, PointStamped, Point
from PID import PIDRegulator
from mavros_msgs.srv import SetMode, SetModeRequest
import numpy as np
from bluerov2_control.srv import SetControlMode, SetControlModeResponse, SetControlModeRequest
from bluerov2_control.msg import ControlMode, Autopilot, FollowWaypointsAction, FollowWaypointsGoal, FollowWaypointsFeedback, FollowWaypointsResult
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Header
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation
from dynamic_reconfigure.server import Server
from bluerov2_control.cfg import pid_reconfigConfig
from sensor_msgs.msg import Range
import actionlib
from uuv_control_msgs.msg import Waypoint, WaypointSet
from math import atan2


class MIMOPID(object):
    def __init__(self, *args):
        self._regulators = [PIDRegulator(*arg) for arg in args]

    def set_p(self, gains):
        for r, g in zip(self._regulators, gains):
            r.p = g

    def get_p(self):
        return [r.p for r in self._regulators]

    def set_i(self, gains):
        for r, g in zip(self._regulators, gains):
            r.i = g
            r.integral = 0

    def get_i(self):
        return [r.i for r in self._regulators]

    def set_d(self, gains):
        for r, g in zip(self._regulators, gains):
            r.d = g

    def get_d(self):
        return [r.d for r in self._regulators]

    def set_sat(self, sats):
        for r, s in zip(self._regulators, sats):
            r.sat = s
            r.integral = 0

    def get_sat(self):
        return [r.sat for r in self._regulators]

    def __call__(self, errors, t):
        return np.array([r.regulate(e, t) for r, e in zip(self._regulators, errors)], dtype=float)


class SimpleCascade(object):
    """
    Nothing fancy here, just a 4 DoF (Surge, Sway, Heave, Yaw)
    Cascade Controller Can be Configured for Different Modes of Control
    Modes
    AccelTeleop -> Acceleration Setpoint Given by AccelStamped Message      [X]
    VelTeleop -> Velocity Setpoint Given by Vector3Stamped Message          [X]
    HoldPosition -> Position Setpoint Set to Latest Odometry Message        [ ]
    LOSPosition -> Position Setpoint Set to Requested PointStamped Message  [ ]
    Autopilot -> Velocity Setpoint Set to Requested Vector3Stamped Message  [ ]

    Outputs WrenchStamped Message

    Requirements

    """

    def __init__(self):
        self._ready = False
        self._first = True
        self._mode = ControlMode()
        self._mode.mode = self._mode.OFF
        self._use_accel_fb = rospy.get_param("~use_accel_fb", default=False)

        #-------- TF Buffer ----------
        self._tf_buff = Buffer()
        TransformListener(self._tf_buff)

        #-------- Service Advertisements ---------
        self._set_control_mode = rospy.Service("controller/set_control_mode", SetControlMode, self._handle_control_mode)
        self._configure_mavros = rospy.ServiceProxy("mavros/set_mode", SetMode)

        #-------- State Config -------------------
        self._control_mode_pub = rospy.Publisher("controller/mode", ControlMode, queue_size=5)
        self._latest_odom_fb = None
        rospy.Subscriber("odometry/filtered", Odometry, self._odom_feedback)

        #-------- Wrench Config ------------------
        self._wrench = WrenchStamped()
        self._wrench_pub = rospy.Publisher("wrench/target", WrenchStamped, queue_size=5)

        #-------- Acceleration Config ------------
        self._latest_accel_sp = None
        rospy.Subscriber("accel/setpoint", AccelStamped, self._accel_sp)
        self._latest_accel_fb = None
        rospy.Subscriber("accel/filtered", AccelWithCovarianceStamped, self._accel_feedback)
        # If using acceleration feedback, then setup the PID Gains and subscribe to the acceleration feedback
        if self._use_accel_fb:
            # Can only use ax, ay, az because ar is not provided by robot_localization
            self._accel_pids = MIMOPID([0, 0, 0, 0],
                                       [0, 0, 0, 0],
                                       [0, 0, 0, 0],
                                       [0, 0, 0, 0])
        # Otherwise, mass and moments of inertia must be given
        else:
            self.mass = rospy.get_param("~pid/mass")
            self.inertial = rospy.get_param("~pid/inertial")
            # update mass, moments of inertia
            self.inertial_tensor = np.array(
                [[self.inertial['ixx'], self.inertial['ixy'], self.inertial['ixz']],
                 [self.inertial['ixy'], self.inertial['iyy'], self.inertial['iyz']],
                 [self.inertial['ixz'], self.inertial['iyz'], self.inertial['izz']]])
            self.mass_inertial_matrix = np.vstack((
                np.hstack((self.mass * np.identity(3), np.zeros((3, 3)))),
                np.hstack((np.zeros((3, 3)), self.inertial_tensor))))

        #--------- Velocity Config ----------
        self._vel_pids = MIMOPID([0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0])
        rospy.Subscriber("vel/setpoint", TwistStamped, self._vel_sp)
        self._vel_limits = np.zeros((4, 1), dtype=float)
        self._latest_vel_sp = None
        self._vel_deadband = rospy.get_param("~pid/deadband", 0.01)

        #---------- Position Config ---------
        self._pos_pids = MIMOPID([0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0])
        rospy.Subscriber("pos/setpoint", PoseStamped, self._pos_sp)
        self._latest_pos_sp = None

        #---------- Autopilot Config ----------
        rospy.Subscriber("autopilot/setpoint", Autopilot, self._ap_sp)
        self._latest_ap_sp = None
        rospy.Subscriber("range", Range, self._range_feedback)
        self._latest_range_fb = None
        self._control_depth = True
        self._control_sway = True
        self._control_yaw = True

        #---------- Waypoint Following Config ----------
        self._los_server = actionlib.SimpleActionServer('follow_waypoints', FollowWaypointsAction, self._los_action_cb, False)
        self._los_server.start()
        self._latest_wp = None

        # -------- Dynamic Reconfigure Server -------
        self._reconfig_serv = Server(pid_reconfigConfig, self._reconfig)

        #---------- Timer Loops -----------
        rospy.Timer(rospy.Duration.from_sec(1.0 / 10.0), self._main_loop)
        self._ready = True

    def _reconfig(self, config, level):
        if self._use_accel_fb:
            self._accel_pids.set_p([config["accel_x_kp"],
                                    config["accel_y_kp"],
                                    config["accel_z_kp"],
                                    config["accel_r_kp"]])
            self._accel_pids.set_i([config["accel_x_ki"],
                                    config["accel_y_ki"],
                                    config["accel_z_ki"],
                                    config["accel_r_ki"]])
            self._accel_pids.set_d([config["accel_x_kd"],
                                    config["accel_y_kd"],
                                    config["accel_z_kd"],
                                    config["accel_r_kd"]])
            self._accel_pids.set_sat([config["accel_x_sat"],
                                    config["accel_y_sat"],
                                    config["accel_z_sat"],
                                    config["accel_r_sat"]])
        self._vel_pids.set_p([config["vel_x_kp"],
                                config["vel_y_kp"],
                                config["vel_z_kp"],
                                config["vel_r_kp"]])
        self._vel_pids.set_i([config["vel_x_ki"],
                                config["vel_y_ki"],
                                config["vel_z_ki"],
                                config["vel_r_ki"]])
        self._vel_pids.set_d([config["vel_x_kd"],
                                config["vel_y_kd"],
                                config["vel_z_kd"],
                                config["vel_r_kd"]])
        self._vel_pids.set_sat([config["vel_x_sat"],
                                config["vel_y_sat"],
                                config["vel_z_sat"],
                                config["vel_r_sat"]])
        self._vel_limits = np.array([[config["vel_x_lim"]],
                                     [config["vel_y_lim"]],
                                     [config["vel_z_lim"]],
                                     [config["vel_r_lim"]]], dtype=float)
        self._pos_pids.set_p([config["pos_x_kp"],
                                config["pos_y_kp"],
                                config["pos_z_kp"],
                                config["pos_r_kp"]])
        self._pos_pids.set_i([config["pos_x_ki"],
                                config["pos_y_ki"],
                                config["pos_z_ki"],
                                config["pos_r_ki"]])
        self._pos_pids.set_d([config["pos_x_kd"],
                                config["pos_y_kd"],
                                config["pos_z_kd"],
                                config["pos_r_kd"]])
        self._pos_pids.set_sat([config["pos_x_sat"],
                                  config["pos_y_sat"],
                                  config["pos_z_sat"],
                                  config["pos_r_sat"]])
        return config

    # ------- CONTROL MODE SERVICE --------
    def _handle_control_mode(self, mode):
        """
            Handles the "set_contol_mode" service.
            When called, places the FCU into manual flight mode and clears the last accel, vel, and pos setpoints.
        """
        res = SetControlModeResponse()
        res.success = False
        if not self._ready:
            return res
        self._mode = mode.mode
        req = SetModeRequest()
        req.base_mode = 0
        #req.custom_mode = "19"  # MANUAL FLIGHT MODE
        req.custom_mode = "0"    # STABILIZE FLIGHT MODE
        # req.custom_mode = "2"    # ALT_HOLD FLIGHT MODE
        self._configure_mavros.wait_for_service()
        res.success = self._configure_mavros(req).mode_sent
        self._control_depth = True
        self._control_sway = True
        self._control_yaw = True
        self._latest_accel_sp = None
        self._latest_vel_sp = None
        self._latest_pos_sp = None
        self._latest_ap_sp = None
        self._latest_wp = None
        return res

    # ------- EXTERNAL SETPOINT HANDLERS --------
    def _vel_sp(self, msg):
        if not self._ready:
            return
        if self._mode.mode == self._mode.VELTELEOP:
            self._latest_vel_sp = msg

    def _accel_sp(self, msg):
        if not self._ready:
            return
        if self._mode.mode == self._mode.ACCELTELEOP:
            self._latest_accel_sp = msg

    def _pos_sp(self, msg):
        if not self._ready:
            return
        self._latest_pos_sp = msg

    def _ap_sp(self, msg):
        if not self._ready:
            return
        self._latest_ap_sp = msg

    # ------- FEEDBACK HANDLERS --------

    def _odom_feedback(self, msg):
        if not self._ready:
            return
        self._latest_odom_fb = msg

    def _accel_feedback(self, msg):
        if not self._ready:
            return
        self._latest_accel_fb = msg

    def _range_feedback(self, msg):
        if not self._ready:
            return
        self._latest_range_fb = msg

    # -------- ACTION CALLBACKS --------
    def _los_action_cb(self, goal):
        r = rospy.Rate(1.0)
        results = FollowWaypointsResult()
        feedback = FollowWaypointsFeedback()
        while self._mode.mode != ControlMode.LOSGUIDANCE:
            rospy.logwarn_throttle(10.0, "{} | Waypoints pending. Set to LOSGUIDANCE mode to execute.".format(rospy.get_name()))
            if self._los_server.is_preempt_requested():
                results.waypoints_completed = 0
                self._los_server.set_preempted(results)
                return
            r.sleep()
        rospy.loginfo("{} | Adding {} waypoint commands to Autopilot.".format(rospy.get_name(), len(goal.waypoints.waypoints)))
        for i, wp in enumerate(goal.waypoints.waypoints):
            # Get sequential waypoints, and set as latest wp
            self._latest_wp = wp
            while not self._calc_3D_distance() < wp.radius_of_acceptance:
                feedback.percentage_complete = float(i) / float(len(goal.waypoints.waypoints)) * 100.0
                self._los_server.publish_feedback(feedback)
                if self._los_server.is_preempt_requested():
                    self._los_server.set_preempted(results)
                    return
                if self._mode.mode != ControlMode.LOSGUIDANCE:
                    self._los_server.set_aborted(results)
                    return
                r.sleep()
            results.waypoints_completed = i + 1
        rospy.loginfo("{} | Waypoint tasks completed.".format(rospy.get_name()))
        self._los_server.set_succeeded(results)

    # -------- CONTROL CALCULATIONS -------
    def _calc_surface(self):
        cmd_accel = AccelStamped(Header(0, rospy.Time.now(), ""), None)
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | SURFACING! No odom feedback.".format(rospy.get_name()))
            cmd_accel.accel.linear.z = 10
        else:
            cmd_accel.accel.linear.z = 10 if self._latest_odom_fb.pose.pose.position.z < -0.5 else 0
        self._latest_accel_sp = cmd_accel
        return True

    def _hold_pos(self):
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No odom feedback.".format(rospy.get_name()))
            return False
        cmd_ap = Autopilot()
        cmd_ap.U = 0
        cmd_ap.Z = self._latest_odom_fb.pose.pose.position.z
        euler = Rotation.from_quat([self._latest_odom_fb.pose.pose.orientation.x,
                                    self._latest_odom_fb.pose.pose.orientation.y,
                                    self._latest_odom_fb.pose.pose.orientation.z,
                                    self._latest_odom_fb.pose.pose.orientation.w]).as_euler("xyz")
        cmd_ap.psi = euler[-1]
        cmd_ap.reference = cmd_ap.DEPTH
        self._control_yaw = False
        self._latest_ap_sp = cmd_ap
        return True

    def _calc_footprint_distance(self):
        if self._latest_wp is None:
            rospy.logwarn_throttle(10.0, "{} | No waypoint provided.".format(rospy.get_name()))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No odom feedback.".format(rospy.get_name()))
            return False
        """
        Calculate euclidean distance from waypoint to vehicle's footprint here
        """
        p = PointStamped(Header(0, rospy.Time.from_sec(0), self._latest_wp.header.frame_id),
                         self._latest_wp.point)
        if self._latest_wp.header.frame_id != self._latest_odom_fb.header.frame_id:
            if self._tf_buff.can_transform(p.header.frame_id, self._latest_odom_fb.header.frame_id,
                                           rospy.Time.from_sec(0), rospy.Duration(5)):
                p = self._tf_buff.transform(p, self._latest_odom_fb.header.frame_id)
            else:
                rospy.logwarn_throttle(10.0, "{} | cannot TF waypoint frame_id: {}".format(rospy.get_name(),
                                                                                           p.header.frame_id))
                return False
        dx = self._latest_odom_fb.pose.pose.position.x - p.point.x
        dy = self._latest_odom_fb.pose.pose.position.y - p.point.y
        return np.sqrt(dx**2+dy**2)

    def _calc_3D_distance(self):
        if self._latest_wp is None:
            rospy.logwarn_throttle(10.0, "{} | No waypoint provided.".format(rospy.get_name()))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No odom feedback.".format(rospy.get_name()))
            return False
        """
        Calculate euclidean distance from waypoint to vehicle's 3D position here
        """
        p = PointStamped(Header(0, rospy.Time.from_sec(0), self._latest_wp.header.frame_id),
                         self._latest_wp.point)
        if self._tf_buff.can_transform(p.header.frame_id, self._latest_odom_fb.header.frame_id, rospy.Time.from_sec(0),
                                       rospy.Duration(5)):
            p = self._tf_buff.transform(p, self._latest_odom_fb.header.frame_id)
        else:
            rospy.logwarn_throttle(10.0,
                                   "{} | cannot TF waypoint frame_id: {}".format(rospy.get_name(), p.header.frame_id))
            return False
        dx = self._latest_odom_fb.pose.pose.position.x - p.point.x
        dy = self._latest_odom_fb.pose.pose.position.y - p.point.y
        if p.point.z <= 0:
            dz = self._latest_odom_fb.pose.pose.position.z - p.point.z
        else:
            dz = self._latest_range_fb.range - p.point.z
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def _calc_los(self):
        """
        Calculate autopilot setpoints
        """
        if self._latest_wp is None:
            rospy.logwarn_throttle(10.0, "{} | No waypoint provided.".format(rospy.get_name()))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No odom feedback.".format(rospy.get_name()))
            return False
        ap = Autopilot()
        ap.Z = self._latest_wp.point.z
        ap.U = self._latest_wp.max_forward_speed if self._calc_footprint_distance() >= self._latest_wp.radius_of_acceptance else 0
        ap.reference = ap.DEPTH if ap.Z <= 0.0 else ap.ALT
        p = PointStamped(Header(0, rospy.Time.from_sec(0), self._latest_wp.header.frame_id),
                         self._latest_wp.point)
        if self._latest_wp.header.frame_id != self._latest_odom_fb.header.frame_id:
            if self._tf_buff.can_transform(p.header.frame_id, self._latest_odom_fb.header.frame_id, rospy.Time.from_sec(0), rospy.Duration(5)):
                p = self._tf_buff.transform(p, self._latest_odom_fb.header.frame_id)
            else:
                rospy.logwarn_throttle(10.0, "{} | cannot TF waypoint frame_id: {}".format(rospy.get_name(), p.header.frame_id))
                return False
        ap.psi = atan2(p.point.y - self._latest_odom_fb.pose.pose.position.y, p.point.x - self._latest_odom_fb.pose.pose.position.x)
        ap.header.stamp = rospy.Time.now()
        self._latest_ap_sp = ap
        return True

    def _calc_autopilot_vel(self):
        """
        Calculate Velocity Setpoints from U (m/s), z (m) and psi (rad)
        Z may be depth, height referenced, or if None will be ignored. Default is depth.
        """
        if self._latest_ap_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No autopilot setpoint.".format(rospy.get_name()))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No odom feedback.".format(rospy.get_name()))
            return False
        cmd_vel = TwistStamped(Header(0, rospy.Time.now(), self._latest_odom_fb.child_frame_id),
                               Twist(None, None))
        cmd_vel.twist.linear.x = self._latest_ap_sp.U
        euler_error = np.array([0,
                                0,
                                self._latest_ap_sp.psi]) - Rotation.from_quat(
            [self._latest_odom_fb.pose.pose.orientation.x,
             self._latest_odom_fb.pose.pose.orientation.y,
             self._latest_odom_fb.pose.pose.orientation.z,
             self._latest_odom_fb.pose.pose.orientation.w]).as_euler("xyz")
        euler_error = np.where(np.bitwise_and(np.abs(euler_error) > np.pi, euler_error < 0), euler_error + 2 * np.pi,
                               euler_error)
        euler_error = np.where(np.bitwise_and(np.abs(euler_error) > np.pi, euler_error > 0), euler_error - 2 * np.pi,
                               euler_error)
        # TODO this gets set to None if load_waypoints gets called too quickly
        if self._latest_ap_sp is None:
            return False
        if self._latest_ap_sp.reference == self._latest_ap_sp.DEPTH:
            z_error = self._latest_ap_sp.Z - self._latest_odom_fb.pose.pose.position.z
            self._control_depth = True
        elif self._latest_ap_sp.reference == self._latest_ap_sp.ALT:
            if self._latest_range_fb is None:
                rospy.logerr_throttle(5, "{} | {}".format(rospy.get_name(), "No ranging info available for altitude control."))
                return False
            z_error = self._latest_ap_sp.Z - self._latest_range_fb.range
            self._control_depth = True
        elif self._latest_ap_sp.reference == self._latest_ap_sp.NONE:
            z_error = 0
            self._control_depth = False
        else:
            rospy.logerr_throttle(5, "{} | {}".format(rospy.get_name(), "Autopilot unknown mode."))
            return False
        pos_err = np.array(
            [[0],
             [0],
             [z_error],
             [euler_error[-1]]
             ],
            dtype=float
        )
        _, _, cmd_vel.twist.linear.z, cmd_vel.twist.angular.z = self._pos_pids(pos_err, rospy.Time.now().to_sec())
        self._control_sway = False
        self._latest_vel_sp = cmd_vel
        return True

    # REGULATE POSITION WITH VELOCITY
    def _calc_vel(self):
        """
        Calculate Velocity Setpoint from Pose Setpoint PID
        """
        if self._latest_pos_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No pos setpoint.".format(rospy.get_name()))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No pos feedback.".format(rospy.get_name()))
            return False
        # Placing the setpoint into body-fixed coordinates (similar to Vessel Parallel Transform)
        cmd_pos = self._latest_pos_sp
        cmd_pos.header.stamp = rospy.Time.from_sec(0.0)  # Get the latest available transform
        body_pose = self._tf_buff.transform(cmd_pos, self._latest_odom_fb.child_frame_id)
        euler_error = Rotation.from_quat([body_pose.pose.orientation.x,
                                          body_pose.pose.orientation.y,
                                          body_pose.pose.orientation.z,
                                          body_pose.pose.orientation.w]).as_euler("xyz")
        euler_error = np.where(np.bitwise_and(np.abs(euler_error) > np.pi, euler_error < 0), euler_error + 2*np.pi, euler_error)
        euler_error = np.where(np.bitwise_and(np.abs(euler_error) > np.pi, euler_error > 0), euler_error - 2*np.pi, euler_error)
        pos_err = np.array(
            [[body_pose.pose.position.x],
             [body_pose.pose.position.y],
             [body_pose.pose.position.z],
             [euler_error[-1]]
             ],
            dtype=float
        )

        vel_sp_msg = TwistStamped()
        vel_sp_msg.header.frame_id = self._latest_odom_fb.child_frame_id
        vel_sp_msg.twist.linear.x, vel_sp_msg.twist.linear.y, vel_sp_msg.twist.linear.z, vel_sp_msg.twist.angular.z = np.clip(self._vel_pids(pos_err, self._latest_odom_fb.header.stamp.to_sec()).reshape(4, 1), -self._vel_limits, self._vel_limits)
        self._latest_vel_sp = vel_sp_msg
        return True

    def _enforce_deadband(self, vector):
        vector.x = 0 if np.abs(vector.x) < self._vel_deadband else vector.x
        vector.y = 0 if np.abs(vector.y) < self._vel_deadband else vector.y
        vector.z = 0 if np.abs(vector.z) < self._vel_deadband else vector.z
        return vector

    # REGULATE VELOCITY WITH ACCELERATION
    def _calc_accel(self):
        """
        Calculate Acceleration Setpoint from Twist Setpoint PID
        """
        if self._latest_vel_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No vel setpoint.".format(rospy.get_name()))
            return False
        if self._latest_odom_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No vel feedback.".format(rospy.get_name()))
            return False
        # Convert setpoint velocity into the correct frame
        if self._latest_vel_sp.header.frame_id != self._latest_odom_fb.child_frame_id and len(self._latest_vel_sp.header.frame_id) > 0:
            if self._tf_buff.can_transform(self._latest_odom_fb.child_frame_id, self._latest_vel_sp.header.frame_id, rospy.Time.from_sec(0.0)):
                cmd_vel = TwistStamped()
                header = Header(self._latest_vel_sp.header.seq, rospy.Time.from_sec(0.0), self._latest_vel_sp.header.frame_id)
                cmd_vel.twist.linear = self._tf_buff.transform(Vector3Stamped(header,
                                                                                self._latest_vel_sp.twist.linear),
                                                                                self._latest_odom_fb.child_frame_id,
                                                                                rospy.Duration(5)).vector
                cmd_vel.twist.angular = self._tf_buff.transform(Vector3Stamped(header,
                                                                               self._latest_vel_sp.twist.angular),
                                                                               self._latest_odom_fb.child_frame_id,
                                                                               rospy.Duration(5)).vector
                cmd_vel.header.stamp = rospy.Time.now()
            else:
                rospy.logwarn_throttle(5, "{} | Cannot TF Velocity SP frame_id: {}".format(rospy.get_name(), self._latest_vel_sp.header.frame_id))
                return False
        else:
            cmd_vel = self._latest_vel_sp
        clean_linear_vel = self._enforce_deadband(self._latest_odom_fb.twist.twist.linear)
        clean_angular_vel = self._enforce_deadband(self._latest_odom_fb.twist.twist.angular)
        vel_err = np.array(
            [[cmd_vel.twist.linear.x - clean_linear_vel.x],
             [cmd_vel.twist.linear.y - clean_linear_vel.y],
             [cmd_vel.twist.linear.z - clean_linear_vel.z],
             [cmd_vel.twist.angular.z - self._latest_odom_fb.twist.twist.angular.z]],
            dtype=float)
        accel_sp_msg = AccelStamped()
        accel_sp_msg.header = cmd_vel.header
        accel_sp_msg.accel.linear.x, accel_sp_msg.accel.linear.y, accel_sp_msg.accel.linear.z, accel_sp_msg.accel.angular.z = self._vel_pids(vel_err, self._latest_odom_fb.header.stamp.to_sec())
        accel_sp_msg.accel.linear.y = 0 if not self._control_sway else accel_sp_msg.accel.linear.y
        accel_sp_msg.accel.linear.z = 0 if not self._control_depth else accel_sp_msg.accel.linear.z
        accel_sp_msg.accel.angular.z = 0 if not self._control_yaw else accel_sp_msg.accel.angular.z
        self._latest_accel_sp = accel_sp_msg
        return True

    def _calc_wrench(self):
        """
        Calculate Wrench Setpoint from Accel Message
        Uses the odometry child_frame_id as the body-fixed frame (ASSUMED FLU)
        Will transform the requested setpoint into the body-fixed frame.
        """
        if self._latest_accel_sp is None:
            rospy.logwarn_throttle(10.0, "{} | No accel setpoint.".format(rospy.get_name()))
            return
        if self._latest_accel_fb is None:
            rospy.logwarn_throttle(10.0, "{} | No accel feedback.".format(rospy.get_name()))
            return
        # Convert setpoint acceleration into the correct frame
        if self._latest_accel_sp.header.frame_id != self._latest_accel_fb.header.frame_id and len(self._latest_accel_sp.header.frame_id) > 0:
            if self._tf_buff.can_transform(self._latest_accel_fb.header.frame_id,
                                           self._latest_accel_sp.header.frame_id, rospy.Time.from_sec(0.0), rospy.Duration(5)):
                header = Header(self._latest_accel_sp.header.seq, rospy.Time.from_sec(0.0), self._latest_accel_sp.header.frame_id)
                cmd_accel = AccelStamped()
                cmd_accel.accel.linear = self._tf_buff.transform(Vector3Stamped(header,
                                                                                self._latest_accel_sp.accel.linear),
                                                                 self._latest_accel_fb.header.frame_id,
                                                                 rospy.Duration(5)).vector
                cmd_accel.accel.angular = self._tf_buff.transform(Vector3Stamped(header,
                                                                                 self._latest_accel_sp.accel.angular),
                                                                  self._latest_accel_fb.header.frame_id,
                                                                  rospy.Duration(5)).vector
            else:
                rospy.logwarn_throttle(5, "{} | Cannot TF Acceleration SP frame_id: {}".format(rospy.get_name(),
                                                                                           self._latest_accel_sp.header.frame_id))
                return False
        else:
            cmd_accel = self._latest_accel_sp  # If it's already in the correct frame then never mind.
        if self._use_accel_fb:
            accel_err = np.array(
                [[cmd_accel.accel.linear.x - self._latest_accel_fb.accel.accel.linear.x],
                 [cmd_accel.accel.linear.y - self._latest_accel_fb.accel.accel.linear.y],
                 [cmd_accel.accel.linear.z - self._latest_accel_fb.accel.accel.linear.z],
                 [cmd_accel.accel.angular.z - self._latest_accel_fb.accel.accel.angular.z]],
                dtype=float)
            tau = self._accel_pids(accel_err, self._latest_accel_fb.header.stamp.to_sec())
        else:
            accel = np.array([[cmd_accel.accel.linear.x],
                              [cmd_accel.accel.linear.y],
                              [cmd_accel.accel.linear.z],
                              [cmd_accel.accel.angular.x],
                              [cmd_accel.accel.angular.y],
                              [cmd_accel.accel.angular.z]])
            tau = self.mass_inertial_matrix.dot(accel)[[0, 1, 2, -1], :]
        self._wrench.header.stamp = rospy.Time.now()
        self._wrench.header.frame_id = cmd_accel.header.frame_id
        self._wrench.wrench.force.x, self._wrench.wrench.force.y, self._wrench.wrench.force.z, self._wrench.wrench.torque.z = tau.squeeze()
        self._wrench_pub.publish(self._wrench)

    def _main_loop(self, event):
        if not self._ready:
            return
        self._control_mode_pub.publish(self._mode)  # send the controller's state
        if self._mode.mode == self._mode.OFF:
            return
        elif self._mode.mode == self._mode.IDLE:
            return
        elif self._mode.mode == self._mode.ACCELTELEOP:
            self._calc_wrench()
        elif self._mode.mode == self._mode.VELTELEOP:
            if self._calc_accel():
                self._calc_wrench()
        elif self._mode.mode == self._mode.HOLDPOSITION:
            if self._hold_pos():
                if self._calc_autopilot_vel():
                    if self._calc_accel():
                        self._calc_wrench()
        elif self._mode.mode == self._mode.AUTOPILOT:
            # Given heading and forward velocity, match it!
            if self._calc_autopilot_vel():
                if self._calc_accel():
                    self._calc_wrench()
        elif self._mode.mode == self._mode.LOSGUIDANCE:
            if self._calc_los():
                if self._calc_autopilot_vel():
                    if self._calc_accel():
                        self._calc_wrench()
        elif self._mode.mode == self._mode.ABORT:
            if self._calc_surface():
                self._calc_wrench()
        else:
            rospy.logerr_throttle(5.0, "{} | Control mode {} not implemented.".format(rospy.get_name(), self._mode.mode))


if __name__=="__main__":
    rospy.init_node("controller")#, log_level=rospy.DEBUG)
    try:
        controller = SimpleCascade()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
