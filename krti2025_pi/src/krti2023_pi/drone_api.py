from math import *
import rospy
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist, TwistStamped, Vector3
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped, GeoPoint
from mavros_msgs.msg import State, Thrust, OverrideRCIn, GlobalPositionTarget, WaypointReached, WaypointList, PositionTarget
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandLong, CommandLongRequest
from mavros_msgs.srv import ParamSet, ParamSetRequest
from mavros_msgs.srv import ParamGet, ParamGetRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest
from mavros_msgs.srv import StreamRate, StreamRateRequest
from sensor_msgs.msg import LaserScan, Imu, NavSatFix, Range, PointCloud2, Image
from quadrotor_msgs.msg import PositionCommand
from serial import Serial

from pygeodesy.geoids import GeoidPGM
import numpy as np
import threading

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

def geoid_height(lat, lon):
    """Calculates AMSL to ellipsoid conversion offset."""
    return _egm96.height(lat, lon)

DEBUG_PERIODE = 1
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

class PID:
    def __init__(self, kp, ki=0, kd=0, dt=0, max_error=2, name=""):
        self.name, self.kp, self.ki, self.kd, self.dt = name, kp, ki, kd, dt
        self.error = self.error_sum = self.error_diff = self.last_error = 0
        self.max_error = max_error

    def update(self, error)->float:
        self.error = error
        self.error_sum += error
        self.error_diff = error - self.last_error
        self.last_error = error
        if(self.max_error != 0):
            self.error_sum = clamp(self.error_sum, -self.max_error, self.max_error)
        return clamp(self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff, -2, 2)
    
    def reset(self):
        self.error = self.error_sum = self.error_diff = self.last_error = 0

class DroneAPI:
    def __init__(self, waypoints: list = [], global_position: dict = {"latitude": -7.265, "longitude": 112.784, "altitude": 1}, parameters: dict = {}, sim:bool = True) -> None:
        self.sim = sim
        self.current_waypoint = 0
        self.follow_waypoint = True
        self.waypoints = waypoints
        self.gps = NavSatFix()
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_cb)
        rospy.sleep(1)
        
        rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.current_state = State()
        rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.mission_wp_reached_cb)
        self.wp_reached = WaypointReached()
            
        self.wait4connect()
        rospy.sleep(0.2)
        if self.gps.status == -1:
            self.set_origin(global_position)

        for name, value in parameters.items():
            self.set_parameter(name, value)
        
        self.imu_heading = -1
        self.current_pose = Odometry()
        self.current_heading = 0.0
        self.local_desired_heading = 0.0
        self.home_heading = -1.0
        self.home_compass = -1.0
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self.pose_cb)

        self.compass = 0.0
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.compass_cb)

        self.current_velocity = TwistStamped()
        rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.velocity_cb)

        rospy.Subscriber('/mavros/imu/data/', Imu, self.imu_cb)
        self.imu = Imu()

        self.lidar_queue = []
        self.lidar_data = LaserScan()
        rospy.Timer(rospy.Duration(0.05), self.lidar_pub)
        
        self.previous_pose = Odometry()
        self.yaw_pid = PID(0.1,0,0.01,0, max_error=0.5)

        self.rangefinder = Range()
        rospy.Subscriber("/mavros/rangefinder/rangefinder", Range, self.rangefinder_cb)

        # ── EGO-Planner interface ──────────────────────────────────────────────
        self.ego_reached = False
        self.ego_active = False
        self._ego_lock = threading.Lock()
        self._last_ego_cmd_time = 0

        # Non-EGO velocity moves (missions that still use move_vel); EGO leg uses only setpoint_raw.
        self._ego_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self._ego_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self._ego_raw_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self._ego_camera_pose_pub = rospy.Publisher("/camera_pose", PoseStamped, queue_size=10)
        
        self.min_depth = 0.0
        rospy.Subscriber("/iris/iris/front_camera/depth/image_raw", Image, self._depth_cb)
        
        self.ego_allow_yaw = True
        # traj_server → quadrotor_msgs/PositionCommand (same fields as common PositionCommand stacks)
        rospy.Subscriber("/planning/pos_cmd", PositionCommand, self._ego_pos_cmd_cb)
        # ── end EGO-Planner ───────────────────────────────────────────────────

        rospy.loginfo("Initialization completed.")
        
    def gps_cb(self, data: NavSatFix): self.gps = data
    def state_cb(self, msg): self.current_state = msg
    def mission_wp_reached_cb(self, msg): self.wp_reached = msg
    def imu_cb(self, msg:Imu): self.imu = msg
    def rangefinder_cb(self, msg: Range): self.rangefinder = msg.range
    def compass_cb(self, msg: Float64): self.compass = msg.data   
    def velocity_cb(self, msg:TwistStamped): self.current_velocity = msg
    def lidar_cb(self, data: LaserScan): self.lidar_data = data.ranges
    
    def _depth_cb(self, msg):
        # Convert raw bytes to numpy array (Assuming float32 from Gazebo)
        try:
            depth_data = np.frombuffer(msg.data, dtype=np.float32)
            # Filter out 0, NaN, and objects too close to the lens (noise/drone body)
            valid_depth = depth_data[np.isfinite(depth_data) & (depth_data > 0.5)]
            if len(valid_depth) > 0:
                self.min_depth = np.min(valid_depth)
            else:
                self.min_depth = 0.0
                
            # --- SYNC: Publish Camera Pose with SAME timestamp as Depth Image ---
            if hasattr(self, 'current_pose'):
                cp = PoseStamped()
                cp.header.stamp = msg.header.stamp
                cp.header.frame_id = "world"
                
                pos = self.current_pose.pose.pose.position
                q = self.current_pose.pose.pose.orientation
                
                # Matrix & Position Calculation (Same as before but synced)
                q0, q1, q2, q3 = q.w, q.x, q.y, q.z
                ww, xx, yy, zz = q0*q0, q1*q1, q2*q2, q3*q3
                q1q2, q0q3, q1q3, q0q2, q2q3, q0q1 = q1*q2, q0*q3, q1*q3, q0*q2, q2*q3, q0*q1
                
                r11, r13 = ww + xx - yy - zz, 2 * (q1q3 + q0q2)
                r21, r23 = 2 * (q1q2 + q0q3), 2 * (q2q3 - q0q1)
                r31, r33 = 2 * (q1q3 - q0q2), ww - xx - yy + zz
                
                ox, oy, oz = 0.1, 0, 0.05
                cp.pose.position.x = pos.x + (r11 * ox + r13 * oz)
                cp.pose.position.y = pos.y + (r21 * ox + r23 * oz)
                cp.pose.position.z = pos.z + (r31 * ox + r33 * oz)
                
                # --- HACK: Force camera to be perfectly horizontal ---
                from tf.transformations import quaternion_multiply, euler_from_quaternion, quaternion_from_euler
                # Get current yaw from odom orientation (tf uses [x,y,z,w])
                curr_q = [q.x, q.y, q.z, q.w]
                (roll, pitch, yaw) = euler_from_quaternion(curr_q)
                
                q_yaw = quaternion_from_euler(0, 0, yaw) # Horizontal Yaw only
                q_cam = [0.5, -0.5, 0.5, -0.5] # Standard Optical Frame
                q_final = quaternion_multiply(q_yaw, q_cam)
                
                cp.pose.orientation.w = q_final[3]
                cp.pose.orientation.x = q_final[0]
                cp.pose.orientation.y = q_final[1]
                cp.pose.orientation.z = q_final[2]
                
                self._ego_camera_pose_pub.publish(cp)
        except Exception as e:
            rospy.logwarn_throttle(10.0, "[EGO] depth/camera_pose publish failed: %s", e)
    
    def pose_cb(self, msg: Odometry):
        self.current_pose = msg
        q = msg.pose.pose.orientation
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z
        psi = atan2((2 * (q0 * q3 + q1 * q2)), (1 - 2 * (pow(q2, 2) + pow(q3, 2))))
        self.imu_heading = degrees(psi)
        self.current_heading = self.imu_heading
        
        # Log VIO Position (Throttled 1s)
        pos = msg.pose.pose.position
        rospy.loginfo_throttle(1.0, f"[VIO-LOC] x: {pos.x:.2f} | y: {pos.y:.2f} | z: {pos.z:.2f}")

        if self.home_heading == -1.0:
            self.home_compass = self.compass
            self.home_heading = self.current_heading
            self.local_desired_heading = self.home_heading
        

    def _ego_position_command_to_setpoint_raw(self, msg: PositionCommand) -> PositionTarget:
        """Map traj_server output (PositionCommand) → MAVROS setpoint_raw (local)."""
        pt = PositionTarget()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = "world"
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pt.type_mask = PositionTarget.IGNORE_YAW_RATE
        pt.position = msg.position
        pt.velocity = msg.velocity
        pt.acceleration_or_force = msg.acceleration
        if self.ego_allow_yaw:
            pt.yaw = msg.yaw
        else:
            pt.yaw = self.current_heading * pi / 180.0
        return pt

    def _ego_hold_position_raw(self, x: float, y: float, z: float, yaw_rad: float):
        """
        Hold / pre-nav climb using the *same* MAVROS topic as the planner stream,
        not geometry_msgs/Twist on cmd_vel (ego stack is PositionCommand-class only).
        """
        pt = PositionTarget()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = "world"
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        pt.type_mask = PositionTarget.IGNORE_YAW_RATE
        pt.position = Point(x, y, z)
        pt.velocity = Vector3(0, 0, 0)
        pt.acceleration_or_force = Vector3(0, 0, 0)
        pt.yaw = yaw_rad
        self._ego_raw_pub.publish(pt)

    def _ego_pos_cmd_cb(self, msg: PositionCommand):
        self._last_ego_cmd_time = rospy.Time.now().to_sec()
        if not self.ego_active:
            return
        self._ego_raw_pub.publish(self._ego_position_command_to_setpoint_raw(msg))

    def _normalize_angle(self, angle):
        while angle > pi: angle -= 2 * pi
        while angle < -pi: angle += 2 * pi
        return angle

    def wait4connect(self):
        rospy.loginfo("Waiting for FCU connection")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(0.1)
        rospy.loginfo("FCU connected")

    def wait4start(self):
        rospy.loginfo("Waiting for user to set mode to GUIDED")
        while not rospy.is_shutdown() and self.current_state.mode != "GUIDED":
            rospy.sleep(0.1)
        rospy.loginfo("Mode set to GUIDED. Starting Mission...")

    def set_mode(self, mode: str = "GUIDED"):
        rospy.wait_for_service("/mavros/set_mode")
        rospy.ServiceProxy("/mavros/set_mode", SetMode)(0, mode)

    def arm(self, status: bool = True):
        rospy.wait_for_service("/mavros/cmd/arming")
        client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        while not rospy.is_shutdown() and self.current_state.armed != status:
            client(CommandBoolRequest(status))
            rospy.sleep(0.1)

    def takeoff(self, altitude: float = 3.0):
        rospy.loginfo(f"Taking off to {altitude}m...")
        self.set_mode("GUIDED")
        self.arm(True)
        rospy.wait_for_service("/mavros/cmd/takeoff")
        rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)(0, 0, 0, 0, altitude)
        while not rospy.is_shutdown() and self.current_pose.pose.pose.position.z < altitude * 0.95:
            rospy.sleep(0.1)
        rospy.loginfo("Takeoff complete.")

    def _ego_make_goal(self, sx, sy, sz):
        """
        One /move_base_simple/goal message the same way RViz + ego_planner expect:
        frame world, position xyz; orientation ignored by waypointCallback (identity).
        """
        g = PoseStamped()
        g.header.stamp = rospy.Time.now()
        g.header.frame_id = "world"
        g.pose.position = Point(sx, sy, sz)
        g.pose.orientation.w = 1.0
        g.pose.orientation.x = 0.0
        g.pose.orientation.y = 0.0
        g.pose.orientation.z = 0.0
        return g

    def navigate_ego(self, goal_x, goal_y, exit_alt=1.3, timeout=60, **kwargs):
        """
        Same data path as stock ego_planner + traj_server:
        - Planning input: /move_base_simple/goal (PoseStamped; orientation unused by FSM).
        - Execution: traj_server publishes quadrotor_msgs/PositionCommand on /planning/pos_cmd;
          this node converts to mavros_msgs/PositionTarget on /mavros/setpoint_raw/local.
        No geometry_msgs/Twist is used on the EGO leg (that would be a second, conflicting API).

        Default mode (**use_segments=False**): one nav goal then wait for XY tolerance.
        **use_segments=True**: short chained goals for dense indoor corridors.

        kwargs (common): use_segments (False), min_alt (0.35), arrival_xy_m (0.45), allow_yaw (True)
        kwargs (segmented): segment_horizon_m, segment_arrival_m, close_depth_m, ...
        """
        use_segments = bool(kwargs.get("use_segments", False))
        min_alt = float(kwargs.get("min_alt", 0.35))
        arrival_xy = float(kwargs.get("arrival_xy_m", kwargs.get("segment_arrival_m", 0.45)))

        self.ego_reached = False
        self.set_mode("GUIDED")
        cruise_z = float(max(min_alt, exit_alt))

        rospy.loginfo(
            "[EGO] Target xy=(%.2f, %.2f) z=%.2f | mode=%s arrival≤%.2fm",
            goal_x,
            goal_y,
            cruise_z,
            "segments" if use_segments else "single (planner-standard)",
            arrival_xy,
        )

        # Climb to cruise Z using setpoint_raw (same bus as PositionCommand), not cmd_vel Twist
        while (
            not rospy.is_shutdown()
            and self.current_pose.pose.pose.position.z < cruise_z - 0.12
        ):
            pos = self.current_pose.pose.pose.position
            yaw_rad = radians(self.current_heading)
            z_next = min(cruise_z, pos.z + clamp((cruise_z - pos.z) * 0.22, 0.04, 0.09))
            self._ego_hold_position_raw(pos.x, pos.y, z_next, yaw_rad)
            rospy.sleep(0.05)

        self.ego_allow_yaw = kwargs.get("allow_yaw", True)
        self.ego_active = True
        gx, gy = float(goal_x), float(goal_y)
        deadline = rospy.Time.now().to_sec() + float(timeout)

        try:
            if not use_segments:
                self._ego_goal_pub.publish(self._ego_make_goal(gx, gy, cruise_z))
                rospy.loginfo("[EGO] Published single /move_base_simple/goal (identity orient)")
                while not rospy.is_shutdown():
                    now = rospy.Time.now().to_sec()
                    if now > deadline:
                        rospy.logwarn("[EGO] Timeout waiting final XY")
                        break
                    pos = self.current_pose.pose.pose.position
                    dfin = hypot(gx - pos.x, gy - pos.y)
                    if dfin <= arrival_xy:
                        self.ego_reached = True
                        rospy.loginfo("[EGO] XY goal reached (Δ=%.2fm)", dfin)
                        break
                    cmd_age = now - float(self._last_ego_cmd_time or 0.0)
                    rospy.loginfo_throttle(
                        2.0,
                        "[EGO] %s | Δfinal=%.2f z=%.2f",
                        "ACTIVE" if cmd_age < 1.3 else "WAIT",
                        dfin,
                        pos.z,
                    )
                    rospy.sleep(0.06)
            else:
                segment_horizon = float(kwargs.get("segment_horizon_m", 2.2))
                segment_arrival = float(kwargs.get("segment_arrival_m", arrival_xy))
                close_depth_m = float(kwargs.get("close_depth_m", 1.25))
                min_segment_m = float(kwargs.get("min_segment_m", 0.85))
                segment_timeout = float(kwargs.get("segment_timeout", 75.0))
                stale_cmd_s = float(kwargs.get("stale_cmd_s", 1.35))
                republish_gap_s = float(kwargs.get("republish_gap_s", 2.5))

                while not rospy.is_shutdown():
                    now = rospy.Time.now().to_sec()
                    if now > deadline:
                        rospy.logwarn("[EGO] Timeout before final XY goal")
                        break

                    pos = self.current_pose.pose.pose.position
                    dx = gx - pos.x
                    dy = gy - pos.y
                    dist_final = hypot(dx, dy)

                    if dist_final <= segment_arrival:
                        self.ego_reached = True
                        rospy.loginfo("[EGO] Final XY reached (%.2fm)", dist_final)
                        break

                    inv = dist_final if dist_final > 1e-6 else 1.0
                    ux, uy = dx / inv, dy / inv

                    step = min(segment_horizon, dist_final)
                    md = float(self.min_depth)
                    if md > 0.06:
                        step = min(step, max(min_segment_m, 0.38 * md))
                        if md < close_depth_m:
                            step = min(step, max(min_segment_m, 0.48 * md))

                    sx = pos.x + ux * step
                    sy = pos.y + uy * step

                    cz = float(pos.z)
                    seg_z = cruise_z
                    dz_band = 0.1
                    if cz < seg_z - dz_band:
                        seg_z = cz + dz_band
                    elif cz > seg_z + dz_band:
                        seg_z = cz - dz_band
                    seg_z = clamp(seg_z, min_alt, cruise_z)

                    goal = self._ego_make_goal(sx, sy, seg_z)
                    self._ego_goal_pub.publish(goal)
                    rospy.loginfo(
                        "[EGO] Segment goal (%.2f, %.2f) z=%.2f step=%.2fm depth=%.2f | remain=%.2fm",
                        sx,
                        sy,
                        seg_z,
                        step,
                        md,
                        dist_final,
                    )

                    seg_deadline = rospy.Time.now().to_sec() + segment_timeout
                    republished_at = rospy.Time.now().to_sec()

                    while not rospy.is_shutdown():
                        now2 = rospy.Time.now().to_sec()
                        pos = self.current_pose.pose.pose.position
                        d_seg = hypot(sx - pos.x, sy - pos.y)
                        d_fin = hypot(gx - pos.x, gy - pos.y)

                        if d_fin <= segment_arrival:
                            self.ego_reached = True
                            rospy.loginfo("[EGO] Final XY reached mid-segment (%.2fm)", d_fin)
                            break

                        if d_seg <= segment_arrival:
                            break

                        if now2 > seg_deadline:
                            rospy.logwarn_throttle(
                                10.0, "[EGO] Segment timeout — next chunk"
                            )
                            break

                        cmd_age = now2 - float(self._last_ego_cmd_time or 0.0)
                        if cmd_age > stale_cmd_s and now2 - republished_at >= republish_gap_s:
                            self._ego_goal_pub.publish(goal)
                            republished_at = now2
                            rospy.logwarn_throttle(
                                10.0, "[EGO] Republished segment (stale pos_cmd)"
                            )

                        rospy.loginfo_throttle(
                            2.0,
                            "[EGO] %s | Δseg=%.2f Δfinal=%.2f z=%.2f",
                            "ACTIVE" if cmd_age < stale_cmd_s else "WAIT",
                            d_seg,
                            d_fin,
                            pos.z,
                        )
                        rospy.sleep(0.06)

                    if self.ego_reached:
                        break

                    rospy.sleep(0.05)

        finally:
            self.ego_active = False
            pos = self.current_pose.pose.pose.position
            self._ego_hold_position_raw(
                pos.x, pos.y, pos.z, radians(self.current_heading)
            )

        rospy.loginfo("[EGO] Navigation finished.")
        return True

    # --- Standard Move Methods (Restored) ---
    def move(self, destination: dict):
        client = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        request = PoseStamped()
        request.header.stamp = rospy.Time.now()
        request.pose.position = Point(x=destination["x"], y=destination["y"], z=destination["z"])
        request.pose.orientation = self.calculate_heading(destination.get("heading", self.home_heading))
        client.publish(request)

    def move_vel(self, velx=0, vely=0, velz=0, heading=None):
        client = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        request = Twist()
        request.linear = Vector3(velx, vely, velz)
        client.publish(request)

    def calculate_heading(self, heading) -> Quaternion:
        yaw = radians(heading)
        q = Quaternion()
        q.z = sin(yaw/2); q.w = cos(yaw/2)
        return q

    def set_parameter(self, name: str, value: float):
        client = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        req = ParamSetRequest(); req.param_id = name; req.value.real = value
        client(req)

    def set_origin(self, origin: dict):
        pub = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
        p = GeoPointStamped(); p.header.stamp = rospy.Time.now(); p.header.frame_id = "global"
        p.position.latitude, p.position.longitude, p.position.altitude = origin["latitude"], origin["longitude"], origin["altitude"]
        pub.publish(p)

    def lidar_pub(self, event): pass # Sim dummy
    def land(self): self.set_mode("LAND")
    def set_ekf_source(self, ekf): 
        client = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        req = CommandLongRequest(); req.command = 42007; req.param1 = ekf; client(req)
