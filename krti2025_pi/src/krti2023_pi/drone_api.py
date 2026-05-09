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
from sensor_msgs.msg import LaserScan, Imu, NavSatFix, Range, PointCloud2
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

        self._ego_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self._ego_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self._ego_raw_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self._ego_camera_pose_pub = rospy.Publisher("/iris/front_camera/depth/pose", PoseStamped, queue_size=10)
        
        rospy.Subscriber("/drone_0_planning/pos_cmd", PositionCommand, self._ego_pos_cmd_cb)
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
    
    def pose_cb(self, msg: Odometry):
        self.current_pose = msg
        q0, q1, q2, q3 = msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
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
        
        # Publish camera pose for EGO
        cp = PoseStamped()
        cp.header = msg.header
        cp.header.frame_id = "world"
        cp.pose = msg.pose.pose
        self._ego_camera_pose_pub.publish(cp)

    def _ego_pos_cmd_cb(self, msg: PositionCommand):
        self._last_ego_cmd_time = rospy.Time.now().to_sec()
        if not self.ego_active: return
        pt = PositionTarget()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = "world"
        pt.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # type_mask: Ignore acceleration and yaw_rate. Use position, velocity, and yaw.
        # IGNORE_AFX(64) + IGNORE_AFY(128) + IGNORE_AFZ(256) + IGNORE_YAW_RATE(2048) = 2496
        pt.type_mask = 2496 
        
        pt.position = msg.position
        pt.velocity = msg.velocity
        
        # Calculate yaw to face velocity direction (Face Forward)
        if abs(msg.velocity.x) > 0.1 or abs(msg.velocity.y) > 0.1:
            target_yaw = atan2(msg.velocity.y, msg.velocity.x)
            pt.yaw = target_yaw
        else:
            pt.yaw = self.current_heading * pi / 180.0 # Maintain current
            
        self._ego_raw_pub.publish(pt)

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

    def navigate_ego(self, goal_x, goal_y, exit_alt=1.3, timeout=60, **kwargs):
        rospy.loginfo(f"[EGO] Target: x={goal_x}, y={goal_y}, z={exit_alt}")
        
        # Raise altitude if needed
        while not rospy.is_shutdown() and self.current_pose.pose.pose.position.z < exit_alt - 0.2:
            t = Twist(); t.linear.z = 0.5; self._ego_vel_pub.publish(t); rospy.sleep(0.1)

        # Send Goal to Planner
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "world"
        goal.pose.position = Point(goal_x, goal_y, exit_alt)
        goal.pose.orientation = self.calculate_heading(self.current_heading)
        
        self.ego_active = True
        self._ego_goal_pub.publish(goal)
        rospy.loginfo("[EGO] Goal sent to Planner Swarm.")

        # Progress loop
        deadline = rospy.Time.now().to_sec() + timeout
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            pos = self.current_pose.pose.pose.position
            dist = sqrt((goal_x - pos.x)**2 + (goal_y - pos.y)**2)
            active = (rospy.Time.now().to_sec() - self._last_ego_cmd_time < 1.0)
            rospy.loginfo_throttle(2, f"[EGO] {'ACTIVE' if active else 'WAITING'} | Dist: {dist:.2f}m")
            if dist < 0.4: break
            rospy.sleep(0.1)
        
        self.ego_active = False
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
