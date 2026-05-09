from math import *
import rospy
from std_msgs.msg import Float64, Empty
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist, TwistStamped, Vector3
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped, GeoPoint
from mavros_msgs.msg import State, Thrust, OverrideRCIn, GlobalPositionTarget, WaypointReached, WaypointList
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandLong, CommandLongRequest
from mavros_msgs.srv import ParamSet, ParamSetRequest
from mavros_msgs.srv import ParamGet, ParamGetRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest
from mavros_msgs.srv import StreamRate, StreamRateRequest
from sensor_msgs.msg import LaserScan, Imu, NavSatFix, Range, PointCloud2
from serial import Serial

from pygeodesy.geoids import GeoidPGM
import numpy as np
import threading

_egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

def geoid_height(lat, lon):
    """Calculates AMSL to ellipsoid conversion offset.
    Uses EGM96 data with 5' grid and cubic interpolation.
    The value returned can help you convert from meters 
    above mean sea level (AMSL) to meters above
    the WGS84 ellipsoid.
    If you want to go from AMSL to ellipsoid height, add the value.
    To go from ellipsoid height to AMSL, subtract this value.
    """
    return _egm96.height(lat, lon)
# IF USING RASPI
# import board
# import busio
# import adafruit_vl53l0x as vl53l0x

# IF USING KHADAS
# https://github.com/pimoroni/vl53l1x-python
# import VL53L1X as VL53
# import VL53L0X as VL53

DEBUG_PERIODE = 1
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

# WIP
class PID:
    def __init__(self, kp, ki=0, kd=0, dt=0, max_error=2, name=""):
        self.name = name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.last_error = 0
        self.max_error = max_error

    
    def update(self, error)->float:
        self.error = error
        self.error_sum += error
        self.error_diff = error - self.last_error
        self.last_error = error
        if(self.max_error != 0):
            self.error_sum = clamp(self.error_sum, -self.max_error, self.max_error)
        pid_val = self.kp * self.error + self.ki * self.error_sum + self.kd * self.error_diff
        pid_val = clamp(pid_val, -2, 2)
        # rospy.logdebug("PID {}: error: {}, error_sum: {}, pid_val: {}".format(self.name, self.error, self.error_sum, pid_val))
        return pid_val
    
    def reset(self):
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.last_error = 0

class DroneAPI:
    """
    Control Functions
    This module is designed to make high level control programming simple.
    """

    """
        TODO:
            1. Emergency cancel (stop all movement) if LIDAR detects an obstacle
               within 1 meter (http://wiki.ros.org/mavros#mavros.2FPlugins.local_position)

    """
    
    def __init__(
        self,
        waypoints: list = [],
        global_position: dict = {
            "latitude": -7.265572783693384,
            "longitude": 112.78452265474213,
            "altitude": 1,
        },
        parameters: dict = {},
        sim:bool = True,

    ) -> None:
        """
        Initialize the drone API

        Args:
            waypoints (list): list of waypoints
            global_position (dict): global position
            parameters (dict): parameters to set
        """
        self.sim = sim
        # Set waypoints
        self.current_waypoint = 0
        self.follow_waypoint = True
        self.waypoints = waypoints
        self.gps = NavSatFix()
        global_pos = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_cb)
        rospy.sleep(3)
        
        # Set state
        state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.current_state = State()

        # mission in auto mode related
        wp_reached_sub = rospy.Subscriber("/mavros/mission/reached", WaypointReached, self.mission_wp_reached_cb)
        self.wp_reached = WaypointReached()
            
        # Wait for connection
        self.wait4connect()

        # Set origin
        # we need to sleep for a while otherwise
        # '0' will be stored in 'now'
        # so the loop will stop immediately
        rospy.sleep(0.2)
        if self.gps.status != -1:
            rospy.loginfo(f"current global position lat: {self.gps.latitude}, lon: {self.gps.longitude}")
        else:
            now = rospy.Time.now()
            while rospy.Time.now() - now < rospy.Duration(1.0):
                rospy.logdebug_throttle(0.2,"set origin")
                self.set_origin(global_position)

        # Set parameters
        for name, value in parameters.items():
            self.set_parameter(name, value)
        
        # Set current pose
        self.imu_heading = -1
        self.current_pose = Odometry()
        self.current_heading = 0.0
        self.local_desired_heading = 0.0
        self.home_heading = -1.0
        self.home_compass = -1.0
        pose_sub = rospy.Subscriber(
            "/mavros/local_position/odom",
            Odometry,
            self.pose_cb,
        )

        self.compass = Float64().data
        compass_sub = rospy.Subscriber(
            "/mavros/global_position/compass_hdg",
            Float64,
            self.compass_cb,
        )

        # set Velocity
        self.current_velocity = TwistStamped()
        velocity_sub = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.velocity_cb)

        # set IMU
        imu_sub = rospy.Subscriber('/mavros/imu/data/', Imu, self.imu_cb)
        self.imu = Imu()

        # LIDAR data
        self.lidar_queue = []
        self.lidar_data = LaserScan()
        # if self.sim:
        #     lidar_sub = rospy.Subscriber("/sensors/lidar/sim", LaserScan, self.lidar_cb)
        # else:
        #     self.setup_lidar()


        rospy.Timer(rospy.Duration(0.05), self.lidar_pub)
        
        # ultrasonic 
        # self.ultrasonic = float()
        # self.ser = Serial('/dev/ttyS0', baudrate=115200) 
        # rospy.Timer(rospy.Duration(0.1), self.ultrasonic_cb)

        self.previous_pose = Odometry()

        self.yaw_pid = PID(0.1,0,0.01,0, max_error=0.5)

        # rangefinder reading from pixhawk
        self.rangefinder = Range()
        rangefinder_sub = rospy.Subscriber("/mavros/rangefinder/rangefinder", Range, self.rangefinder_cb)

        # ── EGO-Planner interface ──────────────────────────────────────────────
        # Parameter EGO-Planner
        self.ego_v_max   = 2.0   # m/s  kecepatan maks drone
        self.ego_a_max   = 3.0   # m/s² akselerasi maks drone
        self.ego_r_safe  = 0.3   # m    jarak aman minimum ke obstacle
        self.ego_dt      = 0.1   # s    interval antar control point B-spline
        self.ego_lam_s   = 10.0  # bobot smoothness cost
        self.ego_lam_c   = 0.5   # bobot collision cost
        self.ego_lam_d   = 0.5   # bobot dynamic feasibility cost
        self.ego_lr      = 0.05  # learning rate gradient descent
        self.ego_iters   = 200   # jumlah iterasi optimasi

        # State EGO-Planner
        self.ego_reached      = False
        self.ego_active       = False
        self.ego_esdf_map     = {}   # dict {(ix,iy,iz): distance} grid 0.1m
        self.ego_esdf_res     = 0.1  # resolusi grid ESDF (meter)
        self.ego_traj_pos     = []   # list posisi trajectory hasil optimasi
        self.ego_traj_vel     = []   # list velocity trajectory hasil optimasi
        self.ego_traj_idx     = 0    # index eksekusi trajectory sekarang
        self._ego_lock        = threading.Lock()

        # Publisher goal ke EGO-Planner (jika pakai external node)
        self.ego_goal_pub = rospy.Publisher("/goal_with_id", PoseStamped, queue_size=1)

        # Publisher cmd_vel hasil trajectory EGO (internal)
        self._ego_vel_pub = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10
        )

        # Subscriber pointcloud dari kamera depan → bangun ESDF
        rospy.Subscriber("/iris/iris/front_camera/points", PointCloud2, self._ego_cloud_cb)

        # Subscriber sinyal "sudah sampai" dari EGO-Planner external (opsional)
        rospy.Subscriber("/planning/reach_end", Empty, self._ego_reached_cb)
        # ── end EGO-Planner ───────────────────────────────────────────────────

        # Print success
        rospy.loginfo("Initialization completed.")
        
    def set_home(self):
        self.home_gps = self.gps
        self.home_compass = self.compass
        self.home_heading = self.current_heading

    def gps_cb(self, data: NavSatFix):
        self.gps = data
        
    
    def ultrasonic_cb(self, msg: float):
        try:
            self.ultrasonic = self.ser.readline().decode('utf-8').split('\n')[0]
            rospy.loginfo(self.ultrasonic)
        except:
            self.ultrasonic = -1

    def move_vel(self, velx = 0, vely = 0, velz = 0, heading :float = None):
        
        cur_pose = self.current_pose
        # Get client
        client = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel_unstamped",
            Twist,
            queue_size=10,
        )
        # rospy.loginfo_throttle(0.3,f"position : {self.current_pose}")
        # rospy.loginfo_throttle(0.3,f"heading : {self.current_heading}")
        
        # If have heading then we set the heading
        if heading is None:
            heading = self.home_heading

        # Set position
        request = Twist()
        request.linear = Vector3(velx,vely,velz)    
        self.local_desired_heading = heading
        rospy.loginfo(f"Move with velocity:\n{request.linear}")
        print(heading)
        print(self.current_heading)
        err_head = heading - self.current_heading

        # if err_head > 180:
        #     err_head -= 360
        # elif err_head < -180:
        #     err_head += 360
        
        # val = self.yaw_pid.update(-err_head)
        val = 0
        request.angular.z = val

        # Send request
        # rospy.logdebug("publishing setpoint_velocity/cmd_vel_unstamped", logger_name="move_vel")
        client.publish(request)
        # rospy.loginfo_throttle_identical(0.1,
        #     f"cmd_vel to x: {velx}; y: {vely}; z: {velz}, yaw: {val}"
        # )
    
    def imu_cb(self, msg:Imu):
        """
        A function for IMU's subscriber callback
        Will set self.imu to the message received
        used in detecting stable motion
        """
        self.imu = msg

    def rangefinder_cb(self, msg: Range):
        self.rangefinder = msg.range
        # rospy.loginfo_throttle(0.3, f"rangefinder : {self.rangefinder}")

    def compass_cb(self, msg: Float64):
        """
        A function for Compass's subscriber callback
        Will set self.ompass to the message received
        used in detecting stable motion
        """
        self.compass = msg.data   
    def lidar_cb(self, data: LaserScan):
        """
        A function for receiving Lidar data from sensors node
        [ 1 2 3 4] consist of 4 range [front, left, back, right] sequentially
        needed for obstacle avoidance
        """
        self.lidar_data = data.ranges
    
    def lidar_pub(self, data):
        """
        A function for publishing Lidar data
        """
        data = LaserScan()
        
        if not self.sim:
            range = self.tof.get_distance()
            self.lidar_queue.append(range)
            
            if len(self.lidar_queue) > 1:
                self.lidar_queue.pop(0)
            
            data.ranges.append(sum(self.lidar_queue) / len(self.lidar_queue))
            self.lidar_data = data

        else:
            data.ranges = self.lidar_data
    
    def state_cb(self, msg):
        """
        A function for state's subscriber callback
        Will set self.current_state to the message received
        """
        self.current_state = msg
    
    def velocity_cb(self, msg:TwistStamped):
        """
        A function for velocity's subscriber callback
        
        """
        self.current_velocity = msg

    def pose_cb(self, msg: Odometry):
        """
        Gets the raw pose of the drone and processes it for use in control.

        Args:
                msg (nav_msgs/Odometry): Raw pose of the drone.
        """
        # Set current pose
        self.current_pose = msg

        # Calculate heading from quarternion to degrees
        q0, q1, q2, q3 = (
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)), (1 - 2 * (pow(q2, 2) + pow(q3, 2))))
        # rospy.logdebug_throttle(0.3, "hdg = " + str(degrees(psi)))
        # rospy.logdebug_throttle(0.3, f"hdg = {self.imu_heading}")

        # Set current heading
        self.imu_heading = degrees(psi)
        self.current_heading = self.imu_heading

        # Set home heading
        if self.home_heading == -1.0:
            self.home_compass = self.compass
            self.home_heading = self.current_heading
            self.local_desired_heading = self.home_heading
            # print("home heading : ", self.home_heading)

    def set_origin(self, origin: dict):
        """
        A function to set origin to custom coordinates
        We need to set this if we're flying without GPS

        Args:
            origin (dict): origin coordinates
                - latitude
                - longitude
                - altitude

        Refer to https://discuss.ardupilot.org/t/guided-mode-with-optical-flow-without-gps-in-simulation/53494/6
        """

        # Get client
        setter = rospy.Publisher(
            "/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10
        )

        # Set position
        position = GeoPointStamped()
        position.header.frame_id = "global"
        position.header.stamp = rospy.Time.now()
        position.position.latitude = origin["latitude"]
        position.position.longitude = origin["longitude"]
        position.position.altitude = origin["altitude"]

        setter.publish(position)
    
    def set_rc_override(self, values: dict = {}):
        client = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=10)
        ch = OverrideRCIn()
        if len(values) != 0:
            for i in values:
                ch.channels[i-1] = values[i]
        rospy.logdebug_throttle(0.2, f"ch override {ch}")
        client.publish(ch)
        


    def get_home_heading(self):
        return self.home_compass

    def get_parameter(self, name: str):
        """
        A function to get parameters

        Args:
            name (str): name of parameter
            value (float): value of parameter
        """

        # Get client
        client = rospy.ServiceProxy("/mavros/param/get", ParamGet)

        # get parameter
        request = ParamGetRequest()
        request.param_id = name

        # Send request
        response = ParamGet()
        response = client(request)
        return response.value

    def stable_motion(self):
        z = self.imu.linear_acceleration.z
        if(z > 9.68 and z < 9.9):
            return True
        return False
    
    def set_parameter(self, name: str, value: float):
        """
        A function to set parameters

        Args:
            name (str): name of parameter
            value (float): value of parameter
        """

        # Get client
        client = rospy.ServiceProxy("/mavros/param/set", ParamSet)

        # Set parameter
        request = ParamSetRequest()
        request.param_id = name
        request.value.real = value

        # Send request
        client(request)
    
    def use_gps(self,use:bool = True):
        self.set_parameter("AHRS_GPS_USE",1 if use else 0)
    
    def set_stream_rate(self, rate: int = 10):
        client = rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)
        request = StreamRateRequest(0, 100, 1)
        # request.message_rate = rate
        # request.on_off = 1
        # request.stream_id = 0
        
        client(request)

    def set_mode(self, mode: str = "GUIDED"):
        """
        A function to set mode

        Args:
            mode (str): mode to set. Default to GUIDED
        """

        # Get client
        rospy.wait_for_service("/mavros/set_mode")
        client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Set mode
        client(SetModeRequest(0, mode))

        # Check if mode is set
        if self.current_state.mode == mode:
            # Print success message
            rospy.loginfo(f"Mode is set to {mode}")
        else:
            # Print failed message
            rospy.loginfo(f"Failed to set mode to {mode}")
            rospy.loginfo(f"Current mode is {self.current_state.mode}")

    def set_thrust(self, thrust: float):
        """
        A function to set thrust

        Args:
            thrust (float): thrust to set. Value should be between 0 and 1
        """

        # Check if thrust is not in range
        if thrust < 0 or thrust > 1:
            print("Illegal thrust value. It should be between 0 and 1 (inclusive).")
            return

        # Get client
        client = rospy.Publisher(
            "/mavros/setpoint_attitude/thrust", Thrust, queue_size=10
        )

        # Create thrust message
        request = Thrust()
        request.header.stamp = rospy.Time.now()
        request.thrust = thrust

        # Set thrust
        client.publish(request)

    def arm(self, status: bool = True):
        """
        A function to arm or disarm the drone

        Args:
            status (bool): True to arm, False to disarm
        """

        # Get client
        rospy.wait_for_service("/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        
        # Arm
        while not rospy.is_shutdown() and not self.current_state.armed:
            arming_client(CommandBoolRequest(status))
        else:
            if status == True:
                rospy.loginfo("Drone is armed and ready to fly")
            else:
                rospy.loginfo("Drone is disarmed")

    def  takeoff(self, altitude: float = 3.0):
        """
        A function to give drone a takeoff command

        Args:
            altitude (float): altitude to takeoff to
        """

        rospy.loginfo("drone current altitude = " + str(self.current_pose.pose.pose.position.z))
        # Arm drone
        self.arm()
        print(f"current altitude = {self.current_pose.pose.pose.position.z}")

        if self.current_pose.pose.pose.position.z > altitude * 0.95 - 0.2:
            rospy.loginfo("altitude already reached")
            return 1
        
        if self.current_pose.pose.pose.position.z > 0.45:
            rospy.loginfo("drone already in air")
            return 1
        
        # Get client
        rospy.wait_for_service("/mavros/cmd/takeoff")
        takeoff_client = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

        # Takeoff
        rospy.loginfo("Taking off ...")
        response = takeoff_client(CommandTOLRequest(0, 0, 0, 0, altitude))
        rospy.sleep(3)

        if response.success:
            rospy.loginfo("Taking off ...")

            # We will return if we are 95% of the way to the target altitude
            while self.current_pose.pose.pose.position.z < altitude * 0.95 - 0.2:
                rospy.loginfo_throttle(0.1,f"current altitude = {self.current_pose.pose.pose.position.z}")

                rospy.sleep(0.1)

            return 1
        
        rospy.logerr("Takeoff failed")
        return 0
        

    def land(self):
        """
        A function to give drone a land command
        """

        # Get client
        rospy.wait_for_service("/mavros/cmd/land")
        client = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)

        # Landing
        client(CommandTOLRequest(0, 0, 0, 0, 0))

        # Print success message
        rospy.loginfo(
            "Landing command sent. Drone should be disarming itself in 10-15 seconds after it touches the ground. ..."
        )

    def stop(self):
        rospy.loginfo("Stopping")
        home_heading = radians(self.get_home_heading())

        x,y,z = self.body2local(0, 0, 0, home_heading)
        dist = {"x": x, "y": y, "z": z, "heading": home_heading}

        # Move command        
        for _ in range(30):
            self.move(dist)
            rospy.sleep(0.1)

        rospy.loginfo("Stop command sent, aircraft should be stopped in a moment")

    def body2local(self, x, y, heading):
        return x * cos(heading) - y * sin(heading), x * sin(heading) + y * cos(heading)
    
    def move(self, destination: dict = None):
        """
            IMPORTANT NOTES:
        - in simulation we need to send the msg  multiple times to make it work
        - in real drone we shouldn't send the msg multiple times
        - ONLY USE MAV_FRAME "LOCAL_NED".

        A function to move the drone to certain position

        Args:
            destination (dict | None): destination coordinates
                - x
                - y
                - z
                - heading (optional, if not passed then it will use the current heading)
            wait_reached: wait until the drone reached the setpoint
                If no destination passed then drone will go to
                current destination in the waypoint list
        """
        rospy.loginfo(f"testing home heading on move method : {self.home_heading}")
        self.previous_pose = self.current_pose
        ref_pose = self.current_pose.pose.pose.position
        # Get client
        client = rospy.Publisher(
            "/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=10,
        )
        rospy.loginfo_throttle(0.2,f"position : {self.current_pose} ")
        rospy.loginfo_throttle(0.2,f"heading : {self.current_heading} ")
        # If no destination is given, use current destination
        # indicated by current waypoint index
        if destination == None:
            destination = self.waypoints[self.current_waypoint]

        # If have heading then we set the heading
        if "heading" in destination:
            heading = destination["heading"]
        else:
            heading = self.home_heading

        # Set position
        request = PoseStamped()
        request.header.stamp = rospy.Time.now()
        request.pose.position = Point(
            x=destination["x"], y=destination["y"], z=destination["z"]
        )

        request.pose.orientation = self.calculate_heading(heading)
        # self.local_desired_heading = heading

        # # === khusus BODY_NED ===
        # cur_waypoint = destination
        # cur_waypoint['x'] += ref_pose.x
        # cur_waypoint['y'] += ref_pose.y
        # cur_waypoint['z'] += ref_pose.z
        # cur_waypoint['heading'] = self.home_heading
        # # === end of khusus BODY_NED ===


        # Send request
        rospy.logdebug("publishing setpoint_position/local", logger_name="move")
        client.publish(request)
        rospy.loginfo(
            f"Moving to x: {destination['x']}; y: {destination['y']}; z: {destination['z']}"
        )

    def move_global_raw(self, coordinate: GeoPoint, heading = None):
        """
            IMPORTANT NOTES:
        - in simulation we need to send the msg  multiple times to make it work
        - in real drone we shouldn't send the msg multiple times but sometimes we need to

        A function to move the drone to certain position in global frame

        """

        # Get client
        client = rospy.Publisher(
            "/mavros/setpoint_raw/global",
            GlobalPositionTarget,
            queue_size=10,
        )

        # http://docs.ros.org/en/api/mavros_msgs/html/msg/GlobalPositionTarget.html
        request = GlobalPositionTarget()
        request.header.stamp = rospy.Time.now()

        request.coordinate_frame = 3
        request.latitude = coordinate.latitude
        request.longitude = coordinate.longitude
        request.altitude = coordinate.altitude
        
        rospy.loginfo_throttle(0.2,f"latitude : {self.gps.latitude} ")
        rospy.loginfo_throttle(0.2,f"longitude : {self.gps.longitude} ")
        rospy.loginfo_throttle(0.2,f"altitude : {self.gps.altitude} ")
        rospy.loginfo_throttle(0.2,f"position : {self.current_pose} ")
        rospy.loginfo_throttle(0.2,f"heading : {self.compass} ")

        if heading is not None:
            request.yaw = radians(heading)
            self.local_desired_heading = heading
        else:
            request.yaw = radians(self.home_compass)
        
        request.type_mask = 1024 # ignore yaw
        
        # Send request
        rospy.logdebug("publishing setpoint_raw/global", logger_name="move_global")
            
        for i in range(30):
            client.publish(request)
            rospy.sleep(0.01)
            
    def move_global(self, coordinate: GeoPoseStamped = None, heading = None,lat:float=None,lon:float=None, alt:float=None):
        """
            IMPORTANT NOTES:
        - in simulation we need to send the msg  multiple times to make it work
        - in real drone we shouldn't send the msg multiple times but sometimes we need to
        A function to move the drone to certain position in global frame
        """

        # Get client
        client = rospy.Publisher(
            "/mavros/setpoint_position/global",
            GeoPoseStamped,
            queue_size=10,
        )
        # coordinate.altitude = gps.altitude-geoid_height(gps.latitude,gps.longitude)+alt
        request = GeoPoseStamped()
        if coordinate is not None:
            request = coordinate
            # request.pose.position.altitude = self.home_gps.altitude - geoid_height(self.gps.latitude,self.gps.longitude) + request.pose.position.altitude
            # request.pose.position.altitude = request.pose.position.altitude
        elif lat is not None and lon is not None and alt is not None:
            request.pose.position.latitude = lat
            request.pose.position.longitude = lon
            request.pose.position.altitude = self.home_gps.altitude - geoid_height(self.gps.latitude,self.gps.longitude) + alt
        request.header.stamp = rospy.Time.now()
        if heading is not None:
            request.pose.orientation = self.calculate_heading(heading)
        else:
            request.pose.orientation = self.calculate_heading(self.home_compass)

        rospy.logdebug("publishing setpoint_position/global", logger_name="move_global")
        for i in range(30):
            client.publish(request)
            rospy.sleep(0.01)

    def send_mavlink_command(self, request: CommandLongRequest):
        """
        A function to send custom MAVLink command.

        Args:
            - request (CommandLongRequest): command and its param
        """
        # Get client
        client = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

        # Send request
        response = client(request)

        return response

    def switch_relay(self, relay: int = 0, status: bool = True):
        """
        A function to switch the relay on/off connected to the autopilot
        this function send a commandLong msg to the autopilot

        Args:
            - relay (int): relay number "index started from 0" default 0
            - status (bool): True to turn on, False to turn off default True
        """

        # get parameter
        request = CommandLongRequest()
        request.command = 181 # MAV_CMD_DO_SET_RELAY
        request.param1 = relay
        request.param2 = 1 if status else 0

        return self.send_mavlink_command(request)
    
    def set_servo(self, servo: int = 9, pwm: int = 1100):
        """
        A function to set the servo pwm value connected to the autopilot
        this function send a commandLong msg to the autopilot

        Args:
            - servo (int):  servo number "index started from 9 to 13", default 9 
                            servo 9-13 means aux out 1-4
            - pwm (int): pwm value to set,  default 1100
        """
        # Get client
        client = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

        # get parameter
        request = CommandLongRequest()
        request.command = 183 # MAV_CMD_DO_SET_SERVO
        request.param1 = servo
        request.param2 = pwm 
        client(request)

        #return self.send_mavlink_command(request)
    
    def set_ekf_source(self, ekf: int = 1):
        """
        A function to switch the relay on/off connected to the autopilot
        this function send a commandLong msg to the autopilot

        Args:
            - ekf (int): EKF source to be used, min value = 0, max value = 3
        """
        valid_ekf = [1,2,3]
        if ekf not in valid_ekf:
            rospy.logerr(f"Invalid EKF source {ekf}. Valid EKF source value is {valid_ekf}")
            return

        # get parameter
        request = CommandLongRequest()
        request.command = 42007 # MAV_CMD_SET_EKF_SOURCE_SET
        request.param1 = ekf

        return self.send_mavlink_command(request)

    def set_speed(self, type:int = 1, speed:int=-2, throttle:int = -1):
        """
        A function to change/set the speed of vehicle
        this function send a commandLong msg to the autopilot

        Args:
            - type (int) : Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
            - speed(int) (m/s): (-1 indicates no change, -2 indicates return to default vehicle speed)
            - throttle :  (-1 indicates no change, -2 indicates return to default vehicle throttle value)
        """

        
        # Get client
        client = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)

        # get parameter
        request = CommandLongRequest()
        request.command = 178 # MAV_CMD_DO_CHANGE_SPEED
        request.param1 = type
        request.param2 = speed
        request.param3 = throttle
        client(request) 

        return self.send_mavlink_command(request)

    def next(self):
        """
        A function to move to next waypoint
        """
        self.current_waypoint += 1
        if self.current_waypoint >= len(self.waypoints) or len(self.waypoints) == 0 or self.current_waypoint > len(self.waypoints)-1:
            return False
        return True

    def mission_wp_reached_cb(self, msg):
        self.wp_reached = msg
        
    def get_wp_reached(self):
        return self.wp_reached.wp_seq
    
    def check_waypoint_reached(self,destination: dict = None, pos_tol = 0.1, head_tol = 0.4):
        """This function checks if the waypoint is reached within given tolerance and returns an int of 1 or 0. This function can be used to check when to request the next waypoint in the mission.
        Args:
                pos_tol (float, optional): Position tolerance under which the drone must be with respect to its position in space. Defaults to 0.03.
                head_tol (float, optional): Heading or angle tolerance under which the drone must be with respect to its orientation in space. Defaults to 0.01.
        Returns:
                1 (int): Waypoint reached successfully.
                0 (int): Failed to reach Waypoint.
        """
        if destination == None:
            destination = self.waypoints[self.current_waypoint]

        dx = abs(self.previous_pose.pose.pose.position.x + destination["x"] - self.current_pose.pose.pose.position.x)
        dy = abs(self.previous_pose.pose.pose.position.y + destination["y"] - self.current_pose.pose.pose.position.y)
        dz = abs(self.previous_pose.pose.pose.position.z + destination["z"] - self.current_pose.pose.pose.position.z)

        dMag = sqrt(pow(dx, 2) + pow(dy, 2)) # so the altitude tolerance is just 0.3 in default

        cosErr = cos(radians(self.current_heading)) - cos(
            radians(self.local_desired_heading)
        )

        sinErr = sin(radians(self.current_heading)) - sin(
            radians(self.local_desired_heading)
        )

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))
        rospy.logdebug_throttle(0.1,f"dx:{dx}, dy:{dy}, dz:{dz}")
        
        # with heading check
        if dMag < pos_tol and dHead < head_tol:
            return 1
        else:
            return 0

    def check_waypoint_reached_global(self,coordinate:GeoPoint ,pos_tol = 2.5, head_tol = 0.4):
        """This function checks if the waypoint is reached within given tolerance and returns an int of 1 or 0. This function can be used to check when to request the next waypoint in the mission.
        Args:
                pos_tol (float, optional): Position tolerance under which the drone must be with respect to its position in space. Defaults to 0.03.
                head_tol (float, optional): Heading or angle tolerance under which the drone must be with respect to its orientation in space. Defaults to 0.01.
        Returns:
                1 (int): Waypoint reached successfully.
                0 (int): Failed to reach Waypoint.
        """
        lat2 = self.gps.latitude
        lon2 = self.gps.longitude
        lat1 = coordinate.latitude
        lon1 = coordinate.longitude
        dist = acos(sin(radians(lat1))*sin(radians(lat2))+cos(radians(lat1))*cos(radians(lat2))*cos(radians(lon2-lon1)))*6400000

        cosErr = cos(radians(self.current_heading)) - cos(
            radians(self.local_desired_heading)
        )

        sinErr = sin(radians(self.current_heading)) - sin(
            radians(self.local_desired_heading)
        )

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))
        rospy.logdebug_throttle(0.2,f"dist from wp :{dist}")
        
        # with heading check
        if dist < pos_tol :
            return 1
        else:
            return 0
        
    def wait4connect(self):
        """
        Wait for connect is a function that will hold the program until communication with the FCU is established.
        Returns:
                0 (int): Connected to FCU.
                -1 (int): Failed to connect to FCU.
        """
        rospy.loginfo("Waiting for FCU connection")
        while not rospy.is_shutdown() and not self.current_state.connected:
            print("connecting")
            rospy.sleep(0.01)
        else:
            if self.current_state.connected:
                rospy.loginfo("FCU connected")
                return 0
            else:
                rospy.logerr("Error connecting to drone's FCU")
                return -1

    def wait4start(self):
        """
        This function will hold the program until the user signals the FCU to mode enter GUIDED mode. This is typically done from a switch on the safety pilot's remote or from the Ground Control Station.
        Returns:
                0 (int): Mission started successfully.
                -1 (int): Failed to start mission.
        """
        rospy.loginfo("Waiting for user to set mode to GUIDED")

        while not rospy.is_shutdown() and self.current_state.mode != "GUIDED":
            rospy.sleep(0.01)
        else:
            # We will not start if mode is not GUIDED and home heading is not set
            if self.current_state.mode == "GUIDED" and self.home_heading != -1.0:
                rospy.loginfo("Mode set to GUIDED. Starting Mission...")
                return 0
            else:
                rospy.logerr("Error starting mission")
                return -1

    def calculate_heading(self, heading) -> Quaternion:
        """
        This function is used to specify the drone's heading in the local reference frame. Psi is a counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.
        Args:
                heading (Float): θ(degree) Heading angle of the drone.
        """
        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        # cy = cos(yaw * 0.5)
        # sy = sin(yaw * 0.5)

        # cr = cos(roll * 0.5)
        # sr = sin(roll * 0.5)

        # cp = cos(pitch * 0.5)
        # sp = sin(pitch * 0.5)

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        q = Quaternion()
        q.x, q.y, q.z, q.w = qx, qy, qz, qw
        
        return q

    def set_heading(self, heading:float):
        self.local_desired_heading = heading

    # ═══════════════════════════════════════════════════════════════════════════
    # EGO-PLANNER — Internal Implementation
    # ═══════════════════════════════════════════════════════════════════════════

    def _ego_reached_cb(self, msg):
        """Callback: EGO-Planner external node melaporkan drone sudah sampai goal."""
        self.ego_reached = True

    def _ego_cloud_cb(self, msg: PointCloud2):
        """
        Callback pointcloud dari kamera depan.
        Bangun ESDF sederhana: untuk setiap titik obstacle, update jarak
        ke semua sel grid di sekitarnya (dalam radius r_safe + margin).

        ESDF grid disimpan sebagai dict {(ix, iy, iz): jarak_terdekat}
        """
        import struct
        res  = self.ego_esdf_res
        r    = self.ego_r_safe + 0.5   # radius update sekitar obstacle
        step = int(r / res) + 1
        new_map = {}

        # Baca setiap titik dari PointCloud2 (format XYZ float32)
        point_step = msg.point_step
        data       = msg.data
        for i in range(msg.width * msg.height):
            off = i * point_step
            ox, oy, oz = struct.unpack_from('fff', data, off)
            if not (isfinite(ox) and isfinite(oy) and isfinite(oz)):
                continue
            # Konversi ke index grid
            cx = int(ox / res)
            cy = int(oy / res)
            cz = int(oz / res)
            # Update sel-sel di sekitar obstacle
            for dx in range(-step, step + 1):
                for dy in range(-step, step + 1):
                    for dz in range(-step, step + 1):
                        key  = (cx + dx, cy + dy, cz + dz)
                        dist = sqrt(dx*dx + dy*dy + dz*dz) * res
                        if key not in new_map or dist < new_map[key]:
                            new_map[key] = dist

        with self._ego_lock:
            self.ego_esdf_map = new_map

    def _ego_esdf_query(self, px: float, py: float, pz: float):
        """
        Query ESDF: kembalikan (jarak, gradien_x, gradien_y, gradien_z)
        di posisi (px, py, pz).

        Gradien dihitung dengan finite difference pada grid ESDF:
            ∇d(p) ≈ (d(p+ε) - d(p-ε)) / (2ε)
        """
        res = self.ego_esdf_res
        eps = res

        def _d(x, y, z):
            if not (isfinite(x) and isfinite(y) and isfinite(z)):
                return 10.0
            key = (int(x/res), int(y/res), int(z/res))
            return self.ego_esdf_map.get(key, 10.0)

        with self._ego_lock:
            d   = _d(px, py, pz)
            gx  = (_d(px+eps, py, pz) - _d(px-eps, py, pz)) / (2*eps)
            gy  = (_d(px, py+eps, pz) - _d(px, py-eps, pz)) / (2*eps)
            gz  = (_d(px, py, pz+eps) - _d(px, py, pz-eps)) / (2*eps)
        return d, gx, gy, gz

    def _ego_init_bspline(self, start, goal, n_ctrl=10):
        """
        Buat initial trajectory sebagai B-spline linear dari start ke goal.
        Kembalikan list control points shape (n_ctrl, 3).

        Control point ke-i:
            Q_i = start + (i / (n_ctrl-1)) * (goal - start)
        """
        ctrl = []
        for i in range(n_ctrl):
            t = i / (n_ctrl - 1)
            q = [start[j] + t * (goal[j] - start[j]) for j in range(3)]
            ctrl.append(q)
        return ctrl  # list of [x, y, z]

    def _ego_bspline_eval(self, ctrl, dt):
        """
        Evaluasi cubic uniform B-spline dari control points.
        Kembalikan (positions, velocities) sebagai list.

        Untuk setiap segment i (dari 1 sampai n-3):
            p(t) = (1/6) * M_cubic * [Q_{i-1}, Q_i, Q_{i+1}, Q_{i+2}]^T
            v(t) = dp/dt

        Matrix cubic B-spline M:
            M = (1/6) * [[-1, 3,-3, 1],
                         [ 3,-6, 3, 0],
                         [-3, 0, 3, 0],
                         [ 1, 4, 1, 0]]
        """
        M = np.array([
            [-1,  3, -3,  1],
            [ 3, -6,  3,  0],
            [-3,  0,  3,  0],
            [ 1,  4,  1,  0]
        ], dtype=float) / 6.0

        positions  = []
        velocities = []
        n = len(ctrl)

        for i in range(1, n - 2):
            Q = np.array(ctrl[i-1:i+3], dtype=float)  # shape (4,3)
            # Evaluasi di t=0.5 (tengah segment)
            for t_norm in [0.0, 0.5]:
                T_pos = np.array([t_norm**3, t_norm**2, t_norm, 1.0])
                T_vel = np.array([3*t_norm**2, 2*t_norm, 1.0, 0.0]) / dt
                pos = T_pos @ M @ Q
                vel = T_vel @ M @ Q
                positions.append(pos.tolist())
                velocities.append(vel.tolist())

        return positions, velocities

    def _ego_optimize(self, ctrl):
        """
        Optimasi control points dengan gradient descent.

        Total cost:
            J = λ_s * J_smooth + λ_c * J_collision + λ_d * J_dynamic

        Gradient update:
            Q_i ← Q_i - lr * ∂J/∂Q_i

        ── Smoothness cost (minimasi jerk) ──
            J_smooth = Σ ||Q_{i+3} - 3Q_{i+2} + 3Q_{i+1} - Q_i||²
            ∂J_smooth/∂Q_i dihitung per control point

        ── Collision cost (ESDF) ──
            Jika d(Q_i) < r_safe:
                J_coll_i = (d(Q_i) - r_safe)²
                ∂J_coll/∂Q_i = 2*(d-r_safe) * ∇d

        ── Dynamic feasibility cost ──
            v_i = (Q_{i+1} - Q_{i-1}) / (2*dt)
            Jika ||v_i|| > v_max:
                J_dyn_i = (||v_i|| - v_max)²
                ∂J_dyn/∂Q_i = 2*(||v_i||-v_max) * v_i/||v_i|| * (1/(2*dt))
        """
        dt   = self.ego_dt
        rs   = self.ego_r_safe
        vmax = self.ego_v_max
        ls   = self.ego_lam_s
        lc   = self.ego_lam_c
        ld   = self.ego_lam_d
        lr   = self.ego_lr
        n    = len(ctrl)

        Q = np.array(ctrl, dtype=float)  # shape (n, 3)

        for _ in range(self.ego_iters):
            grad = np.zeros_like(Q)

            # ── Smoothness: jerk = Q_{i+3} - 3Q_{i+2} + 3Q_{i+1} - Q_i ──
            for i in range(n - 3):
                jerk = Q[i+3] - 3*Q[i+2] + 3*Q[i+1] - Q[i]
                g    = 2.0 * jerk
                grad[i]   -= ls * g
                grad[i+1] += ls * 3 * g
                grad[i+2] -= ls * 3 * g
                grad[i+3] += ls * g

            # ── Collision + Dynamic: loop per control point ──
            for i in range(1, n - 1):
                px, py, pz = Q[i]

                # Collision cost
                d, gx, gy, gz = self._ego_esdf_query(px, py, pz)
                if d < rs:
                    coeff = 2.0 * (d - rs)   # negatif → dorong menjauhi obstacle
                    grad[i][0] += lc * coeff * gx
                    grad[i][1] += lc * coeff * gy
                    grad[i][2] += lc * coeff * gz

                # Dynamic feasibility cost (velocity)
                vi   = (Q[i+1] - Q[i-1]) / (2.0 * dt)
                vnorm = float(np.linalg.norm(vi))
                if vnorm > vmax:
                    coeff = 2.0 * (vnorm - vmax) / (vnorm + 1e-6)
                    dv    = ld * coeff * vi / (2.0 * dt)
                    grad[i-1] -= dv
                    grad[i+1] += dv

            # Update control points (jangan geser start & end)
            Q[1:-1] -= lr * grad[1:-1]

        return Q.tolist()

    def move_ego(self, x: float, y: float, z: float,
                 wait: bool = False, timeout: float = 30.0):
        """
        Navigasi drone ke posisi (x, y, z) menggunakan EGO-Planner.

        Alur:
            1. Ambil posisi drone sekarang sebagai start
            2. Buat initial B-spline linear dari start ke goal
            3. Optimasi control points (smoothness + collision + dynamic)
            4. Evaluasi B-spline → list posisi & velocity
            5. Eksekusi trajectory: publish cmd_vel tiap ego_dt detik
            6. Jika wait=True, tunggu sampai selesai atau timeout

        Args:
            x, y, z  : target posisi dalam frame local (meter)
            wait     : True = blokir sampai drone sampai tujuan
            timeout  : batas waktu tunggu (detik)
        """
        # 1. Posisi start — tunggu sampai odometry valid (bukan NaN)
        deadline = rospy.Time.now().to_sec() + 10.0
        while not rospy.is_shutdown():
            pos = self.current_pose.pose.pose.position
            if isfinite(pos.x) and isfinite(pos.y) and isfinite(pos.z):
                break
            if rospy.Time.now().to_sec() > deadline:
                rospy.logerr("[EGO] Timeout waiting for valid odometry — aborting")
                return
            rospy.sleep(0.05)

        start = [pos.x, pos.y, pos.z]
        goal  = [x, y, z]

        rospy.loginfo(f"[EGO] Start: {start} → Goal: {goal}")

        # 2. Initial B-spline
        ctrl = self._ego_init_bspline(start, goal, n_ctrl=12)

        # 3. Optimasi
        rospy.loginfo("[EGO] Optimizing trajectory...")
        ctrl_opt = self._ego_optimize(ctrl)

        # 4. Evaluasi trajectory
        positions, velocities = self._ego_bspline_eval(ctrl_opt, self.ego_dt)

        rospy.loginfo(f"[EGO] Trajectory ready: {len(positions)} waypoints")

        # 5. Eksekusi trajectory di thread terpisah agar tidak blokir
        self.ego_reached = False
        self.ego_active  = True

        def _execute():
            rate = rospy.Rate(1.0 / self.ego_dt)
            for vel in velocities:
                if rospy.is_shutdown() or not self.ego_active:
                    break
                # Clamp velocity sesuai v_max
                vnorm = sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
                if vnorm > self.ego_v_max:
                    scale = self.ego_v_max / vnorm
                    vel   = [v * scale for v in vel]

                twist = Twist()
                twist.linear = Vector3(vel[0], vel[1], vel[2])
                self._ego_vel_pub.publish(twist)
                rate.sleep()

            # Berhenti setelah trajectory selesai
            stop = Twist()
            self._ego_vel_pub.publish(stop)
            self.ego_reached = True
            self.ego_active  = False
            rospy.loginfo("[EGO] Trajectory execution complete")

        t = threading.Thread(target=_execute, daemon=True)
        t.start()

        # 6. Tunggu jika diminta
        if wait:
            start_t = rospy.Time.now().to_sec()
            while not self.ego_reached and not rospy.is_shutdown():
                if rospy.Time.now().to_sec() - start_t > timeout:
                    rospy.logwarn("[EGO] Timeout! Menghentikan trajectory.")
                    self.ego_active = False
                    break
                rospy.sleep(0.1)

    def ego_stop(self):
        """Hentikan eksekusi trajectory EGO-Planner secara paksa."""
        self.ego_active = False
        stop = Twist()
        self._ego_vel_pub.publish(stop)
        rospy.loginfo("[EGO] Stopped")

    def change_heading(self, angle_deg: float, yaw_rate_deg_per_sec: float = 45.0, tolerance_deg: float = 2.0, timeout_sec: float = 6.0) -> float:
        """
        Rotate the drone by a relative angle using yaw rate commands on the velocity setpoint topic.

        Args:
            angle_deg (float): Relative angle to rotate. Positive means turn right (clockwise), negative means left (counter-clockwise).
            yaw_rate_deg_per_sec (float): Magnitude of yaw rate command to send in deg/s.
            tolerance_deg (float): Stop when within this yaw error to the target heading.
            timeout_sec (float): Safety timeout to stop trying.

        Returns:
            float: The final heading (degrees) the drone attempted to reach.
        """

        def normalize_deg(deg: float) -> float:
            # Normalize to [0, 360)
            return (deg + 360.0) % 360.0

        def shortest_angle_error_deg(current: float, target: float) -> float:
            # Return signed error in [-180, 180]
            err = (target - current + 540.0) % 360.0 - 180.0
            return err

        # Determine absolute target heading (0..360)
        # Positive angle => turn right (CW), which decreases standard CCW yaw
        target_heading = normalize_deg(self.current_heading - angle_deg)

        # Prepare publisher once
        vel_pub = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10
        )

        rate = rospy.Rate(50)  # 50 Hz
        start_time = rospy.Time.now()
        commanded_yaw_rate = radians(abs(yaw_rate_deg_per_sec))

        while not rospy.is_shutdown():
            # Timeout guard
            if (rospy.Time.now() - start_time).to_sec() > timeout_sec:
                break

            # Compute error and direction
            err_deg = shortest_angle_error_deg(self.current_heading, target_heading)
            if abs(err_deg) <= tolerance_deg:
                break

            # Direction: positive err means need CCW (positive yaw), negative err means CW (negative yaw)
            # Convention here: positive angle_deg from caller means turn right (CW), so we computed target accordingly.
            yaw_rate = commanded_yaw_rate if err_deg > 0 else -commanded_yaw_rate

            # If very close, scale down rate to avoid overshoot
            if abs(err_deg) < max(5.0, yaw_rate_deg_per_sec * 0.25):
                yaw_rate = radians(err_deg)  # proportional gentle finish

            twist = Twist()
            twist.linear = Vector3(0.0, 0.0, 0.0)
            twist.angular.z = yaw_rate
            vel_pub.publish(twist)

            rate.sleep()

        # Send zero command to stop rotation
        stop_twist = Twist()
        stop_twist.linear = Vector3(0.0, 0.0, 0.0)
        stop_twist.angular.z = 0.0
        for _ in range(5):
            vel_pub.publish(stop_twist)
            rospy.sleep(0.01)

        # Update desired heading tracker and return
        self.local_desired_heading = target_heading
        return target_heading
