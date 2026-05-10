#!/usr/bin/env python3

#Dengan Menyebut Nama Allah Yang Maha Pengasih dan Yang Maha Penyayang

import sys
import os
import os
import rospkg
rospack = rospkg.RosPack()
try:
    pkg_path = rospack.get_path('krti2024_pi')
    sys.path.insert(0, os.path.join(pkg_path, 'src'))
except rospkg.common.ResourceNotFound:
    pass

from krti2023_pi.drone_api import DroneAPI
from krti2024_pi.msg import DResult
from sensor_msgs.msg import LaserScan
from krti2024_pi.srv import Activate, ActivateRequest, ActivateResponse
# from obstacle_avoidance import ObstacleAvoidanceNode
# from lidar import Lidar360

from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse # attach di sini jg cek repo

from std_msgs.msg import Float32
import rospy
from geographic_msgs.msg import GeoPoint, GeoPoseStamped
from time import sleep
from math import *
from pid import PID

# VELY POSITIF == KIRI
# VELX POSITIF == MAJU
# VELZ POSITIF == NAIK

body2local = lambda x,y,z,heading: (x*cos(heading) - y*sin(heading), x*sin(heading) + y*cos(heading), z, heading)

def getFinalLatLong(lat1, long1, distance, angle) -> GeoPoint:
    # // calculate angles
    radius = 6371000
    delta = distance / radius
    theta = radians(lat1)
    phi   = radians(long1)
    gamma = radians(angle)

    # // calculate sines and cosines
    c_theta = cos(theta)
    s_theta = sin(theta)
    c_phi   = cos(phi)  
    s_phi   = sin(phi)  
    c_delta = cos(delta)
    s_delta = sin(delta)
    c_gamma = cos(gamma)
    s_gamma = sin(gamma)

    # // calculate end vector
    x = c_delta * c_theta * c_phi - s_delta * (s_theta * c_phi * c_gamma + s_phi * s_gamma)
    y = c_delta * c_theta * s_phi - s_delta * (s_theta * s_phi * c_gamma - c_phi * s_gamma)
    z = s_delta * c_theta * c_gamma + c_delta * s_theta

    # // calculate end lat long
    theta2 = asin(z)
    phi2 = atan2(y, x)

    return GeoPoint(latitude=degrees(theta2), longitude=degrees(phi2), altitude=0)

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def height2div(input:float)->float:
    max_height = 1.2
    min_height = 0.3
    min_div = 3.5
    max_div = 1
    return clamp((input-min_height)*(max_div-min_div)/(max_height-min_height)+min_div,1,3.5)
# print(height2div(0.75))

class Game:
    def __init__(self):
        rospy.init_node("main_node", log_level=rospy.DEBUG)

        self.activate_target = rospy.ServiceProxy("/vision/activate/target", Activate)
        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.detach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")
        self.sim = rospy.get_param("/vision/use_sim", False) # awal True
        self.target_sub = rospy.Subscriber(
            "/vision/target/result", DResult, self.target_callback
        )
        
        # self.us_sub = rospy.Subscriber(
        #     "/us", Float32, self.us_callback
        # )

        self.target_data = DResult()
        self.target_data.is_found = False

        waypoints = []

        self.drone = DroneAPI(waypoints=waypoints, sim=True)
        self.x_pid = PID(0.25, 0.02, 0.2, 0, 0.1, 0.25, "x", 0.1)
        self.y_pid = PID(0.25, 0.02, 0.2, 0, 0.1, 0.25, "y", 0.1)
        self.z_pid = PID(0.20, 0.02, 0.2, 0, 0.5, 0.25, "z", 0.1)
        self.us_data = 0.0
        # Tracking attachment status
        self.payload_attached = False
        self.current_payload = None
        # self.is_safe = Lidar360()
        # self.obstacle_avoidance_node_maintain = ObstacleAvoidanceNode()

        # For Obstacle Avoidance in Simulation
        # self.avoidance_vector_x = 0
        # self.avoidance_vector_y = 0
        # self.last_avoidance_timestamp = 0
        # self.avoid = False
        # self.collision_sub = rospy.Subscriber('/sensors/lidar/sim', LaserScan, self.lidar_avoidance_cb)
        
    def us_callback(self,msg:Float32):
        self.us_data = msg.data
        rospy.loginfo_throttle(0.5,f"us jarak: {self.us_data} cm")

    def target_callback(self, msg):
        self.target_data = msg
        # Log untuk debugging deteksi payload
        if msg.is_found:
            rospy.loginfo_throttle(0.5, f"PAYLOAD TERDETEKSI! dx: {msg.dx}, dy: {msg.dy}, x_m: {msg.dx_m:.3f}, y_m: {msg.dy_m:.3f}")
        else:
            rospy.logdebug_throttle(1.0, "Payload tidak terdeteksi")
        rospy.logdebug_throttle(0.2,"target data: {}".format(self.target_data))
    
    def set_alt_vel(self,alt:float):
        self.z_pid.reset()
        cur_alt = self.drone.rangefinder

        while cur_alt < alt *0.95:
            err = alt-cur_alt
            velz = self.z_pid.update(err)
            for _ in range(10):
                self.drone.move_vel(velz=velz)
                rospy.sleep(0.01)
            cur_alt = self.drone.rangefinder
        
        for _ in range(10):
            self.drone.move_vel()
            rospy.sleep(0.01)
        
    def pickup_algorithm(self):
        # self.drone.wait4start()
        now = rospy.Time.now()
        x_done = False
        y_done = False
            # now = rospy.Time.now()
        last_dx = 0 
        last_dy = 0
        velx = 0
        vely = 0
        velz = 0
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")
            # Tambahan info untuk debugging
            # rospy.loginfo_throttle(1.0, f"Vision service aktif: {self.target}, Target type: {getattr(self, 'which_target', 'unknown')}")
            if rospy.Time.now().to_sec() - now.to_sec() > rospy.Duration(20).to_sec():
                rospy.loginfo("timeout")
                return False

        while not rospy.is_shutdown() :
            # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.rangefinder,
            }
            # move_to = {
            #     "x": 0,
            #     "y": 0,
            #     "z": 0,
            # }
            tolerance = 25

            if not (self.target_data.dx > tolerance or self.target_data.dx < - tolerance):
                y_done = True

            if not (self.target_data.dy > tolerance or self.target_data.dy < - tolerance):
                x_done = True
                
            alt = cur_pose["z"]
            target_alt = 0.38

            rospy.logdebug(f"current position : {cur_pose}")
            
            if(self.target_data.is_found):
                rospy.logdebug(f"current position : {cur_pose}")
                velx = self.x_pid.update(self.target_data.dy/180)
                vely = -self.y_pid.update(self.target_data.dx/180)

                # rospy.sleep(1) 
                last_dx = self.target_data.dx
                last_dy = self.target_data.dy
                
                rospy.loginfo_throttle(0.5, "-------- Target_alt = 0.32 FUNGSI TRUE -------")
                err = target_alt - alt
                rospy.loginfo_throttle(0.1,f"Error Value : {err}")
                rospy.loginfo_throttle(0.1,f"Altitude Value : {alt}")
                if x_done and y_done:
                    velz = self.z_pid.update(err)    
                # if x_done and y_done and cur_pose["z"] < 0.32 and self.target_data.is_found:
                if cur_pose["z"] < 0.38 and self.target_data.is_found:
                    rospy.loginfo_throttle(0.1,f"move to : {alt}")
                    rospy.loginfo("target altitude reached")
                    break
            else:
                vely = 0.2 if last_dx < 0 else -0.2 # objek  dikiri center frame , move ke kiri y +
                velx = 0.2 if last_dy > 0 else -0.2
                
            div = height2div(alt)
            print(f"alt:{alt}  div:{div}")
            vely/=div
            velx/=div
            if(alt<0.45):
                velz/=2
                rospy.loginfo_throttle(0.2, f"Keceptan dibawah 45 cm: {velx}, {vely}, {velz}")
                
            if self.drone.stable_motion():
                # pass
                rospy.loginfo("Drone sedang menyesuaikan posisi dengan payload")
                move_detect = self.drone.move_vel(velx, vely, velz)
                rospy.loginfo(f"Move to target: {move_detect}")
                
                # if cur_pose["z"] < 0.50 and self.target_data.is_found:
            
            # rospy.sleep(1)
            
            if x_done and y_done and self.drone.stable_motion() and self.target_data.is_found and alt <= target_alt :
                return True
            # if self.sim:
            #     rospy.sleep(0.1)
            #     # Link them
            #     rospy.loginfo("Attaching drone and payload")
            #     req = AttachRequest()
            #     req.model_name_1 = "iris"
            #     req.link_name_1 = "iris::drone::iris::base_link"
            #     req.model_name_2 = "object kanan" #change to object kiri, when pick up left mode
            #     req.link_name_2 = "object kanan::object::link"

                # resp = self.attach_srv.call(req)
                # rospy.loginfo(f"attach : {resp.ok}")
    
    def drop_grey(self):
        # self.drone.wait4start()
        now = rospy.Time.now()
        x_done = False
        y_done = False
            # now = rospy.Time.now()
        last_dx = 0 
        last_dy = 0
        velx = 0
        vely = 0
        velz = 0
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")
            if rospy.Time.now().to_sec() - now.to_sec() > rospy.Duration(10).to_sec():
                rospy.loginfo("timeout")
                return False

        while not rospy.is_shutdown() :
            # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.rangefinder,
            }
            # move_to = {
            #     "x": 0,
            #     "y": 0,
            #     "z": 0,
            # }
            tolerance = 25

            if not (self.target_data.dx > tolerance or self.target_data.dx < - tolerance):
                y_done = True

            if not (self.target_data.dy > tolerance or self.target_data.dy < - tolerance):
                x_done = True
                
            alt = cur_pose["z"]
            target_alt = 0.75

            rospy.logdebug(f"current position : {cur_pose}")
            
            if(self.target_data.is_found):
                rospy.logdebug(f"current position : {cur_pose}")
                velx = self.x_pid.update(self.target_data.dy/180)
                vely = -self.y_pid.update(self.target_data.dx/180)

                # rospy.sleep(1) 
                last_dx = self.target_data.dx
                last_dy = self.target_data.dy
                
                rospy.loginfo_throttle(0.5, "-------- Target_alt = 0.32 FUNGSI TRUE -------")
                err = target_alt - alt
                rospy.loginfo_throttle(0.1,f"Error Value : {err}")
                rospy.loginfo_throttle(0.1,f"Altitude Value : {alt}")
                if x_done and y_done:
                    velz = self.z_pid.update(err)    
                # if x_done and y_done and cur_pose["z"] < 0.32 and self.target_data.is_found:
                if cur_pose["z"] < 0.75 and self.target_data.is_found:
                    rospy.loginfo_throttle(0.1,f"move to : {alt}")
                    rospy.loginfo("target altitude reached")
                    break
            else:
                vely = 0.2 if last_dx < 0 else -0.2 # objek  dikiri center frame , move ke kiri y +
                velx = 0.2 if last_dy > 0 else -0.2
                
            div = height2div(alt)
            print(f"alt:{alt}  div:{div}")
            vely/=div
            velx/=div
            if(alt<0.45):
                velz/=2
                rospy.loginfo_throttle(0.2, f"Keceptan dibawah 45 cm: {velx}, {vely}, {velz}")
                
            if self.drone.stable_motion():
                # pass
                rospy.loginfo("Drone sedang menyesuaikan posisi dengan payload")
                move_detect = self.drone.move_vel(velx, vely, velz)
                rospy.loginfo(f"Move to target: {move_detect}")
                
                # if cur_pose["z"] < 0.50 and self.target_data.is_found:
            
            # rospy.sleep(1)
            
            if x_done and y_done and self.drone.stable_motion() and self.target_data.is_found and alt <= target_alt :
                return True
        
    # def lidar_avoidance_cb(self, msg):
    #     self.current_2D_scan = msg
    #     self.avoidance_vector_x = 0
    #     self.avoidance_vector_y = 0
    #     self.avoid = False
    #     temp = []
    #     ranges_temp = []
    #     index = []

    #     for i in range(1, len(self.current_2D_scan.ranges)):
    #         d0 = 0.4
    #         k = 0.5

    #         if i % 2 != 1:
    #             continue

    #         rospy.loginfo_throttle(1, f"range {i}: {self.current_2D_scan.ranges[i]}") 

    #         if self.current_2D_scan.ranges[i] < d0 and self.current_2D_scan.ranges[i] > 0.35:
    #             self.avoid = True
    #             x = math.cos(self.current_2D_scan.angle_increment * i)
    #             y = math.sin(self.current_2D_scan.angle_increment * i)
    #             U = -0.5 * k * ((1 / self.current_2D_scan.ranges[i]) - (1 / d0))**2
    #             index.append(math.degrees(i * self.current_2D_scan.angle_increment))
    #             temp.append(U)
    #             ranges_temp.append(self.current_2D_scan.ranges[i])

    #             self.avoidance_vector_x += x * -U
    #             self.avoidance_vector_y += y * -U

    #     rospy.loginfo_throttle(1, f"Angle: {index}") 
    #     rospy.loginfo_throttle(1, f"range: {ranges_temp}")        
    #     rospy.logdebug_throttle(1, f"Nilai U: {temp}")
    #     rospy.logdebug_throttle(1, f"{self.avoidance_vector_x}, {self.avoidance_vector_y}")

    #     # body2local
    #     # home_heading = self.drone.get_home_heading()
    #     # deg2rad = (math.pi/180)
    #     # avoidance_vector_x = self.avoidance_vector_x * math.cos(home_heading * deg2rad) - self.avoidance_vector_y * math.sin(home_heading * deg2rad)
    #     # avoidance_vector_y = self.avoidance_vector_x * math.cos(home_heading * deg2rad) + self.avoidance_vector_y * math.sin(home_heading * deg2rad)

    #     if self.avoid:
    #         magnitude = math.sqrt(self.avoidance_vector_x**2 + self.avoidance_vector_y**2)
    #         if magnitude > 3:
    #             self.avoidance_vector_x = 3 * (self.avoidance_vector_x / magnitude)
    #             self.avoidance_vector_y = 3 * (self.avoidance_vector_y / magnitude)

    #         if self.avoidance_vector_y > 0.15:
    #             self.avoidance_vector_y = 0
    #         #     if self.avoidance_vector_y < 0.15:
    #         #         self.avoidance_vector_y = 0 

    #         # if rospy.Time().now().secs - self.last_avoidance_timestamp > 0.1:
    #         self.lidar_avoidance()

    # def lidar_avoidance(self):
    #     # rospy.logdebug(f"avoid {self.avoid}")
    #     if self.avoid:
    #         home_heading = self.drone.get_home_heading()
    #         head = radians(home_heading)
    #         x,y = body2local(-self.avoidance_vector_y, -self.avoidance_vector_x, head)
    #         if x > 0.15:
    #             x = 0
    #         dist = {"x":  x, "y":  y, "z": 0.6, "heading": home_heading}
    #         self.drone.move(dist)
    #         rospy.logdebug(f"Efek Lidar: {dist}")

    def pickup_algorithm2(self):
        now = rospy.Time.now()
        x_done = False
        y_done = False
        now = rospy.Time.now()
        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")

        while self.target_data.is_found :
        
            # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.current_pose.pose.pose.position.z,
            }
            move_to = {
                "x": 0,
                "y": 0,
                "z": 0,
            }
            if self.target_data.dx > 80:
                rospy.logdebug("move right")
                move_to["y"] += self.target_data.dx_m
            elif self.target_data.dx < -80:
                rospy.logdebug("move left")
                move_to["y"] += self.target_data.dx_m
            if self.target_data.dx > 40:
                rospy.logdebug("move right")
                move_to["y"] += 0.1
            elif self.target_data.dx < -40:
                rospy.logdebug("move left")
                move_to["y"] -= 0.1
            else:
                x_done = True

            if self.target_data.dy > 80:
                rospy.logdebug("move backward")
                move_to["x"] += self.target_data.dy_m

            elif self.target_data.dy < -80:
                rospy.logdebug("move forward")
                move_to["x"] += self.target_data.dy_m
            
            elif self.target_data.dy > 40:
                rospy.logdebug("move backward")
                move_to["x"] += 0.1

            elif self.target_data.dy < -40:
                rospy.logdebug("move forward")
                move_to["x"] -= 0.1
            else:
                y_done = True

            if x_done and y_done and cur_pose["z"] > 1.2:
                move_to["z"] -= 0.2
                self.drone.move(move_to)
                rospy.sleep(0.5)
            
            elif x_done and y_done and cur_pose["z"] < 1.2 and cur_pose["z"] > 0.4:
                move_to["z"] -= 0.1
                rospy.loginfo("move down")
                self.drone.move(move_to)
                rospy.sleep(0.5)

            elif x_done and y_done and cur_pose["z"] < 0.2 and cur_pose["z"] > 0.2:
                
                rospy.loginfo("target altitude reached")
                rospy.sleep(2)
                break

            rospy.logdebug(f"current position : {cur_pose}")
            rospy.logdebug(f"move to : {move_to}")
            if self.drone.stable_motion():
                self.drone.move_vel(move_to["x"], move_to['y'],0)
                # self.drone.move(move_to)

        # cur_pose = {
        #         "x": self.drone.current_pose.pose.pose.position.x,
        #         "y": self.drone.current_pose.pose.pose.position.y,
        #         "z": self.drone.current_pose.pose.pose.position.z,
        # }
        # if self.drone.stable_motion():
        #     self.drone.move(cur_pose)
        return True
    
    def test_color_following(self):
        # self.drone.wait4start()
        now = rospy.Time.now()
        x_done = False
        y_done = False
            # now = rospy.Time.now()
        last_dx = 0 
        last_dy = 0
        velx = 0
        vely = 0
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)
        

        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")
            if rospy.Time.now().to_sec() - now.to_sec() > rospy.Duration(10).to_sec():
                rospy.loginfo("timeout")
                return False

        while not rospy.is_shutdown() :
        
            # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.rangefinder,
            }
            # move_to = {
            #     "x": 0,
            #     "y": 0,
            #     "z": 0,
            # }
            
            tolerance = 30

            if not (self.target_data.dx > tolerance or self.target_data.dx < - tolerance):
                y_done = True

            if not (self.target_data.dy > tolerance or self.target_data.dy < - tolerance):
                x_done = True

            rospy.logdebug(f"current position : {cur_pose}")
            
            if(self.target_data.is_found):
                rospy.logdebug(f"current position : {cur_pose}")
                velx = self.x_pid.update(self.target_data.dy/180)
                vely = -self.y_pid.update(self.target_data.dx/180)

                # rospy.sleep(1)
                last_dx = self.target_data.dx
                last_dy = self.target_data.dy
                   
                # if x_done and y_done and cur_pose["z"] < 0.13 and self.target_data.is_found:
                    # rospy.loginfo_throttle(0.1,f"move to : {alt}")
                    # rospy.loginfo("target altitude reached")
                    # break
            else:
                vely = 0.2 if last_dx < 0 else -0.2 # objek  dikiri center frame , move ke kiri y +
                velx = 0.2 if last_dy > 0 else -0.2
                
            
            if self.drone.stable_motion():
                rospy.loginfo(f"moving with velx : {velx}, vely : {vely} ")
                # pass
                rospy.loginfo("Drone sedang menyesuaikan posisi dengan payload")
                move_detect = self.drone.move_vel(
                    velx, 
                    vely, 
                    0
                    )
                rospy.loginfo(f"Move to target: {move_detect}")
            # rospy.sleep(1)
            
            if x_done and y_done and self.drone.stable_motion() and self.target_data.is_found:
                return True 

    def main(self):
        while not rospy.is_shutdown():
            self.drone.wait4start()
            self.drone.takeoff(1.2)
            try:
                start = rospy.Time.now().to_sec()
                for _ in range(30):
                    self.drone.move({'x': -2, 'y': -0.5, 'z': 1.5})

                rospy.sleep(3)
                rospy.loginfo("searching for target")
                self.activate_target(ActivateRequest(True))
                result = self.pickup_algorithm()
                while not result:
                    result = self.pickup_algorithm()
                self.x_pid.reset()
                rospy.sleep(3)
                for _ in range(30):
                    self.drone.move({'x': -5.5, 'y': -2, 'z': 1.5})
                    
                rospy.sleep(rospy.Duration(3))
                for _ in range(30):
                    self.drone.move({'x': -5.5, 'y': -5, 'z': 1.5})
                
                rospy.sleep(3)
                for _ in range(30):
                    self.drone.move({'x': -3.5, 'y': -5, 'z': 1.5})

                self.x_pid.reset()
                rospy.sleep(3)
                rospy.loginfo("LAND")
                rospy.signal_shutdown("done")

            except KeyboardInterrupt:
                self.drone.set_mode("LAND")
                exit()
            except IndexError:
                pass
        self.drone.set_mode("LAND")
        rospy.signal_shutdown("Mission Finished")

    def test_kiri(self):
        # Drone init
        self.drone.set_ekf_source(2)
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)
        # Destination
        home_heading = self.drone.compass
        print("home heading : ",home_heading)
        # if home_heading > 180:
        #     home_heading -= 180
        # else:
        #     home_heading += 180
        print("home heading : ",home_heading)
        head = radians(float(home_heading))
        rospy.loginfo(f"Changing source to optical flow")
        takeoff_gps = self.drone.gps
        lat = takeoff_gps.latitude
        lon = takeoff_gps.longitude

        # Takeoff
        check = self.drone.takeoff(0.5)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(1)

        ## NWU
        x,y = body2local(2, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 1))
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()
        self.x_pid.reset()
        self.y_pid.reset()
        rospy.sleep(10)

        ## NWU
        x,y = body2local(5, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)

        while self.drone.lidar_data[0] > 1.7:
            # rospy.loginfo(f"Distance forward: {self.drone.lidar_data[0]}")
            rospy.sleep(0.1)
        
        rospy.loginfo("STOPPING")
        x,y = body2local(0, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }

        # aligning to desired rangefinder range

        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")

        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(3)
        rospy.logwarn(rospy.Time.now().to_sec)

        x,y = body2local(3, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to : {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 2))
        result = self.test_color_following()
        start = rospy.Time.now().to_sec() #.secs
        while not result:
            result = self.test_color_following()
            if rospy.Time.now().to_sec() - start > 5:
                break
        self.x_pid.reset()
        self.y_pid.reset()
        self.drone.set_ekf_source(1)
        rospy.sleep(10)
        self.drone.set_mode("AUTO")

    def test_kanan(self):
        # Drone init
        self.drone.set_ekf_source(2)
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)
        # Destination
        home_heading = self.drone.get_home_heading()
        print("home heading : ",home_heading)
        # if home_heading > 180:
        #     home_heading -= 180
        # else:
        #     home_heading += 180
        print("home heading : ",home_heading)
        head = radians(home_heading)
        rospy.loginfo(f"Changing source to optical flow")
        takeoff_gps = self.drone.gps
        lat = takeoff_gps.latitude
        lon = takeoff_gps.longitude

        # Takeoff
        check = self.drone.takeoff(0.75)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(1)

        ## NWU
        x,y = body2local(2, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 1))
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()

        ## NWU
        x,y = body2local(-5, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)

        while self.drone.lidar_data[0] > 750:
            # rospy.loginfo(f"Distance forward: {self.drone.lidar_data[0]}")
            rospy.sleep(0.1)
        
        rospy.loginfo("STOPPING")
        x,y = body2local(0, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }

        # aligning to desired rangefinder range

        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")

        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(3)
        rospy.logwarn(rospy.Time.now().to_sec)

        x,y = body2local(-3, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to : {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 2))
        result = self.test_color_following()
        start = rospy.Time.now().to_sec() #.secs
        while not result:
            result = self.test_color_following()
            if rospy.Time.now().to_sec() - start > 5:
                break
        self.x_pid.reset()
        self.y_pid.reset()
        self.drone.set_ekf_source(1)
        rospy.sleep(10)
        self.drone.set_mode("AUTO")

    def test(self):
        # Drone init
        rospy.sleep(3)
        self.drone.set_ekf_source(2)
        rospy.sleep(3)
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)

        # Destination
        home_heading = self.drone.get_home_heading()
        print("home heading : ",home_heading)
        # if home_heading > 180:
        #     home_heading -= 180
        # else:
        #     home_heading += 180
        print("home heading : ",home_heading)
        head = radians(home_heading)

        rospy.loginfo(f"Changing source to optical flow")

        takeoff_gps = self.drone.gps
        lat = takeoff_gps.latitude
        lon = takeoff_gps.longitude

        # Takeoff
        check = self.drone.takeoff(0.5)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(1)

        ## NWU
        x,y = body2local(2, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 1))
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()


        ## NWU
        x,y = body2local(5, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)
        

        while self.drone.lidar_data[0] > 1.7:
            # rospy.loginfo(f"Distance forward: {self.drone.lidar_data[0]}")
            rospy.sleep(0.1)
        
        rospy.loginfo("STOPPING")
        x,y = body2local(0, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }

        # aligning to desired rangefinder range

        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")

        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(3)
        rospy.logwarn(rospy.Time.now().to_sec)

        x,y = body2local(0, 3, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to : {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 2))
        result = self.test_color_following()
        start = rospy.Time.now().to_sec() #.secs
        while not result:
            result = self.test_color_following()
            if rospy.Time.now().to_sec() - start > 5:
                break
        rospy.sleep(5)
        self.drone.set_ekf_source(1)
        rospy.sleep(10)
        self.drone.set_mode("AUTO")

    def test_color_following(self):
        # self.drone.wait4start()
        now = rospy.Time.now()
        x_done = False
        y_done = False
            # now = rospy.Time.now()
        last_dx = 0 
        last_dy = 0
        velx = 0
        vely = 0
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)
        

        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")
            if rospy.Time.now().to_sec() - now.to_sec() > rospy.Duration(10).to_sec():
                rospy.loginfo("timeout")
                return False

        while not rospy.is_shutdown() :
        
            # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.rangefinder,
            }
            # move_to = {
            #     "x": 0,
            #     "y": 0,
            #     "z": 0,
            # }
            
            tolerance = 30

            if not (self.target_data.dx > tolerance or self.target_data.dx < - tolerance):
                y_done = True

            if not (self.target_data.dy > tolerance or self.target_data.dy < - tolerance):
                x_done = True

            rospy.logdebug(f"current position : {cur_pose}")
            
            if(self.target_data.is_found):
                rospy.logdebug(f"current position : {cur_pose}")
                velx = self.x_pid.update(self.target_data.dy/180)
                vely = -self.y_pid.update(self.target_data.dx/180)

                # rospy.sleep(1)
                last_dx = self.target_data.dx
                last_dy = self.target_data.dy
                   
                # if x_done and y_done and cur_pose["z"] < 0.13 and self.target_data.is_found:
                    # rospy.loginfo_throttle(0.1,f"move to : {alt}")
                    # rospy.loginfo("target altitude reached")
                    # break
            else:
                vely = 0.2 if last_dx < 0 else -0.2 # objek  dikiri center frame , move ke kiri y +
                velx = 0.2 if last_dy > 0 else -0.2
                
            
            if self.drone.stable_motion():
                rospy.loginfo(f"moving with velx : {velx}, vely : {vely} ")
                # pass
                rospy.loginfo("Drone sedang menyesuaikan posisi dengan payload")
                move_detect = self.drone.move_vel(
                    velx, 
                    vely, 
                    0
                    )
                rospy.loginfo(f"Move to target: {move_detect}")
            # rospy.sleep(1)
            
            if x_done and y_done and self.drone.stable_motion() and self.target_data.is_found:
                return True
            
    def main(self):
        while not rospy.is_shutdown():
            self.drone.wait4start()
            self.drone.takeoff(1.2)
            try:
                start = rospy.Time.now().to_sec()
                for _ in range(30):
                    self.drone.move({'x': -2, 'y': -0.5, 'z': 1.5})

                rospy.sleep(3)
                rospy.loginfo("searching for target")
                self.activate_target(ActivateRequest(True))
                result = self.pickup_algorithm()
                while not result:
                    result = self.pickup_algorithm()
                self.x_pid.reset()
                rospy.sleep(3)
                for _ in range(30):
                    self.drone.move({'x': -5.5, 'y': -2, 'z': 1.5})
                    
                rospy.sleep(rospy.Duration(3))
                for _ in range(30):
                    self.drone.move({'x': -5.5, 'y': -5, 'z': 1.5})
                
                rospy.sleep(3)
                for _ in range(30):
                    self.drone.move({'x': -3.5, 'y': -5, 'z': 1.5})

                self.x_pid.reset()
                rospy.sleep(3)
                rospy.loginfo("LAND")
                rospy.signal_shutdown("done")

            except KeyboardInterrupt:
                self.drone.set_mode("LAND")
                exit()
            except IndexError:
                pass
        self.drone.set_mode("LAND")
        rospy.signal_shutdown("Mission Finished")

    def test_kiri(self):
        # Drone init
        self.drone.set_ekf_source(2)
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)
        # Destination
        home_heading = self.drone.compass
        print("home heading : ",home_heading)
        # if home_heading > 180:
        #     home_heading -= 180
        # else:
        #     home_heading += 180
        print("home heading : ",home_heading)
        head = radians(float(home_heading))
        rospy.loginfo(f"Changing source to optical flow")
        takeoff_gps = self.drone.gps
        lat = takeoff_gps.latitude
        lon = takeoff_gps.longitude

        # Takeoff
        check = self.drone.takeoff(0.5)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(1)

        ## NWU
        x,y = body2local(2, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 1))
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()
        self.x_pid.reset()
        self.y_pid.reset()
        rospy.sleep(10)

        ## NWU
        x,y = body2local(5, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)
        

        while self.drone.lidar_data[0] > 1.7:
            # rospy.loginfo(f"Distance forward: {self.drone.lidar_data[0]}")
            rospy.sleep(0.1)
        
        rospy.loginfo("STOPPING")
        x,y = body2local(0, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }

        # aligning to desired rangefinder range

        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")

        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(3)
        rospy.logwarn(rospy.Time.now().to_sec)

        x,y = body2local(3, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to : {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 2))
        result = self.test_color_following()
        start = rospy.Time.now().to_sec() #.secs
        while not result:
            result = self.test_color_following()
            if rospy.Time.now().to_sec() - start > 5:
                break
        self.x_pid.reset()
        self.y_pid.reset()
        self.drone.set_ekf_source(1)
        rospy.sleep(10)
        self.drone.set_mode("AUTO")

    def test_color_following(self):
        # self.drone.wait4start()
        now = rospy.Time.now()
        x_done = False
        y_done = False
            # now = rospy.Time.now()
        last_dx = 0 
        last_dy = 0
        velx = 0
        vely = 0
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)
        

        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")
            if rospy.Time.now().to_sec() - now.to_sec() > rospy.Duration(10).to_sec():
                rospy.loginfo("timeout")
                return False

        while not rospy.is_shutdown() :
        
            # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.rangefinder,
            }
            # move_to = {
            #     "x": 0,
            #     "y": 0,
            #     "z": 0,
            # }
            
            tolerance = 30

            if not (self.target_data.dx > tolerance or self.target_data.dx < - tolerance):
                y_done = True

            if not (self.target_data.dy > tolerance or self.target_data.dy < - tolerance):
                x_done = True

            rospy.logdebug(f"current position : {cur_pose}")
            
            if(self.target_data.is_found):
                rospy.logdebug(f"current position : {cur_pose}")
                velx = self.x_pid.update(self.target_data.dy/180)
                vely = -self.y_pid.update(self.target_data.dx/180)

                # rospy.sleep(1)
                last_dx = self.target_data.dx
                last_dy = self.target_data.dy
                   
                # if x_done and y_done and cur_pose["z"] < 0.13 and self.target_data.is_found:
                    # rospy.loginfo_throttle(0.1,f"move to : {alt}")
                    # rospy.loginfo("target altitude reached")
                    # break
            else:
                vely = 0.2 if last_dx < 0 else -0.2 # objek  dikiri center frame , move ke kiri y +
                velx = 0.2 if last_dy > 0 else -0.2
                
            
            if self.drone.stable_motion():
                rospy.loginfo(f"moving with velx : {velx}, vely : {vely} ")
                # pass
                rospy.loginfo("Drone sedang menyesuaikan posisi dengan payload")
                move_detect = self.drone.move_vel(
                    velx, 
                    vely, 
                    0
                    )
                rospy.loginfo(f"Move to target: {move_detect}")
            # rospy.sleep(1)
            
            if x_done and y_done and self.drone.stable_motion() and self.target_data.is_found:
                return True
            
    def main(self):
        while not rospy.is_shutdown():
            self.drone.wait4start()
            self.drone.takeoff(1.2)
            try:
                start = rospy.Time.now().to_sec()
                for _ in range(30):
                    self.drone.move({'x': -2, 'y': -0.5, 'z': 1.5})

                rospy.sleep(3)
                rospy.loginfo("searching for target")
                self.activate_target(ActivateRequest(True))
                result = self.pickup_algorithm()
                while not result:
                    result = self.pickup_algorithm()
                self.x_pid.reset()
                rospy.sleep(3)
                for _ in range(30):
                    self.drone.move({'x': -5.5, 'y': -2, 'z': 1.5})
                    
                rospy.sleep(rospy.Duration(3))
                for _ in range(30):
                    self.drone.move({'x': -5.5, 'y': -5, 'z': 1.5})
                
                rospy.sleep(3)
                for _ in range(30):
                    self.drone.move({'x': -3.5, 'y': -5, 'z': 1.5})

                self.x_pid.reset()
                rospy.sleep(3)
                rospy.loginfo("LAND")
                rospy.signal_shutdown("done")

            except KeyboardInterrupt:
                self.drone.set_mode("LAND")
                exit()
            except IndexError:
                pass
        self.drone.set_mode("LAND")
        rospy.signal_shutdown("Mission Finished")

    def test_kiri(self):
        # Drone init
        self.drone.set_ekf_source(2)
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)
        # Destination
        home_heading = self.drone.compass
        print("home heading : ",home_heading)
        # if home_heading > 180:
        #     home_heading -= 180
        # else:
        #     home_heading += 180
        print("home heading : ",home_heading)
        head = radians(float(home_heading))
        rospy.loginfo(f"Changing source to optical flow")
        takeoff_gps = self.drone.gps
        lat = takeoff_gps.latitude
        lon = takeoff_gps.longitude

        # Takeoff
        check = self.drone.takeoff(0.5)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(1)

        ## NWU
        x,y = body2local(2, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 1))
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()

        ## NWU
        x,y = body2local(5, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)
        

        while self.drone.lidar_data[0] > 1.7:
            # rospy.loginfo(f"Distance forward: {self.drone.lidar_data[0]}")
            rospy.sleep(0.1)
        
        rospy.loginfo("STOPPING")
        x,y = body2local(0, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }

        # aligning to desired rangefinder range

        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")

        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(3)
        rospy.logwarn(rospy.Time.now().to_sec)

        x,y = body2local(3, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to : {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 2))
        result = self.test_color_following()
        start = rospy.Time.now().to_sec() #.secs
        while not result:
            result = self.test_color_following()
            if rospy.Time.now().to_sec() - start > 5:
                break
        self.x_pid.reset()
        self.y_pid.reset()
        self.drone.set_ekf_source(1)
        rospy.sleep(10)
        self.drone.set_mode("AUTO")

    def test(self):
        # Drone init
        rospy.sleep(3)
        self.drone.set_ekf_source(2)
        rospy.sleep(3)
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)
        # Destination
        home_heading = self.drone.get_home_heading()
        print("home heading : ",home_heading)
        # if home_heading > 180:
        #     home_heading -= 180
        # else:
        #     home_heading += 180
        print("home heading : ",home_heading)
        head = radians(home_heading)

        rospy.loginfo(f"Changing source to optical flow")

        takeoff_gps = self.drone.gps
        lat = takeoff_gps.latitude
        lon = takeoff_gps.longitude

        # Takeoff
        check = self.drone.takeoff(0.5)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(1)

        ## NWU
        x,y = body2local(2, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 1))
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()

        ## NWU
        x,y = body2local(5, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)
        

        while self.drone.lidar_data[0] > 1.7:
            # rospy.loginfo(f"Distance forward: {self.drone.lidar_data[0]}")
            rospy.sleep(0.1)
        
        rospy.loginfo("STOPPING")
        x,y = body2local(0, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }

        # aligning to desired rangefinder range

        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")

        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(3)
        rospy.logwarn(rospy.Time.now().to_sec)

        x,y = body2local(0, 3, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to : {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 2))
        result = self.test_color_following()
        start = rospy.Time.now().to_sec() #.secs
        while not result:
            result = self.test_color_following()
            if rospy.Time.now().to_sec() - start > 5:
                break
        rospy.sleep(5)
        self.drone.set_ekf_source(1)
        rospy.sleep(10)
        self.drone.set_mode("AUTO")

    def test_color_following(self):
        # self.drone.wait4start()
        now = rospy.Time.now()
        x_done = False
        y_done = False
            # now = rospy.Time.now()
        last_dx = 0 
        last_dy = 0
        velx = 0
        vely = 0
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)
        

        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")
            if rospy.Time.now().to_sec() - now.to_sec() > rospy.Duration(10).to_sec():
                rospy.loginfo("timeout")
                return False

        while not rospy.is_shutdown() :
        
            # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.rangefinder,
            }
            # move_to = {
            #     "x": 0,
            #     "y": 0,
            #     "z": 0,
            # }
            
            tolerance = 30

            if not (self.target_data.dx > tolerance or self.target_data.dx < - tolerance):
                y_done = True

            if not (self.target_data.dy > tolerance or self.target_data.dy < - tolerance):
                x_done = True

            rospy.logdebug(f"current position : {cur_pose}")
            
            if(self.target_data.is_found):
                rospy.logdebug(f"current position : {cur_pose}")
                velx = self.x_pid.update(self.target_data.dy/180)
                vely = -self.y_pid.update(self.target_data.dx/180)

                # rospy.sleep(1)
                last_dx = self.target_data.dx
                last_dy = self.target_data.dy
                   
                # if x_done and y_done and cur_pose["z"] < 0.13 and self.target_data.is_found:
                    # rospy.loginfo_throttle(0.1,f"move to : {alt}")
                    # rospy.loginfo("target altitude reached")
                    # break
            else:
                vely = 0.2 if last_dx < 0 else -0.2 # objek  dikiri center frame , move ke kiri y +
                velx = 0.2 if last_dy > 0 else -0.2
                
            
            if self.drone.stable_motion():
                rospy.loginfo(f"moving with velx : {velx}, vely : {vely} ")
                # pass
                rospy.loginfo("Drone sedang menyesuaikan posisi dengan payload")
                move_detect = self.drone.move_vel(
                    velx, 
                    vely, 
                    0
                    )
                rospy.loginfo(f"Move to target: {move_detect}")
            # rospy.sleep(1)
            
            if x_done and y_done and self.drone.stable_motion() and self.target_data.is_found:
                return True
            
    def main(self):
        while not rospy.is_shutdown():
            self.drone.wait4start()
            self.drone.takeoff(1.2)
            try:
                start = rospy.Time.now().to_sec()
                for _ in range(30):
                    self.drone.move({'x': -2, 'y': -0.5, 'z': 1.5})

                rospy.sleep(3)
                rospy.loginfo("searching for target")
                self.activate_target(ActivateRequest(True))
                result = self.pickup_algorithm()
                while not result:
                    result = self.pickup_algorithm()
                self.x_pid.reset()
                rospy.sleep(3)
                for _ in range(30):
                    self.drone.move({'x': -5.5, 'y': -2, 'z': 1.5})
                    
                rospy.sleep(rospy.Duration(3))
                for _ in range(30):
                    self.drone.move({'x': -5.5, 'y': -5, 'z': 1.5})
                
                rospy.sleep(3)
                for _ in range(30):
                    self.drone.move({'x': -3.5, 'y': -5, 'z': 1.5})

                self.x_pid.reset()
                rospy.sleep(3)
                rospy.loginfo("LAND")
                rospy.signal_shutdown("done")

            except KeyboardInterrupt:
                self.drone.set_mode("LAND")
                exit()
            except IndexError:
                pass
        self.drone.set_mode("LAND")
        rospy.signal_shutdown("Mission Finished")

    def test_kiri(self):
        # Drone init
        self.drone.set_ekf_source(2)
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)
        # Destination
        home_heading = self.drone.compass
        print("home heading : ",home_heading)
        # if home_heading > 180:
        #     home_heading -= 180
        # else:
        #     home_heading += 180
        print("home heading : ",home_heading)
        head = radians(float(home_heading))
        rospy.loginfo(f"Changing source to optical flow")
        takeoff_gps = self.drone.gps
        lat = takeoff_gps.latitude
        lon = takeoff_gps.longitude

        # Takeoff
        check = self.drone.takeoff(0.5)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(1)

        ## NWU
        x,y = body2local(2, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 1))
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()

        ## NWU
        x,y = body2local(5, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)
        

        while self.drone.lidar_data[0] > 750:
            # rospy.loginfo(f"Distance forward: {self.drone.lidar_data[0]}")
            rospy.sleep(0.1)
        
        rospy.loginfo("STOPPING")
        x,y = body2local(0, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }

        # aligning to desired rangefinder range

        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")

        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(3)
        rospy.logwarn(rospy.Time.now().to_sec)

        x,y = body2local(0, 3, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to : {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 2))
        result = self.test_color_following()
        start = rospy.Time.now().to_sec() #.secs
        while not result:
            result = self.test_color_following()
            if rospy.Time.now().to_sec() - start > 5:
                break
        rospy.sleep(5)
        self.drone.set_ekf_source(1)
        rospy.sleep(10)
        self.drone.set_mode("AUTO")

    def test_color_following(self):
        # self.drone.wait4start()
        now = rospy.Time.now()
        x_done = False
        y_done = False
            # now = rospy.Time.now()
        last_dx = 0 
        last_dy = 0
        velx = 0
        vely = 0
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)
        

        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")
            if rospy.Time.now().to_sec() - now.to_sec() > rospy.Duration(10).to_sec():
                rospy.loginfo("timeout")
                return False

        while not rospy.is_shutdown() :
        
            # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.rangefinder,
            }
            # move_to = {
            #     "x": 0,
            #     "y": 0,
            #     "z": 0,
            # }
            
            tolerance = 30

            if not (self.target_data.dx > tolerance or self.target_data.dx < - tolerance):
                y_done = True

            if not (self.target_data.dy > tolerance or self.target_data.dy < - tolerance):
                x_done = True

            rospy.logdebug(f"current position : {cur_pose}")
            
            if(self.target_data.is_found):
                rospy.logdebug(f"current position : {cur_pose}")
                velx = self.x_pid.update(self.target_data.dy/180)
                vely = -self.y_pid.update(self.target_data.dx/180)

                # rospy.sleep(1)
                last_dx = self.target_data.dx
                last_dy = self.target_data.dy
                   
                # if x_done and y_done and cur_pose["z"] < 0.13 and self.target_data.is_found:
                    # rospy.loginfo_throttle(0.1,f"move to : {alt}")
                    # rospy.loginfo("target altitude reached")
                    # break
            else:
                vely = 0.2 if last_dx < 0 else -0.2 # objek  dikiri center frame , move ke kiri y +
                velx = 0.2 if last_dy > 0 else -0.2
                
            
            if self.drone.stable_motion():
                rospy.loginfo(f"moving with velx : {velx}, vely : {vely} ")
                # pass
                rospy.loginfo("Drone sedang menyesuaikan posisi dengan payload")
                move_detect = self.drone.move_vel(
                    velx, 
                    vely, 
                    0
                    )
                rospy.loginfo(f"Move to target: {move_detect}")
            # rospy.sleep(1)
            
            if x_done and y_done and self.drone.stable_motion() and self.target_data.is_found:
                return True
            
    def main(self):
        while not rospy.is_shutdown():
            self.drone.wait4start()
            self.drone.takeoff(1.2)
            try:
                start = rospy.Time.now().to_sec()
                for _ in range(30):
                    self.drone.move({'x': -2, 'y': -0.5, 'z': 1.5})

                rospy.sleep(3)
                rospy.loginfo("searching for target")
                self.activate_target(ActivateRequest(True))
                result = self.pickup_algorithm()
                while not result:
                    result = self.pickup_algorithm()
                self.x_pid.reset()
                rospy.sleep(3)
                for _ in range(30):
                    self.drone.move({'x': -5.5, 'y': -2, 'z': 1.5})
                    
                rospy.sleep(rospy.Duration(3))
                for _ in range(30):
                    self.drone.move({'x': -5.5, 'y': -5, 'z': 1.5})
                
                rospy.sleep(3)
                for _ in range(30):
                    self.drone.move({'x': -3.5, 'y': -5, 'z': 1.5})

                self.x_pid.reset()
                rospy.sleep(3)
                rospy.loginfo("LAND")
                rospy.signal_shutdown("done")

            except KeyboardInterrupt:
                self.drone.set_mode("LAND")
                exit()
            except IndexError:
                pass
        self.drone.set_mode("LAND")
        rospy.signal_shutdown("Mission Finished")

    def test_kiri(self):
        # Drone init
        self.drone.set_ekf_source(2)
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)
        # Destination
        home_heading = self.drone.compass
        print("home heading : ",home_heading)
        # if home_heading > 180:
        #     home_heading -= 180
        # else:
        #     home_heading += 180
        print("home heading : ",home_heading)
        head = radians(float(home_heading))
        rospy.loginfo(f"Changing source to optical flow")
        takeoff_gps = self.drone.gps
        lat = takeoff_gps.latitude
        lon = takeoff_gps.longitude

        # Takeoff
        check = self.drone.takeoff(0.5)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(1)

        ## NWU
        x,y = body2local(2, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 1))
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()

        ## NWU
        x,y = body2local(5, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.01)
        

        while self.drone.lidar_data[0] > 750:
            # rospy.loginfo(f"Distance forward: {self.drone.lidar_data[0]}")
            rospy.sleep(0.1)
        
        rospy.loginfo("STOPPING")
        x,y = body2local(0, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }

        # aligning to desired rangefinder range

        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")

        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(3)
        rospy.logwarn(rospy.Time.now().to_sec)

        x,y = body2local(0, 3, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }
        rospy.loginfo(f"We're going to move to : {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 4] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")

        rospy.loginfo("searching for target")
        self.activate_target(ActivateRequest(True, 2))
        result = self.test_color_following()
        start = rospy.Time.now().to_sec() #.secs
        while not result:
            result = self.test_color_following()
            if rospy.Time.now().to_sec() - start > 5:
                break
        rospy.sleep(5)
        self.drone.set_ekf_source(1)
        rospy.sleep(10)
        self.drone.set_mode("AUTO")

    def full_left(self):
        """
            FUNCTION FOR FULL AUTO MISSION, INDOOR + OUTDOOR WITH COMPLETELY COMPUTER VISION
            MAY ALLAH BLESS US
            JALUR KIRI

            Full mission, bismillahirrahmannirrahim:
            --- indoor ---
            0. MAKE SURE THIS FIRST PREP:
                > Activate message for ultrasonic
                > Set GPS as OFF
                > Set home destination
            1. Change source to optical flow
            2. Takeoff to 1m
            3. Call service cam 1
            4. Move forward
            5. Move forward until detect the payload
            6. Test color following activate
            7. Do the pick up algorithm
            8. Drone stop a while
            9. Drone up 1.3 meter after pick up
            10. Turn off cam 1, turn on cam 2
            11. Move forward for move left or right
            12. Use ultrasonnnic for detect wall and move
            13. Drone move to left after detect wall
            14. Drone do scan ember
            15. Drop payload
            16. Move left after drop payload
            17. Increase  altitude 
            18. Move left to go outside

            --- outdoor ---
            19. Change soure to RTK
            20. Set mission for first point
            21. Set mission for first point
            22. Set mission for last point
            23. FINISH
        """
        
        # 0. FIRST PREP 
        rospy.wait_for_message("/us", Float32)
        self.drone.use_gps(False)
        self.drone.wait4start()
        self.drone.move_vel(0,0,0)
        
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to OPTICAL FLOW")
        self.drone.set_ekf_source(1)

        # 2. Takeoff
        check = self.drone.takeoff(1)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        # 3. Call service
        rospy.loginfo("-- CALL SERVICE --")
        self.activate_target(ActivateRequest(True, 1))
        
        # 4. Move forward
        velz = 0
        vely = 0
        velx = 0.3
        
        rospy.loginfo("-- DRONE VELX POSITIF MOVE FORWARD --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Forward] current pose : {self.drone.current_pose.pose.pose.position} ")
        
        # 5. Move until detect the payload
        while not self.target_data.is_found:
            for _ in range(30):
                self.drone.move_vel(0.17, 0, 0)
                rospy.sleep(0.1)
        
        # 6. TEST COLOR FOLLOWING ACTIVATE
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(8).to_sec():
            self.test_color_following()
            
        # 7. DO THE PICKUP ALGORITHM
        rospy.loginfo("-- PICKUP ALGORITHM --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time2 < rospy.Duration(6).to_sec():
            rospy.loginfo("Masuk ke fungsi PICKUP ALGORITHM")
            res = self.pickup_algorithm()
            rospy.sleep(0.01)
            if res:
                break
            
        rospy.sleep(1)
        
        # 8. DRONE STOP A WHILE
        rospy.loginfo("DRONE DO STOPS")
        for _ in range (10):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.01)
        rospy.sleep(2)
        
        # 9. DRONE UP 1.3 METER AFTER PICK UP
        rospy.loginfo("DRONE DO UP 1.30 M")
        self.set_alt_vel(1.3)

        # 10. TURN ON CAM 2
        rospy.loginfo("DRONE TURN ON CAM 2") #Object 2        
        self.activate_target(ActivateRequest(False, 1))
        self.activate_target(ActivateRequest(True, 2))
        
        # 11. MOVE FORWARD FOR MOVE LEFT OR RIGHT
        velz = 0
        vely = 0
        velx = 0.35
        
        rospy.loginfo("-- DRONE VELX POSITIF MOVE FORWARD --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Forward] current pose : {self.drone.current_pose.pose.pose.position} ")
        
        # 12. USE ULTRASONIC FOR DETECT WALL AND MOVE LEFT OR RIGHT
        rospy.loginfo("DRONE USE ULTRASONIC TO DETECT THE WALL AND MOVE RIGHT OR LEFT")
        while self.us_data > 120 and self.us_data < 50:
            for _ in range (30):
                self.drone.move_vel(0.19, 0, 0)
                rospy.sleep(0.1)
        
        rospy.loginfo("DRONE DO STOPS BEFORE MOVE RIGHT OR LEFT")
        for _ in range (20):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.01)
        rospy.sleep(2)
        
        # 13. DRONE MOVE TO LEFT AFTER DETECT WALL
        velz = 0
        vely = 0.35 # LEFT
        velx = 0

        rospy.loginfo("-- DRONE VELY POSITIF MOVE LEFT --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Left] current pose : {self.drone.current_pose.pose.pose.position} ")
            
        while not self.target_data.is_found:
            for _ in range(30):
                self.drone.move_vel(0, 0.25, 0)
                rospy.sleep(0.1)
                
        self.x_pid.reset()
        self.y_pid.reset()
        
        # 14. DRONE DO SCAN EMBER
        rospy.loginfo("-- SEARCHING FOR EMBER --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(10).to_sec():
            self.test_color_following()
        
        # 15. DROP PAYLOAD    
        rospy.loginfo("-- DRONE DO DROP PAYLOAD --")
        self.drone.set_servo(5, 800)
        
        rospy.loginfo("-- DRONE MOVE LEFT --")
        self.activate_target(ActivateRequest(False, 2))
        
        # 16. MOVE LEFT AFTER DROP PAYLOAD
        velz = 0 
        vely = 0.20
        velx = 0
        
        rospy.loginfo("-- DRONE VELY POSITIF MOVE LEFT FTER DROP PAYLOAD --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Left] current pose : {self.drone.current_pose.pose.pose.position} ")
        
        # 17. INCREASE ALTITUDE FOR PASS THE WINDOW
        rospy.loginfo("DRONE DO UP TO 1.60 M")
        self.set_alt_vel(1.6)
        
        # 18. MOVE LEFT TO GO OUTSIDE 
        rospy.loginfo("-- DRONE MOVE RIGHT FOR GET OUT FROM JENDELA --")
        velz = 0 
        vely = 0.30
        velx = 0
        
        rospy.loginfo("-- DRONE GET OUT FROM JENDELA --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(4)
        rospy.loginfo_throttle(0.2,f"[Move Left] current pose : {self.drone.current_pose.pose.pose.position} ")
        
        #////// OUTDOOR MISSION /////////
        # 19. CHANGE SOURCE TO RTK OR ACTIVATE THE GPS
        self.drone.set_ekf_source(2)
        self.drone.use_gps(True)
        self.drone.set_home()
        home_heading = self.drone.compass
        head = radians(home_heading)
        
        # 20. SET MISSION FOR FIRST POINT
        coordinate =  GeoPoint()
        coordinate.altitude = 5
        coordinate.latitude = -7.26455 #CHANGE THIS
        coordinate.longitude = 112.78474 #CHANGE THIS
        
        self.drone.move_global_raw(coordinate)
        
        #>>> CHECK HAS REACHED TARGET OR NOT
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.sleep(0.1)
        
        rospy.loginfo("-- CALL SERVICE CAM 3 --")
        self.activate_target(ActivateRequest(True, 3))
        
        rospy.loginfo("-- SEARCHING FOR RED CARPET --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.activate_target(ActivateRequest(False, 3))
        self.activate_target(ActivateRequest(True, 4))
            
        rospy.loginfo("-- SEARCHING FOR GREY COLOR --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.set_alt_vel(1)
        rospy.sleep(2)
        
        self.drone.set_servo(6, 900)
        
        # 21. SET MISSION FOR SECOND POINT
        coordinate.altitude = 5
        coordinate.latitude = -7.26455 #CHANGE THIS
        coordinate.longitude = 112.78474 #CHANGE THIS
        
        self.drone.move_global_raw(coordinate)
        self.activate_target(ActivateRequest(False, 4))
        
        #>>> CHECK HAS REACHED TARGET OR NOT
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.sleep(0.1)
        
        rospy.loginfo("-- CALL SERVICE CAM 3 --")
        self.activate_target(ActivateRequest(True, 3))
        
        rospy.loginfo("-- SEARCHING FOR RED CARPET --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.activate_target(ActivateRequest(False, 3))
        self.activate_target(ActivateRequest(True, 4))
            
        rospy.loginfo("-- SEARCHING FOR GREY COLOR --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.set_alt_vel(1)
        rospy.sleep(2)
        
        self.drone.set_servo(7, 2300)
        
        # 22. SET MISSION FOR LAST POINT
        coordinate.altitude = 2
        coordinate.latitude = -7.26455 #CHANGE THIS
        coordinate.longitude = 112.78474 #CHANGE THIS
        
        self.activate_target(ActivateRequest(False, 4))
        self.drone.move_global_raw(coordinate)
        
        #>>> CHECK HAS REACHED TARGET OR NOT
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.sleep(0.1)
        
        rospy.loginfo("-- CALL SERVICE CAM 3 --")
        self.activate_target(ActivateRequest(True, 3))
        
        rospy.loginfo("-- SEARCHING FOR RED CARPET --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
        
        # 23. FINISH
        self.drone.set_mode("LAND")
        
    def full_right(self):
        """
            FUNCTION FOR FULL AUTO MISSION, INDOOR + OUTDOOR WITH COMPLETELY COMPUTER VISION
            MAY ALLAH BLESS US
            JALUR KIRI

            Full mission, bismillahirrahmannirrahim:
            --- indoor ---
            0. MAKE SURE THIS FIRST PREP:
                > Activate message for ultrasonic
                > Set GPS as OFF
                > Set home destination
            1. Change source to optical flow
            2. Takeoff to 1m
            3. Call service cam 1
            4. Move forward
            5. Move forward until detect the payload
            6. Test color following activate
            7. Do the pick up algorithm
            8. Drone stop a while
            9. Drone up 1.3 meter after pick up
            10. Turn off cam 1, turn on cam 2
            11. Move forward for move right or right
            12. Use ultrasonnnic for detect wall and move
            13. Drone move to right after detect wall
            14. Drone do scan ember
            15. Drop payload
            16. Move right after drop payload
            17. Increase  altitude 
            18. Move right to go outside

            --- outdoor ---
            19. Change soure to RTK
            20. Set mission for first point
            21. Set mission for first point
            22. Set mission for last point
            23. FINISH
        """
        
        # 0. FIRST PREP 
        rospy.wait_for_message("/us", Float32)
        self.drone.use_gps(False)
        self.drone.wait4start()
        self.drone.move_vel(0,0,0)
        
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to OPTICAL FLOW")
        self.drone.set_ekf_source(1)

        # 2. Takeoff
        check = self.drone.takeoff(1)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        # 3. Call service
        rospy.loginfo("-- CALL SERVICE --")
        self.activate_target(ActivateRequest(True, 1))
        
        # 4. Move forward
        velz = 0
        vely = 0
        velx = 0.3
        
        rospy.loginfo("-- DRONE VELX POSITIF MOVE FORWARD --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Forward] current pose : {self.drone.current_pose.pose.pose.position} ")
        
        # 5. Move until detect the payload
        while not self.target_data.is_found:
            for _ in range(30):
                self.drone.move_vel(0.17, 0, 0)
                rospy.sleep(0.1)
        
        # 6. TEST COLOR FOLLOWING ACTIVATE
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(8).to_sec():
            self.test_color_following()
            
        # 7. DO THE PICKUP ALGORITHM
        rospy.loginfo("-- PICKUP ALGORITHM --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time2 < rospy.Duration(6).to_sec():
            rospy.loginfo("Masuk ke fungsi PICKUP ALGORITHM")
            res = self.pickup_algorithm()
            rospy.sleep(0.01)
            if res:
                break
            
        rospy.sleep(1)
        
        # 8. DRONE STOP A WHILE
        rospy.loginfo("DRONE DO STOPS")
        for _ in range (10):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.01)
        rospy.sleep(2)
        
        # 9. DRONE UP 1.3 METER AFTER PICK UP
        rospy.loginfo("DRONE DO UP 1.30 M")
        self.set_alt_vel(1.3)

        # 10. TURN ON CAM 2
        rospy.loginfo("DRONE TURN ON CAM 2") #Object 2        
        self.activate_target(ActivateRequest(False, 1))
        self.activate_target(ActivateRequest(True, 2))
        
        # 11. MOVE FORWARD FOR MOVE RIGHT
        velz = 0
        vely = 0
        velx = 0.35
        
        rospy.loginfo("-- DRONE VELX POSITIF MOVE FORWARD --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Forward] current pose : {self.drone.current_pose.pose.pose.position} ")
        
        # 12. USE ULTRASONIC FOR DETECT WALL AND MOVE LEFT OR RIGHT
        rospy.loginfo("DRONE USE ULTRASONIC TO DETECT THE WALL AND MOVE RIGHT OR LEFT")
        while self.us_data > 120 and self.us_data < 50:
            for _ in range (30):
                self.drone.move_vel(0.19, 0, 0)
                rospy.sleep(0.1)
        
        rospy.loginfo("DRONE DO STOPS BEFORE MOVE RIGHT OR LEFT")
        for _ in range (20):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.01)
        rospy.sleep(2)
        
        # 13. DRONE MOVE TO RIGHT AFTER DETECT WALL
        velz = 0
        vely = -0.35 # RIGHT
        velx = 0

        rospy.loginfo("-- DRONE VELY POSITIF MOVE RIGHT --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Rigth] current pose : {self.drone.current_pose.pose.pose.position} ")
            
        while not self.target_data.is_found:
            for _ in range(30):
                self.drone.move_vel(0, -0.25, 0)
                rospy.sleep(0.1)
                
        self.x_pid.reset()
        self.y_pid.reset()
        
        # 14. DRONE DO SCAN EMBER
        rospy.loginfo("-- SEARCHING FOR EMBER --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(10).to_sec():
            self.test_color_following()

        rospy.loginfo("DRONE DO DROP PAYLOAD")
        self.drone.set_servo(5, 800)

        rospy.loginfo("DRONE DO MOVE RIIGHT")
        self.activate_target(ActivateRequest(True, 5))
        
        self.set_alt_vel(1.68)
        
        rospy.loginfo("-- SEARCHING FOR EMBER --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(6).to_sec():
            self.test_color_following()
        
        velz = 0
        vely = -0.30
        velx = 0
        
        # # # Move command        
        for _ in range(30):
            rospy.loginfo("DRONE MOVE RIGHT LEPAS PAYLOAD")
            rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(5)
        
        
        self.drone.set_mode("LAND")
        
    def test_relay(self):
        # r = rospy.Rate(20)
        self.drone.wait4start()
        home_heading = self.drone.get_home_heading()
        start = rospy.Time.now().to_sec()
        state = True
        while rospy.Time.now() - start < rospy.Duration(30):
            rospy.loginfo(f"Relay is {state} - should be {'on' if not state else 'off'}")
            self.drone.switch_relay(relay=0, status=state)
            state = not state
            rospy.sleep(2)

    def test_gazebo_attach(self):
        """
            right: bool, default is False

            Full mission, dalam nama Tuhan Yesus:
            --- indoor ---
            0. Set relay as ON
            1. Change source to optical flow
            2. Takeoff to 75cm
            3. Forward 1m
            4. Descend to ~25cm (-0.5m) (skipped)
            5. Pickup algorithm
            6. wait 3s
            7. drop
        

        """

        self.drone.wait4start()

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        # 0. Relay as ON
        self.drone.switch_relay(0, False)

        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to optical flow")
        self.drone.set_ekf_source(2)
        
        # 2. Takeoff
        check = self.drone.takeoff(1)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(2)

        ## 3. Forward 1m
        x,y = body2local(1, 0, head)
        dist = {"x": x, "y": y, "z": 1.2, "heading": home_heading} # in sim the z is in LOCAL_NED idk why 

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        # while not self.drone.check_waypoint_reached(dist):
        #     # self.drone.move(dist)
        #     rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
        #     rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")

        # self.drone.stop()
        rospy.sleep(2)

        ## 4. Skipped, descend will be done by the pickup_algorithm()
        ## 5. Payload search and pickup algorithm
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")

        # Call service
        self.activate_target(ActivateRequest(True, 1))

        # Do the pickup
        result = self.pickup_algorithm()
        # while not result:
        if result == False:
            result = self.pickup_algorithm()

        if result == False:
            # Descend, do the brute force
            target_alt = 0.33
            err = target_alt - self.drone.rangefinder
            self.z_pid.reset()
            while self.drone.rangefinder < target_alt - 0.015 or self.drone.rangefinder > target_alt + 0.015:
                err = target_alt - self.drone.rangefinder
                velz = self.z_pid.update(err)
                self.drone.move_vel(0, 0, velz)

        self.drone.stop()
        rospy.sleep(1)

        # Call service
        self.activate_target(ActivateRequest(False,1))

        # Print result
        rospy.loginfo("-- PAYLOAD SHOULD BE ATTACHED, CONTINUING --")

        x,y = body2local(0, 0, head)
        dist = {"x": x, "y": y, "z": 2, "heading": home_heading} # in sim the z is in LOCAL_NED idk why 
        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)
        
        self.drone.stop()
        rospy.sleep(1)

        # wait 3s
        rospy.sleep(3)
        # if self.sim:
        #     rospy.sleep(0.1)
        #     # Link them
        #     rospy.loginfo("detaching drone and payload")
        #     req = AttachRequest()
        #     req.model_name_1 = "iris"
        #     req.link_name_1 = "iris::drone::iris::base_link"
        #     req.model_name_2 = "object kiri"
        #     req.link_name_2 = "link"

        #     resp = self.detach_srv.call(req)
        #     rospy.loginfo(f"attach : {resp.ok}")

    def gerak_pos(self): #For tes moving with position
        self.drone.wait4start()
        home_heading = self.drone.get_home_heading() # tetep 0
        head = home_heading # current heading
        kiri = home_heading + 90
        kanan = -90
        zeros = 0 
        
        rospy.loginfo("Change to RTK")
        rospy.loginfo(f"home heading: {home_heading}\ncurrent heading: {head}")
        self.drone.set_ekf_source(1)
        # self.drone.set_origin()
        self.drone.arm()
        
        # 2. Drone will take off 0.75 m
        self.drone.takeoff(0.5)

        rospy.sleep(2)
        # TASK: Set where is x(+), x(-), y(+), y(-), z(+), z(-)
        '''
        x(+) = maju (North)
        y(+) = kanan (East)
        z(+) = turun (Down)
        '''
        
        x, y, z, head = body2local(2, 0, 0, head)
        dist = {
            "x": x, 
            "y": y, 
            "z": z, 
            "heading": head
            }
        rospy.loginfo(f"MAJU 2 METER ({dist})")
        rospy.loginfo(f"home heading: {home_heading}\ncurrent heading: {head}")
        for _ in range(20):
            self.drone.move(dist)
            rospy.sleep(0.1)
            
        for _ in range(20):
            self.drone.stop()
            rospy.sleep(0.1)
            
        # rospy.sleep(2)
            
        x, y, z, head = body2local(1, 0, 0, head)
        dist = {
            "x": x, 
            "y": y, 
            "z": z, 
            "heading": head
            }
        rospy.loginfo(f"MAJU 1 METER ({dist})")
        rospy.loginfo(f"home heading: {home_heading}\ncurrent heading: {head}")
        for _ in range(10):
            self.drone.move(dist)
            rospy.sleep(0.1)
        
        for _ in range(50):
            self.drone.stop()
            rospy.sleep(0.1)
        
        # rospy.spin()
        
        self.drone.set_mode("LAND")
        
    def geopoint2geopose(self,point:GeoPoint)->GeoPoseStamped:
        pose = GeoPoseStamped()
        pose.pose.position.latitude = point.latitude
        pose.pose.position.longitude = point.longitude
        pose.pose.position.altitude = point.altitude
        return pose

    def outdoor_kanan(self):
        '''
            Checklist:
            1. Check fungsi mengecek sudah sampai tujuan Lat Long?
            2. Vision berjalan baik, Detec 3 dulu baru 4?
            3. Apakah set_speed bisa mengubah kecepatan?
        
        '''

        self.drone.wait4start()
        # Destination
        self.drone.set_home()
        home_heading = self.drone.compass

        head = radians(home_heading)
        print("compass heading: ",home_heading)
        print("imu heading: ",self.drone.imu_heading)
        print("cur heading: ",self.drone.current_heading)

        # 0. Relay as ON
        # self.drone.switch_relay(0, False)

#       # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to GPS")
        self.drone.set_ekf_source(2)

        gps = self.drone.gps
        coordinate = GeoPoint()
        # coordinate = getFinalLatLong(gps.latitude,gps.longitude,5,home_heading)
        coordinate.altitude = 3
        coordinate.latitude = -7.9152715
        coordinate.longitude = 110.5659067
 
        # 2. Takeoff
        check = self.drone.takeoff(3)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(2)
        
        rospy.loginfo("Change Speed to 3 m/s")
        self.drone.set_speed(0, 3, -1)

        rospy.loginfo("Move to first WP ")
        # self.drone.move_global(coordinate=self.geopoint2geopose(coordinate),heading=0)
        self.drone.move_global_raw(coordinate)
        # BELUM DI CEK WORK ATAU NGGAK 
        # while not self.drone.check_waypoint_reached_global(coordinate):
        #     rospy.sleep(0.1)
        
        rospy.loginfo("-- CALL SERVICE --")
        self.activate_target(ActivateRequest(True, 3))
        
        rospy.loginfo("-- SEARCHING FOR RED CARPET --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.activate_target(ActivateRequest(False, 3))
        self.activate_target(ActivateRequest(True, 4))
            
        rospy.loginfo("-- SEARCHING FOR GREY COLOR --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.set_alt_vel(1)
        rospy.sleep(2)
        
        self.drone.set_servo(6, 900)
        
        # 21. SET MISSION FOR SECOND POINT
        coordinate.altitude = 5
        coordinate.latitude = -7.26455 #CHANGE THIS
        coordinate.longitude = 112.78474 #CHANGE THIS
        
        self.drone.move_global_raw(coordinate)
        self.activate_target(ActivateRequest(False, 4))
        
        #>>> CHECK HAS REACHED TARGET OR NOT
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.sleep(0.1)
        
        rospy.loginfo("-- CALL SERVICE CAM 3 --")
        self.activate_target(ActivateRequest(True, 3))
        
        rospy.loginfo("-- SEARCHING FOR RED CARPET --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.activate_target(ActivateRequest(False, 3))
        self.activate_target(ActivateRequest(True, 4))
            
        rospy.loginfo("-- SEARCHING FOR GREY COLOR --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.set_alt_vel(1)
        rospy.sleep(2)
        
        self.drone.set_servo(7, 2300)
        
        # 22. SET MISSION FOR LAST POINT
        coordinate.altitude = 2
        coordinate.latitude = -7.26455 #CHANGE THIS
        coordinate.longitude = 112.78474 #CHANGE THIS
        
        self.activate_target(ActivateRequest(False, 4))
        self.drone.move_global_raw(coordinate)
        
        #>>> CHECK HAS REACHED TARGET OR NOT
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.sleep(0.1)
        
        rospy.loginfo("-- CALL SERVICE CAM 3 --")
        self.activate_target(ActivateRequest(True, 3))
        
        rospy.loginfo("-- SEARCHING FOR RED CARPET --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
        
        # 23. FINISH
        self.drone.set_mode("LAND")
        
    def full_right(self):
        """
            FUNCTION FOR FULL AUTO MISSION, INDOOR + OUTDOOR WITH COMPLETELY COMPUTER VISION
            MAY ALLAH BLESS US
            JALUR KIRI

            Full mission, bismillahirrahmannirrahim:
            --- indoor ---
            0. MAKE SURE THIS FIRST PREP:
                > Activate message for ultrasonic
                > Set GPS as OFF
                > Set home destination
            1. Change source to optical flow
            2. Takeoff to 1m
            3. Call service cam 1
            4. Move forward
            5. Move forward until detect the payload
            6. Test color following activate
            7. Do the pick up algorithm
            8. Drone stop a while
            9. Drone up 1.3 meter after pick up
            10. Turn off cam 1, turn on cam 2
            11. Move forward for move right or right
            12. Use ultrasonnnic for detect wall and move
            13. Drone move to right after detect wall
            14. Drone do scan ember
            15. Drop payload
            16. Move right after drop payload
            17. Increase  altitude 
            18. Move right to go outside

            --- outdoor ---
            19. Change soure to RTK
            20. Set mission for first point
            21. Set mission for first point
            22. Set mission for last point
            23. FINISH
        """
        
        # 0. FIRST PREP 
        rospy.wait_for_message("/us", Float32)
        self.drone.use_gps(False)
        self.drone.wait4start()
        self.drone.move_vel(0,0,0)
        
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to OPTICAL FLOW")
        self.drone.set_ekf_source(1)

        # 2. Takeoff
        check = self.drone.takeoff(1)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        # 3. Call service
        rospy.loginfo("-- CALL SERVICE --")
        self.activate_target(ActivateRequest(True, 1))
        
        # 4. Move forward
        velz = 0
        vely = 0
        velx = 0.3
        
        rospy.loginfo("-- DRONE VELX POSITIF MOVE FORWARD --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Forward] current pose : {self.drone.current_pose.pose.pose.position} ")
        
        # 5. Move until detect the payload
        while not self.target_data.is_found:
            for _ in range(30):
                self.drone.move_vel(0.17, 0, 0)
                rospy.sleep(0.1)
        
        # 6. TEST COLOR FOLLOWING ACTIVATE
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(8).to_sec():
            self.test_color_following()
            
        # 7. DO THE PICKUP ALGORITHM
        rospy.loginfo("-- PICKUP ALGORITHM --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time2 < rospy.Duration(6).to_sec():
            rospy.loginfo("Masuk ke fungsi PICKUP ALGORITHM")
            res = self.pickup_algorithm()
            rospy.sleep(0.01)
            if res:
                break
            
        rospy.sleep(1)
        
        # 8. DRONE STOP A WHILE
        rospy.loginfo("DRONE DO STOPS")
        for _ in range (10):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.01)
        rospy.sleep(2)
        
        # 9. DRONE UP 1.3 METER AFTER PICK UP
        rospy.loginfo("DRONE DO UP 1.30 M")
        self.set_alt_vel(1.3)

        # 10. TURN ON CAM 2
        rospy.loginfo("DRONE TURN ON CAM 2") #Object 2        
        self.activate_target(ActivateRequest(False, 1))
        self.activate_target(ActivateRequest(True, 2))
        
        # 11. MOVE FORWARD FOR MOVE RIGHT
        velz = 0
        vely = 0
        velx = 0.35
        
        rospy.loginfo("-- DRONE VELX POSITIF MOVE FORWARD --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Forward] current pose : {self.drone.current_pose.pose.pose.position} ")
        
        # 12. USE ULTRASONIC FOR DETECT WALL AND MOVE LEFT OR RIGHT
        rospy.loginfo("DRONE USE ULTRASONIC TO DETECT THE WALL AND MOVE RIGHT OR LEFT")
        while self.us_data > 120 and self.us_data < 50:
            for _ in range (30):
                self.drone.move_vel(0.19, 0, 0)
                rospy.sleep(0.1)
        
        rospy.loginfo("DRONE DO STOPS BEFORE MOVE RIGHT OR LEFT")
        for _ in range (20):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.01)
        rospy.sleep(2)
        
        # 13. DRONE MOVE TO RIGHT AFTER DETECT WALL
        velz = 0
        vely = -0.35 # RIGHT
        velx = 0

        rospy.loginfo("-- DRONE VELY POSITIF MOVE RIGHT --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Rigth] current pose : {self.drone.current_pose.pose.pose.position} ")
            
        while not self.target_data.is_found:
            for _ in range(30):
                self.drone.move_vel(0, -0.25, 0)
                rospy.sleep(0.1)
                
        self.x_pid.reset()
        self.y_pid.reset()
        
        # 14. DRONE DO SCAN EMBER
        rospy.loginfo("-- SEARCHING FOR EMBER --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(10).to_sec():
            self.test_color_following()

        rospy.loginfo("DRONE DO DROP PAYLOAD")
        self.drone.set_servo(5, 800)

        rospy.loginfo("DRONE DO MOVE RIIGHT")
        self.activate_target(ActivateRequest(True, 5))
        
        self.set_alt_vel(1.68)
        
        rospy.loginfo("-- SEARCHING FOR EMBER --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(6).to_sec():
            self.test_color_following()
        
        velz = 0
        vely = -0.30
        velx = 0
        
        # # # Move command        
        for _ in range(30):
            rospy.loginfo("DRONE MOVE RIGHT LEPAS PAYLOAD")
            rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(5)
        
        
        self.drone.set_mode("LAND")
        
    def test_relay(self):
        # r = rospy.Rate(20)
        self.drone.wait4start()
        home_heading = self.drone.get_home_heading()
        start = rospy.Time.now().to_sec()
        state = True
        while rospy.Time.now() - start < rospy.Duration(30):
            rospy.loginfo(f"Relay is {state} - should be {'on' if not state else 'off'}")
            self.drone.switch_relay(relay=0, status=state)
            state = not state
            rospy.sleep(2)

    def test_gazebo_attach(self):
        """
            right: bool, default is False

            Full mission, dalam nama Tuhan Yesus:
            --- indoor ---
            0. Set relay as ON
            1. Change source to optical flow
            2. Takeoff to 75cm
            3. Forward 1m
            4. Descend to ~25cm (-0.5m) (skipped)
            5. Pickup algorithm
            6. wait 3s
            7. drop
        

        """

        self.drone.wait4start()

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        # 0. Relay as ON
        self.drone.switch_relay(0, False)

        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to optical flow")
        self.drone.set_ekf_source(2)
        
        # 2. Takeoff
        check = self.drone.takeoff(1)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(2)

        ## 3. Forward 1m
        x,y = body2local(1, 0, head)
        dist = {"x": x, "y": y, "z": 1.2, "heading": home_heading} # in sim the z is in LOCAL_NED idk why 

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        # while not self.drone.check_waypoint_reached(dist):
        #     # self.drone.move(dist)
        #     rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
        #     rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")

        # self.drone.stop()
        rospy.sleep(2)

        ## 4. Skipped, descend will be done by the pickup_algorithm()
        ## 5. Payload search and pickup algorithm
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")

        # Call service
        self.activate_target(ActivateRequest(True, 1))

        # Do the pickup
        result = self.pickup_algorithm()
        # while not result:
        if result == False:
            result = self.pickup_algorithm()

        if result == False:
            # Descend, do the brute force
            target_alt = 0.33
            err = target_alt - self.drone.rangefinder
            self.z_pid.reset()
            while self.drone.rangefinder < target_alt - 0.015 or self.drone.rangefinder > target_alt + 0.015:
                err = target_alt - self.drone.rangefinder
                velz = self.z_pid.update(err)
                self.drone.move_vel(0, 0, velz)

        self.drone.stop()
        rospy.sleep(1)

        # Call service
        self.activate_target(ActivateRequest(False,1))

        # Print result
        rospy.loginfo("-- PAYLOAD SHOULD BE ATTACHED, CONTINUING --")

        x,y = body2local(0, 0, head)
        dist = {"x": x, "y": y, "z": 2, "heading": home_heading} # in sim the z is in LOCAL_NED idk why 
        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)
        
        self.drone.stop()
        rospy.sleep(1)

        # wait 3s
        rospy.sleep(3)
        # if self.sim:
        #     rospy.sleep(0.1)
        #     # Link them
        #     rospy.loginfo("detaching drone and payload")
        #     req = AttachRequest()
        #     req.model_name_1 = "iris"
        #     req.link_name_1 = "iris::drone::iris::base_link"
        #     req.model_name_2 = "object kiri"
        #     req.link_name_2 = "link"

        #     resp = self.detach_srv.call(req)
        #     rospy.loginfo(f"attach : {resp.ok}")

    def gerak_pos(self): #For tes moving with position
        self.drone.wait4start()
        home_heading = self.drone.get_home_heading() # tetep 0
        head = home_heading # current heading
        kiri = home_heading + 90
        kanan = -90
        zeros = 0 
        
        rospy.loginfo("Change to RTK")
        rospy.loginfo(f"home heading: {home_heading}\ncurrent heading: {head}")
        self.drone.set_ekf_source(1)
        # self.drone.set_origin()
        self.drone.arm()
        
        # 2. Drone will take off 0.75 m
        self.drone.takeoff(0.5)

        rospy.sleep(2)
        # TASK: Set where is x(+), x(-), y(+), y(-), z(+), z(-)
        '''
        x(+) = maju (North)
        y(+) = kanan (East)
        z(+) = turun (Down)
        '''
        
        x, y, z, head = body2local(2, 0, 0, head)
        dist = {
            "x": x, 
            "y": y, 
            "z": z, 
            "heading": head
            }
        rospy.loginfo(f"MAJU 2 METER ({dist})")
        rospy.loginfo(f"home heading: {home_heading}\ncurrent heading: {head}")
        for _ in range(20):
            self.drone.move(dist)
            rospy.sleep(0.1)
            
        for _ in range(20):
            self.drone.stop()
            rospy.sleep(0.1)
            
        # rospy.sleep(2)
            
        x, y, z, head = body2local(1, 0, 0, head)
        dist = {
            "x": x, 
            "y": y, 
            "z": z, 
            "heading": head
            }
        rospy.loginfo(f"MAJU 1 METER ({dist})")
        rospy.loginfo(f"home heading: {home_heading}\ncurrent heading: {head}")
        for _ in range(10):
            self.drone.move(dist)
            rospy.sleep(0.1)
        
        for _ in range(50):
            self.drone.stop()
            rospy.sleep(0.1)
        
        # rospy.spin()
        
        self.drone.set_mode("LAND")
        
    def geopoint2geopose(self,point:GeoPoint)->GeoPoseStamped:

        pose = GeoPoseStamped()
        pose.pose.position.latitude = point.latitude
        pose.pose.position.longitude = point.longitude
        pose.pose.position.altitude = point.altitude
        return pose

    def outdoor_kanan(self):
        '''
            Checklist:
            1. Check fungsi mengecek sudah sampai tujuan Lat Long?
            2. Vision berjalan baik, Detec 3 dulu baru 4?
            3. Apakah set_speed bisa mengubah kecepatan?
        
        '''

        self.drone.wait4start()
        # Destination
        self.drone.set_home()
        home_heading = self.drone.compass

        head = radians(home_heading)
        print("compass heading: ",home_heading)
        print("imu heading: ",self.drone.imu_heading)
        print("cur heading: ",self.drone.current_heading)

        # 0. Relay as ON
        # self.drone.switch_relay(0, False)

#        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to GPS")
        self.drone.set_ekf_source(2)

        gps = self.drone.gps
        coordinate = GeoPoint()
        # coordinate = getFinalLatLong(gps.latitude,gps.longitude,5,home_heading)
        coordinate.altitude = 3
        coordinate.latitude = -7.9152715
        coordinate.longitude = 110.5659067
 
        # 2. Takeoff
        check = self.drone.takeoff(3)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(2)
        
        rospy.loginfo("Change Speed to 3 m/s")
        self.drone.set_speed(0, 3, -1)

        rospy.loginfo("Move to first WP ")
        # self.drone.move_global(coordinate=self.geopoint2geopose(coordinate),heading=0)
        self.drone.move_global_raw(coordinate)
        # BELUM DI CEK WORK ATAU NGGAK 
        # while not self.drone.check_waypoint_reached_global(coordinate):
        #     rospy.sleep(0.1)
        
        rospy.loginfo("-- CALL SERVICE --")
        self.activate_target(ActivateRequest(True, 3))
        
        rospy.loginfo("-- SEARCHING FOR RED CARPET --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.activate_target(ActivateRequest(False, 3))
        self.activate_target(ActivateRequest(True, 4))
            
        rospy.loginfo("-- SEARCHING FOR GREY COLOR --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.set_alt_vel(1)
        rospy.sleep(2)
        
        self.drone.set_servo(6, 900)
        
        # 21. SET MISSION FOR SECOND POINT
        coordinate.altitude = 5
        coordinate.latitude = -7.26455 #CHANGE THIS
        coordinate.longitude = 112.78474 #CHANGE THIS
        
        self.drone.move_global_raw(coordinate)
        self.activate_target(ActivateRequest(False, 4))
        
        #>>> CHECK HAS REACHED TARGET OR NOT
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.sleep(0.1)
        
        rospy.loginfo("-- CALL SERVICE CAM 3--")
        self.activate_target(ActivateRequest(True, 3))

        rospy.loginfo("Move to second WP ")
        self.drone.move_global_raw(coordinate)
        # BELUM DI CEK WORK ATAU NGGAK 
        # rospy.loginfo("-- CHECK REACH OR NOT --")
        # while not self.drone.check_waypoint_reached_global(coordinate):
            # rospy.sleep(0.1)
            
        rospy.loginfo("-- CALL SERVICE --")
        
        rospy.loginfo("-- SEARCHING FOR RED CARPET --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.activate_target(ActivateRequest(False, 3))
        self.activate_target(ActivateRequest(True, 4))
            
        rospy.loginfo("-- SEARCHING FOR GREY COLOR --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
            
        self.set_alt_vel(1)
        rospy.sleep(2)
        
        self.drone.set_servo(7, 2300)
        
        # 22. SET MISSION FOR LAST POINT
        coordinate.altitude = 2
        coordinate.latitude = -7.26455 #CHANGE THIS
        coordinate.longitude = 112.78474 #CHANGE THIS
        
        self.activate_target(ActivateRequest(False, 4))
        self.drone.move_global_raw(coordinate)
        
        #>>> CHECK HAS REACHED TARGET OR NOT
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.sleep(0.1)
        
        rospy.loginfo("-- CALL SERVICE CAM 3 --")
        self.activate_target(ActivateRequest(True, 3))
        
        rospy.loginfo("-- SEARCHING FOR RED CARPET --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time < rospy.Duration(5).to_sec():
            self.test_color_following()
        
        # 23. FINISH
        self.drone.set_mode("LAND")
        
    def full_right(self):
        """
            FUNCTION FOR FULL AUTO MISSION, INDOOR + OUTDOOR WITH COMPLETELY COMPUTER VISION
            MAY ALLAH BLESS US
            JALUR KIRI

            Full mission, bismillahirrahmannirrahim:
            --- indoor ---
            0. MAKE SURE THIS FIRST PREP:
                > Activate message for ultrasonic
                > Set GPS as OFF
                > Set home destination
            1. Change source to optical flow
            2. Takeoff to 1m
            3. Call service cam 1
            4. Move forward
            5. Move forward until detect the payload
            6. Test color following activate
            7. Do the pick up algorithm
            8. Drone stop a while
            9. Drone up 1.3 meter after pick up
            10. Turn off cam 1, turn on cam 2
            11. Move forward for move right or right
            12. Use ultrasonnnic for detect wall and move
            13. Drone move to right after detect wall
            14. Drone do scan ember
            15. Drop payload
            16. Move right after drop payload
            17. Increase  altitude 
            18. Move right to go outside

            --- outdoor ---
            19. Change soure to RTK
            20. Set mission for first point
            21. Set mission for first point
            22. Set mission for last point
            23. FINISH
        """
        
        # 0. FIRST PREP 
        rospy.wait_for_message("/us", Float32)
        self.drone.use_gps(False)
        self.drone.wait4start()
        self.drone.move_vel(0,0,0)
        
        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to OPTICAL FLOW")
        self.drone.set_ekf_source(1)

        # 2. Takeoff
        check = self.drone.takeoff(1)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        # 3. Call service
        rospy.loginfo("-- CALL SERVICE --")
        self.activate_target(ActivateRequest(True, 1))
        
        # 4. Move forward
        velz = 0
        vely = 0
        velx = 0.3
        
        rospy.loginfo("-- DRONE VELX POSITIF MOVE FORWARD --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Forward] current pose : {self.drone.current_pose.pose.pose.position} ")
        
        # 5. Move until detect the payload
        while not self.target_data.is_found:
            for _ in range(30):
                self.drone.move_vel(0.17, 0, 0)
                rospy.sleep(0.1)
        
        # 6. TEST COLOR FOLLOWING ACTIVATE
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(8).to_sec():
            self.test_color_following()
            
        # 7. DO THE PICKUP ALGORITHM
        rospy.loginfo("-- PICKUP ALGORITHM --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() - start_time2 < rospy.Duration(6).to_sec():
            rospy.loginfo("Masuk ke fungsi PICKUP ALGORITHM")
            res = self.pickup_algorithm()
            rospy.sleep(0.01)
            if res:
                break
            
        rospy.sleep(1)
        
        # 8. DRONE STOP A WHILE
        rospy.loginfo("DRONE DO STOPS")
        for _ in range (10):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.01)
        rospy.sleep(2)
        
        # 9. DRONE UP 1.3 METER AFTER PICK UP
        rospy.loginfo("DRONE DO UP 1.30 M")
        self.set_alt_vel(1.3)

        # 10. TURN ON CAM 2
        rospy.loginfo("DRONE TURN ON CAM 2") #Object 2        
        self.activate_target(ActivateRequest(False, 1))
        self.activate_target(ActivateRequest(True, 2))
        
        # 11. MOVE FORWARD FOR MOVE RIGHT
        velz = 0
        vely = 0
        velx = 0.35
        
        rospy.loginfo("-- DRONE VELX POSITIF MOVE FORWARD --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Forward] current pose : {self.drone.current_pose.pose.pose.position} ")
        
        # 12. USE ULTRASONIC FOR DETECT WALL AND MOVE LEFT OR RIGHT
        rospy.loginfo("DRONE USE ULTRASONIC TO DETECT THE WALL AND MOVE RIGHT OR LEFT")
        while self.us_data > 120 and self.us_data < 50:
            for _ in range (30):
                self.drone.move_vel(0.19, 0, 0)
                rospy.sleep(0.1)
        
        rospy.loginfo("DRONE DO STOPS BEFORE MOVE RIGHT OR LEFT")
        for _ in range (20):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.01)
        rospy.sleep(2)
        
        # 13. DRONE MOVE TO RIGHT AFTER DETECT WALL
        velz = 0
        vely = -0.35 # RIGHT
        velx = 0

        rospy.loginfo("-- DRONE VELY POSITIF MOVE RIGHT --")
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        rospy.loginfo_throttle(0.2,f"[Move Rigth] current pose : {self.drone.current_pose.pose.pose.position} ")
            
        while not self.target_data.is_found:
            for _ in range(30):
                self.drone.move_vel(0, -0.25, 0)
                rospy.sleep(0.1)
                
        self.x_pid.reset()
        self.y_pid.reset()
        
        # 14. DRONE DO SCAN EMBER
        rospy.loginfo("-- SEARCHING FOR EMBER --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(10).to_sec():
            self.test_color_following()

        rospy.loginfo("DRONE DO DROP PAYLOAD")
        self.drone.set_servo(5, 800)

        rospy.loginfo("DRONE DO MOVE RIIGHT")
        self.activate_target(ActivateRequest(True, 5))
        
        self.set_alt_vel(1.68)
        
        rospy.loginfo("-- SEARCHING FOR EMBER --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(6).to_sec():
            self.test_color_following()
        
        velz = 0
        vely = -0.30
        velx = 0
        
        # # # Move command        
        for _ in range(30):
            rospy.loginfo("DRONE MOVE RIGHT LEPAS PAYLOAD")
            rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(5)
        
        
        self.drone.set_mode("LAND")
        
    def test_relay(self):
        # r = rospy.Rate(20)
        self.drone.wait4start()
        home_heading = self.drone.get_home_heading()

    def test_gladi(self):
        '''
            H-9 Jam, our objective is pick the payload and goal the payload        
        '''
        self.drone.wait4start()
        self.drone.move_vel(0,0,0)

        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        rospy.loginfo(f"Changing source to OPTICAL FLOW")
        self.drone.set_ekf_source(1)

        rospy.loginfo(f"Changing source to RTK")
        self.drone.set_ekf_source(2)

        self.drone.takeoff(1)
        rospy.sleep(3)

        rospy.loginfo("-- CALL SERVICE --")
        self.activate_target(ActivateRequest(True, 1))

        velz = 0
        vely = 0 
        velx = 0.2

        # # Move command        
        for _ in range(30):
            rospy.loginfo("DRONE VELX POSITIF")
            rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)

        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(8).to_sec():
            self.test_color_following()

        # Do the pickup
        rospy.loginfo("-- PICKUP ALGORITHM --")
        # while rospy.Time.now().to_sec() - now < rospy.Duration(10).to_sec():
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(10).to_sec():
            rospy.loginfo("Masuk ke fungsi PICKUP ALGORITHM")
            res = self.pickup_algorithm()
            rospy.sleep(0.01)
            if res:
                break

        rospy.loginfo("DRONE DO STOPS")
        for _ in range (10):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.01)
        rospy.sleep(2)

        self.activate_target(ActivateRequest(False, 1))
        self.activate_target(ActivateRequest(True, 2))

        self.set_alt_vel(1.3)

        velz = 0
        vely = 0 
        velx = 0.2

        # # Move command        
        for _ in range(30):
            rospy.loginfo("DRONE MAJU SETELAH AMBIL PAYLOAD")
            rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)

        while not self.target_data.is_found:
            for _ in range(30):
                self.drone.move_vel(0,20, 0, 0)
                rospy.sleep(0.1)

        self.x_pid.reset()
        self.y_pid.reset()
        rospy.loginfo(f"Value of pid x: {self.x_pid.reset()} and pid y: {self.y_pid.reset()}")

        rospy.loginfo("-- SEARCHING FOR EMBER --")
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(10).to_sec():
            self.test_color_following()

        rospy.loginfo("DRONE DO DROP PAYLOAD")
        self.drone.set_servo(5, 800)

        rospy.loginfo("DRONE DO MOVE RIIGHT")
        self.activate_target(ActivateRequest(False, 2))

        velz = 0
        vely = -0.20
        velx = 0

        # # Move command        
        for _ in range(30):
            rospy.loginfo("DRONE MOVE RIGHT LEPAS PAYLOAD")
            rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)

        self.drone.set_mode("LAND")
        
    def test_ultrasonik(self):
        rospy.wait_for_message("/us", Float32)
        # self.drone.use_gps(False)
        self.drone.wait4start()
        self.drone.move_vel(0,0,0)
        now = rospy.Time.now().to_sec()

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to GPS RTK")
        self.drone.set_ekf_source(2)

        # 2. Takeoff
        self.drone.takeoff(1)
        rospy.sleep(5)
        
        self.drone.set_servo(6, 900)
        self.drone.set_servo(7, 2300)
        
        # rospy.wait_for_message("/us", Float32)
        # # start = rospy.Time().now().to_sec()
        # # while rospy.Time().now().to_sec() -start < rospy.Duration(8).to_sec():
        # # while self.us_data > 70:
        # #     rospy.loginfo("Di luar rentang ultrasonik")
        #     # rospy.logdebug("Di luar rentang ultrasonik")
        # if self.us_data < 75:
        #     rospy.loginfo("DRONE DEKAT DENGAN TEMBOKKKKKK")
        # else:
        #     rospy.loginfo("DRONE JAUH DENGAN TEMBOK")  
        # # rospy.logdebug_throttle("DRONE DEKAT DENGAN TEMBOKKKKKK")
        
        # # rospy.spin()
        self.drone.set_mode("LAND")
        
    def misi_indor(self):
        rospy.wait_for_message("/us", Float32)
        # self.drone.use_gps(False)
        self.drone.wait4start()
        self.drone.move_vel(0,0,0)
        now = rospy.Time.now().to_sec()

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to GPS RTK")
        self.drone.set_ekf_source(2)

        # 2. Takeoff
        self.drone.takeoff(1)
        rospy.sleep(5)
        
        # Call service
        rospy.loginfo("-- CALL SERVICE --")
        self.activate_target(ActivateRequest(True, 1))

        velz = 0
        vely = 0 
        velx = 0.25
        
        # # Move command        
        for _ in range(30):
            rospy.loginfo("DRONE VELX POSITIF")
            rospy.loginfo_throttle(0.2,f"current pose : {self.drone.current_pose.pose.pose.position} ")
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(1)
        
        # self.drone.stop()
        
        while not self.target_data.is_found:
            for _ in range(30):
                self.drone.move_vel(0.2, 0, 0)
                rospy.sleep(0.01)

        ## 4. Skipped, descend will be done by the pickup_algorithm()
        ## 5. Payload search and pickup algorithm
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")
        start_time = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time < rospy.Duration(8).to_sec():
            self.test_color_following()

        # Do the pickup
        rospy.loginfo("-- PICKUP ALGORITHM --")
        # while rospy.Time.now().to_sec() - now < rospy.Duration(10).to_sec():
        start_time2 = rospy.Time().now().to_sec()
        while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(10).to_sec():
            rospy.loginfo("Masuk ke fungsi PICKUP ALGORITHM")
            res = self.pickup_algorithm()
            rospy.sleep(0.01)
            if res:
                break
            
        # rospy.sleep(1)
        
        rospy.loginfo("DRONE DO STOPS")
        for _ in range (10):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.01)
        rospy.sleep(2)
        
        self.activate_target(ActivateRequest(False, 1))
        self.activate_target(ActivateRequest(True, 2))
        
        self.set_alt_vel(1.3)
        
        self.drone.set_servo(5, 800)
        
        velz = 0
        vely = 0 
        velx = 0.2
        
        # # Move command        
        for _ in range(30):
            rospy.loginfo("DRONE MAJU SETELAH AMBIL PAYLOAD")
            rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        rospy.sleep(2)
        
        # while self.us_data > 120 or self.us_data < 50:
        #     for _ in range(30):
        #         self.drone.move_vel(0.19, 0, 0)
        #         rospy.sleep(0.1)
        
        # rospy.loginfo("DRONE DO STOPS")
        # for _ in range (20):
        #     self.drone.move_vel(0,0,0)
        #     rospy.sleep(0.01)
        # rospy.sleep(3)
        
        # velz = 0
        # vely = -0.20
        # velx = 0
        
        # # # Move command        
        # for _ in range(30):
        #     rospy.loginfo("DRONE KIRI SETELAH DETECT TEMBOK")
        #     rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
        #     rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")
        #     self.drone.move_vel(velx, vely, velz)
        #     rospy.sleep(0.01)
        # rospy.sleep(2)
        
        # while not self.target_data.is_found:
        #     for _ in range(30):
        #         self.drone.move_vel(0, -0.17, 0)
        #         rospy.sleep(0.1)
        
        # self.x_pid.reset()
        # self.y_pid.reset()
        
        # rospy.loginfo("-- SEARCHING FOR EMBER --")
        # start_time2 = rospy.Time().now().to_sec()
        # while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(10).to_sec():
        #     self.test_color_following()
        
        # rospy.loginfo("DRONE DO DROP PAYLOAD")
        # self.drone.set_servo(5, 800)
        
        # rospy.loginfo("DRONE DO MOVE RIIGHT")
        # self.activate_target(ActivateRequest(True, 5))
        
        # self.set_alt_vel(1.68)
        
        # rospy.loginfo("-- SEARCHING FOR EMBER --")
        # start_time2 = rospy.Time().now().to_sec()
        # while rospy.Time().now().to_sec() -start_time2 < rospy.Duration(6).to_sec():
        #     self.test_color_following()
        
        # velz = 0
        # vely = -0.30
        # velx = 0
        
        # # # Move command        
        # for _ in range(30):
        #     rospy.loginfo("DRONE MOVE RIGHT LEPAS PAYLOAD")
        #     rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
        #     rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")
        #     self.drone.move_vel(velx, vely, velz)
        #     rospy.sleep(0.01)
        # rospy.sleep(5)
        
        
        self.drone.set_mode("LAND")
             
    def test_change_mode(self):
        # self.activate_target(ActivateRequest(True, 1))
        # self.activate_target(ActivateRequest(True, 2))
        self.drone.wait4start()
        self.drone.move_vel(0,0,0)
        
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)
        
        rospy.loginfo(f"Changing source to OPTICAL FLOW")
        self.drone.set_ekf_source(2)

        # 2. Takeoff
        self.drone.takeoff(1)
        rospy.sleep(2)
        
        rospy.loginfo("Changing source to OPTICAL FLOW X RTK")
        self.drone.set_ekf_source(1)
        
        rospy.loginfo("Changing to AUTO")
        self.drone.set_mode("AUTO")
    
    def coba_gazebo1(self):
        """
        Misi: drone terbang melewati exit gate menggunakan EGO Planner.
        """
        self.drone.wait4start()
        self.drone.set_ekf_source(1)
        self.drone.arm()
        self.drone.takeoff(0.8)
        rospy.sleep(2)
        self.drone.set_mode("GUIDED")
        rospy.sleep(1)
        rospy.loginfo("[MISI] Takeoff selesai, mulai EGO Planner")

        # ## Payload search and pickup algorithm
        # rospy.loginfo("-- SEARCHING FOR PAYLOAD --")
        # rospy.wait_for_service('/vision/activate/target')
        # self.activate_target(ActivateRequest(True, 1))

        # result = self.pickup_algorithm()
        # if not result:
        #     for _ in range(30):
        #         self.drone.move_vel(0, 0, 0)
        #     self.z_pid.reset()
        #     target_alt = 0.15
        #     while self.drone.rangefinder > target_alt + 0.015:
        #         err = target_alt - self.drone.rangefinder
        #         velz = self.z_pid.update(err)
        #         self.drone.move_vel(0, 0, velz)
        #     result = self.pickup_algorithm()

        # self.drone.stop()
        # self.activate_target(ActivateRequest(False, 1))
        # rospy.loginfo("-- PAYLOAD SHOULD BE ATTACHED, CONTINUING --")
        # for _ in range(30):
        #     self.drone.move_vel(0, 0, 0)


        # # rostopic echo /mavros/local_position/odom | grep -A3 "position"     buat cek posisi

        self.drone.navigate_ego(
            goal_x=-1.23,
            goal_y=0.0,
            exit_alt=0.8,
            timeout=10.0,
            allow_yaw=True,
        )

        self.drone.navigate_ego(
            goal_x=-5.2,
            goal_y=0.0,
            exit_alt=0.8,
            timeout=10.0,
            allow_yaw=True,
        )

        self.drone.navigate_ego(
            goal_x=-5.2,
            goal_y=4.7,
            exit_alt=0.8,
            timeout=120.0,
            allow_yaw=True,
        )

        self.drone.navigate_ego(
            goal_x=-5.2,
            goal_y=8.0,
            exit_alt=1.2,
            timeout=120.0,
            allow_yaw=True,
        )

        self.drone.navigate_ego(
            goal_x=-5.6,
            goal_y=11.2,
            exit_alt=1.0,
            min_alt=0.5,
            timeout=30.0,
            allow_yaw=True,
        )

        self.drone.set_mode("LAND")



    def coba_gazebo(self):
        self.drone.wait4start()

        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to GPS")
        self.drone.set_ekf_source(1)
        
        # self.drone.set_origin()
        self.drone.arm()
        # 2. Takeoff
        check = self.drone.takeoff(0.6)
        # rospy.Timer(rospy.Duration(1/5), self.lidar_avoidance)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(1)

        rospy.loginfo("drone akan MAJU")
        for _ in range(90):
            rospy.sleep(0.01)
            self.drone.move_vel(-0.55, 0, 0)
        rospy.sleep(3)

        # rospy.spin() # for looping this function
        ## 5. Payload search and pickup algorithm
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")

        # # Call service
        self.activate_target(ActivateRequest(True, 1))

        rospy.wait_for_service('/vision/activate/target')
        # try:
        #     activate_service = rospy.Service('/vision/activate/target', Activate)
        #     response = activate_service(ActivateRequest(True, 1))
        #     rospy.loginfo(f"Service response: {response}")
        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Service call failed: {e}")
        #     return

        # Do the pickup
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()

            if result == False:
                for _ in range(30):
                    self.drone.move_vel(0,0,0)
                # Descend, do the brute force
                self.z_pid.reset()
                target_alt = 0.05
                err = target_alt - self.drone.rangefinder
                while self.drone.rangefinder < target_alt - 0.015 or self.drone.rangefinder > target_alt + 0.015:
                    err = target_alt - self.drone.rangefinder
                    velz = self.z_pid.update(err)
                    self.drone.move_vel(0,0,velz)

        rospy.loginfo("-- PAYLOAD SHOULD BE ATTACHED, CONTINUING --")
        for _ in range(100):
            self.drone.move_vel(0,0,0)

        rospy.loginfo("drone akan Naik")
        for _ in range(30):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, 0.2)
        rospy.sleep(3)

        rospy.loginfo("drone akan MAJU")
        for _ in range(30):
            rospy.sleep(0.01)
            self.drone.move_vel(-1.0, 0, 0)
        rospy.sleep(3)
        
        rospy.loginfo("drone akan STOP")
        for _ in range(30):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, 0)
        rospy.sleep(3)

        # rospy.loginfo("drone akan menghadap ke KANAN")
        # hadap = self.drone.change_heading(90)
        # rospy.loginfo(f"Drone menghadap ke: {hadap}")
        # rospy.sleep(1)
        
        # if not self.drone.stable_motion():
        #     rospy.logwarn("Drone tidak stabil, menunggu stabil...")
        #     rospy.sleep(2)

        rospy.loginfo("drone akan MENYAMPING")
        for _ in range(28): #30
            rospy.sleep(0.01)
            self.drone.move_vel(0, 1.5, 0)
        rospy.sleep(3)

        self.activate_target(ActivateRequest(False, 1))
        rospy.loginfo("-- SEARCHING FOR DROP BUCKET --")
        # # Call service
        self.activate_target(ActivateRequest(True, 2))
        rospy.wait_for_service('/vision/activate/target')

        # Activate service
        # self.activate_target(ActivateRequest(True, 2))
        # # Do the search
        # start = rospy.Time.now().to_sec() #.secs
        # result = self.test_color_following_krti()
        # while not result:
        #     result = self.test_color_following_krti()

        rospy.loginfo("drone akan NAIK") #turun
        for _ in range(10): # 30
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, 0.07) #-0.07
        rospy.sleep(3)
        
        rospy.loginfo("-- SEARCH STOPPED, DROPPING OBJECT --")
        
        # Stop drone movement for stability
        for _ in range(10):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, 0)
        
        # Detach payload (single call, not in loop)
        detach_success = self.detach_payload("object kanan")
        if detach_success:
            rospy.loginfo("✅ First payload successfully detached")
        else:
            rospy.logwarn("⚠️ Failed to detach first payload - continuing anyway")
        
        rospy.sleep(3)

        rospy.loginfo("drone akan NAIK")
        for _ in range(10):
            rospy.sleep(0.01)
            # self.drone.stop()
            self.drone.move_vel(0. , 0, 0.25)
        rospy.sleep(3)

        rospy.loginfo("drone akan menuju EXIT GATE")
        for _ in range(30):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 2.5, 0)
        rospy.sleep(3)

        for _ in range(100):
            rospy.sleep(0.01)
            # self.drone.stop()
            self.drone.move_vel(0 , 0, 0)
        rospy.sleep(3)

        # rospy.loginfo("drone akan KARPET KIRI BAWAH")
        # for _ in range(100):
        #     rospy.sleep(0.01)
        #     self.drone.move_vel(1.3, -9.2, 0)
        # rospy.sleep(3)

        # for _ in range(100):
        #     rospy.sleep(0.01)
        #     # self.drone.stop()
        #     self.drone.move_vel(0 , 0, 0)
        # rospy.sleep(3)

        # rospy.loginfo("drone akan KARPET KIRI ATAS")
        # for _ in range(300):
        #     rospy.sleep(0.01)
        #     self.drone.move_vel(-50, -0.05, 0)
        # rospy.sleep(3)

        # for _ in range(100):
        #     rospy.sleep(0.01)
        #     # self.drone.stop()
        #     self.drone.move_vel(0 , 0, 0)
        # rospy.sleep(3)

        # #kurang ke kiri dan ke bawah dikit saja, lihat gambar di Documents/Tutorial Sim
        # rospy.loginfo("drone akan KARPET TERAKHIR")
        # for _ in range(400):
        #     rospy.sleep(0.01)
        #     self.drone.move_vel(52, 50, 0)
        # rospy.sleep(3)

        for _ in range(30):
            rospy.sleep(0.01)
            # self.drone.stop()
            self.drone.move_vel(0 , 0, 0)
        rospy.sleep(3)

        jarak = self.drone.rangefinder
        
        rospy.spin()

        # 3. Land
        self.drone.set_mode("LAND")

    def pickup_algorithm(self):
        rospy.loginfo_throttle(1,"Masuk Fungsi Pickup Algorithm")
        start = rospy.Time.now().to_sec()
        x_done = False
        y_done = False

        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")

            # If it took longer than 10 seconds, we'll do
            # the drop no matter what, believing in God that
            # He will move the object as we want to
            if rospy.Time.now().to_sec() - start > 10:
                return False
        try:
            while self.target_data.is_found :
            
                # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
                cur_pose = {
                    "x": self.drone.current_pose.pose.pose.position.x,
                    "y": self.drone.current_pose.pose.pose.position.y,
                    "z": self.drone.rangefinder,
                }
                move_to = {
                    "x": 0,
                    "y": 0,
                    "z": 0,
                }
                dx = self.target_data.dx
                dy = self.target_data.dy
                if not(dx > 40  or dx < -40):
                    x_done = True
                if not(dy > 40  or dy < -40):
                    y_done = True

                alt = cur_pose["z"]

                if x_done and y_done and cur_pose["z"] < 0.15  and self.target_data.is_found:
                    rospy.loginfo("target altitude reached")
                    break

                rospy.logdebug(f"current position : {cur_pose}")
                rospy.logdebug(f"move to : {move_to}")
                
                rospy.logdebug(f"current position : {cur_pose}")
                velx = self.x_pid.update(-self.target_data.dx/360)
                vely = self.y_pid.update(-self.target_data.dy/360) 
                target_alt = 0.15
                rospy.loginfo_throttle(0.5, "-------- Target_alt = 0.15 FUNGSI TRUE -------")
                err = target_alt - alt
                velz = self.z_pid.update(err)
                rospy.loginfo(f"velx : {velx}, vely : {vely} velz : {velz}")
                if self.drone.stable_motion():
                    # pass      
                    # self.drone.move_vel(-velx, -vely, velz)
                    move_detect = self.drone.move_vel(0, 0, velz)
                    rospy.loginfo(f"Move to target: {move_detect}")
                    # self.drone.move(move_to)
            if self.sim:
                rospy.sleep(0.1)
                # Link them
                rospy.loginfo("Attaching drone and payload")
                req = AttachRequest()
                req.model_name_1 = "iris"
                req.link_name_1 = "iris::drone::iris::base_link"
                req.model_name_2 = "object kanan" #change to object kiri, when pick up left mode
                req.link_name_2 = "object kanan::object::link"

                resp = self.attach_srv.call(req)
                rospy.loginfo(f"attach : {resp.ok}")
                
                if resp.ok:
                    self.payload_attached = True
                    self.current_payload = "object kanan"

            return True
        
        except:
            return False
    
    def detach_payload(self, payload_name):
        """
        Detach the payload from the drone in Gazebo simulation.
        """
        rospy.loginfo_throttle(1,"Masuk Fungsi Detach Algorithm")
        start = rospy.Time.now().to_sec()
        x_done = False
        y_done = False
        z_done = False

        # Wait for target to be found
        while not self.target_data.is_found:
            rospy.loginfo_throttle(0.2,"waiting for target")
            if rospy.Time.now().to_sec() - start > 10:
                rospy.logwarn("Timeout waiting for target")
                return False
        
        rospy.loginfo("Target found, starting alignment process")
        
        try:
            # Phase 1: Reach target altitude (1m)
            target_alt = 0.8
            rospy.loginfo(f"Phase 1: Reaching target altitude {target_alt}m")
            
            while not z_done and self.target_data.is_found:
                cur_alt = self.drone.rangefinder
                alt_error = target_alt - cur_alt
                
                # Check if altitude is within acceptable range
                if abs(alt_error) < 0.05:  # Within 5cm
                    z_done = True
                    rospy.loginfo(f"Target altitude reached: {cur_alt:.3f}m")
                    break
                
                velz = self.z_pid.update(alt_error)
                if self.drone.stable_motion():
                    self.drone.move_vel(0, 0, velz)
                    rospy.loginfo_throttle(0.5, f"Altitude: {cur_alt:.3f}m, Target: {target_alt}m, Error: {alt_error:.3f}m")
                
                rospy.sleep(0.01)
            
            if not z_done:
                rospy.logwarn("Failed to reach target altitude")
                return False
            
            # Phase 2: Align horizontally with target
            rospy.loginfo("Phase 2: Aligning horizontally with target")
            alignment_timeout = rospy.Time.now().to_sec()
            
            while not (x_done and y_done) and self.target_data.is_found:
                # Check timeout
                if rospy.Time.now().to_sec() - alignment_timeout > 10:
                    rospy.logwarn("Timeout during horizontal alignment")
                    break
                
                dx = self.target_data.dx
                dy = self.target_data.dy
                
                # Check if target is centered (within 20 pixels)
                if abs(dx) < 10:
                    x_done = True
                if abs(dy) < 10:
                    y_done = True
                
                # If both aligned, break
                if x_done and y_done:
                    rospy.loginfo("Target aligned successfully")
                    break
                
                # Calculate velocities for alignment
                velx = self.x_pid.update(-dx/360)
                vely = self.y_pid.update(-dy/360)
                
                # Maintain altitude while aligning
                cur_alt = self.drone.rangefinder
                alt_error = target_alt - cur_alt
                velz = self.z_pid.update(alt_error)
                
                if self.drone.stable_motion():
                    self.drone.move_vel(velx, vely, velz)
                    rospy.loginfo_throttle(0.5, f"Aligning - dx: {dx:.1f}, dy: {dy:.1f}, alt: {cur_alt:.3f}m")
                rospy.sleep(0.01)

                # Penyesuaian posisi
                for _ in range(10):
                    rospy.sleep(0.01)
                    self.drone.move_vel(0, 0, 0)
                    rospy.sleep(0.5)
            
            # Phase 3: Detach payload
            rospy.loginfo("Phase 3: Detaching payload")
            if self.sim:
                rospy.sleep(0.1)  # Small delay for stability
                
                try:
                    rospy.loginfo(f"Detaching payload: {payload_name}")
                    req = AttachRequest()
                    req.model_name_1 = "iris"
                    req.link_name_1 = "iris::drone::iris::base_link"
                    req.model_name_2 = payload_name
                    req.link_name_2 = f"{payload_name}::object::link"
                    
                    resp = self.detach_srv.call(req)
                    rospy.loginfo(f"Detach response: {resp.ok}")
                    
                    if resp.ok:
                        self.payload_attached = False
                        self.current_payload = None
                        rospy.loginfo("✅ Payload successfully detached")
                        return True
                    else:
                        rospy.logerr("❌ Detach service returned False")
                        return False
                        
                except Exception as e:
                    rospy.logerr(f"Detach error: {e}")
                    return False
            else:
                rospy.logwarn("Not in simulation mode, skipping detach")
                return False
                
        except Exception as e:
            rospy.logerr(f"Error in detach_payload: {e}")
            return False
    
if __name__ == "__main__":
    game = Game()
    try:
        # UNTUK COMVIS
        # game.movement()
        # UNTUK TANPA COMVIS
        # game.wilayah()
        
        # game.last_test()
        
        # game.check_servo()
        game.coba_gazebo1()  # Complete simulation with proper attach/detach
        # game.self_takeoff()
        
        # MISI OUTDOOR
        # game.outdoor()
        # game.outdoor_kanan()
        # MISI INDOOR
        # game.misi_indor()
        
        # game.test_vision()
        # rospy.spin()
        # game.test_change_mode()
        # game.gerak_pos()
        # game.test_ultrasonik()

        # game.test_gazebo_attach()

        # MISI BIRU = KANAN
        # game.full_right

        # MISI MERAH - KIRI
        # game.full_left

        # 25 September 2023
        # game.full3(right=False)

        # game.test_relay()
        # game.test_kiri()
    except KeyboardInterrupt:
        game.drone.set_mode("LAND")
        exit()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.logdebug("exit")

# Bismillahirrahmannirrahim, semoga sukses selalu