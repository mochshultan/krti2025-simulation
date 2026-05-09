#!/usr/bin/env python3

#Dengan Menyebut Nama Allah Yang Maha Pengasih dan Yang Maha Penyayang

import sys
import os
# Tambahkan parent directory ke sys.path agar bisa import krti2023_pi
sys.path.append(os.path.dirname(__file__))

from krti2023_pi.drone_api import DroneAPI
from krti2024_pi.msg import DResult
from sensor_msgs.msg import LaserScan, Image
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
        rospy.loginfo(f"Result of pickup_algorithm: {result}")
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
        
        # matikan service activate target
        self.activate_target(ActivateRequest(False, 2))

        rospy.loginfo("drone akan NAIK")
        for _ in range(10):
            rospy.sleep(0.01)
            # self.drone.stop()
            self.drone.move_vel(0. , 0, 0.2)
        rospy.sleep(3)

        rospy.loginfo("drone akan BERBELOK KANAN 90 derajat")
        # Stop drone movement for stability before turning
        for _ in range(10):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, 0)
        
        # Turn right 90 degrees
        hadap = self.drone.change_heading(90)
        rospy.loginfo(f"Drone berbelok ke: {hadap} derajat")
        rospy.sleep(2)
        
        # Wait for drone to stabilize after turning
        if not self.drone.stable_motion():
            rospy.logwarn("Drone tidak stabil setelah berbelok, menunggu stabil...")
            rospy.sleep(2)

        rospy.loginfo("drone akan menuju EXIT GATE")
        # Activate gate detection service
        self.activate_target(ActivateRequest(True, 3))  # Assuming gate detection uses target type 3
        rospy.wait_for_service('/vision/activate/target')
        
        go_exit = self.masuk_gate()
        if go_exit:
            rospy.loginfo("✅ Successfully passed through the exit gate")
        else:
            rospy.logwarn("⚠️ Failed to pass through the exit gate - continuing anyway")

        # Landing
        rospy.loginfo("drone akan LAND")
        self.drone.set_mode("LAND")
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

        # for _ in range(30):
        #     rospy.sleep(0.01)
        #     # self.drone.stop()
        #     self.drone.move_vel(0 , 0, 0)
        # rospy.sleep(3)

        # jarak = self.drone.rangefinder
        
        # rospy.spin()

        # # 3. Land
        # self.drone.set_mode("LAND")

    def pickup_algorithm(self):
        rospy.loginfo("Masuk Fungsi Pickup Algorithm")
        start = rospy.Time.now().to_sec()
        threshold = 10  # Jarak pixel dari center

        # Wait for target
        while not self.target_data.is_found:
            rospy.loginfo_throttle(0.2,"waiting for target")
            if rospy.Time.now().to_sec() - start > 10:
                return False
        
        try:
            # Phase 1: Align horizontally while maintaining altitude
            rospy.loginfo("Phase 1: X-Y Horizontal alignment to center")
            while not rospy.is_shutdown():
                if not self.target_data.is_found:
                    break
                    
                dx = self.target_data.dx  # X-axis error (horizontal)
                dy = self.target_data.dy  # Y-axis error (vertical)
                distance_from_center = sqrt(dx*dx + dy*dy)
                
                # Check if both X and Y are centered
                if abs(dx) < threshold and abs(dy) < threshold:
                    rospy.loginfo(f"✅ X-Y CENTERED! dx: {dx}, dy: {dy}, distance: {distance_from_center:.1f}")
                    break
                
                # PID control for both axes
                velx = -self.x_pid.update(dy/180)   # Y error controls X movement (forward/backward)
                vely = self.y_pid.update(dx/180)   # X error controls Y movement (left/right)
                
                if self.drone.stable_motion():
                    self.drone.move_vel(velx, vely, 0)
                    rospy.loginfo_throttle(0.5, f"X-Y Aligning - dx: {dx:+.1f}, dy: {dy:+.1f}, dist: {distance_from_center:.1f}")
                rospy.sleep(0.01)
            
            # Phase 2: Descend to ground while maintaining X-Y center
            rospy.loginfo("Phase 2: Descending while maintaining X-Y center")
            target_alt = 0.05
            while not rospy.is_shutdown():
                if not self.target_data.is_found:
                    break
                    
                alt = self.drone.rangefinder
                dx = self.target_data.dx  # X-axis error
                dy = self.target_data.dy  # Y-axis error
                distance_from_center = sqrt(dx*dx + dy*dy)
                
                # Check if ground reached AND both X-Y centered
                if alt <= target_alt and abs(dx) < threshold and abs(dy) < threshold:
                    rospy.loginfo(f"✅ GROUND + X-Y CENTERED! Alt: {alt:.3f}, dx: {dx}, dy: {dy}")
                    break
                
                # 3-axis PID control
                velx = -self.x_pid.update(dy/180)   # Y error → X movement
                vely = self.y_pid.update(dx/180)   # X error → Y movement  
                err = target_alt - alt
                velz = self.z_pid.update(err)       # Z error → Z movement
                
                if self.drone.stable_motion():
                    self.drone.move_vel(velx, vely, velz)
                    rospy.loginfo_throttle(0.5, f"3D Control - dx: {dx:+.1f}, dy: {dy:+.1f}, alt: {alt:.3f}")
                rospy.sleep(0.01)          
            
            # Phase 3: Stabilize and attach
            rospy.loginfo("Phase 3: Stabilizing for attach")
            for _ in range(10):
                self.drone.move_vel(0, 0, 0)
                rospy.sleep(0.01)
            
            if self.sim:
                rospy.loginfo("Attaching payload")
                req = AttachRequest()
                req.model_name_1 = "iris"
                req.link_name_1 = "iris::drone::iris::base_link"
                req.model_name_2 = "object kanan"
                req.link_name_2 = "object kanan::object::link"
                resp = self.attach_srv.call(req)
                
                if resp.ok:
                    self.payload_attached = True
                    self.current_payload = "object kanan"
                    rospy.loginfo("✅ Payload attached successfully")
                    return True
            return True
        except:
            return False
    
    def detach_payload(self, payload_name):
        """Detach payload menggunakan vision target 2"""
        rospy.loginfo("Masuk Fungsi Detach Algorithm")
        start = rospy.Time.now().to_sec()
        threshold = 5  # Jarak pixel dari center
        target_alt = 0.85

        # Wait for target
        while not self.target_data.is_found:
            rospy.loginfo_throttle(0.2,"waiting for drop target")
            if rospy.Time.now().to_sec() - start > 10:
                return False
        
        try:
            # Phase 1: Position above target at correct altitude with X-Y center
            rospy.loginfo("Phase 1: X-Y-Z Positioning for drop")
            while not rospy.is_shutdown():
                if not self.target_data.is_found:
                    break
                    
                alt = self.drone.rangefinder
                dx = self.target_data.dx  # X-axis error
                dy = self.target_data.dy  # Y-axis error
                distance_from_center = sqrt(dx*dx + dy*dy)
                
                # Check if X-Y centered AND correct altitude
                if abs(dx) < threshold and abs(dy) < threshold and abs(alt - target_alt) < 0.05:
                    rospy.loginfo(f"✅ DROP POSITION PERFECT! dx: {dx}, dy: {dy}, Alt: {alt:.3f}")
                    break
                
                # 3-axis PID control
                velx = -self.x_pid.update(dy/180)   # Y error → X movement
                vely = self.y_pid.update(dx/180)   # X error → Y movement
                err = target_alt - alt
                velz = self.z_pid.update(err)       # Z error → Z movement
                
                if self.drone.stable_motion():
                    self.drone.move_vel(velx, vely, velz)
                    rospy.loginfo_throttle(0.25, f"3D Positioning - dx: {dx:+.1f}, dy: {dy:+.1f}, alt: {alt:.3f}")
                rospy.sleep(0.01)
            
            # Phase 2: Stabilize before detach
            rospy.loginfo("Phase 2: Stabilizing for detach")
            for _ in range(20):
                self.drone.move_vel(0, 0, 0)
            
            # Phase 3: Detach payload
            if self.sim:
                rospy.loginfo(f"Detaching payload: {payload_name}")
                req = AttachRequest()
                req.model_name_1 = "iris"
                req.link_name_1 = "iris::drone::iris::base_link"
                req.model_name_2 = payload_name
                req.link_name_2 = f"{payload_name}::object::link"
                
                resp = self.detach_srv.call(req)
                
                if resp.ok:
                    self.payload_attached = False
                    self.current_payload = None
                    rospy.loginfo("✅ Payload detached successfully")
                    return True
                    
            return False
        except:
            return False
        
    def masuk_gate(self):
        rospy.loginfo("Masuk Gate")
        start = rospy.Time.now().to_sec()
        threshold = 5  # Jarak pixel dari center

        # Wait for target
        while not self.target_data.is_found:
            rospy.loginfo_throttle(0.2,"waiting for gate target")
            if rospy.Time.now().to_sec() - start > 10:
                return False
        try:
            # Phase 1: Position above target at correct altitude with X-Y center
            rospy.loginfo("Phase 1: X-Y-Z Positioning for gate")
            while not rospy.is_shutdown():
                if not self.target_data.is_found:
                    break
                    
                alt = self.drone.rangefinder
                dx = self.target_data.dx  # X-axis error
                dy = self.target_data.dy  # Y-axis error
                distance_from_center = sqrt(dx*dx + dy*dy)
                
                # Check if X-Y centered AND correct altitude
                if abs(dx) < threshold and abs(dy) < threshold:
                    rospy.loginfo(f"✅ GATE POSITION PERFECT! dx: {dx}, dy: {dy}, Alt: {alt:.3f}")
                    break
                
                # 3-axis PID control
                velx = -self.x_pid.update(dx/180)   # X error → X movement
                vely = 0 # hold altitude, no Y movement
                velz = self.z_pid.update(dy/180)     # Y error → Z movement
                
                if self.drone.stable_motion():
                    self.drone.move_vel(velx, vely, velz)
                    rospy.loginfo_throttle(0.25, f"3D Positioning - dx: {dx:+.1f}, dy: {dy:+.1f}, alt: {alt:.3f}")
                rospy.sleep(0.01)
            
            # Phase 2: Stabilize before passing through gate
            rospy.loginfo("Phase 2: Stabilizing for gate pass")
            for _ in range(20):
                self.drone.move_vel(0, 0, 0)

            # Phase 3: Pass through gate
            rospy.loginfo("Phase 3: Passing through gate")
            for _ in range(30):
                self.drone.move_vel(0, 2, 0)
                rospy.sleep(0.01)
            
            return True
        except Exception as e:
            rospy.logerr(f"Error during gate pass: {e}")
            return False

    
if __name__ == "__main__":
    game = Game()
    try:
        game.coba_gazebo()  # Complete simulation with proper attach/detach
        
    except KeyboardInterrupt:
        game.drone.set_mode("LAND")
        exit()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.logdebug("exit")

# Bismillahirrahmannirrahim, semoga sukses selalu