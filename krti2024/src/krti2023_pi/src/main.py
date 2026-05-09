#Dengan Menyebut Nama Allah Yang Maha Pengasih dan Yang Maha Penyayang

from krti2023_pi.drone_api import DroneAPI
from krti2023_pi.msg import DResult
from sensor_msgs.msg import LaserScan
from krti2023_pi.srv import Activate, ActivateRequest, ActivateResponse
#from gazebo_link_attacher_ws.srv import Attach, AttachRequest, AttachResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
#from gazebo_ros_link_detacher.srv import Detach, DetachRequest, DetachResponse
import rospy
from geographic_msgs.msg import GeoPoint
from time import sleep
from math import *
import math
from pid import PID

body2local = lambda x,y,heading: (x*cos(heading) - y*sin(heading), x*sin(heading) + y*cos(heading))

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
        self.sim = rospy.get_param("/vision/use_sim", False)
        self.target_sub = rospy.Subscriber(
            "/vision/target/result", DResult, self.target_callback
        )

        self.target_data = DResult()
        self.target_data.is_found = False

        waypoints = []

        self.drone = DroneAPI(waypoints=waypoints, sim=True)
        self.x_pid = PID(0.4, 0.01, 0.1, 0, 1, 0.3, "x")
        self.y_pid = PID(0.4, 0.01, 0.1, 0, 1, 0.3, "y")
        self.z_pid = PID(0.3, 0.01, 0.2, 0, 1, 0.2, "z", 0.2)

        self.avoidance_vector_x = 0
        self.avoidance_vector_y = 0
        self.last_avoidance_timestamp = 0
        self.avoid = False
        self.collision_sub = rospy.Subscriber('/sensors/lidar/sim', LaserScan, self.lidar_avoidance_cb)


    def target_callback(self, msg):
        self.target_data = msg
        rospy.logdebug_throttle(0.2,"target data: {}".format(self.target_data))

    def pickup_algorithm(self):
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
                if not(dx > 80  or dx < -80):
                    x_done = True
                if not(dy > 40  or dy < -40):
                    y_done = True

                alt = cur_pose["z"]

                if x_done and y_done and cur_pose["z"] < 0.33  and self.target_data.is_found:
                    rospy.loginfo("target altitude reached")
                    break

                rospy.logdebug(f"current position : {cur_pose}")
                rospy.logdebug(f"move to : {move_to}")
                
                rospy.logdebug(f"current position : {cur_pose}")
                velx = self.x_pid.update(-self.target_data.dy/360)
                vely = self.y_pid.update(-self.target_data.dx/360) 
                target_alt = 0.32
                err = target_alt - alt
                velz = self.z_pid.update(err)
                rospy.loginfo(f"velx : {velx}, vely : {vely} velz : {velz}")
                if self.drone.stable_motion():
                    # pass      
                    self.drone.move_vel(-velx, -vely, velz)
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

            return True
        
        except:
            return False
        
    def release_algorithm(self):
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
                if not(dx > 80  or dx < -80):
                    x_done = True
                if not(dy > 40  or dy < -40):
                    y_done = True

                alt = cur_pose["z"]

                if x_done and y_done and cur_pose["z"] < 0.33  and self.target_data.is_found:
                    rospy.loginfo("target altitude reached")
                    break

                rospy.logdebug(f"current position : {cur_pose}")
                rospy.logdebug(f"move to : {move_to}")
                
                rospy.logdebug(f"current position : {cur_pose}")
                velx = self.x_pid.update(-self.target_data.dy/360)
                vely = self.y_pid.update(-self.target_data.dx/360) 
                target_alt = 0.32
                err = target_alt - alt
                velz = self.z_pid.update(err)
                rospy.loginfo(f"velx : {velx}, vely : {vely} velz : {velz}")
                if self.drone.stable_motion():
                    # pass      
                    self.drone.move_vel(-velx, -vely, velz)
                    # self.drone.move(move_to)
            if self.sim:
                rospy.sleep(0.1)
                # Drop Payload
                if self.enable_detach:
                    rospy.loginfo("detaching drone and payload")
                    req = AttachRequest()
                    req.model_name_1 = "iris"
                    req.link_name_1 = "iris::drone::iris::base_link"
                    req.model_name_2 = "object kiri"
                    req.link_name_2 = "link"

                    resp = self.detach_srv.call(req)
                    rospy.loginfo(f"detach : {resp.ok}")

            return True
        
        except:
            return False
        
    def lidar_avoidance_cb(self, msg):
        self.current_2D_scan = msg
        self.avoidance_vector_x = 0
        self.avoidance_vector_y = 0
        self.avoid = False
        temp = []
        ranges_temp = []
        index = []

        for i in range(1, len(self.current_2D_scan.ranges)):
            d0 = 0.4
            k = 0.5

            if i % 2 != 1:
                continue

            rospy.loginfo_throttle(1, f"range {i}: {self.current_2D_scan.ranges[i]}") 

            if self.current_2D_scan.ranges[i] < d0 and self.current_2D_scan.ranges[i] > 0.35:
                self.avoid = True
                x = math.cos(self.current_2D_scan.angle_increment * i)
                y = math.sin(self.current_2D_scan.angle_increment * i)
                U = -0.5 * k * ((1 / self.current_2D_scan.ranges[i]) - (1 / d0))**2
                index.append(math.degrees(i * self.current_2D_scan.angle_increment))
                temp.append(U)
                ranges_temp.append(self.current_2D_scan.ranges[i])

                self.avoidance_vector_x += x * -U
                self.avoidance_vector_y += y * -U

        rospy.loginfo_throttle(1, f"Angle: {index}") 
        rospy.loginfo_throttle(1, f"range: {ranges_temp}")        
        rospy.logdebug_throttle(1, f"Nilai U: {temp}")
        rospy.logdebug_throttle(1, f"{self.avoidance_vector_x}, {self.avoidance_vector_y}")

        # body2local
        # home_heading = self.drone.get_home_heading()
        # deg2rad = (math.pi/180)
        # avoidance_vector_x = self.avoidance_vector_x * math.cos(home_heading * deg2rad) - self.avoidance_vector_y * math.sin(home_heading * deg2rad)
        # avoidance_vector_y = self.avoidance_vector_x * math.cos(home_heading * deg2rad) + self.avoidance_vector_y * math.sin(home_heading * deg2rad)

        if self.avoid:
            magnitude = math.sqrt(self.avoidance_vector_x**2 + self.avoidance_vector_y**2)
            if magnitude > 3:
                self.avoidance_vector_x = 3 * (self.avoidance_vector_x / magnitude)
                self.avoidance_vector_y = 3 * (self.avoidance_vector_y / magnitude)

            if self.avoidance_vector_y > 0.15:
                self.avoidance_vector_y = 0
            #     if self.avoidance_vector_y < 0.15:
            #         self.avoidance_vector_y = 0 

            # if rospy.Time().now().secs - self.last_avoidance_timestamp > 0.1:
            self.lidar_avoidance()

    def lidar_avoidance(self):
        # rospy.logdebug(f"avoid {self.avoid}")
        if self.avoid:
            home_heading = self.drone.get_home_heading()
            head = radians(home_heading)
            x,y = body2local(-self.avoidance_vector_y, -self.avoidance_vector_x, head)
            if x > 0.15:
                x = 0
            dist = {"x":  x, "y":  y, "z": 0.6, "heading": home_heading}
            self.drone.move(dist)
            rospy.logdebug(f"Efek Lidar: {dist}")


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
        self.drone.wait4start()
        now = rospy.Time.now()
        x_done = False
        y_done = False
        now = rospy.Time.now()

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")
            if rospy.Time.now().to_sec() - now > rospy.Duration(5):
                rospy.loginfo("timeout")
                return False

        while not rospy.is_shutdown() :
        
            # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
            cur_pose = {
                "x": 0,
                "y": 0,
                "z": 0,
            }
            

            if not (self.target_data.dx > 40 or self.target_data.dx < - 40):
                x_done = True

            if not (self.target_data.dy > 40 or self.target_data.dy < - 40):
                y_done = True

            rospy.logdebug(f"current position : {cur_pose}")
            velx = self.x_pid.update(-self.target_data.dy/360)
            vely = self.y_pid.update(-self.target_data.dx/360)
            rospy.loginfo(f"velx : {velx}, vely : {vely} ")
            if self.drone.stable_motion():
                # pass
                self.drone.move_vel(velx, vely,0)
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
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 4] current pose : {self.drone.current_pose.pose.pose.position}")
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
        # rospy.sleep(30)
        rospy.logwarn(rospy.Time.now().to_sec)

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

        self.drone.set_speed(type=1, speed=10)
        self.drone.set_speed(type=2, speed=5)
        self.drone.set_speed(type=3, speed=150)

        coordinate = getFinalLatLong(lat, lon,50, 180)
        coordinate.altitude = 25
        rospy.loginfo(f"moving to: {coordinate}")
        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)

        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.logdebug("wait for waypoint reached")
            # rospy.loginfo(f"pos {self.drone.gps.longitude}, ")
            rospy.sleep(1)
        
        cur_gps = self.drone.gps
        lat = cur_gps.latitude
        lon = cur_gps.longitude
        coordinate = getFinalLatLong(lat, lon,100, 270)
        coordinate.altitude = 25
        rospy.loginfo(f"moving to: {coordinate}")
        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.logdebug("wait for waypoint reached")
            # rospy.loginfo(f"pos {self.drone.gps.longitude}, ")
            rospy.sleep(1)

        cur_gps = self.drone.gps
        lat = cur_gps.latitude
        lon = cur_gps.longitude
        coordinate = getFinalLatLong(lat, lon,100, 0)
        coordinate.altitude = 25
        rospy.loginfo(f"moving to: {coordinate}")
        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.logdebug("wait for waypoint reached")
            # rospy.loginfo(f"pos {self.drone.gps.longitude}, ")
            rospy.sleep(1)

        cur_gps = self.drone.gps
        lat = cur_gps.latitude
        lon = cur_gps.longitude
        coordinate = getFinalLatLong(lat, lon,100, 90)
        coordinate.altitude = 25
        rospy.loginfo(f"moving to: {coordinate}")
        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.logdebug("wait for waypoint reached")
            # rospy.loginfo(f"pos {self.drone.gps.longitude}, ")
            rospy.sleep(1)

        cur_gps = self.drone.gps
        lat = cur_gps.latitude
        lon = cur_gps.longitude
        coordinate = getFinalLatLong(lat, lon,50, 180)
        coordinate.altitude = 25
        rospy.loginfo(f"moving to: {coordinate}")
        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)

        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.logdebug("wait for waypoint reached")
            # rospy.loginfo(f"pos {self.drone.gps.longitude}, ")
            rospy.sleep(1)
        
    def test_following(self):
        self.drone.set_ekf_source(2)
        # Drone init
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        
        # Takeoff
        check = self.drone.takeoff(0.75)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        ## NWU
        x,y = body2local(1, 0, head)
        # x, y = 0, 1
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": home_heading
        }

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        while True:
            result = self.test_color_following()

            if result == False:
                rospy.loginfo("Keluar program. Ambil alih.")
                break

    def full(self, right = False):
        """
            right: bool, default is False

            Full mission, bismillahirrahmannirrahim:
            --- indoor ---
            0. Set relay as ON
            1. Change source to optical flow
            2. Takeoff to 75cm
            3. Forward 1m
            4. Descend to ~25cm (-0.5m) (skipped)
            5. Pickup algorithm
            6. Ascent to ~1m (+0.75m)
            7. Forward ~4m till LIDAR detect something
            8. Left/right 5m
            9. Detect drop bucket
            10. Drop payload (relay as OFF)
            11. Change source to GPS
            12. Set mission as AUTO

            --- outdoor ---
            13. Following mission planner
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
        x,y = body2local(1.5, 0, head)
        dist = {"x": x, "y": y, "z": 0, "heading": home_heading}

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
        rospy.sleep(3)

        ## 4. Skipped, descend will be done by the pickup_algorithm()
        ## 5. Payload search and pickup algorithm
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")

        # Call service
        self.activate_target(ActivateRequest(True, 1))

        # Do the pickup
        result = self.pickup_algorithm()
        while not result:
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
        self.activate_target(ActivateRequest(False))

        # Print result
        rospy.loginfo("-- PAYLOAD SHOULD BE ATTACHED, CONTINUING --")

        ## 6. Ascent to ~1m (0.75m up)
        target_alt = 1
        err = target_alt - self.drone.rangefinder
        self.z_pid.reset()
        while self.drone.rangefinder < target_alt - 0.05 or self.drone.rangefinder > target_alt + 0.05:
            err = target_alt - self.drone.rangefinder
            velz = self.z_pid.update(err)
            self.drone.move_vel(0, 0, velz)

        self.drone.stop()
        rospy.sleep(1)

        ## 7. Forward ~4m till LIDAR detect something
        x,y = body2local(4, 0, head)
        dist = {"x": x, "y": y, "z": 0, "heading": home_heading}

        # Move command
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if LIDAR detect something
        while self.drone.ultrasonic > 90:
            rospy.loginfo(f"[WP 4] Distance forward: {self.drone.ultrasonic}")
            rospy.sleep(0.1)
        
        rospy.loginfo(f"[WP 4] Out of loop. Distance forward: {self.drone.ultrasonic}")
        self.drone.stop()
        rospy.sleep(5)

        ## 8. Left/right 5m
        if right == True:
            x,y = body2local(0, -5, head)
        else: 
            x,y = body2local(0, 5, head)

        dist = {"x": x, "y": y, "z": 0, "heading": home_heading}

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 3] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 3] waiting for wp reached")

        self.drone.stop()
        rospy.sleep(1)

        ## 9. Detect drop bucket
        rospy.loginfo("-- SEARCHING FOR DROP BUCKET --")

        # Activate service
        self.activate_target(ActivateRequest(True, 2))

        # Do the search
        start = rospy.Time.now().to_sec() #.secs
        result = self.test_color_following()
        while not result:
            result = self.test_color_following()

            # If it took longer than 5 seconds, we'll do
            # the drop no matter what, believing in God that
            # He will move the object as we want to
            if rospy.Time.now().to_sec() - start > 5:
                break

        rospy.loginfo("-- SEARCH STOPPED, DROPPING OBJECT --")
        

        ## 10. Drop payload
        self.drone.switch_relay(0, True)
        rospy.sleep(3)
        
        ## 11. Change source to GPS
        rospy.loginfo(f"Changing source to GPS")
        self.drone.set_ekf_source(1)

        ## 12. Set mission as AUTO
        rospy.loginfo(f"Setting mode as AUTO")
        self.drone.set_mode("AUTO")
        
    def full2(self, right = False):
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
            6. Ascent to ~1m (+0.75m)
            7. Forward ~4m till LIDAR detect something
            8. Left/right 5m
            9. Detect drop bucket
            10. Drop payload (relay as OFF)
            11. Change source to GPS
            12. Set mission as AUTO

            --- outdoor ---
            13. Following mission planner
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

        velz = 0
        vely = 0 
        velx = 0.3
        
        # Move command        
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        
        
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
        while not result:
            result = self.pickup_algorithm()

            if result == False:
                for _ in range(30):
                    self.drone.move_vel(0,0,0)
                # Descend, do the brute force
                self.z_pid.reset()
                target_alt = 0.33
                err = target_alt - self.drone.rangefinder
                while self.drone.rangefinder < target_alt - 0.015 or self.drone.rangefinder > target_alt + 0.015:
                    err = target_alt - self.drone.rangefinder
                    velz = self.z_pid.update(err)
                    self.drone.move_vel(0,0,velz)

        rospy.loginfo("-- PAYLOAD SHOULD BE ATTACHED, CONTINUING --")
        for _ in range(30):
            self.drone.move_vel(0,0,0)

        # Call service
        self.activate_target(ActivateRequest(False))
        
        # Print result
        rospy.loginfo("-- PAYLOAD SHOULD BE ATTACHED, CONTINUING --")

        ## 6. Ascent to ~1m (0.75m up)
        target_alt = 1
        err = target_alt - self.drone.rangefinder
        self.z_pid.reset()
        while self.drone.rangefinder < target_alt - 0.05 or self.drone.rangefinder > target_alt + 0.05:
            err = target_alt - self.drone.rangefinder
            velz = self.z_pid.update(err)
            self.drone.move_vel(0, 0, velz)

        velz = 0
        vely = 0 
        velx = 0.2
        # Move command        
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.1)

        rospy.sleep(1)

        # Check if LIDAR detect something
        while self.drone.ultrasonic > 90:
            rospy.loginfo(f"[WP 4] Distance forward: {self.drone.ultrasonic}")
            rospy.sleep(0.01)
        
        rospy.loginfo(f"KELUAR LOOP ULTRASONIK")
        
        ## 8. Left/right 5m
        if right == True:
            velz = 0
            vely = 0.2
            velx = 0
        else: 
            velz = 0
            vely = -0.2
            velx = 0
        
        # Move command        
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.1)

        ## 9. Detect drop bucket
        rospy.loginfo("-- SEARCHING FOR DROP BUCKET --")

        # Activate service
        self.activate_target(ActivateRequest(True, 2))

        # Do the search
        start = rospy.Time.now().to_sec() #.secs
        result = self.test_color_following()
        while not result:
            result = self.test_color_following()

            # If it took longer than 5 seconds, we'll do
            # the drop no matter what, believing in God that
            # He will move the object as we want to
            if rospy.Time.now().to_sec() - start > 5:
                break

        self.drone.move_vel(0,0,0)
        rospy.loginfo("-- SEARCH STOPPED, DROPPING OBJECT --")

        ## 10. Drop payload
        self.drone.switch_relay(0, True)
        rospy.sleep(3)
        
        ## 11. Change source to GPS
        rospy.loginfo(f"Changing source to GPS")
        self.drone.set_ekf_source(1)

        ## 12. Set mission as AUTO
        rospy.loginfo(f"Setting mode as AUTO")
        self.drone.set_mode("AUTO")

    def coba(self):
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
        for _ in range(100):
            rospy.sleep(0.01)
            self.drone.move_vel(-0.6, 0, 0)
        rospy.sleep(3)

        # rospy.spin() # for looping this function
        ## 5. Payload search and pickup algorithm
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")

        # # Call service
        self.activate_target(ActivateRequest(True, 1))

        # # # # rospy.wait_for_service('/vision/activate/target')
        # # # # try:
        # # # #     activate_service = rospy.Service('/vision/activate/target', Activate)
        # # # #     response = activate_service(ActivateRequest(True, 1))
        # # # # except rospy.ServiceException as e:
        # # # #     rospy.logerr(f"Service call failed: {e}")
        # # # #     return

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
        for _ in range(100):
            rospy.sleep(0.01)
            self.drone.move_vel(-0.9, 0, 0)
        rospy.sleep(3)

        rospy.loginfo("drone akan MENYAMPING")
        for _ in range(100):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 1.2, 0)
        rospy.sleep(3)

        rospy.loginfo("-- SEARCHING FOR DROP BUCKET --")

        # # self.activate_target(ActivateRequest(False, 1))
        # # Activate service
        self.activate_target(ActivateRequest(True, 2))

        # Do the search
        start = rospy.Time.now().to_sec() #.secs
        result = self.test_color_following()
        while not result:
            result = self.test_color_following()

        rospy.loginfo("drone akan TURUN")
        for _ in range(30):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, -0.07)
        rospy.sleep(3)
        
        rospy.loginfo("-- SEARCH STOPPED, DROPPING OBJECT --")
        for _ in range(30):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, 0)
        rospy.sleep(10)

        rospy.loginfo("drone akan NAIK")
        for _ in range(100):
            rospy.sleep(0.01)
            # self.drone.stop()
            self.drone.move_vel(0 , 0, 0.25)
        rospy.sleep(3)

        rospy.loginfo("drone akan menuju EXIT GATE")
        for _ in range(100):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 1.5, 0)
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

        # for _ in range(100):
        #     rospy.sleep(0.01)
        #     # self.drone.stop()
        #     self.drone.move_vel(0 , 0, 0)
        # rospy.sleep(3)

        jarak = self.drone.rangefinder


        # 3. Land
        # self.drone.set_mode("LAND")

    def full3(self, right = False):
        """
            right: bool, default is False
            Full mission, Bismillahirrahmannirrahim:
            --- indoor ---
            0. Set relay as ON
            1. Change source to optical flow
            2. Takeoff to 75cm
            3. Forward 1m
            4. Descend to ~25cm (-0.5m) (skipped)
            5. Pickup algorithm
            6. Ascent to ~1m (+0.75m)
            7. Forward ~4m till LIDAR detect something
            8. Left/right 5m
            9. Detect drop bucket
            10. Drop payload (relay as OFF)
            11. Change source to GPS
            12. Set mission as AUTO

            --- outdoor ---
            13. Following mission planner
        """

        self.drone.wait4start()

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        # 0. Relay as ON
        self.drone.switch_relay(0, False)

        # 1. Set source to optical flow
        # rospy.loginfo(f"Changing source to optical flow") #ketika self.drone.set_ekf_source(2)
        rospy.loginfo(f"Changing source to GPS")
        self.drone.set_ekf_source(3)

        # 2. Takeoff
        check = self.drone.takeoff(1)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(2)

        velz = 0
        vely = 0 
        velx = 0.3
        
        # Move command        
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        
        
        # Check if waypoint is reached or not
        # while not self.drone.check_waypoint_reached(dist):
        #     # self.drone.move(dist)
        #     rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
        #     rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")

        # Call service
        self.activate_target(ActivateRequest(True, 1))

        # self.drone.stop()
        rospy.sleep(2)

        ## 4. Skipped, descend will be done by the pickup_algorithm()
        ## 5. Payload search and pickup algorithm
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")

        # Do the pickup
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()
            if result == False:
                for _ in range(30):
                    self.drone.move_vel(0,0,0)
                # Descend, do the brute force
                self.z_pid.reset()
                target_alt = 0.33
                err = target_alt - self.drone.rangefinder
                while self.drone.rangefinder < target_alt - 0.015 or self.drone.rangefinder > target_alt + 0.015:
                    err = target_alt - self.drone.rangefinder
                    velz = self.z_pid.update(err)
                    self.drone.move_vel(0,0,velz)
                break

        rospy.loginfo("-- PAYLOAD SHOULD BE ATTACHED, CONTINUING --")
        for _ in range(30):
            self.drone.move_vel(0,0,0)

        # Call service
        self.activate_target(ActivateRequest(False, 1))

        ## 6. Ascent to ~1m (0.75m up)
        target_alt = 1
        err = target_alt - self.drone.rangefinder
        self.z_pid.reset()
        while self.drone.rangefinder < target_alt - 0.05 or self.drone.rangefinder > target_alt + 0.05:
            err = target_alt - self.drone.rangefinder
            velz = self.z_pid.update(err)
            self.drone.move_vel(0, 0, velz)

        rospy.sleep(8)
        self.drone.switch_relay(0, True)

        velz = 0
        vely = 0 
        velx = 0.3
        # Move command        
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.1)

        rospy.sleep(3)
        
        for _ in range(30):
            self.drone.move_vel(0,0,0)

        rospy.sleep(1)
        self.drone.set_mode("LAND")


    def self_takeoff(self):
        self.drone.wait4start()

        # 1. Set source to optical flow
        rospy.loginfo(f"Changing source to optical flow")
        self.drone.set_ekf_source(2)

        # 2. Takeoff
        check = self.drone.takeoff(0.5)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(10)

        # 3. Land
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
        rospy.sleep(3)

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
        if self.sim:
            rospy.sleep(0.1)
            # Link them
            rospy.loginfo("detaching drone and payload")
            req = AttachRequest()
            req.model_name_1 = "iris"
            req.link_name_1 = "iris::drone::iris::base_link"
            req.model_name_2 = "object kiri"
            req.link_name_2 = "link"

            resp = self.detach_srv.call(req)
            rospy.loginfo(f"attach : {resp.ok}")


    

    def test_vision(self):
        
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
            6. Ascent to ~1m (+0.75m)
            7. Forward ~4m till LIDAR detect something
            8. Left/right 5m
            9. Detect drop bucket
            10. Drop payload (relay as OFF)
            11. Change source to GPS
            12. Set mission as AUTO

            --- outdoor ---
            13. Following mission planner
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
        check = self.drone.takeoff(0.5)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(2)

        velz = 0
        vely = 0 
        velx = 0.3
        
        # Move command        
        for _ in range(30):
            self.drone.move_vel(velx, vely, velz)
            rospy.sleep(0.01)
        
        
        # Check if waypoint is reached or not
        # while not self.drone.check_waypoint_reached(dist):
        #     # self.drone.move(dist)
        #     rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
        #     rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")

        # Call service
        self.activate_target(ActivateRequest(True, 1))

        # self.drone.stop()
        rospy.sleep(2)

        ## 4. Skipped, descend will be done by the pickup_algorithm()
        ## 5. Payload search and pickup algorithm
        rospy.loginfo("-- SEARCHING FOR PAYLOAD --")

        # Do the pickup
        result = self.pickup_algorithm()
        while not result:
            result = self.pickup_algorithm()

        rospy.loginfo("-- PAYLOAD SHOULD BE ATTACHED, CONTINUING --")
        for _ in range(30):
            self.drone.move_vel(0,0,0)
        
        # self.drone.set_mode("LAND")
        

if __name__ == "__main__":
    game = Game()
    try:

        #game.coba()
        # game.self_takeoff()
        # game.test_vision()

        game.test_gazebo_attach()

        # MISI BIRU = KIRI
        # game.full(right=False)

        # MISI MERAH - KANAN
        # game.full(right=True)

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
