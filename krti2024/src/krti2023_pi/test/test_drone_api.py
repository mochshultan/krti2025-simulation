from krti2023_pi.drone_api import DroneAPI
import rospy
from geometry_msgs.msg import Twist
from geographic_msgs.msg import GeoPoint
from time import sleep
from math import *
from pid import PID
body2local = lambda x,y,heading: (x*cos(heading) - y*sin(heading), x*sin(heading) + y*cos(heading))

def getFinalLatLong(lat1, long1, distance, angle, radius = 6400000) -> GeoPoint:
    """
    lat tau la
    long
    dist : mau berapa jauh
    angle : 0 itu true north jadi haus dikasih heading sekarang kalo mau body "hipotesis" angle heading ya pasti degree
    move_heading = home_heading - 90
    if move_heading < 0:
        move_heading += 360
    jangan gitu ya lain kali
    """
    # // calculate angles
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
        rospy.on_shutdown(self.shutdown)
        self.drone = DroneAPI(sim=False)
        rospy.sleep(3)
        self.drone.wait4start()
    
    def test_override_move(self, args):
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)
        ## NWU
        x,y = body2local(0, 0, head)    
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0,
            "heading": head
        }
        rospy.loginfo(f"We're going to move to {dist}")

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)
    
    def lock_heading(self):
        err_heading = self.drone.get_home_heading() - self.drone.current_heading
        move_pose = {"x" : 0, "y" : 0, "heading" : 0}
        rospy.loginfo(f"err heading : {err_heading}")

        if err_heading > 180:
            err_heading -= 360
        elif err_heading < -180:
            err_heading += 360

        if abs(err_heading) > 3:
            move_pose["heading"] = err_heading
            self.drone.move(move_pose)
    
    def main(self):
        # while not rospy.is_shutdown():
        self.drone.wait4start()
        self.drone.takeoff(1)
        rospy.sleep(5)

        start = rospy.Time.now().to_sec()

        #  == ini pindah ke WP target ==
        # save posisi sekarang
        wp = {"x":1, "y":0, "z":0, "heading":self.drone.home_heading}

        cur_pose = self.drone.current_pose
        move_pose = wp

        err_heading = self.drone.home_heading - self.drone.current_heading
        rospy.loginfo(f"err heading : {err_heading}")

        if err_heading > 180:
            err_heading -= 360
        elif err_heading < -180:
            err_heading += 360

        move_pose["heading"] = err_heading

        for _ in range(100):
            self.drone.move(move_pose)
            rospy.sleep(0.1)

        start_time = rospy.Time.now()
        while not self.drone.check_waypoint_reached(wp):
            rospy.loginfo_throttle(0.2,f"current pose : {self.drone.current_pose.pose.pose.position} ")
            # check timeout
            if rospy.Time.now() - start_time > rospy.Duration(30):
                rospy.logwarn(" TIMEOUT \n waypoint not reached in 10s ")
                rospy.signal_shutdown("timout tidak mencapai wp target")
        
        rospy.loginfo(f" == 1 == time elapsed: {rospy.Time.now() - start}s")
        rospy.sleep(10)

        rospy.signal_shutdown("Mission Finished")

    def test_takeoff(self):
        # r = rospy.Rate(20)

        self.drone.wait4start()

        check = self.drone.takeoff(2)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
            return
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(10)
        self.drone.set_mode("LAND")
        rospy.signal_shutdown("done")    

    def test_color_following(self):
        self.drone.wait4start()

        rospy.loginfo(f"Changing source to optical flow")
        self.drone.set_ekf_source(2)

        self.drone.stop()

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        now = rospy.Time.now()
        x_done = False
        y_done = False
        now = rospy.Time.now()
        while not self.target_data.is_found :
            rospy.loginfo_throttle(0.2,"waiting for target")
            if rospy.Time.now() - now > rospy.Duration(5):
                rospy.loginfo("timeout")
                return False

        while not rospy.is_shutdown() :
        
            # self.move() maju beberapa cm ke depan untuk mencocokan posisi magnet dengan target
            cur_pose = {
                "x": self.drone.current_pose.pose.pose.position.x,
                "y": self.drone.current_pose.pose.pose.position.y,
                "z": self.drone.current_pose.pose.pose.position.z,
            }

            if self.target_data.dx > 80:
                rospy.logdebug("move right")
                cur_pose["y"] += self.target_data.dx_m
            elif self.target_data.dx < -80:
                rospy.logdebug("move left")
                cur_pose["y"] += self.target_data.dx_m
            if self.target_data.dx > 40:
                rospy.logdebug("move right")
                cur_pose["y"] += 0.03
            elif self.target_data.dx < -40:
                rospy.logdebug("move left")
                cur_pose["y"] -= 0.03
            else:
                x_done = True

            if self.target_data.dy > 80:
                rospy.logdebug("move backward")
                cur_pose["x"] += self.target_data.dy_m

            elif self.target_data.dy < -80:
                rospy.logdebug("move forward")
                cur_pose["x"] += self.target_data.dy_m
            
            elif self.target_data.dy > 40:
                rospy.logdebug("move backward")
                cur_pose["x"] += 0.03

            elif self.target_data.dy < -40:
                rospy.logdebug("move forward")
                cur_pose["x"] -= 0.03
            else:
                y_done = True

            ## NWU
            x,y = body2local(cur_pose['x'], cur_pose['y'], head)
            # x, y = 0, 1
            dist = {
                "x": x, 
                "y": y, 
                "z": 0,
                "heading": home_heading
            }

            rospy.logdebug(f"current position : {dist}")
            self.drone.move(dist)
            rospy.sleep(0.05)

        return True

    def test_move_LOCAL_NED(self):
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

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.sleep(10)
        
        
   
    def test_move_naik_turun(self):
        self.drone.set_ekf_source(2)
        # Drone init
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        
        # Takeoff
        check = self.drone.takeoff(1.2)
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
        x,y = body2local(0, 0, head)
        # x, y = 0, 1
        dist = {
            "x": x, 
            "y": y, 
            "z": 0.5,
            "heading": home_heading
        }

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.sleep(10)
        
        ## NWU
        x,y = body2local(0, 0, head)
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": -0.5,
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
            rospy.loginfo_throttle(0.2,f"current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.sleep(10)

    def test_jalan_bawah(self):
        # Drone init
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        self.drone.switch_relay(relay=0, status=True)
        
        # Takeoff
        check = self.drone.takeoff(1)
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
        x,y = body2local(0, 0, head)
        # x, y = 0, 1
        dist = {
            "x": x, 
            "y": y, 
            "z": -0.25,
            "heading": home_heading
        }

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(5)
        rospy.logwarn(rospy.Time.now().to_sec)
        

        ## NWU
        x,y = body2local(3.5, 0, head)
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
            rospy.loginfo_throttle(0.2,f"[WP 2] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 2] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(5)
        rospy.logwarn(rospy.Time.now().to_sec)

        ## NWU
        x,y = body2local(0, 0, head)
        # x, y = 2, 0
        dist = {
            "x": x, 
            "y": y, 
            "z": 0.1,
            "heading": home_heading
        }

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 3] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 3] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(5)
        rospy.logwarn(rospy.Time.now().to_sec)
        self.drone.switch_relay(relay=0, status=False)
        rospy.sleep(5)

        ## NWU
        x,y = body2local(0, 4, head)
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
            rospy.loginfo_throttle(0.2,f"[WP 5] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 5] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(5)
        rospy.logwarn(rospy.Time.now().to_sec)

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
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 6] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 6] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(5)
        rospy.logwarn(rospy.Time.now().to_sec)
    
    def test_23_agustus(self):
        # Drone init
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)
        
        # Takeoff
        check = self.drone.takeoff(1)
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
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 2] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 2] waiting for wp reached")
        self.drone.stop()
        rospy.loginfo("sampe")
        rospy.logwarn(rospy.Time.now().to_sec)
        rospy.sleep(5)
        rospy.logwarn(rospy.Time.now().to_sec)

    def test_26_agustus(self):
        # Drone init
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        rospy.loginfo(f"Changing source to optical flow")
        self.drone.set_ekf_source(2)
        
        # Takeoff
        check = self.drone.takeoff(1.2)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        ## NWU
        x,y = body2local(1, 0, head)    
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
        rospy.sleep(30)
        rospy.logwarn(rospy.Time.now().to_sec)

    def test_8_september(self):
        # Drone init
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)

        rospy.loginfo(f"Changing source to optical flow")
        self.drone.set_ekf_source(1)
        
        # Takeoff
        check = self.drone.takeoff(20)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        ## NWU
        x,y = body2local(10, 0, head)    
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
        rospy.sleep(30)
        rospy.logwarn(rospy.Time.now().to_sec)

    def test_move_interrupt(self):
        # Drone init
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        # self.drone.switch_relay(relay=0, status=True)

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)
        # rospy.loginfo(f"Changing source to optical flow")
        # self.drone.set_ekf_source(2)

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
        
        # rospy.Timer(rospy.Duration(1),self.test_override_move)
        while self.drone.lidar_data.ranges[0] > 1000:
            rospy.loginfo(f"Distance forward: {self.drone.lidar_data.ranges[0]}")
            rospy.sleep(0.1)
        
        rospy.loginfo(f"Out of loop. Distance forward: {self.drone.lidar_data.ranges[0]}")
        self.drone.stop()

    def test_rc_override(self):
        # self.drone.wait4start()
        home_heading = self.drone.get_home_heading()
        start = rospy.Time.now().to_sec()
        state = True
        while rospy.Time.now():

            if(state):
                self.drone.set_rc_override({8:1500})
            else:
                self.drone.set_rc_override({8:1000})
            state = not state
            rospy.sleep(3)

    def test_heading_using_compass(self):
        # Drone init
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")

        # Destination
        home_heading = self.drone.get_home_heading()

        if home_heading + 90 < 0:
            abcd = home_heading + 90 + 360
        else:
            abcd = home_heading + 90

        head = radians(abcd)
        
        ## NWU
        x,y = body2local(0, 0, head)
        dist = {
            "x": x + self.drone.current_pose.pose.pose.position.x, 
            "y": y + self.drone.current_pose.pose.pose.position.y, 
            "z": 1
        }

        rospy.loginfo(f"We're going to move to {dist}")
        
        # Takeoff
        check = self.drone.takeoff(1)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        # 1. Move command
        for _ in range(30):
            self.drone.move(dist)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            self.drone.move(dist)

            
            rospy.loginfo_throttle(0.2, f"Target heading: {abcd} / imu heading : {self.drone.imu_heading} / compass heading : {self.drone.compass}")
            rospy.loginfo_throttle(0.2,f"current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"waiting for wp reached")

        rospy.sleep(5)

        # 2. Move command   
        ## NWU
        x,y = body2local(1, 0, head)
        dist = {
            "x": x + self.drone.current_pose.pose.pose.position.x, 
            "y": y + self.drone.current_pose.pose.pose.position.y, 
            "z": 1
        }
         
        for _ in range(30):
            self.drone.move(dist)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"waiting for wp reached")

        rospy.sleep(5)
    
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

    def test_outdoor(self):
        gps = self.drone.gps
        # goto1 = gps.latitude

    def test_position(self):
        # r = rospy.Rate(20)
        # self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)
        
        ## NWU
        x,y = body2local(1, 0, head)
        dist = {
            "x": x + self.drone.current_pose.pose.pose.position.x, 
            "y": y + self.drone.current_pose.pose.pose.position.y, 
            "z": 1.2
        }

        rospy.loginfo(f"We're going to move to {dist}")

        while not rospy.is_shutdown():
            # rospy.loginfo_throttle(0.1,f"dest : {dist} ")
            rospy.loginfo_throttle(0.1,f"position : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.1,f"imu heading : {self.drone.current_heading} ")
            rospy.loginfo_throttle(0.1,f"compass heading : {self.drone.compass } ")
            # rospy.loginfo_throttle(0.1,f"position : {self.drone.current_velocity} ")

    def shutdown(self):
        # self.drone.set_mode("LAND")
        rospy.loginfo("shutdown")
    
    def test_ekf(self):
        self.drone.set_ekf_source(1)
        rospy.loginfo("SET EKF SOURCE 1")
        rospy.sleep(10)
        rospy.loginfo("SET EKF SOURCE 2")
        self.drone.set_ekf_source(2)
        rospy.sleep(10)
        rospy.loginfo("SET EKF SOURCE 1")
        self.drone.set_ekf_source(1)
        rospy.sleep(10)
        rospy.loginfo("SET EKF SOURCE 2")
        self.drone.set_ekf_source(2)
        rospy.sleep(10)

    def test_move_global(self):
        self.drone.set_ekf_source(1)
        radius = 6400000
        distance = 50
        
        cur_gps = self.drone.gps
        lat = cur_gps.latitude
        lon = cur_gps.longitude

        coordinate = getFinalLatLong(lat, lon,distance, 180,radius)
        coordinate.altitude = 25
        self.drone.wait4start()
        # Takeoff
        check = self.drone.takeoff(10)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(1)
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
        coordinate = getFinalLatLong(lat, lon,100, 270,radius)
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
        coordinate = getFinalLatLong(lat, lon,100, 0,radius)
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
        coordinate = getFinalLatLong(lat, lon,100, 90,radius)
        coordinate.altitude = 25
        rospy.loginfo(f"moving to: {coordinate}")
        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.logdebug("wait for waypoint reached")
            # rospy.loginfo(f"pos {self.drone.gps.longitude}, ")
            rospy.sleep(1)
    
    def test_servo(self):
        state = True
        rospy.loginfo("started")
        while True:
            if state:
                rospy.loginfo("servo on")
                self.drone.set_servo(9,1100)
                self.drone.set_servo(10,1100)
                self.drone.set_servo(11,1100)
                self.drone.set_servo(12,1100)
            else:
                rospy.loginfo("servo off")
                self.drone.set_servo(9,1900)
                self.drone.set_servo(10,1900)
                self.drone.set_servo(11,1900)
                self.drone.set_servo(12,1900)
            state = not state
            rospy.sleep(3)

    def test_lidar(self):
        self.drone.wait4start()

        rospy.loginfo("test_lidar started!")
        while True:
            rospy.loginfo(f"LIDAR depan: {self.drone.lidar_data.ranges[0]}")

    def program_1_17_aug(self):
        """
            Program 1:
            0. Set source to GPS
            1. Takeoff 15m
            2. Forward 3m (relative to heading)
            3. Backward 3m (relative to heading, back to takeoff position)
        """

        self.drone.wait4start()

        # Destination
        head = self.drone.compass
        rospy.loginfo(f"Compass: {head}")

        # Set source to GPS
        rospy.loginfo(f"Changing source to GPS")
        self.drone.set_ekf_source(1)

        # Takeoff
        check = self.drone.takeoff(15)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        ## 1. Forward 3m
        coordinate = getFinalLatLong(self.drone.gps.latitude, self.drone.gps.longitude, 5, head)
        coordinate.altitude = 15
        rospy.loginfo(f"Moving to: {coordinate}")

        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.loginfo_throttle(0.2,f"[WP 1] Current pose: {self.drone.gps.latitude} - {self.drone.gps.longitude}")
            rospy.loginfo_throttle(0.2,"[WP 1] Waiting for wp reached")
            rospy.sleep(0.1)
        self.drone.stop()
        rospy.sleep(10)

        ## 2. Backward 3m
        coordinate = getFinalLatLong(
            self.drone.gps.latitude, self.drone.gps.longitude, 5, 
            head - 180 if head - 180 > 0 else head - 180 + 360
        )
        coordinate.altitude = 15
        rospy.loginfo(f"Moving to: {coordinate}")

        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.loginfo_throttle(0.2,f"[WP 2] Current pose: {self.drone.gps.latitude} - {self.drone.gps.longitude}")
            rospy.loginfo_throttle(0.2,"[WP 2] Waiting for wp reached")
            rospy.sleep(0.1)
        self.drone.stop()
        rospy.sleep(10)

    def program_2_17_aug(self):
        """
            Program 2:
            0. Set source to GPS
            1. Takeoff 10m
            2. Left 5m -> servo 1
            3. Forward 10m -> servo 2
            4. Right 10m -> servo 3
            5. Backward 10m -> servo 4
            6. Left 5m -> back to starting position
        """

        self.drone.wait4start()

        # Destination
        starting_coordinate = self.drone.gps
        head = self.drone.compass

        # 0. Set source to GPS
        rospy.loginfo(f"Changing source to GPS")
        self.drone.set_ekf_source(1)

        # 1. Takeoff
        check = self.drone.takeoff(15)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        # 2. Left 5m
        coordinate = getFinalLatLong(
            self.drone.gps.latitude, self.drone.gps.longitude, 5, 
            head - 90 if head - 90 > 0 else head - 90 + 360
        )
        coordinate.altitude = 15
        rospy.loginfo(f"Moving to: {coordinate}")

        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.loginfo_throttle(0.2,f"[WP 1] Current pose: {self.drone.gps.latitude} - {self.drone.gps.longitude}")
            rospy.loginfo_throttle(0.2,"[WP 1] Waiting for wp reached")
            rospy.sleep(0.1)
        
        # Servo 1
        self.drone.set_servo(9,1900)
        rospy.sleep(3)

        # 3. Forward 10m
        coordinate = getFinalLatLong(self.drone.gps.latitude, self.drone.gps.longitude, 10, head)
        coordinate.altitude = 15
        rospy.loginfo(f"Moving to: {coordinate}")

        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.loginfo_throttle(0.2,f"[WP 2] Current pose: {self.drone.gps.latitude} - {self.drone.gps.longitude}")
            rospy.loginfo_throttle(0.2,"[WP 2] Waiting for wp reached")
            rospy.sleep(0.1)
        
        # Servo 2
        self.drone.set_servo(10,1900)
        rospy.sleep(3)

        # 4. Right 10m
        coordinate = getFinalLatLong(
            self.drone.gps.latitude, self.drone.gps.longitude, 5, 
            head + 90 if head + 90 > 0 else head + 90 - 360
        )
        coordinate.altitude = 15
        rospy.loginfo(f"Moving to: {coordinate}")

        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.loginfo_throttle(0.2,f"[WP 3] Current pose: {self.drone.gps.latitude} - {self.drone.gps.longitude}")
            rospy.loginfo_throttle(0.2,"[WP 3] Waiting for wp reached")
            rospy.sleep(0.1)
        
        # Servo 3
        self.drone.set_servo(11,1900)
        rospy.sleep(3)

        # 5. Backward 10m
        coordinate = getFinalLatLong(
            self.drone.gps.latitude, self.drone.gps.longitude, 10, 
            head - 180 if head - 180 > 0 else head - 180 + 360
        )
        coordinate.altitude = 15
        rospy.loginfo(f"Moving to: {coordinate}")

        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.loginfo_throttle(0.2,f"[WP 4] Current pose: {self.drone.gps.latitude} - {self.drone.gps.longitude}")
            rospy.loginfo_throttle(0.2,"[WP 4] Waiting for wp reached")
            rospy.sleep(0.1)
        
        # Servo 4
        self.drone.set_servo(12,1900)
        rospy.sleep(3)

        # 6. Back to starting position
        coordinate = GeoPoint(latitude=starting_coordinate.latitude, longitude=starting_coordinate.longitude, altitude=15)
        rospy.loginfo(f"Moving to: {coordinate}")

        for _ in range(100):
            self.drone.move_global(coordinate)
            rospy.sleep(0.01)
        
        while not self.drone.check_waypoint_reached_global(coordinate):
            rospy.loginfo_throttle(0.2,f"[WP 5] Current pose: {self.drone.gps.latitude} - {self.drone.gps.longitude}")
            rospy.loginfo_throttle(0.2,"[WP 5] Waiting for wp reached")
            rospy.sleep(0.1)

    def program_3_17_aug(self):
        """
            Program 3:
            0. Set EKF 2 (optical flow)
            1. Set origin
        """

        self.drone.wait4start()

        # Set source to GPS
        rospy.loginfo(f"Changing source to optical flow")
        self.drone.set_ekf_source(2)

        # Force set origin
        self.drone.set_origin({
            "latitude": -7.265572783693384,
            "longitude": 112.78452265474213,
            "altitude": 1,
        })

        # Arm
        self.drone.arm(True)
        rospy.sleep(2)

        # Disarm
        self.drone.arm(False)

    def flight_test_23_sept(self, right: bool = True):
        """
            Right: bool

            Flight test 23 September:
            --- indoor ---
            0. Set relay as ON
            1. Change source to optical flow
            2. Takeoff to 75cm
            3. Forward 1m
            4. Descend to ~25cm (-0.5m)
            5. Color following
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
        check = self.drone.takeoff(0.75)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(3)

        ## 3. Forward 1m
        x,y = body2local(1, 0, head)
        dist = {"x": x, "y": y, "z": 0, "heading": home_heading}

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 1] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 1] waiting for wp reached")

        self.drone.stop()
        rospy.sleep(1)

        ## 4. Descend to 0.25m (~0.5m down)
        x,y = body2local(0, 0, head)
        dist = {"x": x, "y": y, "z": -0.5, "heading": home_heading}

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 2] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 2] waiting for wp reached")

        self.drone.stop()
        rospy.sleep(1)

        ## 5. Pickup algorithm

        ## 6. Ascent to ~1m (0.75m up)
        x,y = body2local(0, 0, head)
        dist = {"x": x, "y": y, "z": 0.75, "heading": home_heading}

        # Move command        
        for _ in range(30):
            self.drone.move(dist)
            rospy.sleep(0.1)

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 3] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 3] waiting for wp reached")

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
        while self.drone.lidar_data.ranges[0] > 1000:
            rospy.loginfo(f"[WP 4] Distance forward: {self.drone.lidar_data.ranges[0]}")
            rospy.sleep(0.1)
        
        rospy.loginfo(f"[WP 4] Out of loop. Distance forward: {self.drone.lidar_data.ranges[0]}")
        self.drone.stop()
        rospy.sleep(1)

        ## 8. Left/right 5m
        if right == 1: x,y = body2local(0, -5, head)
        else: x,y = body2local(0, 5, head)

        dist = {"x": x, "y": y, "z": 0, "heading": home_heading}

        # Check if waypoint is reached or not
        while not self.drone.check_waypoint_reached(dist):
            # self.drone.move(dist)
            rospy.loginfo_throttle(0.2,f"[WP 3] current pose : {self.drone.current_pose.pose.pose.position} ")
            rospy.loginfo_throttle(0.2,"[WP 3] waiting for wp reached")

        self.drone.stop()
        rospy.sleep(1)

        ## 9. Detect drop bucket

        ## 10. Drop payload
        self.drone.switch_relay(0, True)
        
        ## 11. Change source to GPS
        rospy.loginfo(f"Changing source to GPS")
        self.drone.set_ekf_source(1)

        ## 12. Set mission as AUTO
        rospy.loginfo(f"Setting mode as AUTO")
        self.drone.set_mode("AUTO")

    def move_on_z(self):
        self.drone.wait4start()
        rospy.loginfo(f"Current pose is {self.drone.current_pose.pose.pose.position}")
        
        # Takeoff
        check = self.drone.takeoff(1)
        if not check:
            rospy.logerr("takeoff failed")
            rospy.loginfo("trying again")
        else:
            rospy.loginfo("takeoff success")
        rospy.sleep(5)

        # Destination
        home_heading = self.drone.get_home_heading()
        head = radians(home_heading)
        rospy.logwarn(f"home heading IMU: {home_heading}")
        rospy.logwarn(f"home heading IMU radians: {head}")
        rospy.logwarn(f"home heading compass : {self.drone.compass}")

        rospy.logwarn(f"home heading compass : {self.drone.compass}")

        target_alt = 1.5
        err = target_alt - self.drone.rangefinder
        self.z_pid.reset()
        while self.drone.rangefinder < target_alt - 0.05 or self.drone.rangefinder > target_alt + 0.05:
            err = target_alt - self.drone.rangefinder
            velz = self.z_pid.update(err)
            self.drone.move_vel(0,0,velz)
            # rospy.sleep(0.02)
    
        rospy.loginfo("FINISHED")
        
    def test_heading(self):
        imu = []
        compass = []
        while True:
            a = int(self.drone.imu_heading)
            b = int(self.drone.compass)
            if a not in imu:
                imu.append(a)
            if b not in compass:
                compass.append(b)
            rospy.loginfo(f"IMU heading lsi: {imu}")
            rospy.loginfo(f"compass heading lis: {compass}")
            rospy.loginfo(f"IMU heading: {a}")
            rospy.loginfo(f"compass heading: {b}")
            rospy.sleep(0.2)
            

    def print_rangefinder(self):
        self.drone.wait4start()

        while True:
            rospy.loginfo(f"{self.drone.rangefinder}")

    def print_ultrasonic(self):
        self.drone.wait4start()

        while True:
            rospy.loginfo(f"{self.drone.ultrasonic}")
            
    def test_valid_move_vel(self):
        self.drone.wait4start()
        self.drone.arm()
        rospy.sleep(1)
        rospy.loginfo("MAJU")
        for _ in range(30):
            self.drone.move_vel(2,0,0)
            rospy.sleep(0.1)
            
        rospy.sleep(15)
        rospy.loginfo("STOPPINGGGG")
        for _ in range(30):
            self.drone.move_vel(0,0,0)
            rospy.sleep(0.1)
        
            
    def test_vel_cb(self):
        
        velx = 0
        last_velx = 0
        vely = 0
        last_vely = 0
        velz = 0
        last_velz = 0
        while True:
            velx = self.drone.current_velocity.twist.linear.x
            vely = self.drone.current_velocity.twist.linear.y
            velz = self.drone.current_velocity.twist.linear.z
            # if velx > 0.1:
            #     rospy.loginfo("maju")
            # elif velx < -0.1:
            #     rospy.loginfo("mundur")
            
            # if vely > 0.1:
            #     rospy.loginfo("kiri")
            # elif vely < -0.1:
            #     rospy.loginfo("kanan")
                
            if velz > 0.1:
                rospy.loginfo("naik")
            if velz  < - 0.1:
                rospy.loginfo("turun")
            
    def timer(self):
        start = rospy.Time.now().to_sec()
        x_done = False
        y_done = False

        while True:
            rospy.loginfo_throttle(0.2,"waiting for target")

            # If it took longer than 10 seconds, we'll do
            # the drop no matter what, believing in God that
            # He will move the object as we want to
            if rospy.Time.now().to_sec() - start > 5:
                rospy.loginfo_throttle(0.2,"out of loop")
                return False
        
            

if __name__ == "__main__":
    game = Game()
    try:
        # game.test_jalan_bawah()
        # game.test_26_agustus()
        # game.test_rc_override()
        # game.test_heading_using_compass()
        # game.test_valid_move_vel()
        game.timer()

        # game.test_move_interrupt()
        # result = game.test_color_following()
        # print("timeout" if not result else "finished")

        # game.test_servo()
        # game.test_move_global()
        # game.test_lidar()
        # game.test_ekf()
        # game.test_position()
        # game.test_relay()

        # game.program_1_17_aug()
        # game.program_2_17_aug()
        # game.program_3_17_aug()

        # 20 Aug 23
        # game.test_color_following()
        # game.test_move_naik_turun()
        # game.test_move_LOCAL_NED()

        # game.flight_test_23_sept(right=False)
        # game.print_ultrasonic()
        # game.print_rangefinder()
        
        while True:
            pass
    except KeyboardInterrupt:
        # game.drone.set_mode("LAND")
        exit()
    except rospy.ROSInterruptException:
        pass
