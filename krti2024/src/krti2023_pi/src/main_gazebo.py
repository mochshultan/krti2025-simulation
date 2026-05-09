#!/usr/bin/env python3

#Dengan Menyebut Nama Allah Yang Maha Pengasih dan Yang Maha Penyayang

from krti2023_pi.drone_api import DroneAPI
import rospy
from math import radians

class Game:
    def __init__(self):
        rospy.init_node("main_node", log_level=rospy.DEBUG)
        
        waypoints = []
        self.drone = DroneAPI(waypoints=waypoints, sim=True)

    def coba(self):
        self.drone.wait4start()

        # Set mode OFFBOARD dan arm
        rospy.loginfo("Setting OFFBOARD mode")
        self.drone.set_mode("OFFBOARD")
        rospy.sleep(1)
        
        rospy.loginfo("Arming drone")
        self.drone.arm()
        rospy.sleep(2)
        
        # Takeoff
        rospy.loginfo("Starting takeoff")
        for _ in range(50):
            self.drone.move_vel(0, 0, 0.3)
            rospy.sleep(0.1)
        
        rospy.loginfo("Takeoff complete")
        rospy.sleep(2)

        rospy.loginfo("drone akan MAJU")
        for _ in range(100):
            rospy.sleep(0.01)
            self.drone.move_vel(-0.6, 0, 0)
        rospy.sleep(3)

        rospy.loginfo("drone akan Naik")
        for _ in range(30):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, 0.2)
        rospy.sleep(3)

        rospy.loginfo("drone akan MAJU lagi")
        for _ in range(100):
            rospy.sleep(0.01)
            self.drone.move_vel(-0.9, 0, 0)
        rospy.sleep(3)

        rospy.loginfo("drone akan MENYAMPING")
        for _ in range(100):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 1.2, 0)
        rospy.sleep(3)

        rospy.loginfo("drone akan TURUN")
        for _ in range(30):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, -0.07)
        rospy.sleep(3)
        
        rospy.loginfo("STOP")
        for _ in range(30):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, 0)
        rospy.sleep(3)

        rospy.loginfo("drone akan NAIK")
        for _ in range(100):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, 0.25)
        rospy.sleep(3)

        rospy.loginfo("drone akan menuju EXIT GATE")
        for _ in range(100):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 1.5, 0)
        rospy.sleep(3)

        for _ in range(100):
            rospy.sleep(0.01)
            self.drone.move_vel(0, 0, 0)
        rospy.sleep(3)

        rospy.loginfo("SELESAI - LAND")
        self.drone.set_mode("LAND")

if __name__ == "__main__":
    game = Game()
    try:
        game.coba()
    except KeyboardInterrupt:
        game.drone.set_mode("LAND")
        exit()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.logdebug("exit")
