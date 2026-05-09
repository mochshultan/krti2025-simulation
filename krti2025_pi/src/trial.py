#!/usr/bin/env python

import rospy
from krti2023_pi.drone_api import DroneAPI
from krti2023_pi.msg import DResult
from sensor_msgs.msg import LaserScan
from krti2023_pi.srv import Activate, ActivateRequest, ActivateResponse
#from gazebo_link_attacher_ws.srv import Attach, AttachRequest, AttachResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import rospy
from geographic_msgs.msg import GeoPoint
from time import sleep
import math 
from pid import PID

def body2local(x, y, heading):
    return (x*math.cos(heading) - y*math.sin(heading), x*math.sin(heading) + y*math.cos(heading))


class Main:
    def __init__(self):
        rospy.init_node("main_node", log_level=rospy.DEBUG)

        self.sim = rospy.get_param("/vision/use_sim", False)
        self.collison_sub = rospy.Subscriber('/spur/laser/scan', LaserScan, self.lidar_avoidance_cb)
        self.drone = DroneAPI()
        self.target_data = DResult()
        self.target_data.is_found = False

        self.avoidance_vector_x = 0
        self.avoidance_vector_y = 0
        self.avoid = False

    def lidar_avoidance_cb(self, msg):
        self.current_2D_scan = msg
        self.avoidance_vector_x = 0
        self.avoidance_vector_y = 0
        self.avoid = False

        for i in range(1, len(self.current_2D_scan.ranges.size())):
            d0 = 3
            k = 0.5

            if self.current_2D_scan.ranges[i] < d0 and self.current_2D_scan.ranges[i] > 0.35:
                self.avoid = True
                x = math.cos(self.current_2D_scan.angle_increment * i)
                y = math.sin(self.current_2D_scan.angle_increment * i)
                U = -0.5 * k * ((1 / self.current_2D_scan.ranges[i]) - (1 / d0))**2

                self.avoidance_vector_x += x * U
                self.avoidance_vector_y += y * U

    def self_takeoff(self):
        self.drone.wait4start()

        rospy.loginfo(f"Changing source to GPS")
        self.drone.set_ekf_source(1)

        self.drone.arm()

        self.drone.takeoff(0.5)

if __name__ == "__main__":
    bermain_drone = Main()
    bermain_drone.self_takeoff()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if bermain_drone.avoid:
            rospy.loginfo(f"Avoidance vector: x={bermain_drone.avoidance_vector_x}, y={bermain_drone.avoidance_vector_y}")
        rate.sleep()