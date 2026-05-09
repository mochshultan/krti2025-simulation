#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from krti2023_pi.srv import Activate, ActivateResponse
from krti2023_pi.msg import DResult

import cv2 as cv
import numpy as np
from math import tan, radians
from copy import deepcopy


class Vision:

    target = False
    down_img = None
    alt = -99

    def __init__(self):

        rospy.init_node("vision")

        self.bridge = CvBridge()
        self.which_target = 1
        self.last_time = rospy.Time.now()

        # ===== PARAMETERS =====
        self.down_fov = {
            "x": rospy.get_param("/vision/down_fov_x"),
            "y": rospy.get_param("/vision/down_fov_y"),
        }

        self.target_lower_hsv = np.array(rospy.get_param("/vision/target_lower_hsv"))
        self.target_upper_hsv = np.array(rospy.get_param("/vision/target_upper_hsv"))
        
        self.target2_lower_hsv = np.array(rospy.get_param("/vision/target2_lower_hsv"))
        self.target2_upper_hsv = np.array(rospy.get_param("/vision/target2_upper_hsv"))
        
        self.sim = rospy.get_param("/vision/use_sim")
        self.sim_camera_topic = "/camera/down/image_raw"
        
        rospy.loginfo("=== VISION NODE STARTED ===")
        
        # Setup camera based on sim parameter
        if not self.sim:
            rospy.logerr("WEBCAM MODE NOT IMPLEMENTED")
            raise RuntimeError("Webcam not supported, set use_sim: true")
            
        if self.sim:
            rospy.loginfo("CAMERA SOURCE: GAZEBO /camera/down/image_raw")
            # ===== SUBSCRIBE GAZEBO CAMERA =====
            self.img_sub = rospy.Subscriber(
                self.sim_camera_topic,
                Image,
                self.callback_img
            )

        # ===== SUBSCRIBE ALTITUDE =====
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/odom",
            Odometry,
            self.pose_cb
        )

        # ===== SERVICE =====
        self.activate_srv = rospy.Service(
            "vision/activate/target",
            Activate,
            self.activate_target
        )

        # ===== PUBLISHERS =====
        self.target_result_pub = rospy.Publisher(
            "/vision/target/result",
            DResult,
            queue_size=10
        )

        self.target_img_pub = rospy.Publisher(
            "/vision/target/image",
            Image,
            queue_size=10
        )
        
        rospy.loginfo(f"Target HSV: {self.target_lower_hsv} - {self.target_upper_hsv}")
        rospy.loginfo(f"Down FOV: {self.down_fov}")

    # =========================================
    # CALLBACK IMAGE
    # =========================================
    def callback_img(self, msg):
        rospy.loginfo_throttle(1.0, "Receiving image from Gazebo")
        try:
            self.down_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    # =========================================
    # CALLBACK POSE
    # =========================================
    def pose_cb(self, msg):
        self.alt = msg.pose.pose.position.z

    # =========================================
    # SERVICE ACTIVATE
    # =========================================
    def activate_target(self, req):
        self.target = req.data
        self.which_target = req.target
        rospy.loginfo(f"Target detection: {self.target}, Target: {self.which_target}")
        return ActivateResponse(True)

    # =========================================
    # DETECTION
    # =========================================
    def detect_target(self):

        if self.down_img is None:
            rospy.logwarn_throttle(2.0, "No image received yet")
            return

        img = self.down_img
        img_copy = deepcopy(img)

        Fwidth = img.shape[1]
        Fheight = img.shape[0]

        FWcenter = Fwidth // 2
        FHcenter = Fheight // 2

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # Select HSV based on target
        if self.which_target == 1:
            mask = cv.inRange(hsv, self.target_lower_hsv, self.target_upper_hsv)
        elif self.which_target == 2:
            mask = cv.inRange(hsv, self.target2_lower_hsv, self.target2_upper_hsv)
        else:
            mask = cv.inRange(hsv, self.target_lower_hsv, self.target_upper_hsv)

        # Morphological operations
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv.findContours(
            mask,
            cv.RETR_TREE,
            cv.CHAIN_APPROX_SIMPLE
        )

        if len(contours) == 0:
            self.target_result_pub.publish(DResult(False, 0, 0, 0, 0))
            cv.putText(img_copy, "NO TARGET", (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            msg = self.bridge.cv2_to_imgmsg(img_copy, "bgr8")
            self.target_img_pub.publish(msg)
            return

        largest = max(contours, key=cv.contourArea)

        if cv.contourArea(largest) < 100:
            self.target_result_pub.publish(DResult(False, 0, 0, 0, 0))
            cv.putText(img_copy, "TARGET TOO SMALL", (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            msg = self.bridge.cv2_to_imgmsg(img_copy, "bgr8")
            self.target_img_pub.publish(msg)
            return

        x, y, w, h = cv.boundingRect(largest)

        dx = int(w / 2 + x - FWcenter)
        dy = int(h / 2 + y - FHcenter)

        x_m, y_m = self.calculate_meter_from_pixel(dx, dy, Fwidth, Fheight)

        self.target_result_pub.publish(
            DResult(True, dx, dy, x_m, y_m)
        )

        # Draw visualization
        cv.rectangle(img_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv.line(img_copy, (FWcenter, FHcenter),
                (FWcenter + dx, FHcenter + dy), (0, 0, 255), 2)
        cv.circle(img_copy, (FWcenter, FHcenter), 5, (255, 0, 0), -1)
        cv.putText(img_copy, f"dx:{dx} dy:{dy}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        msg = self.bridge.cv2_to_imgmsg(img_copy, "bgr8")
        self.target_img_pub.publish(msg)

    # =========================================
    # PIXEL → METER
    # =========================================
    def calculate_meter_from_pixel(self, dx, dy, Fw, Fh):

        if self.alt == -99:
            return 0.0, 0.0

        Rx = tan(radians(self.down_fov["x"] / 2)) * self.alt
        Ry = tan(radians(self.down_fov["y"] / 2)) * self.alt

        x = dx * Rx / (Fw / 2)
        y = dy * Ry / (Fh / 2)

        return float(x), float(y)

    # =========================================
    # MAIN LOOP
    # =========================================
    def main(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(5, "Vision node running - waiting for target activation")
            if self.target:
                self.detect_target()
            r.sleep()


if __name__ == "__main__":
    try:
        vision = Vision()
        vision.main()
    except rospy.ROSInterruptException:
        pass
