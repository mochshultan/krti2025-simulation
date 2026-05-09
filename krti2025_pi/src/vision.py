#!/usr/bin/env python3

import rospy

# ROS Image message
from sensor_msgs.msg import Image, CompressedImage

# OpenCV2 for saving an image
import cv2 as cv
import numpy as np

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

from krti2024_pi.srv import Activate, ActivateResponse
from krti2024_pi.msg import DResult

# get range from qr/target/elp
from nav_msgs.msg import Odometry
from math import tan, radians
import random as rng
from copy import deepcopy
#   LIST OF Service TOPIC
#   - vision/activate/target
#   - vision/verbose
#
#   LIST OF Publisher TOPIC
#   - vision/result
#
#   LIST OF Subscriber TOPIC
#   - down_facing_camera/image_raw
#

# TODO:
# -   add a service to request qr and target dx dy in m based on lidar data and fov
#        https://jamboard.google.com/d/1Iu5qJZLyZbIiGC8b8oDcwbF_GfKyBcttvh0o2xGcWHI/viewer?f=6


# Instantiate VERBOSE variable globally for verbose mode



class Vision:
    """
    vision.py
    This Node is responsible for detecting target and publishing the DResult to the '/vision/target/result' topic.
    to activate target detection we should activate
    using service under '/vision/activate/target' topic.
    the main.py is subscribe to this topic and will call the detect_target function
    """

    
    target = False
    front_img = np.array([None for _ in range(10)])
    down_img = np.array([None for _ in range(10)])
    alt= -99

    def __init__(self):
        # initialize node
        rospy.init_node("vision")
        # Instantiate CvBridge for converting ROS Image messages to OpenCV2
        self.bridge = CvBridge()
        self.which_target = 1
        self.last_time = rospy.Time.now()
        # get param from launchfile
        camera_index = rospy.get_param("/vision/camera_index")
        self.down_fov = {
            "x": rospy.get_param("/vision/down_fov_x"),
            "y": rospy.get_param("/vision/down_fov_y"),
        }

        self.target_lower_hsv = np.array(rospy.get_param("/vision/target_lower_hsv"))
        self.target_upper_hsv = np.array(rospy.get_param("/vision/target_upper_hsv"))
        
        self.target2_lower_hsv = np.array(rospy.get_param("/vision/target2_lower_hsv"))
        self.target2_upper_hsv = np.array(rospy.get_param("/vision/target2_upper_hsv"))
        
        self.target3_lower_hsv = np.array(rospy.get_param("/vision/target3_lower_hsv"))
        self.target3_upper_hsv = np.array(rospy.get_param("/vision/target3_upper_hsv"))
        
        self.target4_lower_hsv = np.array(rospy.get_param("/vision/target4_lower_hsv"))
        self.target4_upper_hsv = np.array(rospy.get_param("/vision/target4_upper_hsv"))

        self.sim = rospy.get_param("/vision/use_sim")
        self.sim_camera_topic = "/camera/down/image_raw"
        # setup VideoCapture 
        if not self.sim:
            self.down_cap = cv.VideoCapture(camera_index, cv.CAP_V4L2)
            if not self.down_cap.isOpened():
                rospy.logwarn(f"Camera at index {camera_index} failed, trying index {1-camera_index}")
                self.down_cap = cv.VideoCapture(1-camera_index, cv.CAP_V4L2)
                if not self.down_cap.isOpened():
                    rospy.logerr("Failed to open camera at both /dev/video0 and /dev/video1")
                    raise RuntimeError("Camera initialization failed")
            self.down_cap.set(cv.CAP_PROP_FRAME_WIDTH, 426)
            self.down_cap.set(cv.CAP_PROP_FRAME_HEIGHT, 240)
                
            rospy.Timer(rospy.Duration(0.05), self.read_camera)
            
        if self.sim:
            # subscribe to image_topic from sim
            self.img_sub = rospy.Subscriber(
                self.sim_camera_topic, Image, self.callback_down_img)

        self.pose_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.pose_cb)

        
        print("target lower hsv : {}".format(self.target_lower_hsv))
        print("target upper hsv : {}".format(self.target_upper_hsv))
        print("down fov : {}".format(self.down_fov))

        self.timestamp = rospy.Time.now()
        # to start subscribing to the image_topic and starting the QR code detection
        self.activate_target = rospy.Service(
            "vision/activate/target", Activate, self.activate_target
        )
        
        # PUBLISHER
        # create publisher for target detection result
        self.target_result_pub = rospy.Publisher(
            "/vision/target/result", DResult, queue_size=10
        )

        # publisher for processed image
        self.target_img_pub = rospy.Publisher(
            "/vision/target/image/compressed", CompressedImage, queue_size=10
        )

        self.down_pub = rospy.Publisher("/camera/down/image/compressed", CompressedImage, queue_size=10)
        self.down_raw_pub = rospy.Publisher("/camera/down/image_raw", Image, queue_size=10)
        # Debug publisher untuk raw HSV
        self.debug_hsv_pub = rospy.Publisher("/vision/debug/hsv/compressed", CompressedImage, queue_size=10)
        self.debug_mask_pub = rospy.Publisher("/vision/debug/mask/compressed", CompressedImage, queue_size=10)
        
        # Hough Circle Detection publisher
        self.hough_detection_pub = rospy.Publisher("/vision/hough_detection/image/compressed", CompressedImage, queue_size=10)
        self.hough_detection_raw_pub = rospy.Publisher("/vision/hough_detection/image_raw", Image, queue_size=10)


    def activate_target(self, data):
        """
        This function called when the service '/vision/activate/target' is called.
        It activates the target detection.
        """
        if data.data:            
            # if using real robot
            self.target = True
            self.which_target = data.target
            
            rospy.loginfo("Target detect activated")

        else:
            self.target = False
            rospy.loginfo("Target detect deactivated")
            # self.img_sub.unregister()
        
        return ActivateResponse(True)
    
    def read_camera(self, msg):
        ret_down, self.down_img = self.down_cap.read()
        if ret_down and self.down_img is not None:
            pass  # No flip/rotation applied
        else:
            rospy.logwarn_throttle(1.0, "Failed to read from down camera")
            return
        
        # NYALAKAN CAMERA TANPA COMVIS
        try:
            resized_img = cv.resize(self.down_img,(426,426))
            msg = self.bridge.cv2_to_compressed_imgmsg(resized_img,dst_format="jpg")
            self.down_pub.publish(msg)
            
            raw_msg = self.bridge.cv2_to_imgmsg(resized_img, "bgr8")
            self.down_raw_pub.publish(raw_msg)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Failed to publish camera images: {e}")
            rospy.logerr_throttle(1.0, f"Camera image publish error: {e}")

    def callback_down_img(self, msg):
        """
        This function is called when the down camera image_topic is published.
        It gets the image from the topic and convert from ROS Image msgs to OpenCV2 Image.
        """
        rospy.loginfo_throttle(0.1, "down_image_received")
        try:
            # Convert your ROS Image message to OpenCV2
            self.down_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # JIKA MENGGUNAKAN KAMERA KABEL PANJANG
            # self.down_img = cv.rotate(self.down_img, cv.ROTATE_180)

        except CvBridgeError as e:
            print(Warning("Down image conversion failed: {}".format(e)))

    def pose_cb(self, msg: Odometry):
        """
        Gets the raw pose of the drone and processes it for use in control.

        Args:
                msg (nav_msgs/Odometry): Raw pose of the drone.
        """
        # save current alt
        self.alt = msg.pose.pose.position.z
    
    def create_hsv_mask_hough(self, img, target_lower_hsv, target_upper_hsv, min_area_threshold=150):
        """
        Filter warna HSV dengan threshold area minimum untuk Hough detection
        """
        imgHSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(imgHSV, target_lower_hsv, target_upper_hsv)
        
        # Morphological operations untuk membersihkan noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
        
        # Area threshold filtering untuk reduce noise
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        filtered_mask = np.zeros_like(mask)
        
        for contour in contours:
            area = cv.contourArea(contour)
            if area >= min_area_threshold:
                cv.fillPoly(filtered_mask, [contour], 255)
        
        # Blur untuk deteksi lingkaran yang lebih smooth
        filtered_mask = cv.medianBlur(filtered_mask, 5)
        
        rospy.loginfo_throttle(1.0, f"Hough area threshold: {min_area_threshold} pixels - Filtered {len(contours)} contours")
        return filtered_mask

    def detect_circles_hough(self, mask, param1=200, param2=14, minDist=1000, minRadius=15, maxRadius=200):
        """
        Deteksi lingkaran dengan Hough Transform
        """
        circles = cv.HoughCircles(mask,
                                   cv.HOUGH_GRADIENT,
                                   dp=1,
                                   minDist=minDist,
                                   param1=param1,
                                   param2=param2,
                                   minRadius=minRadius,
                                   maxRadius=maxRadius)
        return circles
    
    def process_hough_detection(self, img, target_lower_hsv, target_upper_hsv):
        """Proses deteksi lingkaran dengan Hough Transform dan publish hasil"""
        h, w = img.shape[:2]
        center_x, center_y = w // 2, h * 5 // 8
        
        # Buat HSV mask dan deteksi lingkaran
        mask = self.create_hsv_mask_hough(img, target_lower_hsv, target_upper_hsv, min_area_threshold=150)
        circles = self.detect_circles_hough(mask)
        
        # Gambar hasil deteksi
        imgWithLines = img.copy()
        detected_circles = []
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            rospy.loginfo_throttle(0.5, f"Hough detected {len(circles[0])} circles")
            
            for circle in circles[0, :]:
                x, y, r = circle[0], circle[1], circle[2]
                distance = np.sqrt((x - center_x)**2 + (y - center_y)**2)
                
                # Simpan data lingkaran
                detected_circles.append({
                    'x': x, 'y': y, 'radius': r, 'distance': distance
                })
                
                # Gambar lingkaran dan garis
                cv.circle(imgWithLines, (x, y), r, (0, 255, 0), 2)
                cv.circle(imgWithLines, (x, y), 2, (255, 0, 0), 2)
                cv.line(imgWithLines, (center_x, center_y), (x, y), (255, 255, 0), 2)
                
                rospy.loginfo_throttle(1.0, f"Circle: center=({x}, {y}), radius={r}, distance={distance:.1f}")
        
        # Gambar titik tengah foto
        cv.circle(imgWithLines, (center_x, center_y), 5, (255, 0, 255), -1)
        
        # Publish gambar dengan deteksi Hough
        try:
            # Compressed version
            img_msg = self.bridge.cv2_to_compressed_imgmsg(cv.resize(imgWithLines, (120, 120)), dst_format="jpg")
            self.hough_detection_pub.publish(img_msg)
            
            # Raw version untuk rqt
            raw_msg = self.bridge.cv2_to_imgmsg(imgWithLines, "bgr8")
            self.hough_detection_raw_pub.publish(raw_msg)
        except Exception as e:
            rospy.logerr(f"Hough image publish error: {e}")
        
        return detected_circles

    def detect_target(self):
        """
        This function is called when the target detection is activated.
        this function called from the main
        It detects the target from the image and publishes the result.
        It will publish to the /vision/target/result topic
        with msg type DResult
        """
        thres = 400
        try:
            img = self.down_img
            if img is None:
                rospy.logwarn_throttle(1.0, "No image available for detection")
                return
            img_copy = deepcopy(img)
            Fwidth = img.shape[1]
            Fheight = img.shape[0]
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Image processing error: {e}")
            return
    #     pass
    # else:
        FWcenter = Fwidth // 2
        FHcenter = Fheight * 5 // 8
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # Gunakan Hough detection untuk semua target
        target_hsv_map = {
            1: (self.target_lower_hsv, self.target_upper_hsv),
            2: (self.target2_lower_hsv, self.target2_upper_hsv),
            3: (self.target3_lower_hsv, self.target3_upper_hsv),
            4: (self.target4_lower_hsv, self.target4_upper_hsv),
            # 5: (self.target5_lower_hsv, self.target5_upper_hsv),
            # 6: (self.target6_lower_hsv, self.target6_upper_hsv),
            # 7: (self.target7_lower_hsv, self.target7_upper_hsv),
        }
        
        if self.which_target in target_hsv_map:
            lower_hsv, upper_hsv = target_hsv_map[self.which_target]
            rospy.loginfo_throttle(1.0, f"Using Hough Circle Detection for Target-{self.which_target}")
            detected_circles = self.process_hough_detection(img, lower_hsv, upper_hsv)
            
            if detected_circles:
                # Ambil lingkaran terdekat dengan center
                closest_circle = min(detected_circles, key=lambda c: c['distance'])
                
                # Hitung dx, dy dari center frame
                dx = closest_circle['x'] - FWcenter
                dy = (closest_circle['y'] - FHcenter) * -1
                
                # Adjustment untuk target tertentu
                # if self.which_target == 1:
                #     dy -= 20
                if self.which_target == 5:
                    dx -= 120
                
                # Konversi ke meter
                x_m, y_m = self.calculate_meter_from_pixel(dx, dy, Fwidth, Fheight)
                
                # Publish hasil - DResult
                self.target_result_pub.publish(DResult(True, dx, dy, x_m, y_m))
                
                # Gambar hasil pada img_copy
                cv.circle(img_copy, (closest_circle['x'], closest_circle['y']), closest_circle['radius'], (0, 255, 0), 2)
                cv.circle(img_copy, (closest_circle['x'], closest_circle['y']), 2, (255, 0, 0), 2)
                cv.line(img_copy, (FWcenter, FHcenter), (closest_circle['x'], closest_circle['y']), (255, 255, 0), 2)
                cv.circle(img_copy, (FWcenter, FHcenter), 5, (255, 0, 255), -1)
                
                cv.putText(img_copy, f"dx:{dx}", (closest_circle['x'] + 10, closest_circle['y']), cv.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1)
                cv.putText(img_copy, f"dy:{dy}", (closest_circle['x'] + 10, closest_circle['y'] + 15), cv.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1)
                cv.putText(img_copy, f"d:{closest_circle['distance']:.1f}", (closest_circle['x'] + 10, closest_circle['y'] + 30), cv.FONT_HERSHEY_PLAIN, 1, (0, 255, 255), 1)
                
                fps = 1 / (rospy.Time.now() - self.last_time).to_sec()
                self.last_time = rospy.Time.now()
                cv.putText(img_copy, f"{round(fps,2)}", (0, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
                
                msg = self.bridge.cv2_to_compressed_imgmsg(cv.resize(img_copy, (120, 120)), dst_format="jpg")
                self.target_img_pub.publish(msg)
                return
            else:
                # Tidak ada lingkaran terdeteksi
                self.target_result_pub.publish(DResult(False, 0, 0, 0, 0))
                cv.putText(img_copy, "NO CIRCLES", (10, 100), cv.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 1)
                fps = 1 / (rospy.Time.now() - self.last_time).to_sec()
                self.last_time = rospy.Time.now()
                cv.putText(img_copy, f"{round(fps,2)}", (0, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
                msg = self.bridge.cv2_to_compressed_imgmsg(cv.resize(img_copy, (120, 120)), dst_format="jpg")
                self.target_img_pub.publish(msg)
                return
        
        # Fallback ke deteksi lama jika target tidak ada di map
        mask = cv.inRange(hsv, self.target_lower_hsv, self.target_upper_hsv)

            
        # DEBUG: Publish HSV dan mask untuk debugging
        try:
            hsv_msg = self.bridge.cv2_to_compressed_imgmsg(cv.resize(hsv, (120, 120)), dst_format="jpg")
            self.debug_hsv_pub.publish(hsv_msg)
            
            mask_colored = cv.applyColorMap(mask, cv.COLORMAP_JET)
            mask_msg = self.bridge.cv2_to_compressed_imgmsg(cv.resize(mask_colored, (120, 120)), dst_format="jpg")
            self.debug_mask_pub.publish(mask_msg)
        except:
            pass
            
        # FILTER
        # morph size for the filter
        MORPH_SIZE = 3
        # create kernel for filter
        element = cv.getStructuringElement(
            cv.MORPH_RECT, (2 * MORPH_SIZE, 2 * MORPH_SIZE), (MORPH_SIZE, MORPH_SIZE)
        )

        # morphological transformation:
        # https://www.youtube.com/watch?v=xSzsD4kXhRw
        # apply filter morphology opening to the image
        # erode and dilate to remove noise
        mask_opening = cv.morphologyEx(mask, cv.MORPH_OPEN, element, iterations=1)
        # apply filter morphology closing to the image
        # dilate and erode to fill holes
        mask_closing = cv.morphologyEx(
            mask_opening, cv.MORPH_CLOSE, element, iterations=2
        )

        # find contours in the masked and filtered image
        contours, hierarchy = cv.findContours(
            mask_closing, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE
        )
        rospy.loginfo_throttle(0.5, f"Contours found: {len(contours)}, Mask pixels: {cv.countNonZero(mask_closing)}")

        # isolate object from background

        res = cv.bitwise_and(img, img, mask=mask_closing)

        # find the biggest contour
        dtype = [("area",float),("boundrect",tuple),("dx",int),("dy",int)]
        data=[]
        # data = [(0,0,0,0)]
        # print("contours", contours)
        if len(contours) == 0:
            rospy.loginfo_throttle(2.0, "NO TARGET DETECTED - Check HSV values and lighting conditions")
            cv.putText(
                    img_copy,
                    "NO TARGET",
                    (10, 100),
                    cv.FONT_HERSHEY_DUPLEX,
                    3,
                    (255, 255, 255),
                    1,
                )
            cv.putText(
                    img_copy,
                    "DETECTED",
                    (10, 200),
                    cv.FONT_HERSHEY_DUPLEX,
                    3,
                    (255, 255, 255),
                    1,
                )
            fps = 1/ ( rospy.Time.now()-self.last_time).to_sec()
            self.last_time = rospy.Time.now()
            cv.putText(img_copy, f"{round(fps,2)}", (0, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
            msg = self.bridge.cv2_to_compressed_imgmsg(cv.resize(img_copy,(120,120)),dst_format="jpg")
            self.target_img_pub.publish(msg)
            return
        for i, contour in enumerate(contours):
            # calculate area of the contours
            area = cv.contourArea(contour)
            rect = cv.boundingRect(contour)
            x, y, w, h = rect
            dx = int(w / 2 + x - FWcenter)
            dy = int((h / 2 + y - FHcenter)* -1)
            #WHEN BELOK KIRI
            if self.which_target == 1:
                dy -= 20
            # if self.which_target == 5: 
                # dx += 120
            #WHEN BELOK KANAN
            elif self.which_target == 5: 
                dx -= 120
            data.append((area,
                            rect,
                            dx,
                            dy))
        
        data = np.array(data, dtype=dtype)
        data = np.sort(data, order="area")
        remove = np.where(data["area"] < thres)
        # print(remove)
        data = np.delete(data, remove)
        # print("data : ", data)
        # rospy.loginfo_throttle(0.2,f"")
        if len(data) == 0:
            self.target_result_pub.publish(DResult(False, 0, 0, 0, 0))
            cv.putText(
                img_copy,
                "NO TARGET",
                (10, 100),
                cv.FONT_HERSHEY_PLAIN,
                3,
                (0, 0, 255),
                1,
            )
            fps = 1/(rospy.Time.now()-self.last_time ).to_sec()
            self.last_time = rospy.Time.now()
            cv.putText(img_copy, f"{round(fps,2)}", (   0, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
            msg = self.bridge.cv2_to_compressed_imgmsg(cv.resize(img_copy,(120,120)),dst_format="jpg")
            self.target_img_pub.publish(msg)
            return
        
        if np.max(data["area"]) < thres :
            # if no target is detected, publish false
            self.target_result_pub.publish(DResult(False, 0, 0, 0, 0))
            cv.putText(
                img_copy,
                "NO TARGET",
                (10, 100),
                cv.FONT_HERSHEY_PLAIN,
                3,
                (0, 0, 255),
                1,
            )
            fps = 1/(rospy.Time.now()-self.last_time ).to_sec()
            self.last_time = rospy.Time.now()
            cv.putText(img_copy, f"{round(fps,2)}", (   0, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1)
            msg = self.bridge.cv2_to_compressed_imgmsg(cv.resize(img_copy,(120,120)),dst_format="jpg")
            self.target_img_pub.publish(msg)
            return

        avgx = np.mean(data["dx"])
        stdx = np.std(data["dx"])
        avgy = np.mean(data["dy"])
        stdy = np.std(data["dy"])
        
        if len(data) > 1 or stdx != 0 or stdy != 0:
            thr = 1.2 
            validx = []
            validy = []
            print("avgx\t: ", avgx)
            print("stdx\t: ", stdx)
            for i in range(len(data)):
                z_scorex = (data["dx"][i]-avgx)/stdx
                z_scorey = (data["dy"][i]-avgy)/stdy

                if abs(z_scorex) < thr:
                    validx.append(data["dx"][i])
                if abs(z_scorey) < thr:
                    validy.append(data["dy"][i])

            # calculate the difference in meters, currently not working as expected
            if len(validx) == 0 or len(validy) == 0:
                dx = int(np.mean(data["dx"]))
                dy = int(np.mean(data["dy"]))
            else:
                dx = int(np.mean(validx))
                dy = int(np.mean(validy))
                print(f"dx\t: {validx}, \tdy\t: {validy}")
                print(f"dx\t: {dx}, \tdy\t: {dy}")
        else:
            dx = int(np.mean(data["dx"]))
            dy = int(np.mean(data["dy"]))
            print(f"dx\t: {dx}, \tdy\t: {dy}")
        x_m, y_m = self.calculate_meter_from_pixel(dx, dy, Fwidth, Fheight)

        rospy.logdebug_throttle(0.2, f"dx\t: {dx}, \tdy\t:, {dy}, \tx_m\t:, {x_m}, \ty_m\t:, {y_m}")
        self.target_result_pub.publish(DResult(True, dx, dy, x_m, y_m))

        color = (
            rng.randint(0, 256),
            rng.randint(0, 256),
            rng.randint(0, 256),
        )

        for i in range(len(contours)):
            cv.drawContours(img_copy, contours, i, (0, 0, 255), 2)
    
        dx = int(dx)
        dy = int(dy)

        cv.line(
            img_copy,
            (FWcenter, FHcenter),
            (FWcenter + dx, FHcenter - dy),
            color,
            3
        )

        cv.circle(img_copy, (FWcenter + dx, FHcenter), 3, (0, 255, 255), -1)
        cv.circle(img_copy, (FWcenter, FWcenter + dy), 3, (0, 255, 255), -1)
        cv.putText(
            img_copy,
            "dx:" + str(dx),
            (FWcenter + dx // 2, FHcenter + 10),
            cv.FONT_HERSHEY_PLAIN,
            1,
            (0, 100, 255),
            1
        )
        cv.putText(
            img_copy,
            "dy:" + str(dy),
            (FWcenter - 10, FHcenter + dy // 2),
            cv.FONT_HERSHEY_PLAIN,
            1,
            (0, 100, 255),
            1
        )
        # # draw horizontal line
        cv.line(
            img_copy,
            (0, FHcenter),
            (int(FWcenter), FHcenter),
            (0, 0, 255),
            2
        )
        cv.line(
            img_copy,
            (int(FWcenter), FHcenter),
            (Fwidth, FHcenter),
            (0, 255, 0),
            2
        )
        cv.line(
            img_copy,
            (FWcenter, 0),
            (FWcenter, FHcenter),
            (0, 0, 255),
            2
        )
        cv.line(
            img_copy,
            (FWcenter, FHcenter),
            (FWcenter, Fheight),
            (0, 255, 0),
            2
        )
        fps = 1/ (rospy.Time.now()-self.last_time).to_sec()
        self.last_time = rospy.Time.now()
        cv.putText(img_copy, f"{round(fps,2)}", (0, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
        try:
            # Convert opencv2 img to ros Image
            msg = self.bridge.cv2_to_compressed_imgmsg(cv.resize(img_copy,(120,120)),dst_format="jpg")
        except CvBridgeError as e:
            print(Warning("Conversion failed: {}".format(e)))
        self.target_img_pub.publish(msg)
    
    def calculate_meter_from_pixel(self, dx, dy, Fw, Fh):
        """
        this function is used to calculate the position in meter we should move
        based on error in pixel, lidar, and the fov of the down camera
        https://jamboard.google.com/d/1lls6bwxasvXhjlHUzlAPdn7H457EWCQQhZ9MEsxx3u0
        """
        
        if self.alt == -99:
            return 0, 0
        Rx = tan(radians(self.down_fov["x"] / 2)) * self.alt
        Ry = tan(radians(self.down_fov["y"] / 2)) * self.alt

        x = dx * Rx / (Fw / 2)
        y = dy * Ry / (Fh / 2)
        return float(x), float(y)
    
    def main(self):
        last = rospy.Time.now()
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(1,"Vision Node Heartbeat")
            if self.target:
                rospy.loginfo_once(f"[Vision] Target-{self.which_target} Activate")
                self.detect_target()
            r.sleep()


if __name__ == "__main__":
    try:
        vision = Vision()
        vision.main()
    except rospy.ROSInterruptException:
        pass
