#!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraDebug:
    def __init__(self):
        rospy.init_node("camera_debug")
        self.bridge = CvBridge()
        
        # Subscribe ke topic kamera simulasi
        self.img_sub = rospy.Subscriber("/camera/down/image_raw", Image, self.callback_img)
        
        # Parameter HSV dari launch file
        self.target_lower_hsv = np.array([0, 200, 200])
        self.target_upper_hsv = np.array([20, 255, 255])
        
        self.img = None
        
    def callback_img(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            print(f"Error: {e}")
    
    def main(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.img is not None:
                # Convert ke HSV
                hsv = cv.cvtColor(self.img, cv.COLOR_BGR2HSV)
                
                # Buat mask
                mask = cv.inRange(hsv, self.target_lower_hsv, self.target_upper_hsv)
                
                # Hasil deteksi
                result = cv.bitwise_and(self.img, self.img, mask=mask)
                
                # Tampilkan info HSV di tengah gambar
                h, w = self.img.shape[:2]
                center_hsv = hsv[h//2, w//2]
                
                # Tampilkan gambar
                cv.putText(self.img, f"HSV: {center_hsv}", (10, 30), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv.putText(self.img, f"Target: {self.target_lower_hsv}-{self.target_upper_hsv}", 
                          (10, 60), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                # Hitung area yang terdeteksi
                detected_pixels = cv.countNonZero(mask)
                total_pixels = mask.shape[0] * mask.shape[1]
                percentage = (detected_pixels / total_pixels) * 100
                
                cv.putText(self.img, f"Detected: {percentage:.1f}%", (10, 90), 
                          cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Tampilkan semua window
                cv.imshow("Original", self.img)
                cv.imshow("HSV", hsv)
                cv.imshow("Mask", mask)
                cv.imshow("Result", result)
                
                key = cv.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                    
            rate.sleep()
        
        cv.destroyAllWindows()

if __name__ == "__main__":
    try:
        debug = CameraDebug()
        debug.main()
    except rospy.ROSInterruptException:
        pass