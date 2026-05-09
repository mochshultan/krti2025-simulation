import cv2 as cv
import numpy as np

img = np.zeros((512, 512, 3), np.uint8)
cv.circle(img, (50, 50), 10, (255, 0, 0), -1)  # blue
cv.circle(img, (512 - 50, 50), 10, (255, 255, 0), -1)  # yellow
cv.circle(img, (50, 512 - 50), 10, (255, 0, 255), -1)  # orange
cv.circle(img, (512 - 50, 512 - 50), 10, (255, 255, 255), -1)  # white
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
circles = cv.HoughCircles(
    gray, cv.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0
)
print(circles)
cv.imshow("img", img)
cv.waitKey(5000)
