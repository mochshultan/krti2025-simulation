import cv2 as cv



if __name__ == "__main__":
    cap = cv.VideoCapture(0)
    try:
        while True:
            _,img = cap.read()
            cv.imshow("wildan asu", img)
            key = cv.waitKey(1)
            if key == ord("x") & 0xFF:
                break
    except KeyboardInterrupt:
        cap.release()