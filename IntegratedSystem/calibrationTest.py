from camcalib import *
import cv2

calib = Calibrate("config.json", 480, 640)

cap = cv2.VideoCapture(0)

while(True):
    _ ,frame = cap.read()
    undistorded = calib.fix(frame)
    cv2.imshow("Undistorted", undistorded)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()