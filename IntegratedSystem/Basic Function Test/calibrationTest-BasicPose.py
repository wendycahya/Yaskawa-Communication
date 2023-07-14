import math

from cvzone.PoseModule import PoseDetector
import cv2

cap = cv2.VideoCapture(2)
detector = PoseDetector()
while True:
    success, img = cap.read()
    img = detector.findPose(img)
    lmList, bboxInfo = detector.findPosition(img, bboxWithHands=False)
    if bboxInfo:
        idrSh, xrSh, yrSh, zrSh = lmList[11]
        idlSh, xlSh, ylSh, zlSh = lmList[12]
        print("===== asli =====")
        print("Id data ", idrSh, "right shoulder x=", xrSh, ", right shoulder y=", yrSh)
        print("Id data ", idlSh, "left shoulder x=", xrSh, ", left shoulder y=", yrSh)
        distance = round(math.sqrt((xrSh - xlSh)**2 + (yrSh - ylSh)**2), 3)
        print("==========")
        print("lebar pixel ", distance)


    cv2.imshow("Image", img)
    cv2.waitKey(1)