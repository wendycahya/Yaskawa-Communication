import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from cvzone.PoseModule import PoseDetector
import cv2
from datetime import datetime
import time
import math as mt
import random
import csv
start_time = datetime.now()
start = time.strftime("%Y%m%d-%H%M%S")
milliseconds = 0
write_file = "TR-"+str(start)+"-SSMNewDemo.csv"
d =0

#=== Calibration Value
real_measurement = 0
error = 0
chestDistance = 0
# Achest = 0.04629
# Bchest = -24.60386
# Cchest = 3870.58679
Achest = 0.04628882081739653
Bchest = -24.603862891449737
Cchest = 3870.586790291231


#Device connection
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(2)
detector = FaceMeshDetector(maxFaces=1)

detector = FaceMeshDetector(maxFaces=1)
detectorPose = PoseDetector()

with open(write_file, "wt", encoding="utf-8") as output:
    #Record data csv opening
    while True:
    # Detect human skeleton
        success, img = cap.read()
        imgMesh, faces = detector.findFaceMesh(img, draw=False)
        fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)
        img = detectorPose.findPose(img)
        lmList, bboxInfo = detectorPose.findPosition(img, bboxWithHands=False)
        if bboxInfo:
            idrSh, xrSh, yrSh, zrSh = lmList[11]
            idlSh, xlSh, ylSh, zlSh = lmList[12]
            # print("===== asli =====")
            # print("Id data ", idrSh, "right shoulder x=", xrSh, ", right shoulder y=", yrSh)
            # print("Id data ", idlSh, "left shoulder x=", xrSh, ", left shoulder y=", yrSh)
            chestDistance = round(mt.sqrt((xrSh - xlSh) ** 2 + (yrSh - ylSh) ** 2), 3)

        if faces:
    #skeleton detection
            face = faces[0]
            print(faces[0])
            pointLeft = face[145]
            pointRight = face[374]
            cv2.line(imgMesh, pointLeft, pointRight, (0, 200, 0), 3)
            cv2.circle(imgMesh, pointLeft, 5, (255, 0, 255), cv2.FILLED)
            cv2.circle(imgMesh, pointRight, 5, (255, 0, 255), cv2.FILLED)
            w, _ = detector.findDistance(pointLeft, pointRight)
            W = 6.3  # default real width eyes distance
            #Finding the focal length

            #d = 60  #distance human and camera
            #f = (w*d) / W
            #print(f)

            #finding distance
            f = 714 #finding the average for focal length
            d = ((W*f) / w) * 10
            print(d)
            d = round(d, 3)

            real_measurement = round((Achest * (chestDistance ** 2)) + (Bchest * chestDistance) + Cchest, 2)
            D = min(d, real_measurement)
            cvzone.putTextRect(img, f'Depth: {D} mm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)
            start_time = datetime.now()
            milliseconds = start_time.microsecond // 1000

        #output.write(str(start_time.strftime("%H:%M:%S")) + ',' + str(milliseconds) + ',' + str(d) + '\n')
        cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()