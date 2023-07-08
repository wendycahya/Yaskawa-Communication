import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
import cv2
from datetime import datetime
import time
import random
import csv
start_time = datetime.now()
start = time.strftime("%Y%m%d-%H%M%S")
milliseconds = 0
write_file = "TR-"+str(start)+"-SSMNewDemo.csv"
d =0

#Device connection
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(2)
detector = FaceMeshDetector(maxFaces=1)

with open(write_file, "wt", encoding="utf-8") as output:
    #Record data csv opening
    while True:
    # Detect human skeleton
        success, img = cap.read()
        imgMesh, faces = detector.findFaceMesh(img, draw=False)
        fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)

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
            cvzone.putTextRect(img, f'Depth: {d} mm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)
            start_time = datetime.now()
            milliseconds = start_time.microsecond // 1000

        #output.write(str(start_time.strftime("%H:%M:%S")) + ',' + str(milliseconds) + ',' + str(d) + '\n')
        cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()