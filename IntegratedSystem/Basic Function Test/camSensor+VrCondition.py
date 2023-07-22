import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from cvzone.PoseModule import PoseDetector
import cv2
from datetime import datetime
import time as t
import math as mt
import random
import csv
start_time = datetime.now()
start = t.strftime("%Y%m%d-%H%M%S")
milliseconds = 0
write_file = "TR-"+str(start)+"-SSMNewDemo.csv"
d =0

def SpMax(Vr_Max, Vh_Max, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr_Max*Ts + ((ac*pow(Ts, 2))/2)
    SpMax   = Vh_Max * (Tr + Ts) + (Vr_Max * Tr) + Ss + Ctot
    return SpMax

def Spmin(C, Zd, Zr):
    SpminVal = C + Zd + Zr
    return SpminVal

def SpPFL(Vr_PFL, Vh, Tr, Ts, ac, C, Zd, Zr):
    global SpPFLVal
    Ctot = C + Zd + Zr
    Ss   = Vr_PFL*Ts + ((ac*pow(Ts,2))/2)
    SpPFLVal   = Vh * ( Tr + Ts ) + (Vr_PFL * Tr) + Ss + Ctot
    return SpPFLVal

def SpSafe(Vr_PFL, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr_PFL*Ts + ((ac*pow(Ts, 2))/2)
    SpSafeVal = Ss + Ctot
    return SpSafeVal

def Vr_SSM(D, Vh, Tr, Ts, ac, C, Zd, Zr, Vr_PFL):
    T = Tr + Ts
    Ctot = C + Zd + Zr
    VrSSM = (((D - (Vh*T) - Ctot)) / T) - ((ac*pow(Ts, 2))/(2*T))
    if VrSSM < Vr_PFL:
        Reduce_Value = Vr_PFL
    else:
        Reduce_Value = VrSSM
    return Reduce_Value

def Vr_SSM2(D, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    VrSSM2 = (D / Ts) - ((ac*Ts)/2) - (Ctot/Ts)
    if VrSSM2 > 0:
        Stop_Value = VrSSM2
    else:
        Stop_Value = 0
    return Stop_Value

# ===== initialization & variables declaration =====
#SSM variables
D = 0
VrPaper = 1000
Vr = 1500
Vr_PFL = 400
Vh_max = 1600
Vh_min = 0
Tr = 0.1
Ts = 0.08
ac = 3000
C_SSM = 200
Zd = 106.7
Zr = 1

#velocity human
Vh = 1600
XnRob = [0, 0, 0]
XnRobEnd = [0, 0, 0]
XnRob_last = [0, 0, 0]
velXR, velYR, velZR = [0, 0, 0]
interval = 0
VrSSM = 0
VrSSM2 = 0

#Robot Velocity
vrchest = 400
vrface = 100
vrstop = 0
vrmax = 1500
vrot = 90
velRob = 0
robotZ = 0
vel = 0
RobotVrmax = 1500

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

# ===== SSM calculation ======
SpPFLVal = SpPFL(Vr_PFL, Vh, Tr, Ts, ac, C_SSM, Zd, Zr)
SpSafeVal = SpSafe(Vr_PFL, Ts, ac, C_SSM, Zd, Zr)
Spfull = SpMax(vrmax, Vr, Tr, Ts, ac, C_SSM, Zd, Zr)
SpminVal = Spmin(C_SSM, Zd, Zr)

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
            data = []  # List to store the input data
            while len(data) < 10:
                face = faces[0]
                #print(faces[0])
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
                #print(d)
                d = round(d, 3)

                real_measurement = round((Achest * (chestDistance ** 2)) + (Bchest * chestDistance) + Cchest, 2)


                # Collect 10 input values

                # print("2. Human Distance ", disHR)
                # print("===========================================")

                # real_measurement = round((Achest * (chestDistance ** 2)) + (Bchest * chestDistance) + Cchest, 2)
                # # print("==========")
                # # print("lebar pixel ", chestDistance)
                # print("Real Chest Distance", real_measurement)

                #D = eye_dist
                D = min(d, real_measurement)
                print("Jarak dari Loop", D)
                data.append(D)

            # Calculate the average
            D = sum(data) / len(data)
            D = round(D, 3)

            start_time = datetime.now()
            milliseconds = start_time.microsecond // 1000



            if D < 0:
                D = 0
            else:
                D = abs(D)
            cvzone.putTextRect(img, f'Depth: {D} mm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)

            # logical SSM send robot
            if D <= SpminVal:
                print("Robot harus berhenti", vrstop)
                mode_collab = 0
                Vr = 1
                # jacoRobot.setSpeed(Vr, vrot)
                # t.sleep(0.5)

            elif D > SpminVal and D <= SpSafeVal:
                print("Robot working on collaboration mode")
                mode_collab = 1
                Vr = Vr_SSM2(D, Tr, Ts, ac, C_SSM, Zd, Zr)
                Vr = round(Vr, 2)
                # jacoRobot.setSpeed(Vr, vrot)
                # t.sleep(0.5)

            elif D > SpSafeVal and D <= SpPFLVal:
                print("Robot speed reduction")
                mode_collab = 2
                Vr = Vr_PFL
                Vr = round(Vr, 2)
                # jacoRobot.setSpeed(Vr, vrot)
                # t.sleep(0.5)

            elif D > SpPFLVal and D <= Spfull:
                mode_collab = 3
                Vr = Vr_SSM(D, Vh, Tr, Ts, ac, C_SSM, Zd, Zr, Vr_PFL)
                Vr = round(Vr, 2)
                # jacoRobot.setSpeed(Vr, vrot)
                # t.sleep(0.5)

            else:
                print("Robot bekerja maximal")
                mode_collab = 4
                Vr = RobotVrmax
                # jacoRobot.setSpeed(Vr, vrot)

        output.write(str(start_time.strftime("%H:%M:%S")) + ',' + str(milliseconds) + ',' + str(D) + ',' + str(Vr) + '\n')
        cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()