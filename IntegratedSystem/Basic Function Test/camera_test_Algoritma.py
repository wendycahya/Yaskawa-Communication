#Robot library
import time as t
#from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot
import threading
from datetime import datetime

#camera library
import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from cvzone.FaceDetectionModule import FaceDetector
from cvzone.PoseModule import PoseDetector
import cv2

import mediapipe as mp
import numpy as np
import math as mt

#computation
from itertools import count
import matplotlib.pyplot as plt
import pandas as pd
from pandas.errors import EmptyDataError
from matplotlib.animation import FuncAnimation
import csv

Sp, Spfull, SpminVal, SpSafeVal, SpPFLVal = 0, 0, 0, 0, 0

#=================function SSM =============================
def SSM_calculation(Vr, Vh, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr*Ts + ((ac*pow(Ts, 2))/2)
    Sp   = Vh * (Tr + Ts) + (Vr * Tr) + Ss + Ctot
    return Sp

def center_point(a,b):
    a = np.array(a)
    b = np.array(b)
    d = mt.sqrt(mt.pow((a[0] - b[0]),2) + mt.pow((a[1] - b[1]),2) + mt.pow((a[2] - b[2]),2))
    mid = [(a[0]+b[0])/2 , (a[1]+b[1])/2, (a[2]+b[2])/2]
    return d, mid

def center_pointXY(a,b):
    a = np.array(a)
    b = np.array(b)
    dXY = mt.sqrt(mt.pow((a[0] - b[0]),2) + mt.pow((a[1] - b[1]),2))
    midXY = [(a[0]+b[0])/2, (a[1]+b[1])/2]
    return dXY, midXY

def velXYZ(Xn, Xn_last, ts):
    velX = (Xn_last[0] - Xn[0]) / ts
    velY = (Xn_last[1] - Xn[1]) / ts
    velZ = (Xn_last[2] - Xn[2]) / ts
    return velX, velY, velZ

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

def inputVRVH(dIn, Pos1, Pos2):
    p1 = np.array(Pos1)
    p2 = np.array(Pos2)
    v1 = np.array(dIn)
    num = p2 - p1
    denum = np.linalg.norm(num)
    calcnumDenum = num/denum
    inputParam = abs(np.dot(v1, calcnumDenum))
    return inputParam

# ===== Robot Function =====
def remap(value, from_low, from_high, to_low, to_high):
    # Clamp the value within the from range
    clamped_value = max(from_low, min(value, from_high))
    # Map the clamped value to the to range
    mapped_value = (clamped_value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
    return mapped_value

#===== new Function to complete Research
def calculate_velocity(distance, time):
    """
    Calculates the velocity given the distance and time.
    :param distance: The distance traveled by the robot (in meters).
    :param time: The time taken by the robot to cover the distance (in seconds).
    :return: The velocity of the robot (in meters per second).
    """
    velocity = distance / time
    return velocity

def get_time_difference_ms(start_time, end_time):
    """
    Calculates the time difference in milliseconds between two datetime objects.
    :param start_time: The starting datetime.
    :param end_time: The ending datetime.
    :return: The time difference in milliseconds.
    """
    time_diff = end_time - start_time
    time_diff_ms = time_diff.total_seconds() * 1000
    return time_diff_ms

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

#Human velocity
Xn1D = 0
Xn_last1D = 0
Xn = [0, 0, 0]
Xn_last = [0, 0, 0]
velX, velY, velZ = [0, 0, 0]
ts = 0.05

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


Spfull = SpMax(vrmax, Vr, Tr, Ts, ac, C_SSM, Zd, Zr)
SpminVal = Spmin(C_SSM, Zd, Zr)
distView = 0
sampleDistance = 1
pause_active = 0

#calibration position variable
zHead = [0, 0]
zChest = [0, 0]
RobTablePos = [0, 0, 0]
robotPos = [0, 0, 0, 0, 0, 0, 0]
distance_traveled, time_diff_ms, time_diff_s, velocity = 0, 0, 0, 0

#distance measurement
#x shoulder in cm
#y real measurement
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


#information
start = datetime.now()
stopwatch_time = t.time()
start_time = datetime.now()
end_time = datetime.now()
elapsed_time = 0
milliseconds = 0
#calibration = 1200
write_file = "SimulationProductivity-"+str(start)+".csv"
mode_collab = 0

#SSM original data
VrOriSSM = 0
mode_SSMori = 0

counter = 0

#====================ROBOT POSITION==========================
start = datetime.now()

# Position Data:
pickPos = [400, -150, 0, 180, 0, 0]
# naik tiap 100mm
liftPos11 = [400, -150, 100, 180, 0, 0]
liftPos12 = [400, -150, 200, 180, 0, 0]
liftPos13 = [400, -150, 300, 180, 0, 0]
liftPos14 = [400, -150, 400, 180, 0, 0]

# movement to goal place
liftPos21 = [500, -100, 500, 180, 60, 0]
liftPos22 = [500, -50, 500, 180, 60, 0]
liftPos23 = [500, 0,   500, 180, 60, 0]
liftPos24 = [500, 100, 500, 180, 60, 0]
liftPos25 = [500, 200, 500, 180, 60, 0]

goalPos = [500, 200, 0, 180, 0, 0]
liftPos2 = [500, 200, 500, 180, 60, 0]
liftPos3 = [500, 200, 100, 180, 0, 0]


#object pic location
objPos1 = [500, 260, 0, 180, 0, 0]
objPos2 = [500, 260, 50, 180, 0, 0]
objPos3 = [500, 260, 100, 180, 0, 0]
objPos3 = [500, 260, 300, 180, 0, 0]

#movement to goal place
movePos21 = [400, 200, 500, 180, 60, 0]
movePos22 = [400, 100, 500, 180, 60, 0]
movePos23 = [400, 0,   500, 180, 60, 0]
movePos24 = [400, -50, 500, 180, 60, 0]
movePos25 = [400, -100, 500, 180, 60, 0]


#Cube A location
APos11 = [400, -120, 400, 180, 0, 0]
APos12 = [400, -120, 300, 180, 0, 0]
APos13 = [400, -120, 200, 180, 0, 0]
APos14 = [400, -120, 100, 180, 0, 0]
APlace = [400, -120, 0, 180, 0, 0]

#Cube B location
BPos11 = [550, -120, 400, 180, 0, 0]
BPos12 = [550, -120, 300, 180, 0, 0]
BPos13 = [550, -120, 200, 180, 0, 0]
BPos14 = [550, -120, 100, 180, 0, 0]
BPlace = [550, -120, 0, 180, 0, 0]


#Cube C location
CPos11 = [400, 40, 400, 180, 0, 0]
CPos12 = [400, 40, 300, 180, 0, 0]
CPos13 = [400, 40, 200, 180, 0, 0]
CPos14 = [400, 40, 100, 180, 0, 0]
CPlace = [400, 40, 0, 180, 0, 0]

#Cube D location
DPos11 = [550, 40, 400, 180, 0, 0]
DPos12 = [550, 40, 300, 180, 0, 0]
DPos13 = [550, 40, 200, 180, 0, 0]
DPos14 = [550, 40, 100, 180, 0, 0]
DPlace = [550, 40, 0, 180, 0, 0]

# def moveToGoal():
#     jacoRobot.setPosition2(movePos21, True)
#     jacoRobot.setPosition2(movePos22, True)
#     jacoRobot.setPosition2(movePos23, True)
#     jacoRobot.setPosition2(movePos24, True)
#     jacoRobot.setPosition2(movePos25, True)
#
# def moveToBack():
#     jacoRobot.setPosition2(movePos25, True)
#     jacoRobot.setPosition2(movePos24, True)
#     jacoRobot.setPosition2(movePos23, True)
#     jacoRobot.setPosition2(movePos22, True)
#     jacoRobot.setPosition2(movePos21, True)
#
# def pick_goDown():
#     jacoRobot.setPosition2(objPos3, True)
#     jacoRobot.setPosition2(objPos1, True)
#
# def pick_goUp():
#     jacoRobot.setPosition2(objPos2, True)
#     jacoRobot.setPosition2(objPos3, True)
#
# def place_A():
#     jacoRobot.setPosition2(APos11, True)
#     jacoRobot.setPosition2(APos12, True)
#     jacoRobot.setPosition2(APos13, True)
#     jacoRobot.setPosition2(APos14, True)
#     jacoRobot.setPosition2(APlace, True)
#
# def place_B():
#     jacoRobot.setPosition2(BPos11, True)
#     jacoRobot.setPosition2(BPos12, True)
#     jacoRobot.setPosition2(BPos13, True)
#     jacoRobot.setPosition2(BPos14, True)
#     jacoRobot.setPosition2(BPlace, True)
#
# def place_C():
#     jacoRobot.setPosition2(CPos11, True)
#     jacoRobot.setPosition2(CPos12, True)
#     jacoRobot.setPosition2(CPos13, True)
#     jacoRobot.setPosition2(CPos14, True)
#     jacoRobot.setPosition2(CPlace, True)
#
# def place_D():
#     jacoRobot.setPosition2(DPos11, True)
#     jacoRobot.setPosition2(DPos12, True)
#     jacoRobot.setPosition2(DPos13, True)
#     jacoRobot.setPosition2(DPos14, True)
#     jacoRobot.setPosition2(DPlace, True)


progress = [0, 0, 0, 0]
finish = [1, 1, 1, 1]

# THREAD FUNCTION:
# Multithread function khusus buat menggerakkan robot
# Program ini akan terus melalukan looping pick & place
# dan tidak terpengaruh dengan main program (pose detection)

# def thread_robotMovement():
#
#     jacoRobot.setSpeed(1200, 90)
#     while True:
#         print("reading the condition", progress)
#
#         jacoRobot.gripperRelease()
#         pick_goDown()
#         jacoRobot.gripperCatch()
#         pick_goUp()
#         moveToGoal()
#
#         # condition A belum terisi
#         if (progress == [0, 0, 0, 0]):
#             progress[0] = 1
#             print("robot mengisi progress[0]", progress)
#             place_A()
#         elif (progress == [0, 1, 0, 0]):
#             progress[0] = 1
#             print("robot mengisi progress[0]", progress)
#             place_A()
#         elif (progress == [0, 0, 1, 0]):
#             progress[0] = 1
#             print("robot mengisi progress[0]", progress)
#             place_A()
#         elif (progress == [0, 0, 0, 1]):
#             progress[0] = 1
#             print("robot mengisi progress[0]", progress)
#             place_A()
#         elif (progress == [0, 1, 1, 0]):
#             progress[0] = 1
#             print("robot mengisi progress[0]", progress)
#             place_A()
#         elif (progress == [0, 1, 0, 1]):
#             progress[0] = 1
#             print("robot mengisi progress[0]", progress)
#             place_A()
#         elif (progress == [0, 0, 1, 1]):
#             progress[0] = 1
#             print("robot mengisi progress[0]", progress)
#             place_A()
#         elif (progress == [0, 1, 1, 1]):
#             progress[0] = 1
#             print("robot mengisi progress[0]", progress)
#             place_A()
#         # condition B belum terisi
#         elif (progress == [1, 0, 0, 0]):
#             progress[1] = 1
#             print("robot mengisi progress[1]", progress)
#             place_B()
#         elif (progress == [1, 0, 1, 0]):
#             progress[1] = 1
#             print("robot mengisi progress[1]", progress)
#             place_B()
#         elif (progress == [1, 0, 0, 1]):
#             progress[1] = 1
#             print("robot mengisi progress[1]", progress)
#             place_B()
#         elif (progress == [1, 0, 1, 1]):
#             progress[1] = 1
#             print("robot mengisi progress[1]", progress)
#             place_B()
#         # condition C belum terisi
#         elif (progress == [1, 1, 0, 0]):
#             progress[2] = 1
#             print("robot mengisi progress[2]", progress)
#             place_C()
#         elif (progress == [1, 1, 0, 1]):
#             progress[2] = 1
#             print("robot mengisi progress[2]", progress)
#             place_C()
#         # condition D belum terisi
#         elif (progress == [1, 1, 1, 0]):
#             progress[3] = 1
#             print("robot mengisi progress[3]", progress)
#             place_D()
#         if (progress == finish):
#             print("Robot Stop\n")
#             jacoRobot.gripperRelease()
#             pick_goUp()
#             print(datetime.now() - start)
#             break
#
#         jacoRobot.gripperRelease()
#         counter = counter + 1
#         t.sleep(1)
#         moveToBack()

# ====================================================
#=== Draw Real-time graph show ===
# Create a figure and axes for live plotting
fig, ax = plt.subplots()
ax2 = ax.twinx()
# Create an empty list to store data for plotting
dataD = []
dataVR = []

# Function to update the plot
def update_plot():
    ax.clear()
    ax.plot(dataD, 'b-')
    ax2.plot(dataVR, 'r--')
    # ax2.plot(dataX, 'r')
    # ax2.plot(dataY, 'g')
    # ax2.plot(dataZ, 'b')
    plt.axis('on')  # Turn off axis labels and ticks
    ax.set_xlabel("Sample Time")
    ax.set_ylabel("Distance (mm)")
    ax2.set_ylabel("Speed (mm/s)")
    plt.tight_layout()  # Adjust the plot to remove any padding
    plt.savefig('temp_plot.png')  # Save the plot as an image

# ROBOT INITIALIZATION:
# ====================================================
# mSim = CoppeliaSim()
# ret = mSim.connect(19997)
# if ret == -1:
#     exit()

# Initialize the robot model
# jacoRobot = CoppeliaArmRobot("Jaco")
# t.sleep(1)


# start thread:
# thr = threading.Thread(target=thread_robotMovement)
# thr.start()

# MAIN PRORGAM:
# ===== camera installation =====
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(2)
cap.set(3, 640)  # width
cap.set(4, 480)  # height

detector = FaceMeshDetector(maxFaces=1)
detectFace = FaceDetector()
detectorPose = PoseDetector()

# ======================================================
# masukkan program utama disini (looping program)
with open(write_file, "wt", encoding="utf-8") as output:
    while True:
    # Detect human skeleton
        success, img = cap.read()
        height, width, channels = img.shape
        imgMesh, faces = detector.findFaceMesh(img, draw=False)
        # imgFace, bboxs = detectFace.findFaces(img)
        fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)
        img = detectorPose.findPose(img)
        lmList, bboxInfo = detectorPose.findPosition(img, bboxWithHands=False)


        elapsed_time = round(t.time() - stopwatch_time, 3)

        # === Robot analysis Velocity ===
        # robotPos = jacoRobot.readPosition()
        print("robot position ", robotPos[0], robotPos[1], robotPos[2])

        XnRob = [robotPos[0], robotPos[1], robotPos[2]]
    # ===== Visualization information ======

        if faces:
            if bboxInfo:
                idrSh, xrSh, yrSh, zrSh = lmList[11]
                idlSh, xlSh, ylSh, zlSh = lmList[12]
                # print("===== asli =====")
                # print("Id data ", idrSh, "right shoulder x=", xrSh, ", right shoulder y=", yrSh)
                # print("Id data ", idlSh, "left shoulder x=", xrSh, ", left shoulder y=", yrSh)
                chestDistance = round(mt.sqrt((xrSh - xlSh) ** 2 + (yrSh - ylSh) ** 2), 3)
            #skeleton detection
            face = faces[0]
            #print(faces[0])
            pointLeft = face[145]
            pointRight = face[374]
            #v2.line(imgMesh, pointLeft, pointRight, (0, 200, 0), 3)
            #cv2.circle(imgMesh, pointLeft, 5, (255, 0, 255), cv2.FILLED)
            #cv2.circle(imgMesh, pointRight, 5, (255, 0, 255), cv2.FILLED)
            w, _ = detector.findDistance(pointLeft, pointRight)
            #center = bboxs[0]["center"]
            #print(center)
            #cv2.circle(img, center, 8, (255, 0, 255), cv2.FILLED)
            #print(w)
            W = 6.3  # default real width eyes distance
            #Finding the focal length
            #d = 60  #distance human and camera
            #f = (w*d) / W
            #print(f)
            #finding distance
            f = 714 #finding the average for focal length
            d = (W*f) / w
            #print(d)
            d = d * 10 # distance in mm
            eye_dist = round(d, 3)

            img.flags.writeable = False

            #skeleton mediapipe migrasion
            # Recolor image to RGB
            # Extract landmarks
            data = []  # List to store the input data

            # Collect 10 input values
            while len(data) < 10:
                # print("2. Human Distance ", disHR)
                xRobPos = 550
                # print("===========================================")

                eye_dist = round(eye_dist, 2)
                # real_measurement = round((Achest * (chestDistance ** 2)) + (Bchest * chestDistance) + Cchest, 2)
                # # print("==========")
                # # print("lebar pixel ", chestDistance)
                # print("Real Chest Distance", real_measurement)

                D = eye_dist
                #D = min(eye_dist, real_measurement)
                value = D
                data.append(value)

            # Calculate the average
            D = sum(data) / len(data)

            #robotPos = jacoRobot.readPosition()
            #print("robot position ", curRobotPos[0], curRobotPos[1], curRobotPos[2])

            XnRob = [robotPos[0], robotPos[1], robotPos[2]]

            start_time = datetime.now()
            milliseconds = start_time.microsecond // 1000
            #
            # #XnRob_last = [0, 0, 0]
            # velXR, velYR, velZR = velXYZ(XnRob, XnRob_last, ts)
            # D = (D + 500) - robotPos[0]
            offset = 0
            # D = (D + offset) - xRobPos
            D = (D - offset)-500
            D = round(D, 3)

            # ===== SSM calculation ======
            SpPFLVal = SpPFL(Vr_PFL, Vh, Tr, Ts, ac, C_SSM, Zd, Zr)
            SpSafeVal = SpSafe(Vr_PFL, Ts, ac, C_SSM, Zd, Zr)

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
                #jacoRobot.setSpeed(Vr, vrot)
                t.sleep(0.5)

            elif D > SpminVal and D <= SpSafeVal:
                print("Robot working on collaboration mode")
                mode_collab = 1
                Vr = Vr_SSM2(D, Tr, Ts, ac, C_SSM, Zd, Zr)
                Vr = round(Vr, 2)
                #jacoRobot.setSpeed(Vr, vrot)
                t.sleep(0.5)

            elif D > SpSafeVal and D <= SpPFLVal:
                print("Robot speed reduction")
                mode_collab = 2
                Vr = Vr_PFL
                Vr = round(Vr, 2)
                #jacoRobot.setSpeed(Vr, vrot)
                t.sleep(0.5)

            elif D > SpPFLVal and D <= Spfull:
                mode_collab = 3
                Vr = Vr_SSM(D, Vh, Tr, Ts, ac, C_SSM, Zd, Zr, Vr_PFL)
                Vr = round(Vr, 2)
                #jacoRobot.setSpeed(Vr, vrot)
                t.sleep(0.5)

            else:
                print("Robot bekerja maximal")
                mode_collab = 4
                Vr = RobotVrmax
                #jacoRobot.setSpeed(Vr, vrot)

            t.sleep(ts)
            # Xn_last = Xn
            Xn_last1D = Xn1D
            XnRob_last = XnRob

            t.sleep(0.5)

        #robotPos = jacoRobot.readPosition()
        end_time = datetime.now()
        XnRobEnd = [robotPos[0], robotPos[1], robotPos[2]]

        distance_traveled = mt.sqrt((XnRobEnd[0] - XnRob[0]) ** 2 + (XnRobEnd[1] - XnRob[1]) ** 2 + (XnRobEnd[2] - XnRob[2]) ** 2)
        time_diff_ms = get_time_difference_ms(start_time, end_time)
        time_diff_s = time_diff_ms / 1000  # converting milliseconds to seconds
        velocity = calculate_velocity(distance_traveled, time_diff_s)
        # Replace this with your actual variable that you want to plot
        # import random
        # new_data = random.randint(0, 100)
        dataD.append(D)
        dataVR.append(velocity)

        # Update the plot
        update_plot()
        output.write(str(end_time.strftime("%H:%M:%S")) + ',' + str(elapsed_time) + ',' + str(D) + ',' + str(Vr) + ',' +
                str(counter) + ',' + str(velocity) + ',' + str(round(robotPos[0],4)) + ',' + str(round(robotPos[1],4)) +
                                                               ',' + str(round(robotPos[2],4)) + '\n')
        print("SUCCESS RECORD ", interval, " !!!")
        print("SUCCESS RECORD counter", counter, " !!!")
        # Load the saved plot image
        plot_img = cv2.imread('temp_plot.png', cv2.IMREAD_UNCHANGED)
        cv2.rectangle(img, (0, 0), (width, 70), (10, 10, 10), -1)
        # Resize the plot image to match the video frame size
        plot_img = cv2.resize(plot_img, (img.shape[1], img.shape[0]))
        cv2.putText(img, "{} s".format(elapsed_time), (10, 30), cv2.FONT_HERSHEY_PLAIN,
                    2, (15, 225, 215), 2)
        cv2.putText(img, "counter {}".format(counter), (10, 60), cv2.FONT_HERSHEY_PLAIN,
                    2, (15, 225, 215), 2)
        # Display the video frame in the 'Video Stream' window
        cv2.imshow('Video Stream', img)

        # Display the plot in the 'Live Plot' window
        cv2.imshow('HR Distance and Robot Velocity', plot_img[:, :, :3])

    # ===== research documentation =====
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()