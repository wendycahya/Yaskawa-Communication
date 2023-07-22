from fs100 import FS100
import os
import threading
import time as t
import cv2
import keyboard
from utilsFS100 import FS100
import threading

import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from cvzone.PoseModule import PoseDetector
import cv2
from datetime import datetime
import time as t
import math as mt
import random
import csv

import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

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

start_time = datetime.now()
start = t.strftime("%Y%m%d-%H%M%S")
milliseconds = 0
write_file = "TestSpeedGraph-"+str(start)+".csv"
d = 0


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

#information
start = datetime.now()
stopwatch_time = t.time()
start_time = datetime.now()
end_time = datetime.now()
elapsed_time = 0
milliseconds = 0


#=== Draw Real-time graph show ===
fig, ax = plt.subplots()
ax2 = ax.twinx()
# Create an empty list to store data for plotting
dataD = []
dataVR = []
dataTime = []

RobTablePos = [0, 0, 0]
robotPos = [0, 0, 0, 0, 0, 0, 0]
distance_traveled, time_diff_ms, time_diff_s, velocity = 0, 0, 0, 0

# ===== SSM calculation ======
SpPFLVal = SpPFL(Vr_PFL, Vh, Tr, Ts, ac, C_SSM, Zd, Zr)
SpSafeVal = SpSafe(Vr_PFL, Ts, ac, C_SSM, Zd, Zr)
Spfull = SpMax(vrmax, Vr, Tr, Ts, ac, C_SSM, Zd, Zr)
SpminVal = Spmin(C_SSM, Zd, Zr)

fpsReader = cvzone.FPS()

# ===== Robot Function =====
def remap(value, from_low, from_high, to_low, to_high):
    # Clamp the value within the from range
    clamped_value = max(from_low, min(value, from_high))
    # Map the clamped value to the to range
    mapped_value = (clamped_value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
    return mapped_value

def convert_mm(x, y, z, rx, ry, rz, re):
    str_x = "{:4d}.{:03d}".format(x // 1000, x % 1000)
    str_y = "{:4d}.{:03d}".format(y // 1000, y % 1000)
    str_z = "{:4d}.{:03d}".format(z // 1000, z % 1000)
    str_rx = "{:4d}.{:04d}".format(rx // 10000, rx % 10000)
    str_ry = "{:4d}.{:04d}".format(ry // 10000, ry % 10000)
    str_rz = "{:4d}.{:04d}".format(rz // 10000, rz % 10000)
    str_re = "{:4d}.{:04d}".format(re // 10000, re % 10000)

    x = float(str_x)
    y = float(str_y)
    z = float(str_z)
    rx = float(str_rx)
    ry = float(str_ry)
    rz = float(str_rz)
    re = float(str_re)

    input = [x, y, z, rx, ry, rz, re]
    return input

speedUpdate = 0

class CustomThread(threading.Thread):
    def __init__(self, name):
        super().__init__(name=name)
        self.is_paused = False
        self.is_stopped = False
        self.pause_event = threading.Event()
        self.stop_event = threading.Event()

    def run(self):
        global speedUpdate
        cap = cv2.VideoCapture(2)
        while True:
            success, img = cap.read()

            cap.set(3, 640)  # width
            cap.set(4, 480)  # height

            detector = FaceMeshDetector(maxFaces=1)
            detectorPose = PoseDetector()
            with open(write_file, "wt", encoding="utf-8") as output:
                # Record data csv opening
                while True:
                    # Detect human skeleton
                    success, img = cap.read()
                    height, width, channels = img.shape
                    imgMesh, faces = detector.findFaceMesh(img, draw=False)
                    fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)
                    img = detectorPose.findPose(img)
                    lmList, bboxInfo = detectorPose.findPosition(img, bboxWithHands=False)

                    cv2.rectangle(img, (0, 0), (width, 70), (10, 10, 10), -1)
                    elapsed_time = round(t.time() - stopwatch_time, 3)

                    if bboxInfo:
                        idrSh, xrSh, yrSh, zrSh = lmList[11]
                        idlSh, xlSh, ylSh, zlSh = lmList[12]
                        # print("===== asli =====")
                        # print("Id data ", idrSh, "right shoulder x=", xrSh, ", right shoulder y=", yrSh)
                        # print("Id data ", idlSh, "left shoulder x=", xrSh, ", left shoulder y=", yrSh)
                        chestDistance = round(mt.sqrt((xrSh - xlSh) ** 2 + (yrSh - ylSh) ** 2), 3)

                    if faces:
                        # skeleton detection
                        data = []  # List to store the input data
                        while len(data) < 10:
                            face = faces[0]
                            # print(faces[0])
                            pointLeft = face[145]
                            pointRight = face[374]
                            cv2.line(imgMesh, pointLeft, pointRight, (0, 200, 0), 3)
                            cv2.circle(imgMesh, pointLeft, 5, (255, 0, 255), cv2.FILLED)
                            cv2.circle(imgMesh, pointRight, 5, (255, 0, 255), cv2.FILLED)
                            w, _ = detector.findDistance(pointLeft, pointRight)
                            W = 6.3  # default real width eyes distance
                            f = 714  # finding the average for focal length
                            d = ((W * f) / w) * 10
                            d = round(d, 3)

                            real_measurement = round(
                                (Achest * (chestDistance ** 2)) + (Bchest * chestDistance) + Cchest, 2)
                            D = min(d, real_measurement)

                            data.append(D)

                        # Calculate the average
                        D = sum(data) / len(data)

                        if FS100.ERROR_SUCCESS == robot.read_position(pos_info, robot_no):
                            x, y, z, rx, ry, rz, re = pos_info['pos']
                        # Distance Calibration results can be integrated here
                        robotPos = convert_mm(x, y, z, rx, ry, rz, re)
                        offset = 168
                        # D = (D + offset) - xRobPos
                        D = (D - offset) - robotPos[0]
                        D = round(D, 3)
                        print("Jarak calibration ", D)

                        if D < 0:
                            D = 0
                        else:
                            D = abs(D)
                        cvzone.putTextRect(img, f'Depth: {D} mm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)

                        # logical SSM send robot
                        if D <= SpminVal:
                            # server.pause()
                            Vr = 0
                            speedUpdate = 0
                            # print("Robot harus berhenti", Vr)
                            mode_collab = 0

                        elif D > SpminVal and D <= SpSafeVal:
                            # server.resume()
                            # print("Robot speed reduction")
                            Vr = Vr_SSM2(D, Tr, Ts, ac, C_SSM, Zd, Zr)
                            Vr = round(Vr, 2)
                            speedUpdate = int(remap(Vr, 0, 1500, 0, 1500))
                            # print("Robot working on collaboration mode")
                            mode_collab = 1


                        elif D > SpSafeVal and D <= SpPFLVal:
                            # server.resume()
                            # print("Robot speed reduction")
                            # print("Robot speed reduction")
                            mode_collab = 2
                            Vr = Vr_PFL
                            Vr = round(Vr, 2)
                            speedUpdate = int(remap(Vr, 0, 1500, 0, 1500))


                        elif D > SpPFLVal and D <= Spfull:
                            # server.resume()
                            Vr = Vr_SSM(D, Vh, Tr, Ts, ac, C_SSM, Zd, Zr, Vr_PFL)
                            Vr = round(Vr, 2)
                            speedUpdate = int(remap(Vr, 0, 1500, 0, 1500))
                            # print("change value speed Reduce: ", Vr)
                            mode_collab = 3

                        else:
                            # server.resume()
                            mode_collab = 4
                            # print("Robot bekerja maximal")
                            # mode_collab = 1
                            Vr = RobotVrmax
                            speedUpdate = int(remap(Vr, 0, 1500, 0, 1500))
                            # print("change value speed maximum: ", VrPaper)

                    dataVel = []
                    # velocity = velocity_group(dataVel)
                    dataD.append(D)
                    dataVR.append(velocity)
                    dataTime.append(elapsed_time)
                    # Update the plot
                    # update_plot()
                    #
                    # output.write(
                    #     str(end_time.strftime("%H:%M:%S")) + ',' + str(elapsed_time) + ',' + str(D) + ',' + str(
                    #         speed) + ',' +
                    #     str(counter) + ',' + str(round(velocity, 3)) + ',' + str(robotPos[0]) + ',' + str(
                    #         robotPos[1]) + ',' + str(robotPos[2]) + '\n')
                    # print("SUCCESS RECORD ", interval, " !!!")
                    # print("SUCCESS RECORD counter", counter, " !!!")
                    # # Load the saved plot image
                    # plot_img = cv2.imread('temp_plot.png', cv2.IMREAD_UNCHANGED)
                    #
                    # # Resize the plot image to match the video frame size
                    # plot_img = cv2.resize(plot_img, (img.shape[1], img.shape[0]))
                    # cv2.putText(img, "{} s".format(elapsed_time), (10, 30), cv2.FONT_HERSHEY_PLAIN,
                    #             2, (15, 225, 215), 2)
                    # cv2.putText(img, "counter {}".format(counter), (10, 60), cv2.FONT_HERSHEY_PLAIN,
                    #             2, (15, 225, 215), 2)
                    # # Display the video frame in the 'Video Stream' window
                    cv2.imshow('Video Stream', img)

                    # Display the plot in the 'Live Plot' window
                    # cv2.imshow('HR Distance and Robot Velocity', plot_img[:, :, :3])

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                cap.release()
                cv2.destroyAllWindows()

    def pause(self):
        self.is_paused = True

    def resume(self):
        self.is_paused = False
        self.pause_event.set()

    def stop(self):
        self.is_stopped = True
        self.pause_event.set()

def rob_command(post1):
    x_coor = post1[0] * 1000
    y_coor = post1[1] * 1000
    z_coor = post1[2] * 1000
    rx_coor = post1[3] * 10000
    ry_coor = post1[4] * 10000
    rz_coor = post1[5] * 10000
    re_coor = post1[6] * 10000

    robot_command = [(int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor))]
    #robot_command = (int(x_coor), int(y_coor), int(z_coor), 0, 0, 0, 0)
    return robot_command

def update_pos():
    while stop_sign.acquire(blocking=False):
        stop_sign.release()
        # let button up take effect
        t.sleep(0.02)

def is_alarmed():
    alarmed = True
    status = {}
    if FS100.ERROR_SUCCESS == robot.get_status(status):
        alarmed = status['alarming']
    return alarmed

def on_reset_alarm():
    robot.reset_alarm(FS100.RESET_ALARM_TYPE_ALARM)
    t.sleep(0.1)
    # reflect the ui
    is_alarmed()

# robot connection
#robot = FS100('192.168.255.1')
p1 = [353.427, -298.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p2 = [353.427, -198.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p3 = [353.427, -98.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p4 = [353.427, 2.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p5 = [353.427, 102.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p6 = [353.427, 202.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p6 = [353.427, 302.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]

## ===== convert robot command =====
#post_1 = rob_command(p1)
post_2 = rob_command(p2)
post_3 = rob_command(p3)
post_4 = rob_command(p4)
post_5 = rob_command(p5)
post_6 = rob_command(p6)

thread1 = CustomThread(name="Thread 1")
thread1.start()


robot = FS100('172.16.0.1')
#robot = FS100('127.0.0.1')
stop_sign = threading.Semaphore()

pos_info = {}
robot_no = 1
status = {}
counter = 0

postMove = [post_2, post_3, post_4, post_5, post_6]

if FS100.ERROR_SUCCESS == robot.get_status(status):
    if not status['servo_on']:
        robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

while True:
    print(speedUpdate)
    for i in postMove:
        robot.move(None, FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
                   FS100.MOVE_SPEED_CLASS_MILLIMETER, speedUpdate, i, wait=True)
        if FS100.ERROR_SUCCESS == robot.read_position(pos_info, robot_no):
            x, y, z, rx, ry, rz, re = pos_info['pos']
            str = "CURRENT POSITION\n" + \
                  "COORDINATE {:12s} TOOL:{:02d}\n".format('ROBOT', pos_info['tool_no']) + \
                  "R{} :X     {:4d}.{:03d} mm       Rx   {:4d}.{:04d} deg.\n".format(robot_no,
                                                                                     x // 1000, x % 1000, rx // 10000,
                                                                                     rx % 10000) + \
                  "    Y     {:4d}.{:03d} mm       Ry   {:4d}.{:04d} deg.\n".format(
                      y // 1000, y % 1000, ry // 10000, ry % 10000) + \
                  "    Z     {:4d}.{:03d} mm       Rz   {:4d}.{:04d} deg.\n".format(
                      z // 1000, z % 1000, rz // 10000, rz % 10000) + \
                  "                            Re   {:4d}.{:04d} deg.\n".format(
                      re // 10000, re % 10000)

            print(str)
        if i == post_6:
            counter = counter + 1
            ## counter information
            print("Robot counter step: ", counter)
            break

robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_OFF)



    # if FS100.ERROR_SUCCESS == robot.get_status(status):
    #     print(status)




