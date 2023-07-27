#================= Library Declaration =======================
import os
from utilsFS100 import FS100
import threading

import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from cvzone.PoseModule import PoseDetector
import cv2
import time as t
import math as mt
import random
import csv

import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

from anrot_module import *

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
write_file = "CloseProposedProductivity-"+str(start)+".csv"
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
robotPos = [0, 0, 0, 0, 0, 0]

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

Achest = 0.04628882081739653
Bchest = -24.603862891449737
Cchest = 3870.586790291231

#information
start = datetime.now()
stopwatch_time = t.time()
end_time = datetime.now()
elapsed_time = 0

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

# Global variables to control the pause and resume state
paused = False

#=========================== Draw Real-time graph show ========================
fig, ax = plt.subplots()
ax2 = ax.twinx()
# Create an empty list to store data for plotting
dataD = []
dataVR = []
dataSPD = []
dataTime = []

# Function to update the plot
def update_plot():
    ax.clear()
    ax.plot(dataTime, dataD, 'b-', label='Distance')
    ax2.plot(dataTime, dataVR, 'r--', label='Velocity')
    ax2.plot(dataTime, dataSPD, 'g-.', label='Speed Command')
    plt.axis('on')  # Turn off axis labels and ticks
    # Add legends to the plot
    # ax.legend(loc='upper left')
    # ax2.legend(loc='upper right')

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (mm)")
    ax2.set_ylabel("Speed (mm/s)")
    plt.tight_layout()  # Adjust the plot to remove any padding
    plt.savefig('temp_plot.png')  # Save the plot as an image


#====================================== Camera Detection ====================================
class CustomThread(threading.Thread):
    def __init__(self, name):
        super().__init__(name=name)
        self.is_paused = False
        self.is_stopped = False
        self.pause_event = threading.Event()
        self.stop_event = threading.Event()

    def run(self):
        global speedUpdate, paused
        cap = cv2.VideoCapture(2)

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
                    #print("Jarak calibration ", D)

                    if D < 0:
                        D = 0
                    else:
                        D = abs(D)
                    cvzone.putTextRect(img, f'Depth: {D} mm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)

                    # logical SSM send robot
                    if D <= SpminVal:
                        pause_event.set()  # Pause the while loop
                        Vr = 0
                        speedUpdate = 0
                        # print("Robot harus berhenti", Vr)
                        mode_collab = 0

                    elif D > SpminVal and D <= SpSafeVal:
                        pause_event.clear()  # Resume the while loop
                        resume_event.set()
                        # print("Robot speed reduction")
                        Vr = Vr_SSM2(D, Tr, Ts, ac, C_SSM, Zd, Zr)
                        Vr = round(Vr, 2)
                        speedUpdate = int(remap(Vr, 0, 1500, 0, 1000))
                        # print("Robot working on collaboration mode")
                        mode_collab = 1

                    elif D > SpSafeVal and D <= SpPFLVal:
                        pause_event.clear()  # Resume the while loop
                        resume_event.set()
                        # print("Robot speed reduction")
                        mode_collab = 2
                        Vr = Vr_PFL
                        Vr = round(Vr, 2)
                        speedUpdate = int(remap(Vr, 0, 1500, 0, 1000))


                    elif D > SpPFLVal and D <= Spfull:
                        pause_event.clear()  # Resume the while loop
                        resume_event.set()
                        Vr = Vr_SSM(D, Vh, Tr, Ts, ac, C_SSM, Zd, Zr, Vr_PFL)
                        Vr = round(Vr, 2)
                        speedUpdate = int(remap(Vr, 0, 1500, 0, 1000))
                        # print("change value speed Reduce: ", Vr)
                        mode_collab = 3

                    else:
                        pause_event.clear()  # Resume the while loop
                        resume_event.set()
                        Vr = RobotVrmax
                        mode_collab = 4
                        speedUpdate = int(remap(Vr, 0, 1500, 0, 1000))
                        # print("change value speed maximum: ", VrPaper)

                # realtimeSPD = speedUpdate / 10
                # dataD.append(D)
                # dataSPD.append(realtimeSPD)
                # #dataVR.append(velocity_avg)
                # dataTime.append(elapsed_time)
                # # Update the plot
                # update_plot()
                #
                output.write(
                    str(end_time.strftime("%H:%M:%S")) + ',' + str(elapsed_time) + ',' + str(D) + ',' + str(
                        speedUpdate) + ',' +
                    str(counter) + ',' + str(round(velocity_avg, 3)) + ',' + str(robotPos[0]) + ',' + str(
                        robotPos[1]) + ',' + str(robotPos[2]) + '\n')
                # print("SUCCESS RECORD ", interval, " !!!")
                # print("SUCCESS RECORD counter", counter, " !!!")
                # # Load the saved plot image
                #plot_img = cv2.imread('temp_plot.png', cv2.IMREAD_UNCHANGED)
                #
                # # Resize the plot image to match the video frame size
                #plot_img = cv2.resize(plot_img, (img.shape[1], img.shape[0]))
                cv2.putText(img, "{} s".format(elapsed_time), (10, 30), cv2.FONT_HERSHEY_PLAIN,
                            2, (15, 225, 215), 2)
                cv2.putText(img, "counter {}".format(counter), (10, 60), cv2.FONT_HERSHEY_PLAIN,
                            2, (15, 225, 215), 2)
                # # Display the video frame in the 'Video Stream' window
                cv2.imshow('Video Stream', img)

                # Display the plot in the 'Live Plot' window
                #cv2.imshow('HR Distance and Robot Velocity', plot_img[:, :, :3])

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


p1=[353.426, -299.669, -308.575, 179.9869, -5.5662, -25.9376, 0]
p2=[354.469, -207.837, -308.435, 179.1151, -4.1320, -24.4397, 0]
p3=[353.436, -94.476, -308.571, 178.7970, -4.2889,  -24.5672, 0]
p4=[353.436, 10.878, -308.570, 178.7971, -4.2899, -24.5655, 0]
p5=[353.432, 108.075, -308.570, 178.7968, -4.2906, -24.5681, 0]
p6=[353.421, 206.070, -308.569, 178.7961, -4.2913, -24.5686, 0]
p7=[353.429, 298.076, -308.571, 178.7984, -4.2911, -24.5633, 0]
p8=[353.434, 207.680, -308.571, 178.7989, -4.2909, -24.5601, 0]
p9=[353.437, 107.280, -308.571, 178.7991, -4.2906, -24.5602, 0]
p10=[353.445, 5.277, -308.572, 178.7995, -4.2895, -24.5614, 0]
p11=[353.624, -99.674, -308.568, 178.8014, -4.2916, -24.5612, 0]

## ===== convert robot command =====
#post_1 = rob_command(p1)
post_2 = rob_command(p2)
post_3 = rob_command(p3)
post_4 = rob_command(p4)
post_5 = rob_command(p5)
post_6 = rob_command(p6)

post_7 = rob_command(p7)
post_8 = rob_command(p8)
post_9 = rob_command(p9)
post_10 = rob_command(p10)
post_11 = rob_command(p11)

# robot connection
#robot = FS100('192.168.255.1')
robot = FS100('172.16.0.1')
stop_sign = threading.Semaphore()

pos_info = {}
robot_no = 1
status = {}
status_move = {}
counter = 0

# Function to pause the thread
pause_event = threading.Event()
resume_event = threading.Event()

def robot_working():
    global counter
    t.sleep(3)
    if FS100.ERROR_SUCCESS == robot.get_status(status):
        if not status['servo_on']:
            robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

    postMove = [post_2, post_3, post_4, post_5, post_6, post_7, post_8, post_9, post_10, post_11]
    while True:
        for i in postMove:
            print(speedUpdate)
            while pause_event.is_set():  # Pause the loop if 'paused' is True
                print("Loop paused...")
            #wait_thread()
            robot.move(None, FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
                       FS100.MOVE_SPEED_CLASS_MILLIMETER, speedUpdate, i, wait=True)

            if i == post_11:
                counter = counter + 1
                ## counter information
                print("Robot counter step: ", counter)
                break

    robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_OFF)

velocity_avg = 0

#===================Main Program Thread Execution======================================
def velocity_calculator_thread():
    global velocity_avg
    m_IMU = anrot_module('./config_IMU.json')
    while True:
        data = m_IMU.get_module_data(10)

        x = data['acc'][0]['X']
        y = data['acc'][0]['Y']
        z = data['acc'][0]['Z']

        # Calculate velocity

        if vx is not None and vy is not None and vz is not None:
            # Calculate the magnitude of velocity
            magnitude_velocity = self.calculate_magnitude_velocity(vx, vy, vz)
            #print(f"Time: {t.time()}, X: {x}, Y: {y}, Z: {z}")
            #print(f"Velocity: Vx: {vx}, Vy: {vy}, Vz: {vz}")
            #print(f"Magnitude Velocity: {magnitude_velocity}")
            velocity_avg = magnitude_velocity
        else:
            # Print when there is not enough data to calculate velocity
            #print(f"Time: {t.time()}, X: {x}, Y: {y}, Z: {z}")
            print("Not enough data points to calculate velocity.")
        t.sleep(0.5)

if __name__ == '__main__':
    thread1 = CustomThread(name="Thread 1")
    thread1.start()
    thread2 = threading.Thread(target=robot_working)
    thread2.start()
    velocity_calculator_thread = threading.Thread(target=velocity_calculator_thread)
    # Start the thread for the VelocityCalculator
    velocity_calculator_thread.start()
    while True:
        print("Alhamdulillah yok bisa yok", velocity_avg)