import threading
import time

class Job(threading.Thread):

    def __init__(self, *args, **kwargs):
        super(Job, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()     # 用于暂停线程的标识
        self.__flag.set()       # 设置为True
        self.__running = threading.Event()      # 用于停止线程的标识
        self.__running.set()      # 将running设置为True

    def run(self):
        while self.__running.isSet():

            for i in range(0, 100):
                self.__flag.wait()
                print(i)
                print(Vr)
                time.sleep(1)

    def pause(self):
        self.__flag.clear()     # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()    # 设置为True, 让线程停止阻塞

    def stop(self):
        self.__flag.set()       # 将线程从暂停状态恢复, 如何已经暂停的话
        self.__running.clear()        # 设置为False


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

start_time = datetime.now()
start = t.strftime("%Y%m%d-%H%M%S")
milliseconds = 0
write_file = "NewProductivity-"+str(start)+".csv"
d = 0

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
counter = 0

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


#calibration position variable
RobTablePos = [0, 0, 0]
robotPos = [0, 0, 0, 0, 0, 0, 0]
distance_traveled, time_diff_ms, time_diff_s, velocity = 0, 0, 0, 0

# ===== SSM calculation ======
SpPFLVal = SpPFL(Vr_PFL, Vh, Tr, Ts, ac, C_SSM, Zd, Zr)
SpSafeVal = SpSafe(Vr_PFL, Ts, ac, C_SSM, Zd, Zr)
Spfull = SpMax(vrmax, Vr, Tr, Ts, ac, C_SSM, Zd, Zr)
SpminVal = Spmin(C_SSM, Zd, Zr)


if __name__ == '__main__':
    server = Job()
    server.start()
    # Device connection
    fpsReader = cvzone.FPS()
    cap = cv2.VideoCapture(2)
    cap.set(3, 640)  # width
    cap.set(4, 480)  # height

    detector = FaceMeshDetector(maxFaces=1)
    detectorPose = PoseDetector()
    with open(write_file, "wt", encoding="utf-8") as output:
    #Record data csv opening
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
                    # # print("==========")
                    # # print("lebar pixel ", chestDistance)
                    # print("Real Chest Distance", real_measurement)

                    #D = eye_dist
                    D = min(d, real_measurement)
                    print("Jarak dari Loop", D)
                    data.append(D)

                # Calculate the average
                D = sum(data) / len(data)

                # Distance Calibration results can be integrated here

                D = round(D, 3)

                # read robot start time
                start_time = datetime.now()
                milliseconds = start_time.microsecond // 1000
                XnRob = [robotPos[0], robotPos[1], robotPos[2]]


                if D < 0:
                    D = 0
                else:
                    D = abs(D)
                cvzone.putTextRect(img, f'Depth: {D} mm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)

                # logical SSM send robot
                if D <= SpminVal:
                    server.pause()
                    print("Robot harus berhenti", vrstop)
                    mode_collab = 0
                    Vr = 1
                    # jacoRobot.setSpeed(Vr, vrot)


                elif D > SpminVal and D <= SpSafeVal:
                    server.resume()
                    print("Robot working on collaboration mode")
                    mode_collab = 1
                    Vr = Vr_SSM2(D, Tr, Ts, ac, C_SSM, Zd, Zr)
                    Vr = round(Vr, 2)
                    # jacoRobot.setSpeed(Vr, vrot)


                elif D > SpSafeVal and D <= SpPFLVal:
                    server.resume()
                    print("Robot speed reduction")
                    mode_collab = 2
                    Vr = Vr_PFL
                    Vr = round(Vr, 2)
                    # jacoRobot.setSpeed(Vr, vrot)


                elif D > SpPFLVal and D <= Spfull:
                    server.resume()
                    mode_collab = 3
                    Vr = Vr_SSM(D, Vh, Tr, Ts, ac, C_SSM, Zd, Zr, Vr_PFL)
                    Vr = round(Vr, 2)
                    # jacoRobot.setSpeed(Vr, vrot)

                else:
                    server.resume()
                    print("Robot bekerja maximal")
                    mode_collab = 4
                    Vr = RobotVrmax
                    # jacoRobot.setSpeed(Vr, vrot)

            interval = interval + 1

            #read robot position end time
            end_time = datetime.now()

            distance_traveled = mt.sqrt(
                (XnRobEnd[0] - XnRob[0]) ** 2 + (XnRobEnd[1] - XnRob[1]) ** 2 + (XnRobEnd[2] - XnRob[2]) ** 2)
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

            output.write(str(start_time.strftime("%H:%M:%S")) + ',' + str(elapsed_time) + ',' + str(D) + ',' + str(Vr) + '\n')
            # Load the saved plot image
            plot_img = cv2.imread('../temp_plot.png', cv2.IMREAD_UNCHANGED)

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


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
