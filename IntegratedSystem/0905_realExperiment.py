Powered by Anonymousemail → Join Us!

#1. copy assets folder
#2. Test the speed control the movement
#3.
#Robot library
import math
import time as t
import threading
from datetime import datetime
from fs100 import FS100

#camera library
import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from cvzone.FaceDetectionModule import FaceDetector
import cv2

#computation
import mediapipe as mp
import numpy as np
import math as mt
import csv

#GUI
import pygame

#====================INITIALIZE GUI=========================
# Initialize
pygame.init()

# Create Window/Display
width, height = 1280, 700
window = pygame.display.set_mode((width, height))
pygame.display.set_caption("Human Robot Safety Collaboration")

# Initialize Clock for FPS
fps = 30
clock = pygame.time.Clock()

#font and coloring
font = pygame.font.Font('assets/Inter-SemiBold.otf', 24)
font_big = pygame.font.Font('assets/Inter-SemiBold.otf', 48)
font_reg = pygame.font.Font('assets/Inter-Regular.otf', 24)
green, yellow, blue, red, purple, gray = (20, 128, 10), (236, 190, 35), (0, 103, 230), (197, 54, 55), (123, 97, 255), (229, 229, 229)
window.fill((255, 255, 255))

# ===layout interface===
pygame.draw.rect(window, gray, (667, 50, 590, 213), border_radius=5)
pygame.draw.rect(window, gray, (18, 513, 640, 172), border_radius=5)
pygame.draw.rect(window, gray, (667, 276, 590, 409), border_radius=5)
#mode collaboration
pygame.draw.rect(window, purple, (442, 523, 203, 152), border_radius=5)

#warning head and chest
pygame.draw.rect(window, purple, (1104, 414, 148, 127), border_radius=5)
pygame.draw.rect(window, gray, (1112, 444, 134, 95), border_radius=5)
text = font.render("WARNING!!!", True, (242, 242, 247))
window.blit(text, (1112, 414))

#img assets
imgKinova = pygame.image.load("assets/kinova.png").convert()
imgKinova = pygame.transform.scale(imgKinova, (166, 167))
window.blit(imgKinova, (693, 89))
imgHuman = pygame.image.load("assets/human.png").convert()
imgHuman = pygame.transform.scale(imgHuman, (361, 360))
window.blit(imgHuman, (693, 325))

# ===Title Text===
text = font.render("Safety Human Robot Collaboration", True, (50, 50, 50))
window.blit(text, (748, 13))



# ====Robot domain===
text_Robot = font.render("Robot Domain", True, (50, 50, 50))
window.blit(text_Robot, (693, 59))
text_RobotPos = font_reg.render("Robot Position", True, (50, 50, 50))
window.blit(text_RobotPos, (822, 89))
text_xRobot = font_reg.render("x:", True, (50, 50, 50))
window.blit(text_xRobot, (825, 119))
text_yRobot = font_reg.render("y:", True, (50, 50, 50))
window.blit(text_yRobot, (825, 156))
text_zRobot = font_reg.render("z:", True, (50, 50, 50))
window.blit(text_zRobot, (825, 194))
text_rmRobot = font_reg.render("Robot Movement", True, (50, 50, 50))
window.blit(text_rmRobot, (1006, 89))
text_spRobot = font_reg.render("Speed:", True, (50, 50, 50))
window.blit(text_spRobot, (1006, 125))
text_vrRobot = font_reg.render("Vr:", True, (50, 50, 50))
window.blit(text_vrRobot, (1006, 156))

# ====Human domain===
text_Human = font.render("Human Domain", True, (50, 50, 50))
window.blit(text_Human, (693, 292))
text_mvHuman = font_reg.render("Human Movement", True, (50, 50, 50))
window.blit(text_mvHuman, (987, 292))
text_fHuman = font_reg.render("FACE", True, (50, 50, 50))
window.blit(text_fHuman, (860, 329))
text_fminHuman = font_reg.render("max:", True, (50, 50, 50))
window.blit(text_fminHuman, (860, 358))
text_fmaxHuman = font_reg.render("min:", True, (50, 50, 50))
window.blit(text_fmaxHuman, (860, 391))
text_cHuman = font_reg.render("CHEST", True, (50, 50, 50))
window.blit(text_cHuman, (860, 435))
text_cminHuman = font_reg.render("max:", True, (50, 50, 50))
window.blit(text_cminHuman, (860, 472))
text_cmaxHuman = font_reg.render("min:", True, (50, 50, 50))
window.blit(text_cmaxHuman, (860, 505))
text_vhHuman = font_reg.render("Vh:", True, (50, 50, 50))
window.blit(text_vhHuman, (987, 329))


# ====SSM Information===
text_SSMInf = font.render("SSM Information", True, (50, 50, 50))
window.blit(text_SSMInf, (28, 519))
text_currDistance = font_reg.render("Current Distance:", True, (50, 50, 50))
window.blit(text_currDistance, (28, 555))
text_spSSM = font_reg.render("Sp:", True, (50, 50, 50))
window.blit(text_spSSM, (28, 584))
text_spminSSM = font_reg.render("Sp min:", True, (50, 50, 50))
window.blit(text_spminSSM, (28, 616))
text_vrMax = font_reg.render("Vr Max:", True, (50, 50, 50))
window.blit(text_vrMax, (28, 646))


# ===mode collaboration===
text_mode = font_reg.render("Mode", True, (242, 242, 247))
window.blit(text_mode, (511, 526))
#robot task
pygame.draw.rect(window, purple, (913, 547, 339, 127), border_radius=5)
pygame.draw.rect(window, gray, (1077, 557, 166, 110), border_radius=5)

text_Rtask = font.render("Robot Task", True, (242, 242, 247))
window.blit(text_Rtask, (929, 557))

pygame.draw.rect(window, purple, (808, 549, 99, 125), border_radius=5)
pygame.draw.rect(window, gray, (816, 585, 84, 82), border_radius=5)
text_titcounter = font_reg.render("COUNT", True, (242, 242, 247))
window.blit(text_titcounter, (812, 553))



# =================function SSM =============================
def SSM_calculation(Vr, Vh, Tr, ac, C, Zd, Zr):
    Tb = Vr / ac
    Ss  = pow(Vr, 2) / (2*ac)
    Ctot = C + Zd + Zr
    Sp = Vh * ( Tr + Tb ) + (Vr * Tr) + Ss + Ctot
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

def Vr_max(Sp, Vh, Vr, Tr, ac, C, Zd, Zr):
    Ts = Vr / ac
    T = Tr + Ts
    Ctot = C + Zd + Zr
    Vrmax = ((Sp - (Vh*T) - Ctot) / T) - (ac*pow(Ts, 2)/(2*T))
    return Vrmax

def Spmin(Vh, Tr, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    SpminVal = Vh * Tr + Ctot
    return SpminVal

# ===== Robot Function =====
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

def move_convert(post_original):
    x = post_original[0] * 1000
    y = post_original[1] * 1000
    z = post_original[2] * 1000
    rx = post_original[3] * 10000
    ry = post_original[4] * 10000
    rz = post_original[5] * 10000
    re = post_original[0] * 10000
    post_robot = (x, y, z, rx, ry, rz, re)
    return post_robot

def move_distance(post1, post2):
    print("nilai post akhir", post2[2])
    print("nilai post awal", post1[2])
    x_coor = post2[0] - post1[0]
    y_coor = post2[1] - post1[1]
    z_coor = post2[2] - post1[2]
    rx_coor = post2[3] - post1[3]
    ry_coor = post2[4] - post1[4]
    rz_coor = post2[5] - post1[5]
    re_coor = post2[6] - post1[6]

    dist = math.sqrt(math.pow((post2[0] - post1[0]), 2) + math.pow((post2[1] - post1[1]), 2) + math.pow((post2[2] - post1[2]), 2))

    move_coor = (int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor))
    return move_coor, int(dist)

def time_robot(speed, distance, delay_rob):
    distance = distance / 1000
    speed = speed / 10
    time_move = (distance / speed) + delay_rob
    return time_move

def rob_command(post1):
    x_coor = post1[0] * 1000
    y_coor = post1[1] * 1000
    z_coor = post1[2] * 1000
    #rx_coor = post1[3] * 1000
    #ry_coor = post1[4] * 1000
    #rz_coor = post1[5] * 1000
    #re_coor = post1[6] * 1000

    #robot_command = (int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor))
    robot_command = (int(x_coor), int(y_coor), int(z_coor), 0, 0, 0, 0)
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

# ===== initialization & variables declaration =====
#SSM variables
Vrinitial = 200
Vr = Vrinitial
Vh = 1600
Tr = 0.41
ac = 200
C = 1200
Zd = 90
Zr = 25

#velocity
Xn1D = 0
Xn_last1D = 0
Xn = [0, 0, 0]
Xn_last = [0, 0, 0]
velX, velY, velZ = [0, 0, 0]
ts = 0.05

#velocity human
velHum = 1600

XnRob = [0, 0, 0]
XnRob_last = [0, 0, 0]
velXR, velYR, velZR = [0, 0, 0]
#Sp = SSM_calculation(Vr, Vh, Tr, ac, C, Zd, Zr)
interval = 0
Vrmax = 0
Vr_max_command = 0

#Robot Velocity
vrchest = 100
vrface = 60
vrstop = 0
vrmax = 200
vrot = 90
velRob = 0
robotZ = 0
vel = 0
RobotVrmax = 200


distView = 0
sampleDistance = 1
pause_active = 0

Spmax = SSM_calculation(Vrinitial, Vh, Tr, ac, C, Zd, Zr)
Sp = Spmax
Scurrent = Spmax + 1000
SpminInitial = Spmin(Vh, Tr, ac, C, Zd, Zr)
SpminVal = SpminInitial
#calibration position variable
zHead = [0, 0]
zChest = [0, 0]
RobTablePos = [0, 0, 0]

#distance measurement
#x shoulder in cm
#y real measurement
x = [-2, -4, -9, -15, -17, -23, -38, -50, -76, -80]
y = [150, 140, 130, 120, 110, 70, 60, 50, 40, 30]

coff = np.polyfit(x, y, 2)
A, B, C = coff

xHeight = [0.2707, 0.2799, 0.2703, 0.2716, 0.29167, 0.634, 0.634, 0.6299, 0.6025, 0.6072, 0.1201, 0.1188, 0.0990, 0.1237, 0.1666]
yHeight = [140, 140, 140, 140, 140, 90, 90, 90, 90, 90, 150, 150, 150, 150, 150]

coffHeight = np.polyfit(xHeight, yHeight, 2)
Ah, Bh, Ch = coffHeight

Anose, Bnose, Cnose = -0.0001612723204521041, -0.23012162009342785, 180.10792676079961


#initialization
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

noseRAW = 0
midshoulderRAW = 0
midHipsRAW = 0

#information
write_file = "SSMdata_Analysis.csv"
mode_collab = 0

#SSM original data
VrOriSSM = 0
mode_SSMori = 0


# ===== Yaskawa Connect Robot =====
# robot connection
robot = FS100('192.168.255.1')
stop_sign = threading.Semaphore()

# Initialize the robot model
pos_info = {}
robot_no = 1
status = {}
speed = 0
x, y, z, rx, ry, rz, re = 0, 0, 0, 0, 0, 0, 0
delay_rob = 0.1
# ===== Movement Position List =====
pointHome = 0
# point 1 <-- home
pointHomeA = [445.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
pointHomeB = [435.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
pointHomeC = [425.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
pointHomeD = [415.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
# point 1 down to point 2
point1 = [405.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]

point121 = [405.916, -387.580, 320.821, 178.6969, -0.0029, -0.0306, 0]
point122 = [405.916, -387.580, 295.821, 178.6969, -0.0029, -0.0306, 0]
point123 = [405.916, -387.580, 270.821, 178.6969, -0.0029, -0.0306, 0]
point124 = [405.916, -387.580, 245.821, 178.6969, -0.0029, -0.0306, 0]
point125 = [405.916, -387.580, 220.821, 178.6969, -0.0029, -0.0306, 0]
point126 = [405.916, -387.580, 195.821, 178.6969, -0.0029, -0.0306, 0]
point127 = [405.916, -387.580, 170.821, 178.6969, -0.0029, -0.0306, 0]
point128 = [405.916, -387.580, 145.821, 178.6969, -0.0029, -0.0306, 0]
point129 = [405.916, -387.580, 120.821, 178.6969, -0.0029, -0.0306, 0]
point1210 = [405.916, -387.580, 95.821, 178.6969, -0.0029, -0.0306, 0]
point1211 = [405.916, -387.580, 70.821, 178.6969, -0.0029, -0.0306, 0]
point1212 = [405.916, -387.580, 45.821, 178.6969, -0.0029, -0.0306, 0]
point1213 = [405.916, -387.580, 20.821, 178.6969, -0.0029, -0.0306, 0]
point1214 = [405.916, -387.580, -4.179, 178.6969, -0.0029, -0.0306, 0]
point1215 = [405.916, -387.580, -29.179, 178.6969, -0.0029, -0.0306, 0]
point1216 = [405.916, -387.580, -54.179, 178.6969, -0.0029, -0.0306, 0]
point1217 = [405.916, -387.580, -79.179, 178.6969, -0.0029, -0.0306, 0]
point1218 = [405.916, -387.580, -104.179, 178.6969, -0.0029, -0.0306, 0]
point1219 = [405.916, -387.580, -129.179, 178.6969, -0.0029, -0.0306, 0]
point1220 = [405.916, -387.580, -154.179, 178.6969, -0.0029, -0.0306, 0]
point1221 = [405.916, -387.580, -179.179, 178.6969, -0.0029, -0.0306, 0]

point2 = [405.929, -387.592, -201.391, 178.7000, 0.0002, -0.0261, 0]
#point 2 up to point 3 same as point 1 backfrom down to up

#point 1 goes to point 4 --> change y
point141 = [405.916, -362.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point142 = [405.916, -337.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point143 = [405.916, -312.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point144 = [405.916, -287.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point145 = [405.916, -262.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point146 = [405.916, -237.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point147 = [405.916, -212.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point148 = [405.916, -187.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point149 = [405.916, -162.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1410 = [405.916, -137.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1411 = [405.916, -112.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1412 = [405.916, -87.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1413 = [405.916, -62.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1414 = [405.916, -37.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1415 = [405.916, -12.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1416 = [405.916, 12.420, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1417 = [405.916, 37.420, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1418 = [405.916, 62.420, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1419 = [405.916, 87.420, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1420 = [405.916, 112.420, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1421 = [405.916, 137.420, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1422 = [405.916, 162.420, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1423 = [405.916, 187.420, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1424 = [405.916, 212.420, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1425 = [405.916, 237.420, 345.821, 178.6969, -0.0029, -0.0306, 0]

point4 = [405.919, 253.778, 345.819, 178.6967, -0.0031, -0.0308, 0]

point451 = [405.919, 253.778, 320.819, 178.6967, -0.0031, -0.0308, 0]
point452 = [405.919, 253.778, 295.819, 178.6967, -0.0031, -0.0308, 0]
point453 = [405.919, 253.778, 270.819, 178.6967, -0.0031, -0.0308, 0]
point454 = [405.919, 253.778, 245.819, 178.6967, -0.0031, -0.0308, 0]
point455 = [405.919, 253.778, 220.819, 178.6967, -0.0031, -0.0308, 0]
point456 = [405.919, 253.778, 195.819, 178.6967, -0.0031, -0.0308, 0]
point457 = [405.919, 253.778, 170.819, 178.6967, -0.0031, -0.0308, 0]
point458 = [405.919, 253.778, 145.819, 178.6967, -0.0031, -0.0308, 0]
point459 = [405.919, 253.778, 120.819, 178.6967, -0.0031, -0.0308, 0]
point4510 = [405.919, 253.778, 95.819, 178.6967, -0.0031, -0.0308, 0]
point4511 = [405.919, 253.778, 70.819, 178.6967, -0.0031, -0.0308, 0]
point4512 = [405.919, 253.778, 45.819, 178.6967, -0.0031, -0.0308, 0]
point4513 = [405.919, 253.778, 20.819, 178.6967, -0.0031, -0.0308, 0]
point4514 = [405.919, 253.778, -4.819, 178.6967, -0.0031, -0.0308, 0]
point4515 = [405.919, 253.778, -29.819, 178.6967, -0.0031, -0.0308, 0]
point4516 = [405.919, 253.778, -54.819, 178.6967, -0.0031, -0.0308, 0]
point4517 = [405.919, 253.778, -79.819, 178.6967, -0.0031, -0.0308, 0]
point4518 = [405.919, 253.778, -104.819, 178.6967, -0.0031, -0.0308, 0]
point4519 = [405.919, 253.778, -129.819, 178.6967, -0.0031, -0.0308, 0]
point4520 = [405.919, 253.778, -154.819, 178.6967, -0.0031, -0.0308, 0]
point4521 = [405.919, 253.778, -179.819, 178.6967, -0.0031, -0.0308, 0]
#point 4 goes to point 5 --> change z
point5 = [405.932, 253.783, -201.391, 178.6961, -0.0045, -0.0303, 0]

#point 5 goes to point 6 (point 4) --> up chage z

#point6 goes to point 7 pointHomeA --> left change y
#point4 = [405.919, 253.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H1 = [425.919, 228.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H2 = [430.919, 203.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H3 = [425.919, 178.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H4 = [430.919, 153.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H5 = [435.919, 128.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H6 = [440.919, 103.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H7 = [445.919, 78.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H8 = [445.919, 53.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H9 = [445.919, 28.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H10 = [445.919, 3.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H11 = [445.919, -21.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H12 = [445.919, -46.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H13 = [445.919, -71.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H14 = [445.919, -96.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H15 = [445.919, -121.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H16 = [445.919, -146.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H17 = [445.919, -171.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H18 = [445.919, -196.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H19 = [445.919, -221.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H20 = [445.919, -246.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H21 = [445.919, -271.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H22 = [445.919, -296.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H23 = [445.919, -321.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H24 = [445.919, -346.222, 345.819, 178.6967, -0.0031, -0.0308, 0]
point4H25 = [445.919, -371.222, 345.819, 178.6967, -0.0031, -0.0308, 0]

#pointHomeA = [445.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]

progress = [0, 0, 0, 0]
finish = [1, 1, 1, 1]
start_time = datetime.now()

class Job(threading.Thread):
    def __init__(self, *args, **kwargs):
        super(Job, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()     # 用于暂停线程的标识
        self.__flag.set()       # 设置为True
        self.__running = threading.Event()      # 用于停止线程的标识
        self.__running.set()      # 将running设置为True

    def run(self):

        # set speed
        SPEED_XYZ = (10, 150, 200)
        SPEED_R_XYZE = (10, 50, 100)

        speed_class = FS100.MOVE_SPEED_CLASS_MILLIMETER
        speed = SPEED_XYZ[2]

        # ==counter position==
        counter = 0

        pygame.draw.rect(window, purple, (929, 602, 140, 29), border_radius=5)
        text_process = font_reg.render("PROCESS", True, (242, 242, 247))
        window.blit(text_process, (940, 600))
        pygame.draw.rect(window, gray, (1119, 562, 99, 99), border_radius=5)
        imgPro = pygame.image.load("assets/process.png").convert()
        imgPro = pygame.transform.scale(imgPro, (99, 99))
        window.blit(imgPro, (1119, 562))

        pygame.draw.rect(window, gray, (816, 585, 84, 82), border_radius=5)
        text_fillcounter = font_big.render("0", True, (50, 50, 50))
        window.blit(text_fillcounter, (836, 590))

        while self.__running.is_set():

            # Read initial position
            if FS100.ERROR_SUCCESS == robot.read_position(pos_info, robot_no):
                x, y, z, rx, ry, rz, re = pos_info['pos']
                pointHome = (x, y, z, 0, 0, 0, 0)
                straaa = "CURRENT POSITION\n" + \
                      "COORDINATE {:12s} TOOL:{:02d}\n".format('ROBOT', pos_info['tool_no']) + \
                      "R{} :X     {:4d}.{:03d} mm       Rx   {:4d}.{:04d} deg.\n".format(robot_no,
                                                                                         x // 1000, x % 1000,
                                                                                         rx // 10000,
                                                                                         rx % 10000) + \
                      "    Y     {:4d}.{:03d} mm       Ry   {:4d}.{:04d} deg.\n".format(
                          y // 1000, y % 1000, ry // 10000, ry % 10000) + \
                      "    Z     {:4d}.{:03d} mm       Rz   {:4d}.{:04d} deg.\n".format(
                          z // 1000, z % 1000, rz // 10000, rz % 10000) + \
                      "                            Re   {:4d}.{:04d} deg.\n".format(
                          re // 10000, re % 10000)

            print(straaa)
            # ===== convert robot command =====
            robHome = pointHome
            robHome1 = rob_command(pointHomeA)
            robHome2 = rob_command(pointHomeB)
            robHome3 = rob_command(pointHomeC)
            robHome4 = rob_command(pointHomeD)

            rob1 = rob_command(point1)

            rob121 = rob_command(point121)
            rob122 = rob_command(point122)
            rob123 = rob_command(point123)
            rob124 = rob_command(point124)
            rob125 = rob_command(point125)
            rob126 = rob_command(point126)
            rob127 = rob_command(point127)
            rob128 = rob_command(point128)
            rob129 = rob_command(point129)
            rob1210 = rob_command(point1210)
            rob1211 = rob_command(point1211)
            rob1212 = rob_command(point1212)
            rob1213 = rob_command(point1213)
            rob1214 = rob_command(point1214)
            rob1215 = rob_command(point1215)
            rob1216 = rob_command(point1216)
            rob1217 = rob_command(point1217)
            rob1218 = rob_command(point1218)
            rob1219 = rob_command(point1219)
            rob1220 = rob_command(point1220)
            rob1221 = rob_command(point1221)

            rob2 = rob_command(point2)

            rob141 = rob_command(point141)
            rob142 = rob_command(point142)
            rob143 = rob_command(point143)
            rob144 = rob_command(point144)
            rob145 = rob_command(point145)
            rob146 = rob_command(point146)
            rob147 = rob_command(point147)
            rob148 = rob_command(point148)
            rob149 = rob_command(point149)
            rob1410 = rob_command(point1410)
            rob1411 = rob_command(point1411)
            rob1412 = rob_command(point1412)
            rob1413 = rob_command(point1413)
            rob1414 = rob_command(point1414)
            rob1415 = rob_command(point1415)
            rob1416 = rob_command(point1416)
            rob1417 = rob_command(point1417)
            rob1418 = rob_command(point1418)
            rob1419 = rob_command(point1419)
            rob1420 = rob_command(point1420)
            rob1421 = rob_command(point1421)
            rob1422 = rob_command(point1422)
            rob1423 = rob_command(point1423)
            rob1424 = rob_command(point1424)
            rob1425 = rob_command(point1425)


            rob4 = rob_command(point4)

            rob451 = rob_command(point451)
            rob452 = rob_command(point452)
            rob453 = rob_command(point453)
            rob454 = rob_command(point454)
            rob455 = rob_command(point455)
            rob456 = rob_command(point456)
            rob457 = rob_command(point457)
            rob458 = rob_command(point458)
            rob459 = rob_command(point459)
            rob4510 = rob_command(point4510)
            rob4511 = rob_command(point4511)
            rob4512 = rob_command(point4512)
            rob4513 = rob_command(point4513)
            rob4514 = rob_command(point4514)
            rob4515 = rob_command(point4515)
            rob4516 = rob_command(point4516)
            rob4517 = rob_command(point4517)
            rob4518 = rob_command(point4518)
            rob4519 = rob_command(point4519)
            rob4520 = rob_command(point4520)
            rob4521 = rob_command(point4521)

            rob5 = rob_command(point5)

            rob4H1 = rob_command(point4H1)
            rob4H2 = rob_command(point4H2)
            rob4H3 = rob_command(point4H3)
            rob4H4 = rob_command(point4H4)
            rob4H5 = rob_command(point4H5)
            rob4H6 = rob_command(point4H6)
            rob4H7 = rob_command(point4H7)
            rob4H8 = rob_command(point4H8)
            rob4H9 = rob_command(point4H9)
            rob4H10 = rob_command(point4H10)
            rob4H11 = rob_command(point4H11)
            rob4H12 = rob_command(point4H12)
            rob4H13 = rob_command(point4H13)
            rob4H14 = rob_command(point4H14)
            rob4H15 = rob_command(point4H15)
            rob4H16 = rob_command(point4H16)
            rob4H17 = rob_command(point4H17)
            rob4H18 = rob_command(point4H18)
            rob4H19 = rob_command(point4H19)
            rob4H20 = rob_command(point4H20)
            rob4H21 = rob_command(point4H21)
            rob4H22 = rob_command(point4H22)
            rob4H23 = rob_command(point4H23)
            rob4H24 = rob_command(point4H24)
            rob4H25 = rob_command(point4H25)

            # ===== move and distance =========
            # move from home to point 1
            #post1_move, distance1 = move_distance(robHome, rob1)
            #post2_move, distance2 = move_distance(rob1, rob2)

            postH1_move, distanceH1 = move_distance(robHome, robHome1)
            postH2_move, distanceH2 = move_distance(robHome2, robHome3)
            postH3_move, distanceH3 = move_distance(robHome3, robHome4)

            post1_move, distance1 = move_distance(robHome4, rob1)

            # move from point 1 to point 2 -> move down

            post121_move, distance121 = move_distance(rob1, rob121)
            post122_move, distance122 = move_distance(rob121, rob122)
            post123_move, distance123 = move_distance(rob122, rob123)
            post124_move, distance124 = move_distance(rob123, rob124)
            post125_move, distance125 = move_distance(rob124, rob125)
            post126_move, distance126 = move_distance(rob125, rob126)
            post127_move, distance127 = move_distance(rob126, rob127)
            post128_move, distance128 = move_distance(rob127, rob128)
            post129_move, distance129 = move_distance(rob128, rob129)
            post1210_move, distance1210 = move_distance(rob129, rob1210)
            post1211_move, distance1211 = move_distance(rob1210, rob1211)
            post1212_move, distance1212= move_distance(rob1211, rob1212)
            post1213_move, distance1213 = move_distance(rob1212, rob1213)
            post1214_move, distance1214 = move_distance(rob1213, rob1214)
            post1215_move, distance1215 = move_distance(rob1214, rob1215)
            post1216_move, distance1216 = move_distance(rob1215, rob1216)
            post1217_move, distance1217 = move_distance(rob1216, rob1217)
            post1218_move, distance1218 = move_distance(rob1217, rob1218)
            post1219_move, distance1219 = move_distance(rob1218, rob1219)
            post1220_move, distance1220 = move_distance(rob1219, rob1220)
            post1221_move, distance1221 = move_distance(rob1220, rob1221)

            # move from point 2 to point 3(1) -> move up
            post231_move, distance231 = move_distance(rob1221, rob1220)
            post232_move, distance232 = move_distance(rob1220, rob1219)
            post233_move, distance233 = move_distance(rob1219, rob1218)
            post234_move, distance234 = move_distance(rob1218, rob1217)
            post235_move, distance235 = move_distance(rob1217, rob1216)
            post236_move, distance236 = move_distance(rob1216, rob1215)
            post237_move, distance237 = move_distance(rob1215, rob1214)
            post238_move, distance238 = move_distance(rob1214, rob1213)
            post239_move, distance239 = move_distance(rob1213, rob1212)
            post2310_move, distance2310 = move_distance(rob1212, rob1211)
            post2311_move, distance2311 = move_distance(rob1211, rob1210)
            post2312_move, distance2312 = move_distance(rob1210, rob129)
            post2313_move, distance2313 = move_distance(rob129, rob128)
            post2314_move, distance2314 = move_distance(rob128, rob127)
            post2315_move, distance2315 = move_distance(rob127, rob126)
            post2316_move, distance2316 = move_distance(rob126, rob125)
            post2317_move, distance2317 = move_distance(rob125, rob124)
            post2318_move, distance2318 = move_distance(rob124, rob123)
            post2319_move, distance2319 = move_distance(rob123, rob122)
            post2320_move, distance2320 = move_distance(rob122, rob121)
            post2321_move, distance2321 = move_distance(rob121, rob1)


            # move position 3(1) to pos 4 -> move right
            post141_move, distance141 = move_distance(rob1, rob141)
            post142_move, distance142 = move_distance(rob141, rob142)
            post143_move, distance143 = move_distance(rob142, rob143)
            post144_move, distance144 = move_distance(rob143, rob144)
            post145_move, distance145 = move_distance(rob144, rob145)
            post146_move, distance146 = move_distance(rob145, rob146)
            post147_move, distance147 = move_distance(rob146, rob147)
            post148_move, distance148 = move_distance(rob147, rob148)
            post149_move, distance149 = move_distance(rob148, rob149)
            post1410_move, distance1410 = move_distance(rob149, rob1410)
            post1411_move, distance1411 = move_distance(rob1410, rob1411)
            post1412_move, distance1412= move_distance(rob1411, rob1412)
            post1413_move, distance1413 = move_distance(rob1412, rob1413)
            post1414_move, distance1414 = move_distance(rob1413, rob1414)
            post1415_move, distance1415 = move_distance(rob1414, rob1415)
            post1416_move, distance1416 = move_distance(rob1415, rob1416)
            post1417_move, distance1417 = move_distance(rob1416, rob1417)
            post1418_move, distance1418 = move_distance(rob1417, rob1418)
            post1419_move, distance1419 = move_distance(rob1418, rob1419)
            post1420_move, distance1420 = move_distance(rob1419, rob1420)
            post1421_move, distance1421 = move_distance(rob1420, rob1421)
            post1422_move, distance1422 = move_distance(rob1421, rob1422)
            post1423_move, distance1423 = move_distance(rob1422, rob1423)
            post1424_move, distance1424 = move_distance(rob1423, rob1424)
            post1425_move, distance1425 = move_distance(rob1424, rob1425)

            post4_move, distance4 = move_distance(rob1425, rob4)

            # move down from point 4 to point 5 -> move down
            post451_move, distance451 = move_distance(rob4, rob451)
            post452_move, distance452 = move_distance(rob451, rob452)
            post453_move, distance453 = move_distance(rob452, rob453)
            post454_move, distance454 = move_distance(rob453, rob454)
            post455_move, distance455 = move_distance(rob454, rob455)
            post456_move, distance456 = move_distance(rob455, rob456)
            post457_move, distance457 = move_distance(rob456, rob457)
            post458_move, distance458 = move_distance(rob457, rob458)
            post459_move, distance459 = move_distance(rob458, rob459)
            post4510_move, distance4510 = move_distance(rob459, rob4510)
            post4511_move, distance4511 = move_distance(rob4510, rob4511)
            post4512_move, distance4512= move_distance(rob4511, rob4512)
            post4513_move, distance4513 = move_distance(rob4512, rob4513)
            post4514_move, distance4514 = move_distance(rob4513, rob4514)
            post4515_move, distance4515 = move_distance(rob4514, rob4515)
            post4516_move, distance4516 = move_distance(rob4515, rob4516)
            post4517_move, distance4517 = move_distance(rob4516, rob4517)
            post4518_move, distance4518 = move_distance(rob4517, rob4518)
            post4519_move, distance4519 = move_distance(rob4518, rob4519)
            post4520_move, distance4520 = move_distance(rob4519, rob4520)
            post4521_move, distance4521 = move_distance(rob4520, rob4521)


            # move point 5 to point 6(4) -> move up
            post561_move, distance561 = move_distance(rob4521, rob4520)
            post562_move, distance562 = move_distance(rob4520, rob4519)
            post563_move, distance563 = move_distance(rob4519, rob4518)
            post564_move, distance564 = move_distance(rob4518, rob4517)
            post565_move, distance565 = move_distance(rob4517, rob4516)
            post566_move, distance566 = move_distance(rob4516, rob4515)
            post567_move, distance567 = move_distance(rob4515, rob4514)
            post568_move, distance568 = move_distance(rob4514, rob4513)
            post569_move, distance569 = move_distance(rob4513, rob4512)
            post5610_move, distance5610 = move_distance(rob4512, rob4511)
            post5611_move, distance5611 = move_distance(rob4511, rob4510)
            post5612_move, distance5612 = move_distance(rob4510, rob459)
            post5613_move, distance5613 = move_distance(rob459, rob458)
            post5614_move, distance5614 = move_distance(rob458, rob457)
            post5615_move, distance5615 = move_distance(rob457, rob456)
            post5616_move, distance5616 = move_distance(rob456, rob455)
            post5617_move, distance5617 = move_distance(rob455, rob454)
            post5618_move, distance5618 = move_distance(rob454, rob453)
            post5619_move, distance5619 = move_distance(rob453, rob452)
            post5620_move, distance5620 = move_distance(rob452, rob451)
            post5621_move, distance5621 = move_distance(rob451, rob4)

            # move point 6(4) to point 7(near home) -> move left
            post7H1_move, distance7H1 = move_distance(rob4, rob4H1)
            post7H2_move, distance7H2 = move_distance(rob4H1, rob4H2)
            post7H3_move, distance7H3 = move_distance(rob4H2, rob4H3)
            post7H4_move, distance7H4 = move_distance(rob4H3, rob4H4)
            post7H5_move, distance7H5 = move_distance(rob4H4, rob4H5)
            post7H6_move, distance7H6 = move_distance(rob4H5, rob4H6)
            post7H7_move, distance7H7 = move_distance(rob4H6, rob4H7)
            post7H8_move, distance7H8 = move_distance(rob4H7, rob4H8)
            post7H9_move, distance7H9 = move_distance(rob4H8, rob4H9)
            post7H10_move, distance7H10 = move_distance(rob4H9, rob4H10)
            post7H11_move, distance7H11 = move_distance(rob4H10, rob4H11)
            post7H12_move, distance7H12= move_distance(rob4H11, rob4H12)
            post7H13_move, distance7H13 = move_distance(rob4H12, rob4H3)
            post7H14_move, distance7H14 = move_distance(rob4H13, rob4H14)
            post7H15_move, distance7H15 = move_distance(rob4H14, rob4H15)
            post7H16_move, distance7H16 = move_distance(rob4H15, rob4H16)
            post7H17_move, distance7H17 = move_distance(rob4H16, rob4H17)
            post7H18_move, distance7H18 = move_distance(rob4H17, rob4H18)
            post7H19_move, distance7H19 = move_distance(rob4H18, rob4H19)
            post7H20_move, distance7H20 = move_distance(rob4H19, rob4H20)
            post7H21_move, distance7H21 = move_distance(rob4H20, rob4H21)
            post7H22_move, distance7H22 = move_distance(rob4H21, rob4H22)
            post7H23_move, distance7H23 = move_distance(rob4H22, rob4H23)
            post7H24_move, distance7H24 = move_distance(rob4H23, rob4H24)
            post7H25_move, distance7H25 = move_distance(rob4H24, rob4H25)

            # servo on check
            if FS100.ERROR_SUCCESS == robot.get_status(status):
                if not status['servo_on']:
                    robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

            # ===== list movement task ========
            pos_updater = threading.Thread(target=update_pos)
            index = 0
            tredON = False

            postMove = [postH1_move, postH2_move, postH3_move, post1_move, post121_move, post122_move, post123_move, post124_move, post125_move, post126_move, post127_move, post128_move, post129_move, post1210_move,
                        post1211_move, post1212_move, post1213_move, post1214_move, post1215_move, post1216_move, post1217_move, post1218_move, post1219_move, post1220_move, post1221_move,
                        post231_move, post232_move, post233_move, post234_move, post235_move, post236_move,
                        post237_move, post238_move, post239_move, post2310_move,
                        post2311_move, post2312_move, post2313_move, post2314_move, post2315_move, post2316_move,
                        post2317_move, post2318_move, post2319_move, post2320_move, post2321_move,
                        post141_move, post142_move, post143_move, post144_move, post145_move, post146_move,
                        post147_move, post148_move, post149_move, post1410_move,
                        post1411_move, post1412_move, post1413_move, post1414_move, post1415_move, post1416_move,
                        post1417_move, post1418_move, post1419_move, post1420_move, post1421_move, post1422_move, post1423_move, post1424_move, post1425_move, post4_move,
                        post451_move, post452_move, post453_move, post454_move, post455_move, post456_move,
                        post457_move, post458_move, post459_move, post4510_move,
                        post4511_move, post4512_move, post4513_move, post4514_move, post4515_move, post4516_move,
                        post4517_move, post4518_move, post4519_move, post4520_move, post4521_move,
                        post561_move, post562_move, post563_move, post564_move, post565_move, post566_move,
                        post567_move, post568_move, post569_move, post5610_move,
                        post5611_move, post5612_move, post5613_move, post5614_move, post5615_move, post5616_move,
                        post5617_move, post5618_move, post5619_move, post5620_move, post5621_move,
                        post7H1_move, post7H2_move, post7H3_move, post7H4_move, post7H5_move, post7H6_move,
                        post7H7_move, post7H8_move, post7H9_move, post7H10_move,
                        post7H11_move, post7H12_move, post7H13_move, post7H14_move, post7H15_move, post7H16_move,
                        post7H17_move, post7H18_move, post7H19_move, post7H20_move, post7H21_move, post7H22_move,
                        post7H23_move, post7H24_move, post7H25_move
                        ]
            dist = [
                    distanceH1, distanceH2, distanceH3, distance1, distance121, distance122, distance123, distance124,
                    distance125, distance126, distance127, distance128, distance129, distance1210,
                    distance1211, distance1212, distance1213, distance1214, distance1215, distance1216, distance1217,
                    distance1218, distance1219, distance1220, distance1221,
                    distance231, distance232, distance233, distance234, distance235, distance236,
                    distance237, distance238, distance239, distance2310,
                    distance2311, distance2312, distance2313, distance2314, distance2315, distance2316,
                    distance2317, distance2318, distance2319, distance2320, distance2321,
                    distance141, distance142, distance143, distance144, distance145, distance146,
                    distance147, distance148, distance149, distance1410,
                    distance1411, distance1412, distance1413, distance1414, distance1415, distance1416,
                    distance1417, distance1418, distance1419, distance1420, distance1421, distance1422, distance1423,
                    distance1424, distance1425, distance4,
                    distance451, distance452, distance453, distance454, distance455, distance456,
                    distance457, distance458, distance459, distance4510,
                    distance4511, distance4512, distance4513, distance4514, distance4515, distance4516,
                    distance4517, distance4518, distance4519, distance4520, distance4521,
                    distance561, distance562, distance563, distance564, distance565, distance566,
                    distance567, distance568, distance569, distance5610,
                    distance5611, distance5612, distance5613, distance5614, distance5615, distance5616,
                    distance5617, distance5618, distance5619, distance5620, distance5621,
                    distance7H1, distance7H2, distance7H3, distance7H4, distance7H5, distance7H6,
                    distance7H7, distance7H8, distance7H9, distance7H10,
                    distance7H11, distance7H12, distance7H13, distance7H14, distance7H15, distance7H16,
                    distance7H17, distance7H18, distance7H19, distance7H20, distance7H21, distance7H22,
                    distance7H23, distance7H24, distance7H25

                     ]


            for i in postMove:
                self.__flag.wait()
                time_d = time_robot(speed, dist[index], delay_rob)
                print(time_d)
                print("nilai x yang masuk ", index, "sebesar ", i)
                if FS100.ERROR_SUCCESS == robot.one_move(FS100.MOVE_TYPE_LINEAR_INCREMENTAL_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT, speed_class, speed, i):
                    t.sleep(time_d)  # robot may not update the status
                    if not is_alarmed() and tredON == False:
                        pos_updater.start()
                        tredON = True
                index = index + 1
                print("Finished step ", index)

            counter = counter + 1
            ## counter information
            print("Robot counter: ", counter)
            pygame.draw.rect(window, gray, (816, 585, 84, 82), border_radius=5)
            text_fillcounter = font_big.render(str(counter), True, (50, 50, 50))
            window.blit(text_fillcounter, (836, 590))

            if counter == 2:
                finish_task = datetime.now() - start_time
                finish_task = str(finish_task)
                print(datetime.now() - start_time)
                pygame.draw.rect(window, gray, (1119, 562, 99, 99), border_radius=5)
                imgSuc = pygame.image.load("assets/success.png").convert()
                imgSuc = pygame.transform.scale(imgSuc, (99, 99))
                window.blit(imgSuc, (1119, 562))
                pygame.draw.rect(window, purple, (929, 602, 140, 29), border_radius=5)
                text_process = font_reg.render("FINISH", True, (242, 242, 247))
                window.blit(text_process, (940, 600))
                pygame.draw.rect(window, purple, (916, 638, 157, 29), border_radius=5)
                text_process = font_reg.render(finish_task[0:9], True, (242, 242, 247))
                window.blit(text_process, (922, 638))
                break

        robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_ON)
        # a hold off in case we switch to teach/play mode
        robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_OFF)


    def pause(self):
        self.__flag.clear()     # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()    # 设置为True, 让线程停止阻塞

    def stop(self):
        self.__flag.set()       # 将线程从暂停状态恢复, 如何已经暂停的话
        self.__running.clear()        # 设置为False


if __name__ == '__main__':
    server = Job()
    server.start()
    # MAIN PRORGAM:
    # ===== camera installation =====
    fpsReader = cvzone.FPS()
    cap = cv2.VideoCapture(0)
    cap.set(3, 640)  # width
    cap.set(4, 480)  # height

    detector = FaceMeshDetector(maxFaces=1)
    detectFace = FaceDetector()
    #print("Nilai S Current adalah ", Scurrent)
    # ======================================================
    # masukkan program utama disini (looping program)
    with open(write_file, "wt", encoding="utf-8") as output:
        while True:
            # Get Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    start = False
                    pygame.quit()

    # ================= Apply Logic =================
            # Detect human skeleton
            success, img = cap.read()
            imgMesh, faces = detector.findFaceMesh(img, draw=False)
            imgFace, bboxs = detectFace.findFaces(img)

            # === Robot analysis Velocity ===
            # Read initial position
            if FS100.ERROR_SUCCESS == robot.read_position(pos_info, robot_no):
                x, y, z, rx, ry, rz, re = pos_info['pos']
                #pointHome = (x, y, z, 0, 0, 0, 0)

            curRobotPos = convert_mm(x, y, z, rx, ry, rz, re)
            #print("robot position ", curRobotPos[0], curRobotPos[1], curRobotPos[2])
            xRob = round(curRobotPos[0], 2)
            yRob = round(curRobotPos[1], 2)
            zRob = round(curRobotPos[2], 2)
            XnRob = [xRob, yRob, zRob]

            xTrob = round(xRob + 900, 2)
            yTrob = round(yRob + 900, 2)
            zTrob = round(zRob + 900, 2)
            RobTablePos = [xTrob, yTrob, zTrob]
            #print("Robot Position X Y Z: ", xRob, yRob, zRob)
            #print("Robot Last Position X Y Z: ", XnRob_last[0], XnRob_last[1], XnRob_last[2])
            velR = velXYZ(XnRob, XnRob_last, ts)
            #print("Robot velocity X Y Z: ", velR[0], velR[1], velR[2])
            # velR[0] = round(velR[0], 2)
            # velR[1] = round(velR[1], 2)
            # velR[2] = round(velR[2], 2)
            VelRnew = math.sqrt(velR[0]**2 + velR[1]**2 + velR[2]**2)
            VelRnew = abs(VelRnew)
            if VelRnew > RobotVrmax:
                VelRnew = RobotVrmax
            #print("Robot average velocity", VelRnew)
            SpStatis = SSM_calculation(Vrinitial, Vh, Tr, ac, C, Zd, Zr)
            #print("SSM Statis", SpStatis)
            # ===== SSM calculation ======
            Sp = SSM_calculation(VelRnew, velHum, Tr, ac, C, Zd, Zr)
            #print("SSM Dynamic", Sp)

            if faces:
                with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
                    # skeleton detection
                    face = faces[0]
                    # print(faces[0])
                    pointLeft = face[145]
                    pointRight = face[374]
                    w, _ = detector.findDistance(pointLeft, pointRight)
                    W = 6.3  # default real width eyes distance
                    f = 714  # finding the average for focal length
                    d = (W * f) / w
                    d = d * 10  # distance in mm
                    eye_dist = round(d, 3)

                    # skeleton mediapipe migrasion
                    # Recolor image to RGB
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img.flags.writeable = False
                    # Make detection
                    results = pose.process(img)
                    # Recolor back to BGR
                    img.flags.writeable = True
                    #img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

                    # Extract landmarks
                    try:
                        landmarks = results.pose_landmarks.landmark
                        # Get coordinates
                        xyzFoot = [landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].x,
                                   landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].y,
                                   landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].z]

                        xyzKnee = [landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x,
                                   landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y,
                                   landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].z]

                        xyzNose = [landmarks[mp_pose.PoseLandmark.NOSE.value].x,
                                   landmarks[mp_pose.PoseLandmark.NOSE.value].y,
                                   landmarks[mp_pose.PoseLandmark.NOSE.value].z]
                        # landmarks shoulder left and right
                        rightShoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,
                                         landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y,
                                         landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].z]
                        leftShoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                                        landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y,
                                        landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].z]

                        rightHip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x,
                                    landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y,
                                    landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].z]
                        leftHip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,
                                   landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y,
                                   landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].z]

                        # Calculate angle
                        Shodis, Shomid = center_point(leftShoulder, rightShoulder)
                        Hipdis, Hipmid = center_point(leftHip, rightHip)

                        # distance mid
                        middis, midCoor = center_point(Shomid, Hipmid)

                        # read the Xn
                        RAWdist = round(Shomid[2] * 100, 3)
                        distanceCM = A * RAWdist ** 2 + B * RAWdist + C
                        # shoulder distance
                        Xn1D = round(Shodis, 4)
                        vel = (Xn1D - Xn_last1D) / ts

                        vel = abs(vel)
                        # Xn = [round(Shomid[0],4), round(Shomid[1],4), round(distanceCM,4)]
                        # velX, velY, veLZ = velXYZ(Xn, Xn_last, ts)
                        # print("Nilai Vel x: ", velX , "Nilai Vel y: ", velY ," Nilai Vel z:", velZ)

                        # Scol active pada saat terdapat vr pada rentang kecepatan

                        # Human Height detection
                        nilai_nose = tuple(np.multiply([xyzNose[0], xyzNose[1]], [640, 480]).astype(int))
                        nilai_shoulderMid = tuple(np.multiply([Shomid[0], Shomid[1]], [640, 480]).astype(int))
                        nilai_HipMid = tuple(np.multiply([Hipmid[0], Hipmid[1]], [640, 480]).astype(int))

                        print("Data raw nose ", nilai_nose[1])
                        print("Read raw mid shoulder ", nilai_shoulderMid[1])
                        print("Read raw mid hips ", nilai_HipMid[1])

                        noseRAW = nilai_nose[1]
                        midshoulderRAW = nilai_shoulderMid[1]
                        midHipsRAW = nilai_HipMid[1]

                        noseLoc = Anose*(nilai_nose[1]**2) + Bnose*nilai_nose[1] + Cnose
                        shoulderLoc = Anose*(nilai_shoulderMid[1]**2)+Bnose*nilai_shoulderMid[1]+Cnose
                        hipsLoc = Anose*(nilai_HipMid[1]**2)+Bnose*nilai_HipMid[1]+Cnose


                        #Sp = 400
                        disHR = distanceCM / 100
                        #Spmin = 100
                        SpminVal = Spmin(velHum, Tr, ac, C, Zd, Zr)

                        if SpminVal > Sp or SpminVal < 1000:
                            SpminVal = 1000
                            Sp = Sp + 800
                        # separation protective condition
                        # if Spmin > disHR:
                        #    cv2.putText(image, 'Mode = STOPPPPPPPPPPP',
                        #            (420,60),
                        #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                        #            )
                        #    cv2.circle(image, (380,40), radius = 10, color =(0,0,255), thickness = 20)

                        Vr_max_command = Vr_max(Sp, Vh, VelRnew, Tr, ac, C, Zd, Zr)
                        Vr_max_command = abs(Vr_max_command)
                        # ===== information visualization =====
                        # left monitoring input
                        velHum = vel * 1000
                        velRob = VelRnew
                        ShodisXY, ShoXYmid = center_pointXY(leftShoulder, rightShoulder)


                        #  right monitoring output

                        # Skeleton visualization
                        cv2.putText(img, str(Shodis),
                                    tuple(np.multiply([Shomid[0], Shomid[1]], [640, 480]).astype(int)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                    )

                        cv2.circle(img, tuple(np.multiply([Shomid[0], Shomid[1]], [640, 480]).astype(int)), radius=5,
                                   color=(255, 255, 0), thickness=10)

                        # Visualize
                        cv2.putText(img, str(Hipdis),
                                    tuple(np.multiply([Hipmid[0], Hipmid[1]], [640, 480]).astype(int)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                    )

                        cv2.circle(img, tuple(np.multiply([Hipmid[0], Hipmid[1]], [640, 480]).astype(int)), radius=5,
                                   color=(255, 255, 0), thickness=10)

                        cv2.line(img, tuple(np.multiply([Shomid[0], Shomid[1]], [640, 480]).astype(int)),
                                 tuple(np.multiply([Hipmid[0], Hipmid[1]], [640, 480]).astype(int)), (255, 255, 0),
                                 thickness=2)

                        cv2.circle(img, tuple(np.multiply([midCoor[0], midCoor[1]], [640, 480]).astype(int)), radius=5,
                                   color=(255, 255, 0), thickness=10)

                        cv2.circle(img, tuple(np.multiply([xyzNose[0], xyzNose[1]], [640, 480]).astype(int)), radius=5,
                                   color=(255, 255, 0), thickness=10)

                        #print("Nilai nose Linear Regression ", noseLocRL)
                        print("===========================================")


                        minHead = noseLoc * 10 - 150
                        maxHead = noseLoc * 10 + 150
                        zHead = [minHead, maxHead]
                        minChest = hipsLoc * 10
                        maxChest = shoulderLoc * 10
                        zChest = [minChest, maxChest]
                        print("Read Head Position ", zHead)
                        print("Read Chest Position ", zChest)
                        #print("Nilai S Current adalah ", Scurrent)
                        Scurrent = eye_dist
                        Scurrent = round(Scurrent, 2)



                        # logical SSM send robot
                        if Scurrent < SpminVal:
                            server.pause()
                            print("Robot harus berhenti", vrstop)
                            mode_collab = 4
                            Vr = 0
                            #mode SSM ori stop = 3
                            mode_SSMori = 3
                            VrOriSSM = 0
                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_stop = font_reg.render("Stop", True, (242, 242, 247))
                            window.blit(text_stop, (467, 555))
                            pygame.draw.rect(window, red, (460, 588, 166, 81), border_radius=5)

                            t.sleep(0.5)
                        elif SpminVal <= Scurrent and Sp > Scurrent:
                            server.resume()
                            print("Robot working on collaboration mode")
                            mode_collab = 3

                            #mode SSM ori stop = 3
                            mode_SSMori = 2
                            VrOriSSM = 0

                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_coll = font_reg.render("Collaboration", True, (242, 242, 247))
                            window.blit(text_coll, (467, 555))
                            pygame.draw.rect(window, blue, (460, 588, 166, 81), border_radius=5)

                            #pygame.draw.rect(window, gray, (1142, 442, 90, 90), border_radius=5)
                            #imgHead = pygame.image.load("assets/safe.png").convert()
                            #imgHead = pygame.transform.scale(imgHead, (90, 90))
                            #window.blit(imgHead, (1134, 442))
                            print("Nilai current robot position read", XnRob)
                            print("Nilai current robot position + table", RobTablePos)
                            print("Nilai perbandingan robot dan zHead min max", zHead[0], zRob, zHead[1])
                            if zHead[0] < RobTablePos[2] and zHead[1]+100 >= RobTablePos[2]:
                                #print("Velocity limitation on head area: ", vrface)
                                #pygame.draw.rect(window, gray, (1142, 442, 90, 90), border_radius=5)
                                imgHead = pygame.image.load("assets/head.png").convert()
                                imgHead = pygame.transform.scale(imgHead, (90, 90))
                                window.blit(imgHead, (1134, 445))
                                Vr = vrface
                                speed = 150
                                print("change value speed", speed)
                                #jacoRobot.setSpeed(Vr, vrot)
                                #jacoRobot.message("Jaco Speed Vr face")
                                print("Succes send speed VrFace")
                                t.sleep(0.5)

                            elif zChest[0] < RobTablePos[2] and zChest[1] >= RobTablePos[2]:
                                #print("Velocity limitation on chest: ", vrchest)
                                #pygame.draw.rect(window, gray, (1142, 442, 90, 90), border_radius=5)
                                imgchest = pygame.image.load("assets/chest.png").convert()
                                imgchest = pygame.transform.scale(imgchest, (90, 90))
                                window.blit(imgchest, (1134, 445))
                                Vr = vrchest
                                speed = 250
                                print("change value speed 250: ", speed)
                                #jacoRobot.setSpeed(Vr, vrot)
                                #jacoRobot.message("Jaco Speed Vrchest")
                                print("Succes send speed VrChest")
                                t.sleep(0.5)
                            else:
                                Vr = Vr_max_command
                                if Vr_max_command <= vrface:
                                    Vr = 50
                                    speed = 500
                                    print("change value speed 500: ", speed)
                                    #jacoRobot.setSpeed(Vr, vrot)
                                    #jacoRobot.message("Jaco Speed Vr warning")
                                    #print("Succes send speed Vr Command")
                                    t.sleep(0.5)
                            #jacoRobot.message("Collaboration speed")
                            t.sleep(0.5)
                        elif Sp <= Scurrent and Sp + 150 >= Scurrent:
                            server.resume()
                            #print("Robot speed reduction")
                            mode_collab = 2
                            # calculate the Vmax allowable
                            #print("Vmax allowable in this workspace: ", Vr_max_command)
                            # Vr = Vr_max_command
                            Vr = 100
                            speed = 500
                            print("change value speed 500: ", speed)
                            #jacoRobot.setSpeed(Vr, vrot)

                            #mode SSM ori reduce speed = 2
                            mode_SSMori = 2
                            VrOriSSM = 100

                            print("Succes send speed Vr Mid")
                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_reduce = font_reg.render("Reduce Speed", True, (242, 242, 247))
                            window.blit(text_reduce, (467, 555))
                            pygame.draw.rect(window, yellow, (460, 588, 166, 81), border_radius=5)
                            #jacoRobot.message("Robot speed reduction")
                            t.sleep(0.5)
                        else:
                            server.resume()
                            #print("Robot bekerja maximal")
                            mode_collab = 1
                            Vr = vrmax
                            speed = 750
                            print("change value speed 750: ", speed)
                            #jacoRobot.setSpeed(Vr, vrot)

                            #mode SSM ori full speed = 1
                            mode_SSMori = 1
                            VrOriSSM = vrmax
                            #print("Succes send speed Vr Full Speed")
                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_freespeed = font_reg.render("Full Speed", True, (242, 242, 247))
                            window.blit(text_freespeed, (467, 555))
                            pygame.draw.rect(window, green, (460, 588, 166, 81), border_radius=5)
                            #jacoRobot.message("Robot free speed")
                            t.sleep(0.5)


                        t.sleep(ts)
                        # Xn_last = Xn
                        Xn_last1D = Xn1D
                        XnRob_last = XnRob
                    except:
                        if Scurrent < SpminVal:
                            server.pause()
                            #print("Robot harus berhenti", vrstop)
                            mode_collab = 4
                            Vr = 0

                            #mode SSM ori stop speed = 3
                            mode_SSMori = 3
                            VrOriSSM = 0

                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_coll = font_reg.render("Stop", True, (242, 242, 247))
                            window.blit(text_coll, (467, 555))
                            pygame.draw.rect(window, red, (460, 588, 166, 81), border_radius=5)
                            #jacoRobot.message("Robot stop")
                            #print("Be careful! Sensor not detected")
                            #jacoRobot.message("Your position is too close. Be careful sensor not detected")
                            t.sleep(0.5)
                        elif Scurrent > Sp:
                            server.resume()
                            #print("Robot bekerja maximal")
                            mode_collab = 1
                            Vr = vrmax
                            speed = 750
                            print("change value speed max 750: ", speed)
                            #jacoRobot.setSpeed(Vr, vrot)
                            #mode SSM ori full speed = 1
                            mode_SSMori = 1
                            VrOriSSM = 100
                            #print("Succes send speed Vr Full Speed")
                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_coll = font_reg.render("Full Speed", True, (242, 242, 247))
                            window.blit(text_coll, (467, 555))
                            pygame.draw.rect(window, green, (460, 588, 166, 81), border_radius=5)
                            #jacoRobot.message("Robot free speed")
                            t.sleep(0.5)

                    t.sleep(0.5)
                    # distance calculation robot speed

                # Render detections
                mp_drawing.draw_landmarks(img, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                          mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                          mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                                          )


            #print("Scurrent update ", Scurrent)

            # ======= Information Visualization =========

            # SSM output value
            pygame.draw.rect(window, gray, (233, 555, 196, 29), border_radius=5)
            text_currval = font_reg.render(str(Scurrent) + " mm", True, (50, 50, 50))
            window.blit(text_currval, (233, 555))

            Sp = round(Sp, 2)
            pygame.draw.rect(window, gray, (74, 584, 196, 29), border_radius=5)
            text_Spval = font_reg.render(str(Sp) + " mm", True, (50, 50, 50))
            window.blit(text_Spval, (74, 584))

            SpminVal = round(SpminVal, 2)
            pygame.draw.rect(window, gray, (115, 616, 196, 29), border_radius=5)
            text_Spminval = font_reg.render(str(SpminVal) + " mm", True, (50, 50, 50))
            window.blit(text_Spminval, (115, 616))

            Vr_max_command = round(Vr_max_command, 2)
            pygame.draw.rect(window, gray, (118, 647, 196, 29), border_radius=5)
            text_vmaxcmdval = font_reg.render(str(Vr_max_command) + " mm/s", True, (50, 50, 50))
            window.blit(text_vmaxcmdval, (118, 647))

            # ============== Robot Domain ===============
            pygame.draw.rect(window, gray, (845, 119, 150, 29), border_radius=5)
            text_xRval = font_reg.render(str(RobTablePos[0]) + " mm", True, (50, 50, 50))
            window.blit(text_xRval, (855, 119))

            pygame.draw.rect(window, gray, (845, 156, 150, 29), border_radius=5)
            text_yRval = font_reg.render(str(RobTablePos[1]) + " mm", True, (50, 50, 50))
            window.blit(text_yRval, (855, 156))

            pygame.draw.rect(window, gray, (845, 194, 150, 29), border_radius=5)
            text_zRval = font_reg.render(str(RobTablePos[2]) + " mm", True, (50, 50, 50))
            window.blit(text_zRval, (855, 194))

            Vr = round(Vr, 2)
            pygame.draw.rect(window, gray, (1088, 125, 165, 29), border_radius=5)
            text_speedRval = font_reg.render(str(Vr) + " mm/s", True, (50, 50, 50))
            window.blit(text_speedRval, (1088, 125))

            VelRnew = round(VelRnew, 2)
            pygame.draw.rect(window, gray, (1045, 160, 170, 29), border_radius=5)
            text_veloRval = font_reg.render(str(VelRnew) + " mm/s", True, (50, 50, 50))
            window.blit(text_veloRval, (1045, 156))

            # === Human domain filter data

            if zHead[0] > 2000:
                zHead[0] = 1800
            zHead[0] = abs(round(zHead[0], 2))
            pygame.draw.rect(window, gray, (929, 391, 165, 29), border_radius=5)
            text_minHead = font_reg.render(str(zHead[0]) + " mm", True, (50, 50, 50))
            window.blit(text_minHead, (929, 391))
            if zHead[1] > 2000:
                zHead[1] = 1800
            zHead[1] = abs(round(zHead[1], 2))
            pygame.draw.rect(window, gray, (929, 358, 165, 29), border_radius=5)
            text_maxHead = font_reg.render(str(zHead[1]) + " mm", True, (50, 50, 50))
            window.blit(text_maxHead, (929, 358))

            if zChest[0] > 2000:
                zChest[0] = 0
            zChest[0] = abs(round(zChest[0], 2))
            pygame.draw.rect(window, gray, (935, 508, 165, 29), border_radius=5)
            text_minZchest = font_reg.render(str(zChest[0]) + " mm", True, (50, 50, 50))
            window.blit(text_minZchest, (935, 508))

            if zChest[1] > 2000:
                zChest[1] = 0
            zChest[1] = abs(round(zChest[1], 2))
            pygame.draw.rect(window, gray, (935, 475, 165, 29), border_radius=5)
            text_maxZchest = font_reg.render(str(zChest[1]) + " mm", True, (50, 50, 50))
            window.blit(text_maxZchest, (935, 475))

            velHum = abs(round(velHum, 2))
            pygame.draw.rect(window, gray, (1031, 329, 165, 29), border_radius=5)
            text_velHum = font_reg.render(str(velHum) + " mm/s", True, (50, 50, 50))
            window.blit(text_velHum, (1031, 329))

            imgRGB = np.rot90(img)
            frame = pygame.surfarray.make_surface(imgRGB).convert()
            frame = pygame.transform.flip(frame, True, False)

            window.blit(frame, (18, 18))
            # ===== research documentation =====
            interval = interval + 1
            # nilai calibrasi data raw real hip, real shoulder, real nose, pixel hip, pixel shoulder, pixel nose
            # output.write(str(interval) + ',' + str(Scurrent) + ',' + str(VrOriSSM) + ',' + str(Vr) + ',' + str(mode_SSMori) + ',' + str(mode_collab) + '\n')
            print("SUCCESS RECORD!!!")
            # Update Display
            pygame.display.update()
            # Set FPS
            clock.tick(fps)