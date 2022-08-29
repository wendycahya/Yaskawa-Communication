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

    input = (x, y, z, rx, ry, rz, re)
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

def time_robot(speed, distance):
    distance = distance / 1000
    speed = speed / 10
    time_move = (distance / speed) + 0.5
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
# Movement Position List
pointHome = 0
pointHomeA = [445.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
pointHomeB = [435.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
pointHomeC = [425.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
pointHomeD = [415.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]

point1 = [405.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point1A = [405.929, -387.592, 335.821, 178.7000, 0.0002, -0.0261, 0]
point1B = [405.929, -387.592, 325.821, 178.7000, 0.0002, -0.0261, 0]
point1C = [405.929, -387.592, 315.821, 178.7000, 0.0002, -0.0261, 0]
point1D = [405.929, -387.592, 305.821, 178.7000, 0.0002, -0.0261, 0]
point1E = [405.929, -387.592, 295.821, 178.7000, 0.0002, -0.0261, 0]
point1F = [405.929, -387.592, 285.821, 178.7000, 0.0002, -0.0261, 0]
point1G = [405.929, -387.592, 275.821, 178.7000, 0.0002, -0.0261, 0]
point1H = [405.929, -387.592, 265.821, 178.7000, 0.0002, -0.0261, 0]
point1I = [405.929, -387.592, 255.821, 178.7000, 0.0002, -0.0261, 0]
point1J = [405.929, -387.592, 245.821, 178.7000, 0.0002, -0.0261, 0]
point1K = [405.929, -387.592, 235.821, 178.7000, 0.0002, -0.0261, 0]
point1L = [405.929, -387.592, 225.821, 178.7000, 0.0002, -0.0261, 0]
point1M = [405.929, -387.592, 215.821, 178.7000, 0.0002, -0.0261, 0]
point1N = [405.929, -387.592, 195.821, 178.7000, 0.0002, -0.0261, 0]
point1O = [405.929, -387.592, 185.821, 178.7000, 0.0002, -0.0261, 0]
point1P = [405.929, -387.592, 175.821, 178.7000, 0.0002, -0.0261, 0]

point2 = [405.929, -387.592, -201.391, 178.7000, 0.0002, -0.0261, 0]

point4 = [405.919, 253.778, 345.819, 178.6967, -0.0031, -0.0308, 0]

point5 = [405.932, 253.783, -201.391, 178.6961, -0.0045, -0.0303, 0]

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

        pygame.draw.rect(window, purple, (929, 602, 140, 29), border_radius=5)
        text_process = font_reg.render("PROCESS", True, (242, 242, 247))
        window.blit(text_process, (940, 600))
        pygame.draw.rect(window, gray, (1119, 562, 99, 99), border_radius=5)
        imgPro = pygame.image.load("assets/process.png").convert()
        imgPro = pygame.transform.scale(imgPro, (99, 99))
        window.blit(imgPro, (1119, 562))
        while self.__running.is_set():
            print("reading condition", progress)
            # Read initial position
            if FS100.ERROR_SUCCESS == robot.read_position(pos_info, robot_no):
                x, y, z, rx, ry, rz, re = pos_info['pos']
                pointHome = (x, y, z, 0, 0, 0, 0)
                str = "CURRENT POSITION\n" + \
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

            print(str)
            # ===== convert robot command =====
            robHome = pointHome
            robHome1 = rob_command(pointHomeA)
            robHome2 = rob_command(pointHomeB)
            robHome3 = rob_command(pointHomeC)
            robHome4 = rob_command(pointHomeD)

            rob1 = rob_command(point1)
            rob1A = rob_command(point1A)
            rob1B = rob_command(point1B)
            rob1C = rob_command(point1C)
            rob1D = rob_command(point1D)
            rob1E = rob_command(point1E)
            rob1F = rob_command(point1F)
            rob1G = rob_command(point1G)
            rob1H = rob_command(point1H)
            rob1I = rob_command(point1I)
            rob1J = rob_command(point1J)
            rob1K = rob_command(point1K)
            rob1L = rob_command(point1L)
            rob1M = rob_command(point1M)
            rob1N = rob_command(point1N)
            rob1O = rob_command(point1O)
            rob1P = rob_command(point1P)

            rob2 = rob_command(point2)
            rob4 = rob_command(point4)
            rob5 = rob_command(point5)
            # ===== move and distance =========
            #post1_move, distance1 = move_distance(robHome, rob1)
            #post2_move, distance2 = move_distance(rob1, rob2)

            post1_move, distance1 = move_distance(robHome, robHome1)
            post1A_move, distance1A = move_distance(robHome1, robHome2)
            post1B_move, distance1B = move_distance(robHome2, robHome3)
            post1C_move, distance1C = move_distance(robHome3, robHome4)

            post2_move, distance2 = move_distance(rob1, rob1A)
            post2A_move, distance2A = move_distance(rob1B, rob1C)
            post2B_move, distance2B = move_distance(rob1C, rob1D)
            post2C_move, distance2C = move_distance(rob1D, rob1E)
            post2D_move, distance2D = move_distance(rob1E, rob1F)
            post2E_move, distance2E = move_distance(rob1F, rob1G)
            post2F_move, distance2F = move_distance(rob1G, rob1H)
            post2G_move, distance2G = move_distance(rob1H, rob1I)
            post2H_move, distance2H = move_distance(rob1I, rob1J)
            post2I_move, distance2I = move_distance(rob1J, rob1K)
            post2J_move, distance2J = move_distance(rob1K, rob1L)
            post2K_move, distance2K = move_distance(rob1L, rob1M)
            post2L_move, distance2L = move_distance(rob1M, rob1N)
            post2M_move, distance2M = move_distance(rob1N, rob1O)
            post2N_move, distance2N = move_distance(rob1O, rob1P)
            post2O_move, distance2O = move_distance(rob1P, rob2)



            post3_move, distance3 = move_distance(rob2, rob1)
            post4_move, distance4 = move_distance(rob1, rob4)
            post5_move, distance5 = move_distance(rob4, rob5)
            post6_move, distance6 = move_distance(rob5, rob4)
            post7_move, distance7 = move_distance(rob4, robHome)

            # servo on check
            if FS100.ERROR_SUCCESS == robot.get_status(status):
                if not status['servo_on']:
                    robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

            # ===== list movement task ========
            pos_updater = threading.Thread(target=update_pos)
            index = 0
            tredON = False

            postMove = [post1_move, post1A_move, post1B_move, post1C_move, post2_move, post2A_move, post2B_move, post2C_move, post2D_move, post2E_move, post2F_move, post2G_move, post2H_move, post2I_move, post2J_move, post2K_move, post2L_move, post2M_move, post2N_move, post2O_move, post3_move, post4_move, post5_move, post6_move, post7_move]
            dist = [distance1, distance1A, distance1B, distance1C, distance2, distance2A, distance2B, distance2C, distance2D, distance2E, distance2F, distance2G, distance2H, distance2I, distance2J, distance2K, distance2L, distance2M, distance2N, distance2O, distance3, distance4, distance5, distance6, distance7]


            for i in postMove:
                self.__flag.wait()
                time_d = time_robot(speed, dist[index])
                print(time_d)
                print("nilai x yang masuk ", index, "sebesar ", i)
                if FS100.ERROR_SUCCESS == robot.one_move(FS100.MOVE_TYPE_LINEAR_INCREMENTAL_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT, speed_class, speed, i):
                    t.sleep(time_d)  # robot may not update the status
                    if not is_alarmed() and tredON == False:
                        pos_updater.start()
                        tredON = True
                index = index + 1
                print("Finished step ", index)
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
            curRobotPos = convert_mm(x,y, z, rx, ry, rz, re)
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
                                    speed = 100
                                    print("change value speed 100: ", speed)
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
                            speed = 250
                            print("change value speed 250: ", speed)
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
                            speed = 100
                            print("change value speed 100: ", speed)
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
                            speed = 250
                            print("change value speed max 250: ", speed)
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