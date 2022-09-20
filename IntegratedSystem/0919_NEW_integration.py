import math
import time as t
import threading
from datetime import datetime
#from fs100 import FS100
from utilsFS100 import FS100

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
text_spRobot = font_reg.render("Vsend:", True, (50, 50, 50))
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

# def move_distance(post1, post2):
#     print("nilai post akhir", post2[2])
#     print("nilai post awal", post1[2])
#     x_coor = post2[0] - post1[0]
#     y_coor = post2[1] - post1[1]
#     z_coor = post2[2] - post1[2]
#     rx_coor = post2[3] - post1[3]
#     ry_coor = post2[4] - post1[4]
#     rz_coor = post2[5] - post1[5]
#     re_coor = post2[6] - post1[6]
#
#     dist = math.sqrt(math.pow((post2[0] - post1[0]), 2) + math.pow((post2[1] - post1[1]), 2) + math.pow((post2[2] - post1[2]), 2))
#
#     move_coor = (int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor))
#     return move_coor, int(dist)

# def time_robot(speed, distance, delay_rob):
#     distance = distance / 1000
#     speed = speed / 10
#     time_move = (distance / speed) + delay_rob
#     return time_move

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

# ===== initialization & variables declaration =====
#SSM variables
Vrinitial = 200
Vr = Vrinitial
Vh = 1600
Tr = 0.41
ac = 200
C = 1000
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
vrchest = 250
vrface = 60
vrstop = 0
vrmax = 750
vrot = 90
velRob = 0
robotZ = 0
vel = 0
RobotVrmax = 300


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
write_file = "0920-Trial_SSMdata_Analysis.csv"
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
p1 = [489.918,58.66,-232.798,179.9994,0.0013,180,0]
p2 = [503.826,49.941,-232.801,179.9994,0.0008,179.9983,0]
p3 = [517.714,34.156,-232.792,179.9993,0.0025,179.9994,0]
p4 = [524.313,19.02,-232.794,179.9994,0.0018,179.9998,0]
p5 = [527.394,1.638,-232.795,179.9995,0.0018,179.9996,0]
p6 = [524.974,-16.781,-232.8,179.9995,0.0019,179.9984,0]
p7 = [519.415,-31.248,-232.798,179.9996,0.0021,179.9978,0]
p8 = [506.061,-48.025,-232.799,179.9996,0.0013,179.9974,0]
p9 = [493.358,-56.936,-232.796,179.9997,0.002,179.9982,0]
p10 = [474.845,-63.793,-232.795,179.9998,0.0021,179.9975,0]
p11 = [456.362,-64.478,-232.797,179.9998,0.0014,179.9998,0]
p12 = [440.952,-61.331,-232.798,179.9998,0.0012,179.9982,0]
p13 = [421.762,-50.653,-232.796,179.9998,0.0016,179.9989,0]
p14 = [410.766,-39.078,-232.798,179.9998,0.0015,179.998,0]
p15 = [401.572,-22.85,-232.797,179.9998,0.0017,179.9982,0]
p16 = [397.933,-5.843,-232.798,179.9997,0.0023,179.9979,0]
p17 = [398.326,10.806,-232.798,179.9996,0.0023,-179.9997,0]
p18 = [404.386,28.79,-232.801,179.9996,0.0012,-179.9995,0]
p19 = [412.581,41.702,-232.799,179.9995,0.002,179.9985,0]
p20 = [428.429,55.264,-232.8,179.9995,0.0009,180,0]
p21 = [443.412,61.939,-232.799,179.9994,0.0016,179.999,0]
p22 = [462.422,65.001,-232.794,179.9993,0.0021,179.9986,0]
p23 = [462.416,109.998,-132.797,179.9991,0.0013,179.999,0]
p24 = [470.503,109.681,-232.794,179.9993,0.0011,179.9989,0]
p25 = [485.965,107.438,-232.797,179.9992,0.0017,-179.999,0]
p26 = [506.907,100.326,-232.797,179.9994,0.0007,-179.9995,0]
p27 = [525.95,89.765,-232.801,179.9993,0.0008,179.9998,0]
p28 = [538.583,79.249,-232.795,179.9992,0.0015,-179.9995,0]
p29 = [552.121,63.228,-232.799,179.9993,0.0011,-179.9995,0]
p30 = [563.13,44.215,-232.797,179.9993,0.0019,-179.9995,0]
p31 = [568.362,29.424,-232.8,179.9993,0.0022,179.9996,0]
p32 = [571.894,7.664,-232.801,179.9994,0.0007,179.9997,0]
p33 = [571.534,-13.912,-232.794,179.9994,0.0019,179.9996,0]
p34 = [568.081,-30.176,-232.799,179.9995,0.0014,-179.9993,0]
p35 = [560.067,-50.098,-232.795,179.9995,0.0016,179.9988,0]
p36 = [548.88,-67.991,-232.795,179.9996,0.0015,179.9975,0]
p37 = [537.262,-80.423,-232.797,179.9995,0.0006,179.9968,0]
p38 = [520.125,-93.362,-232.799,179.9998,0.0019,179.9984,0]
p39 = [501.68,-102.75,-232.793,180,0.0029,179.9975,0]
# p40 = [484.636,-107.57,-232.795,-179.9999,0.003,179.9988,0]
# p41 = [461.272,-109.815,-232.797,180,0.0019,179.9982,0]
# p42 = [443.311,-108.321,-232.8,179.9999,0.0015,179.9982,0]
# p43 = [424.35,-102.966,-232.802,179.9998,0.0009,179.9991,0]
# p44 = [403.698,-92.896,-232.799,179.9999,0.0011,179.9988,0]
# p45 = [389.853,-82.602,-232.801,179.9999,0.0009,179.9989,0]
# p46 = [375.287,-66.728,-232.797,180,0.0016,179.9989,0]
# p47 = [363.663,-48.383,-232.797,180,0.0017,179.9984,0]
# p48 = [357.454,-32.567,-232.801,179.9998,0.0003,179.9971,0]
# p49 = [353.139,-10.412,-232.797,179.9998,0.0016,179.998,0]
# p50 = [352.833,9.588,-232.799,179.9997,0.0013,179.9998,0]
# p51 = [356.379,28.46,-232.8,179.9997,0.0015,179.9997,0]
# p52 = [364.089,48.928,-232.8,179.9997,0.0007,179.9995,0]
# p53 = [373.321,64.516,-232.8,179.9995,0.0015,179.9992,0]
# p54 = [385.686,78.546,-232.798,179.9995,0.0011,-179.9996,0]
# p55 = [402.779,92.206,-232.797,179.9995,0.001,-179.9998,0]
# p56 = [419.114,101.117,-232.797,179.9994,0.001,179.9988,0]
# p57 = [437.329,106.886,-232.798,179.9993,0.0015,-179.9998,0]
# p58 = [460.201,109.847,-232.795,179.9992,0.0017,179.9991,0]
# p59 = [462.414,116.575,-132.798,179.999,0.0013,179.9992,0]
# p60 = [465.434,139.253,-232.798,179.9991,0.0015,179.9999,0]
# p61 = [486.266,137.277,-232.797,179.999,0.0021,179.9985,0]
# p62 = [503.132,133.213,-232.798,179.999,0.002,179.9993,0]
# p63 = [522.537,125.46,-232.794,179.9992,0.0013,179.9989,0]
# p64 = [540.846,114.863,-232.797,179.999,0.0023,179.9998,0]
# p65 = [558.328,100.947,-232.796,179.9991,0.0019,179.9994,0]
# p66 = [570.774,87.593,-232.793,179.9991,0.002,-179.9996,0]
# p67 = [581.04,72.955,-232.792,179.9993,0.0006,179.9998,0]
# p68 = [590.436,54.436,-232.797,179.9992,0.0024,179.9993,0]
# p69 = [597.489,33.239,-232.799,179.9993,0.0008,-179.9991,0]
# p70 = [601.204,11.871,-232.794,179.9993,0.0022,179.9996,0]
# p71 = [601.639,-5.443,-232.796,179.9994,0.0008,179.9988,0]
# p72 = [599.731,-23.222,-232.796,179.9994,0.0022,-179.9997,0]
# p73 = [594.439,-43.808,-232.801,179.9995,0.0017,179.9997,0]
# p74 = [585.556,-64.752,-232.796,179.9996,0.0021,179.9989,0]
# p75 = [574.326,-82.997,-232.795,179.9997,0.0023,179.9992,0]
# p76 = [563.044,-96.296,-232.799,179.9996,0.0015,179.999,0]
# p77 = [546.735,-110.631,-232.798,179.9997,0.0016,179.9975,0]
# p78 = [529.328,-121.932,-232.798,179.9998,0.0017,179.9979,0]
# p79 = [509.288,-131.122,-232.797,179.9998,0.0014,179.9993,0]
# p80 = [491.946,-136.16,-232.799,179.9998,0.0013,179.9994,0]
# p81 = [474.793,-138.734,-232.801,179.9999,0.0014,179.9989,0]
# p82 = [452.498,-138.739,-232.796,180,0.0017,179.997,0]
# p83 = [430.309,-135.369,-232.798,180,0.0013,179.9982,0]
# p84 = [410.017,-129.101,-232.798,179.9997,0.0002,179.9989,0]
# p85 = [394.857,-121.847,-232.795,180,0.0013,179.9989,0]
# p86 = [376.768,-109.616,-232.795,-179.9998,0.0019,179.9988,0]
# p87 = [360.675,-94.846,-232.796,-179.9998,0.0019,179.9984,0]
# p88 = [346.682,-77.517,-232.801,180,0.0012,179.9983,0]
# p89 = [337.962,-62.616,-232.798,-179.9998,0.0029,179.9989,0]
# p90 = [330.174,-43.271,-232.798,180,0.0012,179.9966,0]
# p91 = [325.003,-21.535,-232.798,179.9999,0.0013,179.9966,0]
# p92 = [323.126,1.171,-232.8,179.9998,0.0012,179.9992,0]
# p93 = [324.318,18.459,-232.799,179.9997,0.0022,179.999,0]
# p94 = [328.163,36.84,-232.799,179.9996,0.0022,179.9996,0]
# p95 = [335.952,57.888,-232.797,179.9997,0.0008,179.9991,0]
# p96 = [347.237,78.209,-232.798,179.9998,0.0001,-179.9997,0]
# p97 = [358.778,93.125,-232.797,179.9995,0.0012,-179.9994,0]
# p98 = [371.246,105.301,-232.799,179.9993,0.0016,-179.9988,0]
# p99 = [388.397,117.791,-232.798,179.9993,0.0013,179.9997,0]
# p100 = [408.676,128.319,-232.8,179.9992,0.0017,179.9987,0]
# p101 = [429.236,135.32,-232.799,179.9995,0.0005,179.9991,0]
# p102 = [446.864,138.4,-232.795,179.9993,0.0011,-179.9993,0]
# p103 = [462.418,139.332,-192.114,179.9991,0.0014,-179.9999,0]
# p104 = [462.417,139.332,-132.797,179.999,0.0014,-179.9998,0]


start_time = datetime.now()
# ==counter position==
counter = 0

class Job(threading.Thread):
    def __init__(self, *args, **kwargs):
        super(Job, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()     # 用于暂停线程的标识
        self.__flag.set()       # 设置为True
        self.__running = threading.Event()      # 用于停止线程的标识
        self.__running.set()      # 将running设置为True

    def run(self):

        # set speed
        SPEED_XYZ = (10, 150, 500)
        SPEED_R_XYZE = (10, 50, 100)

        #speed_class = FS100.MOVE_SPEED_CLASS_MILLIMETER
        speed = SPEED_XYZ[2]



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
        index = 0
        start = datetime.now()
        stop = datetime.now()

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
            post_1 = rob_command(p1)
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
            post_12 = rob_command(p12)
            post_13 = rob_command(p13)
            post_14 = rob_command(p14)
            post_15 = rob_command(p15)
            post_16 = rob_command(p16)
            post_17 = rob_command(p17)
            post_18 = rob_command(p18)
            post_19 = rob_command(p19)
            post_20 = rob_command(p20)
            post_21 = rob_command(p21)
            post_22 = rob_command(p22)
            post_23 = rob_command(p23)
            post_24 = rob_command(p24)
            post_25 = rob_command(p25)
            post_26 = rob_command(p26)
            post_27 = rob_command(p27)
            post_28 = rob_command(p28)
            post_29 = rob_command(p29)
            post_30 = rob_command(p30)
            post_31 = rob_command(p31)
            post_32 = rob_command(p32)
            post_33 = rob_command(p33)
            post_34 = rob_command(p34)
            post_35 = rob_command(p35)
            post_36 = rob_command(p36)
            post_37 = rob_command(p37)
            post_38 = rob_command(p38)
            post_39 = rob_command(p39)
            # post_40 = rob_command(p40)
            # post_41 = rob_command(p41)
            # post_42 = rob_command(p42)
            # post_43 = rob_command(p43)
            # post_44 = rob_command(p44)
            # post_45 = rob_command(p45)
            # post_46 = rob_command(p46)
            # post_47 = rob_command(p47)
            # post_48 = rob_command(p48)
            # post_49 = rob_command(p49)
            # post_50 = rob_command(p50)
            #
            # post_51 = rob_command(p51)
            # post_52 = rob_command(p52)
            # post_53 = rob_command(p53)
            # post_54 = rob_command(p54)
            # post_55 = rob_command(p55)
            # post_56 = rob_command(p56)
            # post_57 = rob_command(p57)
            # post_58 = rob_command(p58)
            # post_59 = rob_command(p59)
            # post_60 = rob_command(p60)
            #
            # post_61 = rob_command(p61)
            # post_62 = rob_command(p62)
            # post_63 = rob_command(p63)
            # post_64 = rob_command(p64)
            # post_65 = rob_command(p65)
            # post_66 = rob_command(p66)
            # post_67 = rob_command(p67)
            # post_68 = rob_command(p68)
            # post_69 = rob_command(p69)
            # post_70 = rob_command(p70)
            #
            # post_71 = rob_command(p71)
            # post_72 = rob_command(p72)
            # post_73 = rob_command(p73)
            # post_74 = rob_command(p74)
            # post_75 = rob_command(p75)
            # post_76 = rob_command(p76)
            # post_77 = rob_command(p77)
            # post_78 = rob_command(p78)
            # post_79 = rob_command(p79)
            # post_80 = rob_command(p80)
            #
            # post_81 = rob_command(p81)
            # post_82 = rob_command(p82)
            # post_83 = rob_command(p83)
            # post_84 = rob_command(p84)
            # post_85 = rob_command(p85)
            # post_86 = rob_command(p86)
            # post_87 = rob_command(p87)
            # post_88 = rob_command(p88)
            # post_89 = rob_command(p89)
            # post_90 = rob_command(p90)
            #
            # post_91 = rob_command(p91)
            # post_92 = rob_command(p92)
            # post_93 = rob_command(p93)
            # post_94 = rob_command(p94)
            # post_95 = rob_command(p95)
            # post_96 = rob_command(p96)
            # post_97 = rob_command(p97)
            # post_98 = rob_command(p98)
            # post_99 = rob_command(p99)
            # post_100 = rob_command(p100)
            #
            # post_101 = rob_command(p101)
            # post_102 = rob_command(p102)
            # post_103 = rob_command(p103)
            # post_104 = rob_command(p104)


            # servo on check
            if FS100.ERROR_SUCCESS == robot.get_status(status):
                if not status['servo_on']:
                    robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

            # ===== list movement task ========
            pos_updater = threading.Thread(target=update_pos)
            index = 0
            tredON = False

            postMove = [post_1,post_2,post_3,post_4,post_5,post_6,post_7,post_8,post_9,post_10,
                         post_11,post_12,post_13,post_14,post_15,post_16,post_17,post_18,post_19,post_20,
                         post_21,post_22,post_23,post_24,post_25,post_26,post_27,post_28,post_29,post_30,
                         post_31,post_32,post_33,post_34,post_35,post_36,post_37,post_38,post_39
                        ]

                        # post_40,
                        #  post_41,post_42,post_43,post_44,post_45,post_46,post_47,post_48,post_49,post_50,
                        #  post_51,post_52,post_53,post_54,post_55,post_56,post_57,post_58,post_59,post_60,
                        #  post_61,post_62,post_63,post_64,post_65,post_66,post_67,post_68,post_69,post_70,
                        #  post_71,post_72,post_73,post_74,post_75,post_76,post_77,post_78,post_79,post_80,
                        #  post_81,post_82,post_83,post_84,post_85,post_86,post_87,post_88,post_89,post_90,
                        #  post_91,post_92,post_93,post_94,post_95,post_96,post_97,post_98,post_99,post_100,
                        #  post_101,post_102,post_103,post_104]



            for i in postMove:
                self.__flag.wait()
                #time_d = time_robot(speed, dist[index], delay_rob)
                if status == FS100.TRAVEL_STATUS_START:
                    start = datetime.now()
                print("nilai x yang masuk ", index, "sebesar ", i)
                robot.move(None, FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
                           FS100.MOVE_SPEED_CLASS_PERCENT, speed, i)
                if status == FS100.TRAVEL_STATUS_END:
                    stop = datetime.now()
                print("nilai start", start)
                print("nilai stop", stop)
                diff_seconds = (stop - start)
                robot_time = diff_seconds.seconds + 3
                print("nilai second", robot_time)
                t.sleep(robot_time)  # robot may not update the status
                index = index + 1
                print("Finished step ", index)
                #exception
                if i == post_39:
                    counter = counter + 1
                    ## counter information
                    print("Robot counter: ", counter)
                    pygame.draw.rect(window, gray, (816, 585, 84, 82), border_radius=5)
                    text_fillcounter = font_big.render(str(counter), True, (50, 50, 50))
                    window.blit(text_fillcounter, (836, 590))
                    break
            if counter == 1:
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

            xTrob = round(xRob + 850, 2)
            yTrob = round(yRob + 850, 2)
            zTrob = round(zRob + 850, 2)
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
                                    speed = 150
                                    Vr = 50
                                    print("change value speed 150: ", speed)
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
                            speed = 500
                            Vr = speed
                            print("change value speed 500: ", speed)
                            #jacoRobot.setSpeed(Vr, vrot)

                            #mode SSM ori reduce speed = 2
                            mode_SSMori = 2
                            VrOriSSM = speed

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
                            speed = 750
                            Vr = speed

                            print("change value speed 750: ", speed)
                            #jacoRobot.setSpeed(Vr, vrot)

                            #mode SSM ori full speed = 1
                            mode_SSMori = 1
                            VrOriSSM = speed
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
                            speed = 750
                            Vr = speed
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
            output.write(str(interval) + ',' + str(Scurrent) + ',' + str(VrOriSSM) + ',' + str(Vr) + ',' + str(mode_SSMori) + ',' + str(mode_collab) + ',' + str(XnRob[0]) + ',' + str(XnRob[1]) + ',' + str(XnRob[2]) + '\n')
            print("SUCCESS RECORD ", interval, " !!!" )
            # Update Display
            pygame.display.update()
            # Set FPS
            clock.tick(fps)