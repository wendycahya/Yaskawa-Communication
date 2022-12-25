import math
import time as t
import threading
from datetime import datetime
from utilsFS100 import FS100
import numpy as np

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

def time_robot(speed, distance, delay_rob):
    distance = distance / 1000
    speed = speed / 10
    time_move = (distance / speed) + delay_rob
    return time_move

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
start = datetime.now()
stop = datetime.now()

# ===== Movement Position List =====
p1 = [430.901,98.295,169.992,178.2827,-0.1132,-0.1875,0]
p2 = [411.019,124.178,107.8,178.2986,-0.1397,-0.1497,0]
p3 = [380.551,144.635,50.579,178.3884,-0.1335,-0.101,0]
p4 = [342.569,157.805,1.305,178.5439,-0.0767,-0.0556,0]
p5 = [309.341,162.762,-31.25,178.7044,0.009,-0.0365,0]
p6 = [314.991,163.073,-52.623,177.462,-0.6217,-0.332,0]  #pos1
p7 = [319.842,163.448,-80.433,176.6027,-1.117,-0.4684,0]
p8 = [321.86,163.715,-108.341,176.5,-1.1554,-0.4555,0] #pos1 down


p9 = [508.511,153.34,-555.573,178.4581,-0.1072,-0.0775,0]
p10 = [508.739,148.288,-523.971,178.7084,-0.0013,-0.0353,0]
p11 = [551.146,61.817,-523.882,178.8027,0.0184,-0.0429,0]
p12 = [574.571,-39.419,-523.802,178.8527,0.0077,-0.0454,0]
p13 = [574.202,-148.208,-523.781,178.8485,-0.0114,-0.0447,0]
p14 = [548.139,-258.364,-523.838,178.7893,-0.0211,-0.041,0]
p15 = [504.63,-349.822,-523.982,178.699,-0.0044,-0.0365,0]
p16 = [506.131,-351.776,-585.825,178.6614,0.0289,-0.0463,0]
p17 = [505.269,-350.673,-612.892,178.6825,0.0104,-0.0398,0]
p18 = [504.747,-349.991,-525.94,178.6959,-0.0021,-0.0376,0]
p19 = [532.784,-296.2,-523.887,178.7566,-0.0178,-0.0409,0]
p20 = [568.144,-186.476,-523.787,178.8339,-0.0171,-0.0432,0]
p21 = [577.264,-76.391,-523.789,178.8573,0.0013,-0.045,0]
p22 = [561.727,28.069,-523.855,178.8256,0.018,-0.0428,0]
p23 = [525.514,119.664,-523.942,178.7455,0.0104,-0.0383,0]
p24 = [518.973,155.785,-523.848,178.6884,-0.0089,-0.0372,0]
p25 = [615.082,157.229,-523.697,178.6751,-0.0125,-0.0382,0]
p26 = [655.908,154.839,-539.6,178.6841,-0.0082,-0.0362,0]
p27 = [671.43,154.71,-624.921,178.691,-0.005,-0.037,0]
p28 = [667.099,155.056,-591.561,178.6719,-0.0108,-0.0398,0]
p29 = [651.418,154.581,-523.98,178.6984,-0.0044,-0.0312,0]
p30 = [662.771,64.28,-524.002,178.782,0.0147,-0.0294,0]
p31 = [654.86,-57.997,-523.977,178.8474,0.0075,-0.0289,0]
p32 = [621.487,-173.173,-523.939,178.8489,-0.0142,-0.0304,0]
p33 = [565.047,-277.031,-523.93,178.7881,-0.0243,-0.0348,0]
p34 = [504.62,-349.832,-523.982,178.699,-0.0043,-0.0359,0]
p35 = [506.013,-351.622,-592.405,178.6639,0.0257,-0.0447,0]
p36 = [505.58,-351.071,-605.977,178.6745,0.0167,-0.0419,0]
p37 = [504.623,-349.837,-524.059,178.6988,-0.004,-0.0348,0]
p38 = [504.62,-349.832,-523.982,178.699,-0.0043,-0.0359,0]
p39 = [504.62,-349.832,-523.982,178.699,-0.0043,-0.0359,0]
p40 = [504.62,-349.832,-523.982,178.699,-0.0043,-0.0359,0]
p41 = [504.62,-349.832,-523.982,178.699,-0.0043,-0.0359,0]
p42 = [504.62,-349.832,-523.982,178.699,-0.0043,-0.0359,0]

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

# set speed
SPEED_XYZ = (10, 50, 500)
SPEED_R_XYZE = (10, 50, 100)

# speed_class = FS100.MOVE_SPEED_CLASS_MILLIMETER
speed = SPEED_XYZ[2]

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
post_40 = rob_command(p40)
post_41 = rob_command(p41)
post_42 = rob_command(p42)

#             # ==counter position==
counter = 0
#             # servo on check
if FS100.ERROR_SUCCESS == robot.get_status(status):
    if not status['servo_on']:
        robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

index = 0
tredON = False

postMove = [post_1,post_2, post_3,post_4,post_5,post_6,post_7,post_8]

#post_3,post_4,post_5]

# post_6,post_7,post_8,post_9,post_10,
#              post_11,post_12,post_13,post_14,post_15,post_16,post_17,post_18,post_19,post_20,
#              post_21,post_22,post_23,post_24,post_25,post_26,post_27,post_28,post_29,post_30,
#             post_31,post_32,post_33,post_34,post_35,post_36,post_37,post_38,post_39, post_40,
#             post_41,post_42]

for i in postMove:
    # time_d = time_robot(speed, dist[index], delay_rob)
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

robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_ON)
# a hold off in case we switch to teach/play mode
robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_OFF)