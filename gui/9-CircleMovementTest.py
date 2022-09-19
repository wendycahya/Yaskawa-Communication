from utilsFS100 import FS100
import os
import threading
import time
import math
from datetime import datetime

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
    x_coor = (post2[0] - post1[0])
    y_coor = (post2[1] - post1[1])
    z_coor = (post2[2] - post1[2])
    rx_coor = (post2[3] - post1[3])
    ry_coor = (post2[4] - post1[4])
    rz_coor = (post2[5] - post1[5])
    re_coor = (post2[6] - post1[6])

    dist = math.sqrt(math.pow((post2[0] - post1[0]), 2) + math.pow((post2[1] - post1[1]), 2) + math.pow((post2[2] - post1[2]), 2))

    move_coor = (int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor))
    return move_coor, int(dist)

def time_robot(speed, distance):
    distance = distance / 1000
    speed = speed / 10
    time = (distance / speed)
    return time

def rob_command(post1):
    x_coor = post1[0] * 1000
    y_coor = post1[1] * 1000
    z_coor = post1[2] * 1000
    rx_coor = post1[3] * 10000
    ry_coor = post1[4] * 10000
    rz_coor = post1[5] * 10000
    re_coor = post1[6] * 10000

    robot_command = [(int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor))]
    return robot_command

def update_pos():
    while stop_sign.acquire(blocking=False):
        stop_sign.release()
        # let button up take effect
        time.sleep(0.02)

def is_alarmed():
    alarmed = True
    status = {}
    if FS100.ERROR_SUCCESS == robot.get_status(status):
        alarmed = status['alarming']
    return alarmed

def on_reset_alarm(self):
    self.robot.reset_alarm(FS100.RESET_ALARM_TYPE_ALARM)
    time.sleep(0.1)
    # reflect the ui
    is_alarmed()


# robot connection
robot = FS100('192.168.255.1')
stop_sign = threading.Semaphore()

pos_info = {}
robot_no = 1

post_1 = [0, 0, 0, 0, 0, 0, 0]
if FS100.ERROR_SUCCESS == robot.read_position(pos_info, robot_no):
    x, y, z, rx, ry, rz, re = pos_info['pos']
    post_ori = (x, y, z, rx, ry, rz, re)
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

MAX_XYZ = 90000
MAX_R_XYZE = 180000
SPEED_XYZ = (10, 150, 500)
SPEED_R_XYZE = (10, 50, 100)

dx = -6000
dy = -433741
dz = -471740

if dx != 0 or dy != 0 or dz != 0:
    speed_class = FS100.MOVE_SPEED_CLASS_PERCENT
    speed = SPEED_XYZ[2]
else:
    speed_class = FS100.MOVE_SPEED_CLASS_DEGREE
    speed = SPEED_R_XYZE[1]

# ====== datapoint ======
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
p40 = [484.636,-107.57,-232.795,-179.9999,0.003,179.9988,0]
p41 = [461.272,-109.815,-232.797,180,0.0019,179.9982,0]
p42 = [443.311,-108.321,-232.8,179.9999,0.0015,179.9982,0]
p43 = [424.35,-102.966,-232.802,179.9998,0.0009,179.9991,0]
p44 = [403.698,-92.896,-232.799,179.9999,0.0011,179.9988,0]
p45 = [389.853,-82.602,-232.801,179.9999,0.0009,179.9989,0]
p46 = [375.287,-66.728,-232.797,180,0.0016,179.9989,0]
p47 = [363.663,-48.383,-232.797,180,0.0017,179.9984,0]
p48 = [357.454,-32.567,-232.801,179.9998,0.0003,179.9971,0]
p49 = [353.139,-10.412,-232.797,179.9998,0.0016,179.998,0]
p50 = [352.833,9.588,-232.799,179.9997,0.0013,179.9998,0]
p51 = [356.379,28.46,-232.8,179.9997,0.0015,179.9997,0]
p52 = [364.089,48.928,-232.8,179.9997,0.0007,179.9995,0]
p53 = [373.321,64.516,-232.8,179.9995,0.0015,179.9992,0]
p54 = [385.686,78.546,-232.798,179.9995,0.0011,-179.9996,0]
p55 = [402.779,92.206,-232.797,179.9995,0.001,-179.9998,0]
p56 = [419.114,101.117,-232.797,179.9994,0.001,179.9988,0]
p57 = [437.329,106.886,-232.798,179.9993,0.0015,-179.9998,0]
p58 = [460.201,109.847,-232.795,179.9992,0.0017,179.9991,0]
p59 = [462.414,116.575,-132.798,179.999,0.0013,179.9992,0]
p60 = [465.434,139.253,-232.798,179.9991,0.0015,179.9999,0]
p61 = [486.266,137.277,-232.797,179.999,0.0021,179.9985,0]
p62 = [503.132,133.213,-232.798,179.999,0.002,179.9993,0]
p63 = [522.537,125.46,-232.794,179.9992,0.0013,179.9989,0]
p64 = [540.846,114.863,-232.797,179.999,0.0023,179.9998,0]
p65 = [558.328,100.947,-232.796,179.9991,0.0019,179.9994,0]
p66 = [570.774,87.593,-232.793,179.9991,0.002,-179.9996,0]
p67 = [581.04,72.955,-232.792,179.9993,0.0006,179.9998,0]
p68 = [590.436,54.436,-232.797,179.9992,0.0024,179.9993,0]
p69 = [597.489,33.239,-232.799,179.9993,0.0008,-179.9991,0]
p70 = [601.204,11.871,-232.794,179.9993,0.0022,179.9996,0]
p71 = [601.639,-5.443,-232.796,179.9994,0.0008,179.9988,0]
p72 = [599.731,-23.222,-232.796,179.9994,0.0022,-179.9997,0]
p73 = [594.439,-43.808,-232.801,179.9995,0.0017,179.9997,0]
p74 = [585.556,-64.752,-232.796,179.9996,0.0021,179.9989,0]
p75 = [574.326,-82.997,-232.795,179.9997,0.0023,179.9992,0]
p76 = [563.044,-96.296,-232.799,179.9996,0.0015,179.999,0]
p77 = [546.735,-110.631,-232.798,179.9997,0.0016,179.9975,0]
p78 = [529.328,-121.932,-232.798,179.9998,0.0017,179.9979,0]
p79 = [509.288,-131.122,-232.797,179.9998,0.0014,179.9993,0]
p80 = [491.946,-136.16,-232.799,179.9998,0.0013,179.9994,0]
p81 = [474.793,-138.734,-232.801,179.9999,0.0014,179.9989,0]
p82 = [452.498,-138.739,-232.796,180,0.0017,179.997,0]
p83 = [430.309,-135.369,-232.798,180,0.0013,179.9982,0]
p84 = [410.017,-129.101,-232.798,179.9997,0.0002,179.9989,0]
p85 = [394.857,-121.847,-232.795,180,0.0013,179.9989,0]
p86 = [376.768,-109.616,-232.795,-179.9998,0.0019,179.9988,0]
p87 = [360.675,-94.846,-232.796,-179.9998,0.0019,179.9984,0]
p88 = [346.682,-77.517,-232.801,180,0.0012,179.9983,0]
p89 = [337.962,-62.616,-232.798,-179.9998,0.0029,179.9989,0]
p90 = [330.174,-43.271,-232.798,180,0.0012,179.9966,0]
p91 = [325.003,-21.535,-232.798,179.9999,0.0013,179.9966,0]
p92 = [323.126,1.171,-232.8,179.9998,0.0012,179.9992,0]
p93 = [324.318,18.459,-232.799,179.9997,0.0022,179.999,0]
p94 = [328.163,36.84,-232.799,179.9996,0.0022,179.9996,0]
p95 = [335.952,57.888,-232.797,179.9997,0.0008,179.9991,0]
p96 = [347.237,78.209,-232.798,179.9998,0.0001,-179.9997,0]
p97 = [358.778,93.125,-232.797,179.9995,0.0012,-179.9994,0]
p98 = [371.246,105.301,-232.799,179.9993,0.0016,-179.9988,0]
p99 = [388.397,117.791,-232.798,179.9993,0.0013,179.9997,0]
p100 = [408.676,128.319,-232.8,179.9992,0.0017,179.9987,0]
p101 = [429.236,135.32,-232.799,179.9995,0.0005,179.9991,0]
p102 = [446.864,138.4,-232.795,179.9993,0.0011,-179.9993,0]
p103 = [462.418,139.332,-192.114,179.9991,0.0014,-179.9999,0]
p104 = [462.417,139.332,-132.797,179.999,0.0014,-179.9998,0]



status = {}


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
post_43 = rob_command(p43)
post_44 = rob_command(p44)
post_45 = rob_command(p45)
post_46 = rob_command(p46)
post_47 = rob_command(p47)
post_48 = rob_command(p48)
post_49 = rob_command(p49)
post_50 = rob_command(p50)

post_51 = rob_command(p51)
post_52 = rob_command(p52)
post_53 = rob_command(p53)
post_54 = rob_command(p54)
post_55 = rob_command(p55)
post_56 = rob_command(p56)
post_57 = rob_command(p57)
post_58 = rob_command(p58)
post_59 = rob_command(p59)
post_60 = rob_command(p60)

post_61 = rob_command(p61)
post_62 = rob_command(p62)
post_63 = rob_command(p63)
post_64 = rob_command(p64)
post_65 = rob_command(p65)
post_66 = rob_command(p66)
post_67 = rob_command(p67)
post_68 = rob_command(p68)
post_69 = rob_command(p69)
post_70 = rob_command(p70)

post_71 = rob_command(p71)
post_72 = rob_command(p72)
post_73 = rob_command(p73)
post_74 = rob_command(p74)
post_75 = rob_command(p75)
post_76 = rob_command(p76)
post_77 = rob_command(p77)
post_78 = rob_command(p78)
post_79 = rob_command(p79)
post_80 = rob_command(p80)

post_81 = rob_command(p81)
post_82 = rob_command(p82)
post_83 = rob_command(p83)
post_84 = rob_command(p84)
post_85 = rob_command(p85)
post_86 = rob_command(p86)
post_87 = rob_command(p87)
post_88 = rob_command(p88)
post_89 = rob_command(p89)
post_90 = rob_command(p90)

post_91 = rob_command(p91)
post_92 = rob_command(p92)
post_93 = rob_command(p93)
post_94 = rob_command(p94)
post_95 = rob_command(p95)
post_96 = rob_command(p96)
post_97 = rob_command(p97)
post_98 = rob_command(p98)
post_99 = rob_command(p99)
post_100 = rob_command(p100)

post_101 = rob_command(p101)
post_102 = rob_command(p102)
post_103 = rob_command(p103)
post_104 = rob_command(p104)


print("Post ori before move: ", post_ori)
print("Post 1 before move: ", post_1)
print("Post 2 before move: ", post_2)
print("Post 2 before move: ", post_3)
print("Post 2 before move: ", post_4)

#list_move = [pos1_move, pos2_move, pos3_move]

if FS100.ERROR_SUCCESS == robot.get_status(status):
    if not status['servo_on']:
        robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

# move0, distance0 = move_distance(post_ori, post_1)
# move1, distance1 = move_distance(post_1, post_2)
# move2, distance2 = move_distance(post_2, post_3)
# move3, distance3 = move_distance(post_3, post_4)
# print("list move 1: ", move1)
# print("========================")
# print("list move 2: ", move2)
# print("=========================")
# print("list move 2: ", move3)
# print("=========================")
list_move = [post_1,post_2,post_3,post_4,post_5,post_6,post_7,post_8,post_9,post_10,
             post_11,post_12,post_13,post_14,post_15,post_16,post_17,post_18,post_19,post_20,
             post_21,post_22,post_23,post_24,post_25,post_26,post_27,post_28,post_29,post_30,
             post_31,post_32,post_33,post_34,post_35,post_36,post_37,post_38,post_39,post_40,
             post_41,post_42,post_43,post_44,post_45,post_46,post_47,post_48,post_49,post_50,
             post_51,post_52,post_53,post_54,post_55,post_56,post_57,post_58,post_59,post_60,
             post_61,post_62,post_63,post_64,post_65,post_66,post_67,post_68,post_69,post_70,
             post_71,post_72,post_73,post_74,post_75,post_76,post_77,post_78,post_79,post_80,
             post_81,post_82,post_83,post_84,post_85,post_86,post_87,post_88,post_89,post_90,
             post_91,post_92,post_93,post_94,post_95,post_96,post_97,post_98,post_99,post_100,
             post_101,post_102,post_103,post_104]
#list_move = [move1, move2, move3]
#dist = [distance0, distance1, distance2, distance3]
#print(dist)
index = 0
start = datetime.now()
stop = datetime.now()

for x in list_move:
#declare the post

#function movement and distance
#time calculation
    if status == FS100.TRAVEL_STATUS_START:
        start = datetime.now()
    #time_d = time_robot(speed, dist[index])
    robot.move(None, FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,FS100.MOVE_SPEED_CLASS_PERCENT, speed, x)
    if status == FS100.TRAVEL_STATUS_END:
        stop = datetime.now()
    print("nilai start", start)
    print("nilai stop", stop)
    diff_seconds = (stop - start)
    robot_time = diff_seconds.seconds + 3
    print("nilai second", robot_time)
    time.sleep(robot_time)  # robot may not update the status
    index = index + 1
    print("Finished step ", index)


if FS100.ERROR_SUCCESS == robot.read_position(pos_info, robot_no):
    x, y, z, rx, ry, rz, re = pos_info['pos']

    str_update = "CURRENT POSITION\n" + \
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

print(str_update)

robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_ON)
# a hold off in case we switch to teach/play mode
robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_OFF)