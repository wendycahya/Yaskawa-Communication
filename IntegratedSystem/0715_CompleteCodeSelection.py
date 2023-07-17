from utilsFS100 import FS100
import threading
from datetime import datetime

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

def move_convert(post_original):
    x = post_original[0] * 1000
    y = post_original[1] * 1000
    z = post_original[2] * 1000
    rx = post_original[3] * 10000
    ry = post_original[4] * 10000
    rz = post_original[5] * 10000
    re = post_original[0] * 10000
    robotPos = (x, y, z, rx, ry, rz, re)
    return robotPos

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

    dist = mt.sqrt(mt.pow((post2[0] - post1[0]), 2) + mt.pow((post2[1] - post1[1]), 2) + mt.pow((post2[2] - post1[2]), 2))

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

# Initialize the robot model
robot = FS100('172.16.0.1')
speed = 0
counter = 0
stop_sign = threading.Semaphore()

#====================ROBOT POSITION==========================
start = datetime.now()

 # Initialize the robot model
pos_info = {}
robot_no = 1
status = {}
x, y, z, rx, ry, rz, re = 0, 0, 0, 0, 0, 0, 0
delay_rob = 0.1
# # ===== Movement Position List =====
# p1 = [723.086, 54.302, 379.545, -167.291, -80.048, -8.599, 0]
# p2 = [730.100, 72.261, 345.694, -167.375, -76.904, -7.293, 0]
# p3 = [735.083, 90.800, 310.952, -167.492, -73.691, -5.964, 0]
# p4 = [737.850, 109.369, 276.195, -167.630, -70.481, -4.654, 0]
# p5 = [738.423,127.829,241.500,-167.792,-67.272,-3.361,0]
# p6 = [736.815,146.175,206.694,-167.988,-64.043,-2.067,0]
# p7 = [733.049,164.151,172.148,-168.207,-60.819,-0.792,0]
# p8 = [727.335,181.233,138.696,-168.447,-57.670,0.436,0]
# p9 = [719.535,197.959,105.147,-168.722,-54.479,1.670,0]
# p10 = [709.706,214.064,71.832,-169.027,-51.267,2.895,0]
# p11=[697.991,229.299,39.12,-169.361,-48.063,4.1,0]
# p12=[684.788,243.265,7.776,-169.72,-44.938,5.261,0]
# p13=[669.596,256.5,-23.542,-170.118,-41.748,6.428,0]
# p14=[652.79,268.577,-54.057,-170.55,-38.564,7.573,0]
# p15=[634.782,279.267,-83.232,-171.011,-35.439,8.68,0]
# p16=[615.129,288.809,-111.902,-171.514,-32.277,9.778,0]
# p17=[594.54,296.878,-139.139,-172.047,-29.172,10.833,0]
# p18=[572.501,303.646,-165.726,-172.63,-26.029,11.882,0]
# p19=[549.641,308.913,-190.971,-173.249,-22.925,12.89,0]
# p20=[525.744,312.712,-215.211,-173.919,-19.81,13.874,0]
p21=[501.472,314.963,-237.871,-174.625,-16.756,14.813,0]
p22=[476.421,315.713,-259.436,-175.387,-13.694,15.722,0]
p23=[450.526,314.917,-279.968,-176.214,-10.606,16.605,0]
p24=[424.692,312.598,-298.828,-177.085,-7.583,17.432,0]
p25=[398.848,308.81,-316.177,-178.011,-4.602,18.209,0]
p26=[372.555,303.483,-332.35,-179.015,-1.6,18.947,0]
p27=[361.073,300.693,-338.943,-179.476,-0.295,19.253,0]
p28=[362.639,302.177,-332.124,-179.437,-0.283,19.255,0]
p29=[365.315,304.713,-319.433,-179.373,-0.259,19.262,0]
p30=[367.712,306.982,-306.722,-179.316,-0.24,19.27,0]
p31=[369.88,309.032,-293.622,-179.264,-0.222,19.278,0]
p32=[371.733,310.782,-280.634,-179.221,-0.206,19.289,0]
p33=[373.323,312.282,-267.472,-179.184,-0.194,19.297,0]
p34=[374.609,313.492,-254.438,-179.155,-0.184,19.307,0]
p35=[375.62,314.441,-241.261,-179.132,-0.175,19.317,0]
p36=[376.342,315.114,-228.338,-179.117,-0.17,19.326,0]
p37=[376.801,315.539,-214.795,-179.108,-0.166,19.333,0]
p38=[376.961,315.682,-201.63,-179.107,-0.164,19.341,0]
p39=[376.845,315.565,-188.352,-179.111,-0.166,19.344,0]
p40=[376.431,315.166,-174.99,-179.122,-0.17,19.349,0]
p41=[375.747,314.514,-161.94,-179.14,-0.175,19.348,0]
p42=[374.776,313.589,-148.719,-179.164,-0.184,19.347,0]
p43=[373.538,312.417,-135.736,-179.194,-0.194,19.342,0]
p44=[371.959,310.925,-122.31,-179.231,-0.208,19.335,0]
p45=[370.156,309.226,-109.44,-179.273,-0.222,19.323,0]
p46=[368.045,307.238,-96.451,-179.321,-0.238,19.31,0]
p47=[365.641,304.976,-83.443,-179.375,-0.259,19.291,0]
p48=[362.947,302.45,-70.539,-179.434,-0.28,19.27,0]
p49=[361.071,300.691,-62.32,-179.476,-0.295,19.253,0]
p50=[364.156,296.667,-62.32,-179.479,-0.294,19.252,0]
p51=[373.679,283.668,-62.315,-179.492,-0.294,19.254,0]
p52=[382.931,270.076,-62.301,-179.504,-0.292,19.253,0]
p53=[391.559,256.39,-62.303,-179.515,-0.289,19.254,0]
p54=[399.823,242.221,-62.29,-179.526,-0.286,19.253,0]
p55=[407.51,227.895,-62.292,-179.535,-0.283,19.252,0]
p56=[414.747,213.226,-62.284,-179.544,-0.28,19.254,0]
p57=[421.352,198.539,-62.282,-179.553,-0.276,19.254,0]
p58=[427.489,183.547,-62.275,-179.56,-0.273,19.253,0]
p59=[433.057,168.48,-62.269,-179.566,-0.27,19.254,0]
p60=[438.121,153.134,-62.268,-179.571,-0.264,19.254,0]
p61=[442.679,137.52,-62.262,-179.576,-0.261,19.253,0]
p62=[446.616,122.014,-62.267,-179.58,-0.255,19.255,0]
p63=[450.045,106.282,-62.262,-179.583,-0.252,19.254,0]
p64=[452.881,90.692,-62.262,-179.586,-0.248,19.254,0]
p65=[455.173,75.036,-62.258,-179.587,-0.244,19.255,0]
p66=[456.968,58.969,-62.254,-179.588,-0.241,19.254,0]
p67=[458.185,42.868,-62.255,-179.588,-0.237,19.253,0]
p68=[458.841,27,-62.252,-179.588,-0.235,19.255,0]
p69=[458.943,11.008,-62.254,-179.586,-0.232,19.253,0]
p70=[458.477,-4.954,-62.257,-179.585,-0.229,19.254,0]
p71=[457.493,-20.631,-62.254,-179.582,-0.228,19.255,0]
p72=[455.926,-36.597,-62.253,-179.579,-0.227,19.255,0]
p73=[453.829,-52.248,-62.261,-179.575,-0.225,19.253,0]
p74=[451.185,-67.914,-62.26,-179.571,-0.225,19.255,0]
p75=[448.028,-83.347,-62.259,-179.566,-0.226,19.254,0]
p76=[444.273,-98.873,-62.264,-179.561,-0.226,19.253,0]
p77=[439.989,-114.231,-62.27,-179.555,-0.228,19.255,0]
p78=[435.283,-129.075,-62.27,-179.548,-0.23,19.254,0]
p79=[430.089,-143.736,-62.27,-179.542,-0.234,19.253,0]
p80=[424.208,-158.63,-62.277,-179.535,-0.237,19.254,0]

p81=[417.913,-173.068,-62.283,-179.527,-0.241,19.254,0]
p82=[411.076,-187.369,-62.29,-179.52,-0.247,19.253,0]
p83=[403.865,-201.195,-62.292,-179.513,-0.252,19.255,0]
p84=[396.25,-214.647,-62.295,-179.506,-0.26,19.254,0]
p85=[388.063,-228.011,-62.303,-179.498,-0.268,19.253,0]
p86=[379.423,-241.063,-62.306,-179.49,-0.276,19.255,0]
p87=[370.413,-253.699,-62.315,-179.482,-0.286,19.253,0]
p88=[362.108,-264.586,-62.32,-179.475,-0.295,19.254,0]
p89=[362.766,-264.992,-64.781,-179.466,-0.31,19.261,0]
p90=[365.782,-266.847,-76.853,-179.42,-0.378,19.29,0]
p91=[368.552,-268.546,-89.275,-179.377,-0.442,19.315,0]
p92=[371.041,-270.067,-101.966,-179.338,-0.5,19.334,0]
p93=[373.172,-271.366,-114.532,-179.305,-0.551,19.349,0]
p94=[375.007,-272.484,-127.344,-179.276,-0.595,19.36,0]
p95=[376.506,-273.392,-140.108,-179.253,-0.632,19.366,0]
p96=[377.665,-274.09,-152.806,-179.234,-0.661,19.368,0]
p97=[378.512,-274.596,-165.718,-179.22,-0.684,19.365,0]
p98=[379.018,-274.893,-178.351,-179.211,-0.698,19.363,0]
p99=[379.204,-274.997,-191.272,-179.207,-0.704,19.356,0]
p100=[379.067,-274.904,-203.892,-179.209,-0.704,19.349,0]
p101=[378.592,-274.605,-216.781,-179.216,-0.694,19.337,0]
p102=[377.799,-274.114,-229.44,-179.227,-0.677,19.327,0]
p103=[376.676,-273.425,-242.146,-179.244,-0.651,19.315,0]
p104=[375.225,-272.537,-254.797,-179.267,-0.617,19.304,0]
p105=[373.454,-271.458,-267.381,-179.295,-0.575,19.291,0]
p106=[371.342,-270.174,-279.974,-179.328,-0.524,19.28,0]
p107=[368.987,-268.746,-292.103,-179.366,-0.466,19.271,0]
p108=[366.193,-267.054,-304.69,-179.41,-0.397,19.261,0]
p109=[363.184,-265.234,-316.702,-179.456,-0.321,19.253,0]
p110=[362.109,-264.585,-320.718,-179.474,-0.295,19.251,0]
p111=[362.109,-264.585,-320.718,-179.474,-0.295,19.251,0]
p112=[362.109,-264.585,-320.718,-179.474,-0.295,19.251,0]
p113=[362.109,-264.585,-320.718,-179.474,-0.295,19.251,0]
p114=[362.109,-264.585,-320.718,-179.474,-0.295,19.251,0]
p115=[362.109,-264.585,-320.718,-179.474,-0.295,19.251,0]

## ===== convert robot command =====
# post_1 = rob_command(p1)
# post_2 = rob_command(p2)
# post_3 = rob_command(p3)
# post_4 = rob_command(p4)
# post_5 = rob_command(p5)
# post_6 = rob_command(p6)
# post_7 = rob_command(p7)
# post_8 = rob_command(p8)
# post_9 = rob_command(p9)
# post_10 = rob_command(p10)
#
# post_11 = rob_command(p11)
# post_12 = rob_command(p12)
# post_13 = rob_command(p13)
# post_14 = rob_command(p14)
# post_15 = rob_command(p15)
# post_16 = rob_command(p16)
# post_17 = rob_command(p17)
# post_18 = rob_command(p18)
# post_19 = rob_command(p19)
# post_20 = rob_command(p20)
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
post_105 = rob_command(p105)
post_106 = rob_command(p106)
post_107 = rob_command(p107)
post_108 = rob_command(p108)
post_109 = rob_command(p109)
post_110 = rob_command(p110)

post_111 = rob_command(p111)
post_112 = rob_command(p112)
post_113 = rob_command(p113)
post_114 = rob_command(p114)
post_115 = rob_command(p115)


class Job(threading.Thread):

    def __init__(self, *args, **kwargs):
        super(Job, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()     # 用于暂停线程的标识
        self.__flag.set()       # 设置为True
        self.__running = threading.Event()      # 用于停止线程的标识
        self.__running.set()      # 将running设置为True

    def run(self):
        t.sleep(5)  # delay for initialization
        global speed
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
        if FS100.ERROR_SUCCESS == robot.get_status(status):
            if not status['servo_on']:
                robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)
        #
        # # ===== list movement task ========
        pos_updater = threading.Thread(target=update_pos)
        index = 0
        tredON = False
        #post_1, post_2, post_3, post_4, post_5, post_6, post_7, post_8, post_9, post_10, post_11, post_12, post_13, post_14, post_15, post_16, post_17, post_18, post_19, post_20,
        postMove = [
                    post_21, post_22, post_23, post_24, post_25, post_26, post_27, post_28, post_29, post_30,
                    post_31, post_32, post_33, post_34, post_35, post_36, post_37, post_38, post_39, post_40,
                    post_41, post_42, post_43, post_44, post_45, post_46, post_47, post_48, post_49, post_50,
                    post_51, post_52, post_53, post_54, post_55, post_56, post_57, post_58, post_59, post_60,
                    post_61, post_62, post_63, post_64, post_65, post_66, post_67, post_68, post_69, post_70,
                    post_71, post_72, post_73, post_74, post_75, post_76, post_77, post_78, post_79, post_80,
                    post_81, post_82, post_83, post_84, post_85, post_86, post_87, post_88, post_89, post_90,
                    post_91, post_92, post_93, post_94, post_95, post_96, post_97, post_98, post_99, post_100,
                    post_101, post_102, post_103, post_104, post_105, post_106, post_107, post_108, post_109,
                    post_110,
                    post_111, post_112, post_113, post_114, post_115
                    ]
        #
        global counter

        while self.__running.isSet():

            for i in postMove:
                self.__flag.wait()
                # time_d = time_robot(speed, dist[index], delay_rob)
                # if status == FS100.TRAVEL_STATUS_START:
                #     start = datetime.now()
                # print("nilai x yang masuk ", index, "sebesar ", i)
                robot.move(None, FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
                           FS100.MOVE_SPEED_CLASS_PERCENT, speed, i)

                t.sleep(0.20)  # robot may not update the status
                index = index + 1
                # print("Finished step ", index)
                #             #exception
                if i == post_115:
                    counter = counter + 1
                    ## counter information
                    print("Robot counter step: ", counter)
                    break

        robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_ON)
        #     # a hold off in case we switch to teach/play mode
        robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_OFF)

    def pause(self):
        self.__flag.clear()     # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()    # 设置为True, 让线程停止阻塞

    def stop(self):
        self.__flag.set()       # 将线程从暂停状态恢复, 如何已经暂停的话
        self.__running.clear()        # 设置为False


# ======== camera detection =====

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
write_file = "1500-NewProductivity-"+str(start)+".csv"
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

                    data.append(D)

                # Calculate the average
                D = sum(data) / len(data)

                # read robot start time
                if FS100.ERROR_SUCCESS == robot.read_position(pos_info, robot_no):
                    x, y, z, rx, ry, rz, re = pos_info['pos']
                    pointHome = (x, y, z, 0, 0, 0, 0)
                    straaa = "CURRENT POSITION\n" + \
                             "COORDINATE {:12s} TOOL:{:02d}\n".format('ROBOT', pos_info['tool_no']) + \
                             "R{} :X     {:4d}.{:03d} mm       Rx   {:4d}.{:04d} deg.\n".format(robot_no,
                                                                                                x // 1000,
                                                                                                x % 1000,
                                                                                                rx // 10000,
                                                                                                rx % 10000) + \
                             "    Y     {:4d}.{:03d} mm       Ry   {:4d}.{:04d} deg.\n".format(
                                 y // 1000, y % 1000, ry // 10000, ry % 10000) + \
                             "    Z     {:4d}.{:03d} mm       Rz   {:4d}.{:04d} deg.\n".format(
                                 z // 1000, z % 1000, rz // 10000, rz % 10000) + \
                             "                            Re   {:4d}.{:04d} deg.\n".format(
                                 re // 10000, re % 10000)

                # print(straaa)

                robotPos = convert_mm(x, y, z, rx, ry, rz, re)
                start_time = datetime.now()
                milliseconds = start_time.microsecond // 1000
                XnRob = [robotPos[0], robotPos[1], robotPos[2]]

                # Distance Calibration results can be integrated here
                offset = 60
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
                    server.pause()
                    Vr = 0
                    speed = 0
                    print("Robot harus berhenti", Vr)
                    mode_collab = 0

                elif D > SpminVal and D <= SpSafeVal:
                    server.resume()
                    # print("Robot speed reduction")
                    Vr = Vr_SSM2(D, Tr, Ts, ac, C_SSM, Zd, Zr)
                    Vr = round(Vr, 2)
                    speed = int(remap(Vr, 0, 1500, 0, 800))
                    print("Robot working on collaboration mode")
                    mode_collab = 1


                elif D > SpSafeVal and D <= SpPFLVal:
                    server.resume()
                    print("Robot speed reduction")
                    # print("Robot speed reduction")
                    mode_collab = 2
                    Vr = Vr_PFL
                    Vr = round(Vr, 2)
                    speed = int(remap(Vr, 0, 1500, 0, 800))


                elif D > SpPFLVal and D <= Spfull:
                    server.resume()
                    Vr = Vr_SSM(D, Vh, Tr, Ts, ac, C_SSM, Zd, Zr, Vr_PFL)
                    Vr = round(Vr, 2)
                    speed = int(remap(Vr, 0, 1500, 0, 800))
                    # print("change value speed Reduce: ", Vr)
                    mode_collab = 3

                else:
                    server.resume()
                    mode_collab = 4
                    # print("Robot bekerja maximal")
                    # mode_collab = 1
                    Vr = RobotVrmax
                    speed = int(remap(Vr, 0, 1500, 0, 800))

            interval = interval + 1

            #read robot position end time
            if FS100.ERROR_SUCCESS == robot.read_position(pos_info, robot_no):
                x, y, z, rx, ry, rz, re = pos_info['pos']
                pointHome = (x, y, z, 0, 0, 0, 0)
                straaa = "CURRENT POSITION\n" + \
                         "COORDINATE {:12s} TOOL:{:02d}\n".format('ROBOT', pos_info['tool_no']) + \
                         "R{} :X     {:4d}.{:03d} mm       Rx   {:4d}.{:04d} deg.\n".format(robot_no,
                                                                                            x // 1000,
                                                                                            x % 1000,
                                                                                            rx // 10000,
                                                                                            rx % 10000) + \
                         "    Y     {:4d}.{:03d} mm       Ry   {:4d}.{:04d} deg.\n".format(
                             y // 1000, y % 1000, ry // 10000, ry % 10000) + \
                         "    Z     {:4d}.{:03d} mm       Rz   {:4d}.{:04d} deg.\n".format(
                             z // 1000, z % 1000, rz // 10000, rz % 10000) + \
                         "                            Re   {:4d}.{:04d} deg.\n".format(
                             re // 10000, re % 10000)

            # print(straaa)

            robotPos = convert_mm(x, y, z, rx, ry, rz, re)
            end_time = datetime.now()

            XnRobEnd = [robotPos[0], robotPos[1], robotPos[2]]

            end_time = datetime.now()

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

            output.write(str(end_time.strftime("%H:%M:%S")) + ',' + str(elapsed_time) + ',' + str(D) + ',' + str(speed) + ',' +
                         str(counter) + ',' + str(round(velocity, 3)) + ',' + str(robotPos[0]) + ',' + str(robotPos[1]) + ',' + str(
                    robotPos[2]) + '\n')
            print("SUCCESS RECORD ", interval, " !!!")
            print("SUCCESS RECORD counter", counter, " !!!")
            # Load the saved plot image
            plot_img = cv2.imread('temp_plot.png', cv2.IMREAD_UNCHANGED)

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
