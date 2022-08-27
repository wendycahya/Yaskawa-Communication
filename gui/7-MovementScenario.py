from fs100 import FS100
import os
import threading
import time
import math

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

    post1 = list(post1)
    post2 = list(post2)

    x_coor = (post2[0] - post1[0])
    y_coor = (post2[1] - post1[1])
    z_coor = (post2[2] - post1[2])
    rx_coor = (post2[3] - post1[3])
    ry_coor = (post2[4] - post1[4])
    rz_coor = (post2[5] - post1[5])
    re_coor = (post2[6] - post1[6])

    dist = math.sqrt(math.pow((post2[0] - post1[0]), 2) + math.pow((post2[1] - post1[1]), 2) + math.pow((post2[2] - post1[2]), 2))

    move_coor = tuple(int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor))
    return move_coor, int(dist)

def time_robot(speed, distance):
    distance = distance / 1000
    speed = speed / 10
    time = (distance / speed) + 0.5
    return time

def rob_command(post1):
    x_coor = post1[0] * 1000
    y_coor = post1[1] * 1000
    z_coor = post1[2] * 1000
    rx_coor = post1[3] * 1000
    ry_coor = post1[4] * 1000
    rz_coor = post1[5] * 1000
    re_coor = post1[6] * 1000

    robot_command = (int(x_coor), int(y_coor), int(z_coor), int(rx_coor), int(ry_coor), int(rz_coor), int(re_coor))
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
    speed_class = FS100.MOVE_SPEED_CLASS_MILLIMETER
    speed = SPEED_XYZ[2]
else:
    speed_class = FS100.MOVE_SPEED_CLASS_DEGREE
    speed = SPEED_R_XYZE[1]


status = {}

#post_1 = (457413, -5034, 293332, 1786969, -24, -303, 0)

#===== point movement mm =====
pointHome = [457.413, -5.049, 293.318, 178.6969, -0.0019, -0.0283, 0]
point1 = [405.916, -387.580, 345.821, 178.6969, -0.0029, -0.0306, 0]
point2 = [405.929, -387.592, -201.391, 178.7000, 0.0002, -0.0261, 0]
point4 = [405.919, 253.778, 345.819, 178.6967, -0.0031, -0.0308, 0]
point5 = [405.932, 253.783, -201.391, 178.6961, -0.0045, -0.0303, 0]



# ===== convert robot command =====
robHome = rob_command(pointHome)
rob1 = rob_command(point1)
rob2 = rob_command(point2)
rob4 = rob_command(point4)
rob5 = rob_command(point5)

# ===== move and distance =========


# ===== list movement task ========

post_1 = post_ori
print("Post 1: ", post_1)
post_2 = post_1
print("Post 2[2]: ", post_2[2])
lst = list(post_2)
lst[2] = lst[2] + 50000
post_2 = tuple(lst)
print("Post new post 2: ", post_2)
# post_2[2] = post_2[2] + 10000
print("Post 2: ", post_2)

post_3 = post_2
lst = list(post_3)
lst[2] = lst[2] + 50000
post_3 = tuple(lst)

post_4 = post_3
lst = list(post_4)
lst[1] = lst[1] + 50000
post_4 = tuple(lst)

print("Post ori before move: ", post_ori)
print("Post 1 before move: ", post_1)
print("Post 2 before move: ", post_2)
print("Post 2 before move: ", post_3)
print("Post 2 before move: ", post_4)

#list_move = [pos1_move, pos2_move, pos3_move]

if FS100.ERROR_SUCCESS == robot.get_status(status):
    if not status['servo_on']:
        robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

pos_updater = threading.Thread(target=update_pos)
index = 0
tredON = False

move1, distance1 = move_distance(post_1, post_2)
move2, distance2 = move_distance(post_2, post_3)
move3, distance3 = move_distance(post_3, post_4)
print("list move 1: ", move1)
print("========================")
print("list move 2: ", move2)
print("=========================")
print("list move 2: ", move3)
print("=========================")
list_move = [move1, move2, move3]
dist = [distance1, distance2, distance3]
print(dist)

for x in list_move:
#declare the post

#function movement and distance
#time calculation
    time_d = time_robot(speed, dist[index])
    print(time_d)
    if FS100.ERROR_SUCCESS == robot.one_move(FS100.MOVE_TYPE_LINEAR_INCREMENTAL_POS,FS100.MOVE_COORDINATE_SYSTEM_ROBOT, speed_class, speed, x):
        time.sleep(time_d)  # robot may not update the status
        if not is_alarmed() and tredON == False:
            pos_updater.start()
            tredON = True
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