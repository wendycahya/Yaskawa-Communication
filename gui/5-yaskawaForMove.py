import numpy as np

from fs100 import FS100
import os
import threading
import time
from decimal import Decimal

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

def on_reset_alarm():
    robot.reset_alarm(FS100.RESET_ALARM_TYPE_ALARM)
    time.sleep(0.1)
    # reflect the ui
    is_alarmed()

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


def move_distance(post1, post2):
    x_coor = post2[0] - post1[0]
    y_coor = post2[1] - post1[1]
    z_coor = post2[2] - post1[2]
    rx_coor = post2[3] - post1[3]
    ry_coor = post2[4] - post1[4]
    rz_coor = post2[5] - post1[5]
    re_coor = post2[6] - post1[6]

    move_coor = [x_coor, y_coor, z_coor, rx_coor, ry_coor, rz_coor, re_coor]
    return move_coor

# robot connection
robot = FS100('192.168.255.1')
stop_sign = threading.Semaphore()

pos_info = {}
robot_no = 1

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

pos1_move = (dx, dy, dz, 0, 0, 0, 0)
pos2_move = (0, 648012, 0, 0, 0, 0, 0)
pos3_move = (63521, -214271, 471780, 0, 0, 0, 0)
status = {}

list_move = [pos1_move, pos2_move, pos3_move]

if FS100.ERROR_SUCCESS == robot.get_status(status):
    if not status['servo_on']:
        robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

pos_updater = threading.Thread(target=update_pos)
index = 0
TredON = False

for x in list_move:
    if FS100.ERROR_SUCCESS == robot.one_move(FS100.MOVE_TYPE_LINEAR_INCREMENTAL_POS,FS100.MOVE_COORDINATE_SYSTEM_ROBOT, speed_class, speed, x):
        time.sleep(30)  # robot may not update the status
        if not is_alarmed() and TredON == False:
            pos_updater.start()
            TredON = True

    index = index + 1
    print("robot step ", index)

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