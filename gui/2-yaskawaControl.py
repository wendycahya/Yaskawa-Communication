from fs100 import FS100
import os
import threading
import time


def update_pos(self):
    while stop_sign.acquire(blocking=False):
        stop_sign.release()
        # let button up take effect
        time.sleep(0.02)

def is_alarmed(self):
    alarmed = False
    status = {}
    if FS100.ERROR_SUCCESS == self.robot.get_status(status):
        alarmed = status['alarming']
    return alarmed

def on_reset_alarm(self):
    self.robot.reset_alarm(FS100.RESET_ALARM_TYPE_ALARM)
    time.sleep(0.1)
    # reflect the ui
    self.is_alarmed()

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
SPEED_XYZ = (10, 100, 500)
SPEED_R_XYZE = (10, 50, 100)

dx = 0
dy = 0
dz = 50000

if dx != 0 or dy != 0 or dz != 0:
    speed_class = FS100.MOVE_SPEED_CLASS_MILLIMETER
    speed = SPEED_XYZ[1]
else:
    speed_class = FS100.MOVE_SPEED_CLASS_DEGREE
    speed = SPEED_R_XYZE[1]

pos1_move = (dx, dy, dz, 0, 0, 0, 0)
pos2_move = (dx, dy, dz, 0, 0, 0, 0)
status = {}

if FS100.ERROR_SUCCESS == robot.get_status(status):
    if not status['servo_on']:
        robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)

pos_updater = threading.Thread(target=update_pos)
if FS100.ERROR_SUCCESS == robot.one_move(FS100.MOVE_TYPE_LINEAR_INCREMENTAL_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT, speed_class, speed, pos1_move):
    time.sleep(0.1)  # robot may not update the status
    if not is_alarmed():
        pos_updater.start()

robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_ON)
# a hold off in case we switch to teach/play mode
robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_OFF)