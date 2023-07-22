import time as t
import math as mt
from utilsFS100 import FS100
import threading

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

# robot connection
#robot = FS100('192.168.255.1')
p1 = [353.427, -298.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p2 = [353.427, -198.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p3 = [353.427, -98.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p4 = [353.427, 2.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p5 = [353.427, 102.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p6 = [353.427, 202.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]
p6 = [353.427, 302.333, -307.424, -180.0132, -4.4338, -24.0585, 0.0000]

## ===== convert robot command =====
#post_1 = rob_command(p1)
post_2 = rob_command(p2)
post_3 = rob_command(p3)
post_4 = rob_command(p4)
post_5 = rob_command(p5)
post_6 = rob_command(p6)


robot = FS100('172.16.0.1')
stop_sign = threading.Semaphore()

pos_info = {}
robot_no = 1
status = {}
status_move = {}
counter = 0

postMove = [post_2, post_3, post_4, post_5, post_6]

speedUpdate = 200

def robot_working():
    global counter, paused, resume_event

    if FS100.ERROR_SUCCESS == robot.get_status(status):
        if not status['servo_on']:
            robot.switch_power(FS100.POWER_TYPE_SERVO, FS100.POWER_SWITCH_ON)
    t.sleep(3)
    while True:
        for i in postMove:
            #print(speedUpdate)
            robot.move(None, FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
                       FS100.MOVE_SPEED_CLASS_MILLIMETER, speedUpdate, i, wait=True)

            if i == post_6:
                counter = counter + 1
                ## counter information
                #print("Robot counter step: ", counter)
                break


                #while pause_event.is_set():

                #
                # if speedUpdate == 0:
                #     robot.stop()
                # else:
                #     robot.move(None, FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS, FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
                #                FS100.MOVE_SPEED_CLASS_MILLIMETER, speedUpdate, i, wait=True)
                #
                #     print("Status Success Movement= ", status['one_cycle'])

            robot.switch_power(FS100.POWER_TYPE_HOLD, FS100.POWER_SWITCH_OFF)

class VelocityCalculator:
    def __init__(self):
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.prev_time = None

    def update_position(self, x, y, z):
        if self.prev_x is None or self.prev_y is None or self.prev_z is None:
            # If it's the first position measurement, store it and return None as velocity
            self.prev_x = x
            self.prev_y = y
            self.prev_z = z
            self.prev_time = t.time()
            return None, None, None

        # Calculate the distance traveled along each axis
        delta_x = x - self.prev_x
        delta_y = y - self.prev_y
        delta_z = z - self.prev_z

        # Calculate the time elapsed
        current_time = t.time()
        delta_time = current_time - self.prev_time

        # Calculate the velocity components along each axis
        velocity_x = delta_x / delta_time
        velocity_y = delta_y / delta_time
        velocity_z = delta_z / delta_time

        # Update previous positions and time
        self.prev_x = x
        self.prev_y = y
        self.prev_z = z
        self.prev_time = current_time

        return velocity_x, velocity_y, velocity_z

    @staticmethod
    def calculate_magnitude_velocity(velocity_x, velocity_y, velocity_z):
        # Calculate the magnitude of velocity using the formula
        magnitude = mt.sqrt(velocity_x ** 2 + velocity_y ** 2 + velocity_z ** 2)
        return magnitude

# Example usage:
velocity_calculator = VelocityCalculator()

resume_event = threading.Event()
resume_event.set()  # Set the event to allow the thread to start
# Start the thread that runs the loop
thread2 = threading.Thread(target=robot_working)
thread2.start()

# Simulate the object moving along the x, y, and z axes over time

while True:
    if FS100.ERROR_SUCCESS == robot.read_position(pos_info, robot_no):
        x, y, z, rx, ry, rz, re = pos_info['pos']
    robotPosA = convert_mm(x, y, z, rx, ry, rz, re)

    x = robotPosA[0]
    y = robotPosA[1]
    z = robotPosA[2]

    # Calculate velocity
    vx, vy, vz = velocity_calculator.update_position(x, y, z)

    if vx is not None and vy is not None and vz is not None:
        # Calculate the magnitude of velocity
        magnitude_velocity = velocity_calculator.calculate_magnitude_velocity(vx, vy, vz)
        print(f"Time: {t.time()}, X: {x}, Y: {y}, Z: {z}")
        print(f"Velocity: Vx: {vx}, Vy: {vy}, Vz: {vz}")
        print(f"Magnitude Velocity: {magnitude_velocity}")
    else:
        # Print when there is not enough data to calculate velocity
        print(f"Time: {t.time()}, X: {x}, Y: {y}, Z: {z}")
        print("Not enough data points to calculate velocity.")

    t.sleep(1)  # Simulate time passing (in a real application, you'd get the position from your data source)
