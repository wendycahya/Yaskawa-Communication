import threading
from datetime import datetime
import time as t
import math as mt

import random
import csv

import numpy as np
import cv2
import matplotlib.pyplot as plt

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

speed = 0
counter = 0
stop_sign = threading.Semaphore()
stopwatch_time = t.time()
#=== Draw Real-time graph show ===
# Create a figure and axes for live plotting
fig, ax = plt.subplots()
ax2 = ax.twinx()
# Create an empty list to store data for plotting
#dataD = []
dataVR = []

# Function to update the plot
def update_plot():
    ax.clear()
    #ax.plot(dataD, 'b-')
    ax2.plot(dataVR, 'r--')
    # ax2.plot(dataX, 'r')
    # ax2.plot(dataY, 'g')
    # ax2.plot(dataZ, 'b')
    plt.axis('on')  # Turn off axis labels and ticks
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (mm)")
    ax2.set_ylabel("Speed (mm/s)")
    plt.tight_layout()  # Adjust the plot to remove any padding
    plt.savefig('temp_plot.png')  # Save the plot as an image


#====================ROBOT POSITION==========================

 # Initialize the robot model
pos_info = {}
robot_no = 1
status = {}
x, y, z, rx, ry, rz, re = 0, 0, 0, 0, 0, 0, 0
delay_rob = 0.1
speed = 0

# # ===== Movement Position List =====
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

distance_traveled, time_diff_ms, time_diff_s, velocity = 0, 0, 0, 0


def velocity_group(data):
    while len(data) < 3:
        start_time = datetime.now()
        a = random.random()
        X_first = [a+1, a+2, a+10]
        print("Lokasi pertama: ", X_first)
        # Measure the time taken

        #t.sleep(3)  # Simulate some time delay during the movement
        X_second = [a+3, a+2, a+7]
        print("Lokasi kedua: ", X_second)
        end_time = datetime.now()
        distance_traveled = mt.sqrt(
            (X_second[0] - X_first[0]) ** 2 + (X_second[1] - X_first[1]) ** 2 + (X_second[2] - X_first[2]) ** 2)
        print("Distance Traveled ", distance_traveled)
        time_diff_ms = get_time_difference_ms(start_time, end_time)
        time_diff_s = time_diff_ms / 1000  # converting milliseconds to seconds
        print(time_diff_s)
        velocity = calculate_velocity(distance_traveled, time_diff_s)
        print("Total velocity: ", velocity)
        data.append(velocity)

    velocity_avg = sum(data) / len(data)
    return velocity_avg

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
        #
        # # ===== list movement task ========
        pos_updater = threading.Thread(target=update_pos)
        tredON = False
        #post_1, post_2, post_3, post_4, post_5, post_6, post_7, post_8, post_9, post_10, post_11, post_12, post_13, post_14, post_15, post_16, post_17, post_18, post_19, post_20,
        #
        global counter
        # time_d = time_robot(speed, dist[index], delay_rob)
        # if status == FS100.TRAVEL_STATUS_START:
        #     start = datetime.now()
        # print("nilai x yang masuk ", index, "sebesar ", i)
        #post_1,
        postMove = [post_2, post_3, post_4, post_5, post_6]
        while self.__running.isSet():

            for i in postMove:
                self.__flag.wait()
                # read robot start time
                print("Robot move on ", i)

                # t.sleep(0.20)  # robot may not update the status
                # print("Finished step ", index)
                #             #exception
                if i == post_6:
                    counter = counter + 1
                    ## counter information
                    print("Robot counter step: ", counter)
                    break


    def pause(self):
        self.__flag.clear()     # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()    # 设置为True, 让线程停止阻塞

    def stop(self):
        self.__flag.set()       # 将线程从暂停状态恢复, 如何已经暂停的话
        self.__running.clear()        # 设置为False


start = t.strftime("%Y%m%d-%H%M%S")
write_file = "VelocityAnalysis-"+str(start)+".csv"
velocity = 0
if __name__ == '__main__':
    server = Job()
    server.start()
    cap = cv2.VideoCapture(2)
    cap.set(3, 640)  # width
    cap.set(4, 480)  # height
    with open(write_file, "wt", encoding="utf-8") as output:
        while True:
            success, img = cap.read()
            height, width, channels = img.shape

            speed = 1500
            elapsed_time = round(t.time() - stopwatch_time, 3)
            data = []

            velocity = velocity_group(data)
            #velocity = random.random()
            dataVR.append(velocity)

            # Update the plot
            update_plot()
            end_time = datetime.now()
            output.write(str(end_time.strftime("%H:%M:%S")) + ',' + str(elapsed_time) + ',' + str(velocity) + '\n')
            # Load the saved plot image
            plot_img = cv2.imread('temp_plot.png', cv2.IMREAD_UNCHANGED)

            # Resize the plot image to match the video frame size
            plot_img = cv2.resize(plot_img, (img.shape[1], img.shape[0]))

            # Display the video frame in the 'Video Stream' window
            cv2.imshow('Video Stream', img)

            # Display the plot in the 'Live Plot' window
            cv2.imshow('HR Distance and Robot Velocity', plot_img[:, :, :3])


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

