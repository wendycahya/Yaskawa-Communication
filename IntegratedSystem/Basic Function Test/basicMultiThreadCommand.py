import threading
from datetime import datetime
import time as t
import math as mt
import random

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

def calculate_velocity(distance, time):
    """
    Calculates the velocity given the distance and time.
    :param distance: The distance traveled by the robot (in meters).
    :param time: The time taken by the robot to cover the distance (in seconds).
    :return: The velocity of the robot (in meters per second).
    """
    velocity = distance / time
    return velocity

time_diff_ms = 0
velocity = 0
def group_average_velocity(data):
    data = []
    # Perform some computation or task
    while len(data) < 5:
        start_time = datetime.now()
        a = random.random()
        X_first = [a, a+5, a+7]
        print("Lokasi pertama: ", X_first)
        # Measure the time taken

        t.sleep(3)  # Simulate some time delay during the movement
        X_second = [a+1, a+12, a+15]
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

    velocity_return = sum(data) / len(data)

    return velocity_return


if __name__ == '__main__':
    while True:

        data = []
        velocity = group_average_velocity(data)
        # Access the result from the shared variable
        print("Result from main:", velocity)