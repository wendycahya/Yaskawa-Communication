import time
from datetime import datetime

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

# Example usage
start_datetime = datetime.now() # Example starting datetime
time.sleep(10)
end_datetime = datetime.now()

distance_traveled = 10 # in meters

time_diff_ms = get_time_difference_ms(start_datetime, end_datetime)
time_diff_s = time_diff_ms / 1000  # converting milliseconds to seconds

velocity = calculate_velocity(distance_traveled, time_diff_s)
print(f"The robot start is {start_datetime} s.\n")
print(f"The robot start is {end_datetime} s.\n")
print(f"The velocity of the robot is {velocity} mm/s.")