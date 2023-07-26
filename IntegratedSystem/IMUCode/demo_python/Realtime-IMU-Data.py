import numpy as np
import matplotlib.pyplot as plt
from anrot_module import *
import time
import json
import math

#=========================== Draw Real-time graph show ========================
fig, ax = plt.subplots()
ax2 = ax.twinx()
# Create an empty list to store data for plotting
dataD = []
dataVR = []
dataSPD = []
dataTime = []

# Function to update the plot
def update_plot():
    ax.clear()
    ax.plot(dataTime, dataD, 'b-', label='Distance')
    ax2.plot(dataTime, dataVR, 'r--', label='Velocity')
    ax2.plot(dataTime, dataSPD, 'g-.', label='Speed Command')
    plt.axis('on')  # Turn off axis labels and ticks
    # Add legends to the plot
    # ax.legend(loc='upper left')
    # ax2.legend(loc='upper right')

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (mm)")
    ax2.set_ylabel("Speed (mm/s)")
    plt.tight_layout()  # Adjust the plot to remove any padding
    plt.savefig('temp_plot.png')  # Save the plot as an image

#log_file = 'chlog.csv'

# Time interval in seconds
delta_t = 0.01

# Initialize velocity components (initialized to zero)
vx = 0.0
vy = 0.0
vz = 0.0
gravity = 9.81 * 1000
if __name__ == '__main__':

    m_IMU = anrot_module('./config.json')
    print("Press Ctrl-C to terminate while statement.")

    # uncomment following line to enable csv logger
    # m_IMU.create_csv(log_file) #uncomment this line to enable

    while True:
        try:
            data = m_IMU.get_module_data(10)

            # print("Data X:", data['acc'][0]['X'])
            # print("Data Y:", data['acc'][0]['Y'])
            # print("Data Z:", data['acc'][0]['Z'])

            ax = data['acc'][0]['X'] * gravity
            ay = data['acc'][0]['Y'] * gravity
            az = data['acc'][0]['Z'] * gravity

            # Integrate acceleration to calculate velocity
            vx = ax * delta_t
            vy = ay * delta_t
            vz = az * delta_t

            # Calculate the magnitude of velocity (speed)
            speed = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

            print("Data speed:", speed)

            # uncomment following line to enable csv logger
            #m_IMU.write2csv(data, log_file)

            #print(data['euler'], end="\r")  # print 'euler'. It can be replaced with 'id', 'timestamp', 'acc', 'gyr', 'mag', 'quat'.
            # print(data, end="\r") #print all

        except KeyboardInterrupt:
            print("Serial is closed.")
            m_IMU.close()
            break
        except:
            print("Error")
            pass
