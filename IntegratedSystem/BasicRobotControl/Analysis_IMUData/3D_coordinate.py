import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

def apply_calibration(raw_measurement, calibration_params):
    # Apply bias correction
    calibrated_measurement = raw_measurement - calibration_params['accel_bias']

    # Apply scale factor correction (if applicable)

    # Apply noise compensation (if applicable)

    return calibrated_measurement

df = pd.read_csv('dataMeneng.csv')

column_acc = df[['accX', 'accY', 'accZ']]
selected_array = column_acc.values
raw_data = np.array(selected_array)
#print(raw_data)

accX = raw_data[:, 0]
accY = raw_data[:, 1]
accZ = raw_data[:, 2]

# Sample accelerometer data for X, Y, and Z axes
accel_x = np.array([0.1, 0.5, 1.0, 1.5, 2.0])  # Replace with your actual data
accel_y = np.array([0.2, 0.6, 1.2, 1.8, 2.4])  # Replace with your actual data
accel_z = np.array([0.3, 0.7, 1.4, 2.1, 2.8])  # Replace with your actual data

# Create a 3D scatter plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Plot the accelerometer data points
ax.scatter(accX, accY, accZ, c='b', marker='o', label='Accelerometer Data')
#ax.scatter(accel_x, accel_y, accel_z, c='b', marker='o', label='Accelerometer Data')
#ax.scatter(accel_x+2, accel_y-1, accel_z-1, c='r', marker='o', label='Calibration Data')

# Step 2: Compute Mean
mean_accel = np.mean(raw_data[:, 0:3], axis=0)

# Step 5: Compute Calibration Parameters
calibration_params = {
    'accel_bias': mean_accel,
    # Include scale factor and noise parameters if applicable
}

# Example usage:
calibrated_data = apply_calibration(raw_data, calibration_params)
print(calibration_params)
cal_accX = calibrated_data[:, 0]
cal_accY = calibrated_data[:, 1]
cal_accZ = calibrated_data[:, 2]

ax.scatter(cal_accX, cal_accX, cal_accX, c='r', marker='o', label='Calibrated Data')

# Set labels and title
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title('3D Accelerometer Data Scatter Plot')

# Add a legend
ax.legend()

# Show the plot
plt.show()