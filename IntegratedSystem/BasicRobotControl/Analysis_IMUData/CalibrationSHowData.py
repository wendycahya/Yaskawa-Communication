import numpy as np
import matplotlib.pyplot as plt

# Sample calibration data
time = np.array([1, 2, 3, 4, 5])  # Time data (e.g., seconds)
accel_x = np.array([0.1, 0.5, 1.0, 1.5, 2.0])  # Calibration data for accelerometer X-axis
gyro_y = np.array([0.2, 0.6, 1.2, 1.8, 2.4])   # Calibration data for gyroscope Y-axis

# Create the plot
plt.figure(figsize=(8, 6))  # Optional: Set the figure size

# Plot accelerometer calibration data
plt.plot(time, accel_x, label='Accelerometer X-axis')

# Plot gyroscope calibration data
plt.plot(time, gyro_y, label='Gyroscope Y-axis')

# Add labels and title
plt.xlabel('Time (seconds)')
plt.ylabel('Calibration Data')
plt.title('IMU Calibration Data')
plt.legend()  # Add a legend to the plot

# Show the plot
plt.grid(True)  # Optional: Add grid lines to the plot
plt.show()