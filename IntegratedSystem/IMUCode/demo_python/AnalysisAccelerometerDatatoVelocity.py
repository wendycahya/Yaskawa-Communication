import pandas as pd

# Read accelerometer data from CSV
df = pd.read_csv("IMUDataAnalaysis-20230728-041117.csv")

# Assuming 'Time' is in seconds and is sorted in ascending order
time_values = df['Time'].values

# Calculate time intervals (delta t)
delta_t = time_values[1:] - time_values[:-1]

# Acceleration components (x, y, z) in m/s^2
accel_x = df['accX'].values * 0.00980665
accel_y = df['accY'].values * 0.00980665
accel_z = df['accZ'].values * 0.00980665

# Integrate acceleration to get velocity components
velocity_x = (accel_x[1:] + accel_x[:-1]) * 0.5 * delta_t
velocity_y = (accel_y[1:] + accel_y[:-1]) * 0.5 * delta_t
velocity_z = (accel_z[1:] + accel_z[:-1]) * 0.5 * delta_t

# Calculate total velocity (Euclidean norm)
velocity_magnitude = (velocity_x**2 + velocity_y**2 + velocity_z**2)**0.5

# Convert velocity components and magnitude to DataFrame
velocity_df = pd.DataFrame({
    'Time': time_values[1:],
    'Velocity_X': velocity_x,
    'Velocity_Y': velocity_y,
    'Velocity_Z': velocity_z,
    'Velocity_Magnitude': velocity_magnitude
})

# Save the velocity data to a new CSV file
velocity_df.to_csv("velocity_data.csv", index=False)