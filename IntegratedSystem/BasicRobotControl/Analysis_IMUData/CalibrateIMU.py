import numpy as np
import pandas as pd

# Step 1: Collect Raw Data
# Assume raw_data is a Nx3 numpy array where each row is [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
# Read the CSV file into a pandas DataFrame
df = pd.read_csv('dataMeneng.csv')
# Extract the 'ColumnB' data into a NumPy array
column_accX = df['accX'].values
column_accY = df['accY'].values
column_accZ = df['accZ'].values
column_acc = df[['accX', 'accY', 'accZ']]

# Combine column arrays into a 2D array
column_array = np.array([column_accX, column_accY, column_accZ])
row_array = np.transpose(column_array)
selected_array = column_acc.values
raw_data = np.array(selected_array)
print(raw_data)
# Step 2: Compute Mean
mean_accel = np.mean(raw_data[:, 0:3], axis=0)

# Step 3: Compute Scale Factors (Optional)
# If you have scale factor errors, you can estimate them here

# Step 4: Estimate Noise (Optional)
# If you want to estimate noise, you can do it here

# Step 5: Compute Calibration Parameters
calibration_params = {
    'accel_bias': mean_accel,
    # Include scale factor and noise parameters if applicable
}

# Step 6: Apply Calibration
def apply_calibration(raw_measurement, calibration_params):
    # Apply bias correction
    calibrated_measurement = raw_measurement - calibration_params['accel_bias']

    # Apply scale factor correction (if applicable)

    # Apply noise compensation (if applicable)

    return calibrated_measurement

# Example usage:
calibrated_data = apply_calibration(raw_data, calibration_params)