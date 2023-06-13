import numpy as np
from scipy import stats
import pandas as pd
import matplotlib.pyplot as plt

# Define your data as a numpy array
# Load the CSV file into a pandas DataFrame
data_frame = pd.read_csv('0609-SSMNewDemo.csv')

# Extract the data column from the DataFrame
data = data_frame['d']

# Calculate the IQR
q1 = np.percentile(data, 25)
q3 = np.percentile(data, 75)
iqr = q3 - q1

# Set the threshold for outliers
threshold = 1.5

# Create a mask for outliers
outlier_mask = (data < q1 - threshold * iqr) | (data > q3 + threshold * iqr)

# Filter the data
filtered_data = data[~outlier_mask]

# Print the filtered data
print(filtered_data)

plt.figure(figsize=(8, 6))
#plt.plot(time, data, label='Original Data')
plt.plot(filtered_data, label='Filtered Data')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.title('Anomaly Amplitude Filtering')
plt.legend()
plt.grid(True)
plt.show()