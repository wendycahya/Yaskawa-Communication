import numpy as np
from scipy import stats

# Define your data as a numpy array
data = np.array([1, 2, 3, 10, 4, 5, 6, 100, 7, 8, 9])

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