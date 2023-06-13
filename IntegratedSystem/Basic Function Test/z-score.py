import numpy as np
import matplotlib.pyplot as plt

# Generate sample time series data
time = np.arange(0, 10, 0.1)
data = np.sin(time) + np.random.normal(0, 0.2, len(time))  # Add noise to the sine wave

# Calculate z-scores for the data
z_scores = (data - np.mean(data)) / np.std(data)

# Define a threshold for anomaly detection
threshold = 2.0

# Filter anomalous data points based on z-scores
filtered_data = data[np.abs(z_scores) <= threshold]
print(filtered_data)
# Plot the original data and filtered data
plt.figure(figsize=(8, 6))
plt.plot(time, data, label='Original Data')
plt.plot(time, filtered_data, label='Filtered Data')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.title('Anomaly Amplitude Filtering')
plt.legend()
plt.grid(True)
plt.show()