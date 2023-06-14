import matplotlib.pyplot as plt
import numpy as np

# Generate some example data
x = np.linspace(0, 10, 100)
y = np.sin(x)

# Define the threshold for changing the face color
threshold = 5

# Create a figure and axis
fig, ax = plt.subplots()

# Iterate over x-values and set face color based on condition
for i in range(len(x)):
    if x[i] > threshold:
        ax.plot(x[i], y[i], 'o', color='red', markersize=5, facecolor='lightgray')

    else:
        ax.plot(x[i], y[i], 'o', color='blue', markersize=5, facecolor='lightgreen')

# Show the plot
plt.show()