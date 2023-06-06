import matplotlib.pyplot as plt
import numpy as np

# Create an empty plot
plt.ion()

# Create a figure and axes
fig, ax = plt.subplots()
x_data = []
y_data = []

# Create a line object
line, = ax.plot(x_data, y_data)

# Set up the plot axes
ax.set_xlim(0, 10)
ax.set_ylim(0, 1)

# Start the loop to update the plot
while True:
    # Generate new data point
    x = len(x_data) + 1
    y = np.random.rand()

    # Append the new data point to the arrays
    x_data.append(x)
    y_data.append(y)

    # Update the line data
    line.set_data(x_data, y_data)

    # Redraw the plot
    plt.draw()
    plt.pause(0.1)

    # Break the loop condition if desired
    if x >= 10:
        break

# Keep the plot displayed after the loop ends
plt.ioff()
plt.show()