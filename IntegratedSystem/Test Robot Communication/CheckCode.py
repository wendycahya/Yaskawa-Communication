import matplotlib.pyplot as plt
import numpy as np

# Function to update the plot with new data
def update_plot():
    global x, y
    # Generate new data (sine wave in this example)
    x = np.linspace(x[-1], x[-1] + 2 * np.pi, num_points)
    y = np.sin(x)
    # Update the plot data
    line.set_data(x, y)
    # Adjust plot limits if needed
    ax.relim()
    ax.autoscale_view()
    # Redraw the plot
    plt.draw()

# Global variables
num_points = 100
x = np.linspace(0, 2 * np.pi, num_points)
y = np.sin(x)

# Set up the plot
fig, ax = plt.subplots()
line, = ax.plot(x, y)

# Main loop to update the plot periodically
while True:
    update_plot()
    plt.pause(0.1)  # Pause for a short duration to allow the plot to update
