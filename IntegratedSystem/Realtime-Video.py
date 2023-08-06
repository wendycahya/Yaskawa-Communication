import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Global variables
csv_file = 'RealtimePlot.csv'  # Replace with the path to your CSV file
update_interval = 1000  # Update plot every 1000 milliseconds (1 second)
fig, ax = plt.subplots()
ax2 = ax.twinx()


# Function to update the plot
def update_plot():
    df = pd.read_csv(csv_file)
    plt.cla()  # Clear the current plot
    ax.plot(df['Time'], df['Distance'], 'b-', label='Distance')
    # ax2.plot(dataTime, dataVR, 'r--', label='Velocity')
    ax2.plot(df['Time'], df['Speed'], 'g-.', label='Speed Command')
    plt.axis('on')  # Turn off axis labels and ticks

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (mm)")
    ax2.set_ylabel("Speed (mm/s)")

    plt.title('Distance and Speed Command Real-Time Plot')
    # Adjust plot limits if needed
    ax.relim()
    ax.autoscale_view()
    # Redraw the plot
    plt.draw()


if __name__ == '__main__':
    # Main loop to update the plot periodically
    while True:
        update_plot()
        plt.pause(0.1)  # Pause for a short duration to allow the plot to update
