import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Global variables
csv_file = '3-CProp.csv'  # Replace with the path to your CSV file
update_interval = 1000  # Update plot every 1000 milliseconds (1 second)

# Read the CSV file
df = pd.read_csv(csv_file)

# Create the figure and axis objects
fig, ax = plt.subplots()

# Function to update the plot
def update_plot(frame):
    plt.cla()  # Clear the current plot

    # Plot the data up to the current frame
    current_data = df.iloc[:frame]
    #ax.plot(current_data['Time'], current_data['Distance'], 'b-', label='Distance')
    ax.plot(current_data['Time'], current_data['Speed'], 'g-.', label='Speed Command')

    plt.axis('on')  # Turn off axis labels and ticks

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (mm/s)")

    plt.title('Robot Speed')

# Main function to create the real-time plot
def main():
    num_frames = len(df)  # Number of frames equals the number of rows in the CSV file
    ani = FuncAnimation(fig, update_plot, frames=num_frames, interval=update_interval)
    plt.show()

if __name__ == '__main__':
    main()