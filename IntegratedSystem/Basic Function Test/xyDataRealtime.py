import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Function to update the graph in real-time
def update_graph(frame):
    # Generate new data points or fetch them from a source
    new_x = frame  # Replace 'frame' with your method of getting new x values
    new_y = np.sin(new_x) * 100 + height // 2  # Example: Use sine function as y values, replace with your data source

    # Append new data points to the lists
    x_data.append(new_x)
    y_data.append(new_y)

    # Clear the previous plot
    plt.clf()

    # Plot the data using Matplotlib
    plt.plot(x_data, y_data, 'r-')
    plt.xlabel('X Axis')
    plt.ylabel('Y Axis')
    plt.title('Real-time Graph')

# Create a blank white image to serve as the canvas for the graph
height, width = 400, 600
canvas = np.ones((height, width, 3), dtype=np.uint8) * 255

# Set up Matplotlib figure for plotting
fig, ax = plt.subplots(figsize=(4, 3), dpi=100)

# Initialize empty lists to hold data points
x_data = []
y_data = []

# Start capturing from the camera
cap = cv2.VideoCapture(2)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # Resize the frame to match the canvas size
    frame = cv2.resize(frame, (width, height))

    # Display the camera feed on the OpenCV window
    cv2.imshow("Camera Feed", frame)

    # Update the graph on the same OpenCV window
    update_graph(frame)

    # Render the Matplotlib figure to an image
    fig.canvas.draw()
    plot_img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    plot_img = plot_img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

    # Combine the camera feed and the graph plot
    combined_img = np.concatenate((frame, plot_img), axis=1)

    # Show the combined image in the OpenCV window
    cv2.imshow("Camera + Graph", combined_img)

    if cv2.waitKey(1) & 0xFF == 27:  # Press 'Esc' key to exit
        break

# Release the video capture and close OpenCV windows when finished
cap.release()
cv2.destroyAllWindows()
