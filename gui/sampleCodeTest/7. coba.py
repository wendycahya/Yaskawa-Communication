import cv2
import matplotlib.pyplot as plt

# Read an image using OpenCV
image = cv2.imread('test.png')

# Convert BGR to RGB for Matplotlib
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Create a Matplotlib figure with subplots
fig, (ax1, ax2) = plt.subplots(1, 2)

# Display the OpenCV image in the first subplot
ax1.imshow(image_rgb)
ax1.set_title('OpenCV Image')

# Plot a graph in the second subplot
x = [1, 2, 3, 4, 5]
y = [3, 5, 2, 8, 1]
ax2.plot(x, y)
ax2.set_title('Matplotlib Plot')

# Adjust spacing between subplots
plt.tight_layout()

# Show the figure with both the image and plot
plt.show()