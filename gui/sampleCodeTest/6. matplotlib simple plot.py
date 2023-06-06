import matplotlib.pyplot as plt

x = [1, 2, 3, 4, 5]
y = [10, 5, 8, 3, 6]

plt.plot(x, y)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Plot')

# Save the plot as an image file
#plt.savefig('plot.png')

# Alternatively, display the plot using an external viewer
plt.show()