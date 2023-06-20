import matplotlib.pyplot as plt

# Create the figure and axes
fig, ax = plt.subplots()

# Sample data
x = [1, 2, 3, 4, 5]
y = [10, 15, 7, 12, 8]

# Plot the data
ax.plot(x, y)

# Set x-tick positions and labels
xtick_positions = range(min(x), max(x)+1)
xtick_labels = [str(xtick) for xtick in xtick_positions]
ax.set_xticks(xtick_positions)
ax.set_xticklabels(xtick_labels)

# Customize x-ticks appearance
plt.xticks(rotation=45, fontsize=8)

# Display the plot
plt.show()