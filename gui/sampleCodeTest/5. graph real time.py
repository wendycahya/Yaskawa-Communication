import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

matplotlib.use('TkAgg')
def animate(i):
    data = pd.read_csv('0605-SSMNewDemo.csv',
                       skiprows=1)
    x = data['no']  # assigning 'Time' column to x variable
    y = data['c']  # assigning 'HRR' column to y variable

    plt.cla()  # clear axis after plotting individual lines
    plt.plot(x, y, label='Velocity')  # selecting the x and y variables to plot
    plt.xlabel('Time (s)')  # label x axis
    plt.ylabel('Speed (mm/s)')  # label y axis
    plt.title('Robot Speed Detection')


# plt.gcf (get current figure) will reload the plat based on the data saved in the 'data' DataFrame
# animate argument will call the function defined above
# interval = 2000 milli second. The frames will be updated every 2 seconds.
# frames = 200. After plotting 200 frames, the animation will stop.

ani = FuncAnimation(plt.gcf(), animate, interval=200, frames=500, repeat=False)

# video = ani.to_html5_video()
# html = display.HTML(video)
# display.display(html)
# plt.close()

plt.tight_layout()  # adds padding to the graph
plt.show(block=True)  # show graph
