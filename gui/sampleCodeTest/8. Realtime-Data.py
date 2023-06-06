from itertools import count
import matplotlib.pyplot as plt
import pandas as pd
plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []

index = count()

def animate(i):
    try:
        data = pd.read_csv('0605-SSMNewDemo.csv') #random value between 0 and 50
        #data = pd.read_csv(r'\Users\User\source\repos\LsNcroINGRAM\ReadForceSensor\Project1\data.csv') #from force sensor
        x = data['no']
        y = data['c']
        plt.cla() #make sure the color of the plot is the same
        plt.plot(x, y, label = 'Speed')
        plt.legend(loc = 'upper left')
        plt.ylim(0, 2000)

    except EmptyDataError:
        pass

ani = FuncAnimation(plt.gcf(), animate, interval = 5, cache_frame_data=False)

plt.tight_layout()
plt.show()