# python_live_plot.py

import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')

x_values = []
y1_values = []
y2_values = []
index = count()


def animate(i):
    data = pd.read_csv('0605-SSMNewDemo.csv')
    x_values = data['time']
    y1_values = data['Vr']
    y2_values = data['VrPaper']

    plt.cla()
    plt.plot(x_values, y1_values)
    plt.plot(x_values, y2_values)
    plt.xlabel('time(s)')
    plt.ylabel('Robot Speed(mm/s)')
    plt.title('Comparison Performance')
    plt.gcf().autofmt_xdate()
    plt.tight_layout()

ani = FuncAnimation(plt.gcf(), animate, 5000)

plt.tight_layout()
plt.show()