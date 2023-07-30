import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def update_plot(i):
    # 讀取CSV文件
    df = pd.read_csv(csv_file)

    # 確保CSV文件中含有"Time"和"Speed"這兩個header
    if "Time" not in df.columns or "Speed" not in df.columns:
        raise ValueError("CSV文件中缺少'Time'或'Speed'這兩個header")

    # 將時間轉換成適合繪圖的格式
    df['Time'] = pd.to_datetime(df['Time'])

    # 設置x軸為整數索引
    df = df.set_index(df.index + 1)

    df.index = (df.index/44.5)*2

    # 只顯示CSV文件中的前i行數據，即模擬即時更新效果
    df = df.head(i+1)

    # 清除當前圖表
    plt.clf()

    # 繪製圖表
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (mm/s)')
    plt.title('Robot Command Speed')
    plt.plot(df.index, df['Speed']/10)

if __name__ == "__main__":
    csv_file = "3-CProp.csv"  # 替換成你的CSV文件名稱

    # 計算CSV文件的行數，用於設置FuncAnimation的frames參數
    num_rows = sum(1 for line in open(csv_file)) - 1

    # 使用FuncAnimation不斷更新圖表
    ani = FuncAnimation(plt.gcf(), update_plot, frames=num_rows, interval=4.45)  # 100毫秒更新一次（0.1秒）

    # 顯示圖表
    plt.show()