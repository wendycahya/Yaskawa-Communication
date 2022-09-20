import time
import math
from datetime import datetime


start = datetime.now()
print("nilai second start: ", start)
time.sleep(3)
stop = datetime.now()
print("nilai second stop: ", stop)

diff_seconds = (stop - start)
timer = diff_seconds.seconds + 0.5
print("nilai second", timer)