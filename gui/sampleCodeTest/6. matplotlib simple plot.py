import io
import numpy as np
import matplotlib.pyplot as plt

arr = np.array([1,2,3,4])

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.plot(arr)
plt.show()
plt.close(fig)

backend_orig = plt.get_backend()
plt.switch_backend("TkAgg")

buf = io.BytesIO()
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.plot(arr)
plt.savefig(buf, format="png")
plt.close(fig)

plt.switch_backend(backend_orig)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.plot(arr)
plt.show()
plt.close(fig)