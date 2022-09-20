from robodk.robolink import *  # RoboDK API

from tempfile import TemporaryDirectory
import numpy as np
from matplotlib import pyplot as plt

#----------------------------------
# Get the simulated camera from RoboDK
RDK = Robolink()

cam_item = RDK.Item('Depth Camera', ITEM_TYPE_CAMERA)
if not cam_item.Valid():
    cam_item = RDK.Cam2D_Add(RDK.ActiveStation(), 'DEPTH')
    cam_item.setName('Depth Camera')
cam_item.setParam('Open', 1)

#----------------------------------------------
# Get the image from RoboDK
td = TemporaryDirectory(prefix='robodk_')
tf = td.name + '/temp.grey32'
if RDK.Cam2D_Snapshot(tf, cam_item) == 1:
    grey32 = np.fromfile(tf, dtype='>u4')
    w, h = grey32[:2]
    grey32 = np.flipud(np.reshape(grey32[2:], (h, w)))
else:
    raise

#----------------------------------------------
# Display
grey32[grey32 == 0] = 2**32 - 1  # This is for display purposes only! Values of 0 do not have any measurements.
plt.imshow(grey32, 'gray')
plt.show()
