
import Andor
import matplotlib.pyplot as plt
import numpy as np

cam = Andor.Camera()
cam.SetDemoReady()
cam.StartAcquisition()
data = []
cam.GetAcquiredData(data)

data = np.array(data)
data = data.reshape((256,1024))
plt.imshow(data)
plt.show()

cam.ShutDown()