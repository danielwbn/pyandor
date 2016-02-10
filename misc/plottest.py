
import Andor
import matplotlib.pyplot as plt

cam = Andor.Camera()
cam.SetDemoReady()
cam.StartAcquisition()
data = []
cam.GetAcquiredData(data)

plt.imshow(data)
plt.show()

cam.ShutDown()