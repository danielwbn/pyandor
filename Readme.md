This is a small Python wrapper for Andor cameras and spectrometers. It tries to stick to the same function naming that Andor does. 
Therefore, it should be fairly trivial how to use it.
The package is object oriented and keeps some information inside the class such as gain, preampgain, gainRange etc.

The package contains two modules, Andor and Shamrock. The Andor module is for use with Andor cameras, whereas the Shamrock Module is for Andor Spectrograph control.

Simple example:

```
import Andor

cam = Andor.Camera()
cam.SetDemoReady()
cam.StartAcquisition()
data = []
cam.GetAcquiredData(data)
cam.SaveAsTxt("raw.txt")
cam.ShutDown()
```