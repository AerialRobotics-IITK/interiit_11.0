from libOcam3 import oCams
import numpy as np

ocam = oCams("/dev/video2", verbose=1)
ocam.Set((("fmt", 1280, 960, 60)))
ocam.Start()
frame = ocam.GetFrame()
ocam.Stop()
print(frame.shape)
