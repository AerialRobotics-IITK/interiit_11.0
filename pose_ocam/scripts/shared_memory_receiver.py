import numpy as np
from multiprocessing import shared_memory

class Pose:

	def __init__(self):
		self.existing_shm=shared_memory.SharedMemory(name='aruco_pose')

	def getPose(self):
		pose = np.ndarray((3,), dtype=np.float64, buffer=self.existing_shm.buf)
		self.existing_shm.close()
		self.existing_shm.unlink()
		return pose

getter = Pose()
curr_pos = getter.getPose()
print(curr_pos)