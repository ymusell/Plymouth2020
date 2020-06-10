import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


f = open('imu_magnetometer_calibration.txt')
magx = []
magy = []
magz = []
for i in f:
	s = i.split(' ')  #For the spaces
	if s[0] != "And":
		magx.append(float(s[0]))
		magy.append(float(s[1]))
		magz.append(float(s[2].split('\n')[0]))
mX = np.array(magx)
mY = np.array(magy)
mZ = np.array(magz)
 
fig = plt.figure()
plt.suptitle('Before/After calibration values of the IMU', size = 'x-large')
ax = fig.add_subplot(111, projection='3d')
ax.plot(mX, mY, mZ,label='before the calibration')

# After the second type of calibration, with the .exe Magneto 1.2, to see the README.md
listA = [[1.013172,-0.000964,-0.006329],[-0.000964,1.024569,0.006336],[-0.006329,0.006336,0.974003]]
A = np.array(listA)
bias = np.array([0.624667,2.099800,-23.159913])
newMX = []
newMY = []
newMZ = []
for k in range (mX.shape[0]):
	h = np.array([mX[k],mY[k],mZ[k]])
	hCal = np.dot(A,(h-bias))
	newMX.append(hCal[0])
	newMY.append(hCal[1])
	newMZ.append(hCal[2])
arrayNewMX = np.array(newMX)
arrayNewMY = np.array(newMY)
arrayNewMZ = np.array(newMZ)

ax.plot(arrayNewMX, arrayNewMY, arrayNewMZ,'ro',label='After the calibration')
ax.set_xlabel('magnet_X')
ax.set_ylabel('magnet_Y')
ax.set_zlabel('magnet_Z')
ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(-20, 20)
ax.legend()

plt.show()