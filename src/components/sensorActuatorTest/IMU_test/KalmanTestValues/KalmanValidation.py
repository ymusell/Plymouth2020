import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

######################### Data acquisition #####################

f_3 = open('kalman_try_shape_3.txt')
yaw_3 = []
pitch_3 = []
roll_3 = []
for i in f_3:
	s = i.split(' ')  #For the spaces
	if s[0] != "And":
		yaw_3.append(float(s[0]))
		pitch_3.append(float(s[1]))
		roll_3.append(float(s[2].split('\n')[0]))
mYaw_3 = np.array(yaw_3)
mPitch_3 = np.array(pitch_3)
mRoll_3 = np.array(roll_3)
f_3.close()

f_2 = open('kalman_try_shape_2.txt')
yaw_2 = []
pitch_2 = []
roll_2 = []
for i in f_2:
	s = i.split(' ')  #For the spaces
	if s[0] != "And":
		yaw_2.append(float(s[0]))
		pitch_2.append(float(s[1]))
		roll_2.append(float(s[2].split('\n')[0]))
mYaw_2 = np.array(yaw_2)
mPitch_2 = np.array(pitch_2)
mRoll_2 = np.array(roll_2)
f_2.close()

f_time = open('kalman_try_time.txt')
timer = []
sum_time = 0
for i in f_time:
	s = i.split('\n')  #For the spaces
	timer.append(float(s[0])+sum_time)
	sum_time += float(s[0])
time = np.array(timer)
f_time.close()

######################### Data processing #####################
fig = plt.figure()
plt.suptitle('The three Euler angles in relation to time whith the two types of Kalman filter', size = 'x-large')
ax1 = fig.add_subplot(221)
ax1.set_title("Yaw in relation to the time")
ax1.plot(time,mYaw_3,'r', label='Kalman with 3 components')
ax1.plot(time,mYaw_2,'b', label='Kalman with 2 components')
ax1.legend()
ax2 = fig.add_subplot(222)
ax2.set_title("Pitch in relation to the time")
ax2.plot(time,mPitch_3,'r', label='Kalman with 3 components')
ax2.plot(time,mPitch_2,'b', label='Kalman with 2 components')
ax2.legend()
ax3 = fig.add_subplot(223)
ax3.set_title("Roll in relation to the time")
ax3.plot(time,mRoll_3,'r', label='Kalman with 3 components')
ax3.plot(time,mRoll_2,'b', label='Kalman with 2 components')
ax3.legend()
# ax4 = fig.add_subplot(224)
# ax4.set_title("Value of the yaw, to see if the Kalman 3 dimension is needed or not")
# ax4.plot(time,mRoll_3,'r', label='Kalman with 3 components')
# ax4.plot(time,mRoll_2,'b', label='Kalman with 2 components')
# ax4.legend()

plt.show()
 
# fig = plt.figure()
# plt.suptitle('Before/After calibration values of the IMU', size = 'x-large')
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(mX, mY, mZ,label='before the calibration')
