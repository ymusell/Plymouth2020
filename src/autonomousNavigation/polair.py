import matplotlib.pyplot as plt
import numpy as np

angle = np.arange(0, 360, 10)
pol = [0,0,0,0.5,4,6,6.5,6.9,7.1,7.3,7.5,7.7,7.9,8,8,7.5,6.5,6,5.7]
inv = pol[::-1]
r = np.array(pol+inv[1:len(inv)-1])/8

theta = np.deg2rad(angle)

fig = plt.figure()

ax = fig.add_subplot(111, projection='polar')

ax.scatter(theta,r)
ax.plot(theta,r,'r')
ax.set_theta_zero_location('N')

# plt.savefig("boat_polar_speed_normalize.png", bbox_inches='tight')

# plt.show()

def nbr_action_to_polar(nbr):
	angle_taken = np.arange(0, 360, 360/nbr)
	liste_polar = np.interp(angle_taken, angle, r)  
	return liste_polar

print(nbr_action_to_polar(8))
