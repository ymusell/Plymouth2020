#!/usr/bin/env python2
import numpy as np
import utm
import h5py
import random as rd

import sys
import rospkg
rospack = rospkg.RosPack()
pkg = rospack.get_path('plymouth2020')
sys.path.append(pkg+'/src/my_libs')
import control_lib as cl

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from gps_common.msg import GPSFix
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion


def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

##############################################################################################
#      ROS
##############################################################################################

def sub_xy(data):
	global pos_x, pos_y
	#rospy.loginfo("Pos x : %s, Pos y : %s",data.x, data.y)
	pos_x = data.x
	pos_y = data.y

def sub_gps_origin(data): # Vector3
    global lat_lon_origin
    lat_lon_origin[0] = [data.x, data.y]
    lat_lon_origin[1] = utm.from_latlon(data.x, data.y)

def sub_gps(data): # Vector3
	global pos_x, pos_y
	res = utm.from_latlon(data.latitude, data.longitude)
	pos_x = -(lat_lon_origin[1][1]-res[1])
	pos_y = lat_lon_origin[1][0]-res[0]

def sub_wind_direction(data):
	global psi
	psi = data.data

def sub_wind_force(data):
	global awind
	awind = data.data

def sub_theta(data):
	global theta
	#rospy.loginfo("Imu heading %s ",data.orientation.x)
	theta = data.x

def sub_zone_to_stay(data): # Vector3
	global zone_to_stay

	res = utm.from_latlon(data.x, data.y)
	zone_to_stay[0] = -(lat_lon_origin[1][1]-res[1])
	zone_to_stay[1] = (lat_lon_origin[1][0]-res[0])
	# print(zone_to_stay) 

##############################################################################################
#      Q-learning
##############################################################################################

def zone_boat(zone_position, wind_direction, boat_position):
	"""
	pos_x = boat_position[0]
	zone_to_stay[O] = zone_position[0]
	pos_y = boat_position[1]
	zone_to_stay[1] = zone_position[1]
	"""
	vect_boat = np.array([boat_position[0]-zone_position[0],boat_position[1]-zone_position[1]])
	theta_proj = np.arccos(vect_boat[1]/np.linalg.norm(vect_boat))
	angle_boat_wind = wind_direction - theta_proj
	if angle_boat_wind<0:
		angle_boat_wind += 2*np.pi
	angle = int(angle_boat_wind/(2*np.pi/nbr_of_angle_part))
	# print("angle",angle)
	ray = int(np.linalg.norm(vect_boat)/range_of_ray)
	if ray >= nb_ray:
		ray = nb_ray-1
	zone = nbr_of_angle_part*ray + angle
	return zone,ray,angle


def rewards(previous_zone, current_zone, action):
	if int((current_zone-previous_zone)/nbr_of_angle_part)>=1:
		reward = -1
	if int((current_zone-previous_zone)/nbr_of_angle_part)<=-1:
		reward = 1
	else:
		reward = 0.05
	reward += polair_value[action]
	return reward

def chose_action(epsilon, ray, angle, Q):
	action_available = Q[angle,ray,:]
	#Exploration
	if rd.uniform(0, 1) > epsilon:
		act = np.random.choice(actions)
		theta_ba = act*(2*np.pi/nbr_of_angle_part)
	#Exploit
	else:
		act = actions[np.argmax(action_available)]
		theta_ba = act*(2*np.pi/nbr_of_angle_part)
	theta_ba = (np.pi/2)-theta_ba
	return theta_ba,act

def reset_map():
	reset_msg.data = True
	pub_send_reset.publish(reset_msg)

def save_data(table):
	f1 = h5py.File(sys.path[0]+'/data/station_keeping_Qlearning.hdf5', "w")
	dset = f1.create_dataset("data", (nbr_of_angle_part, nb_ray,8), dtype='i', data=table)
	f1.close()

##############################################################################################
#      Main
##############################################################################################

pos_x, pos_y= 0, 0
awind,psi = 0, 0
theta = 0
lat_lon_origin = [[0,0],[0,0]]
zone_to_stay = [0,0]
u = [0,0]

###### Machine learning part ############################
learning_rate = 0.85
actualization_rate = 0.99
nbr_episodes = 100	#5000
ending_time = 25	# 25 sec of sail
range_of_ray = 1   	# 1 meter
nb_ray = 20 	# 20 range of ray
nbr_of_angle_part = 8  # The map is devided in 8 parts
actions = np.arange(0,8,1)
polair_value = [0,0.625,0.9125,1,0.7125,1,0.9125,0.625]	#See the program polair.py



if __name__ == '__main__':

	node_name = 'controler_station_keeping'
	rospy.init_node(node_name)
	calibration_matrix = rospy.get_param('calibration_matrix')
	rospy.loginfo("the value of the calibration_matrix in {}".format(calibration_matrix))
	rospy.Subscriber("simu_send_theta", Vector3, sub_theta)
	rospy.Subscriber("simu_send_wind_direction", Float32, sub_wind_direction)
	rospy.Subscriber("simu_send_wind_force", Float32, sub_wind_force)
	rospy.Subscriber("launch_send_gps_origin", Vector3, sub_gps_origin)
	rospy.Subscriber("simu_send_gps", GPSFix, sub_gps)
	rospy.Subscriber("control_send_zone_to_stay", Vector3, sub_zone_to_stay)

	pub_send_u_rudder = rospy.Publisher('control_send_u_rudder', Float32, queue_size=10)
	pub_send_u_sail   = rospy.Publisher('control_send_u_sail', Float32, queue_size=10)
	pub_send_reset    = rospy.Publisher('reset_map', Bool, queue_size=10)
	u_rudder_msg = Float32()
	u_sail_msg   = Float32()
	reset_msg    = Bool()

	while lat_lon_origin == [[],[]] and not rospy.is_shutdown():
		rospy.sleep(0.5)
		rospy.loginfo("[{}] Waiting GPS origin".format(node_name))
	rospy.loginfo("[{}] Got GPS origin {}".format(node_name,lat_lon_origin))
	while zone_to_stay == [0,0] and not rospy.is_shutdown():
		rospy.sleep(0.5)
		rospy.loginfo("[{}] Waiting zone_to_stay {}".format(node_name, zone_to_stay))
	rospy.loginfo("[{}] Zone_to_stay {}".format(node_name,zone_to_stay))
	

	##### Definition of the map/table for the Q-learning ##############
	if (calibration_matrix == 1):
		Q_table = np.zeros((nbr_of_angle_part, nb_ray,8), dtype='f') # 8 is the nbr of directions avalaibles
		# saving the matrix
		f1 = h5py.File(sys.path[0]+'/data/station_keeping_Qlearning.hdf5', "w")
		dset = f1.create_dataset("data", (nbr_of_angle_part, nb_ray,8), dtype='i', data=Q_table)
		f1.close()
		rospy.loginfo("Creation of the matrix ended")


	if (calibration_matrix == 0): 	#Take the value of the saved Q_table
		f1 = h5py.File(sys.path[0]+'/data/station_keeping_Qlearning.hdf5', "r")
		dset1 = f1['data']
		Q_table = dset1[:].astype(float)
		f1.close()


	rate = rospy.Rate(10)
	nbr_ite = 0
	while not rospy.is_shutdown():
		while (nbr_ite <nbr_episodes): 
			# Initialisation de la postion du bateau
			reset_map()
			epsilon = float(nbr_ite)/float(nbr_episodes)
			prev_zone,prev_ray,prev_angle = zone_boat(zone_to_stay, psi, np.array([pos_x,pos_y]))
			thetabar,act = chose_action(epsilon, prev_ray, prev_angle, Q_table)
			begin_time = rospy.get_time()
			while (rospy.get_time()-begin_time<ending_time):
				x = np.array([[pos_x,pos_y,theta]]).T
				
				fut_x = x + 1*np.array([[np.cos(theta),np.sin(theta),0]]).T

				# reinsforcement learning
				new_zone,new_ray,new_angle = zone_boat(zone_to_stay, psi, np.array([pos_x,pos_y]))
				if new_zone!=prev_zone:
					#Chose an action 
					thetabar,act = chose_action(epsilon, new_ray, new_angle, Q_table)
					# Update the Q-table
					reward = rewards(prev_zone, new_zone, act)
					Q_table[prev_angle,prev_ray,act] = Q_table[prev_angle,prev_ray,act] + learning_rate*((reward+actualization_rate*(np.argmax(Q_table[new_angle,new_ray,:])))-Q_table[prev_angle,prev_ray,act])
				u = cl.control_station_keeping(thetabar,fut_x,psi)	
				# rospy.loginfo(" Zone {}, ray de {}, angle de {}".format(new_zone,new_ray,new_angle))
				
				prev_zone,prev_ray,prev_angle = new_zone,new_ray,new_angle
				u_rudder_msg.data = u[0,0]
				u_sail_msg.data = u[1,0]
				pub_send_u_rudder.publish(u_rudder_msg)
				pub_send_u_sail.publish(u_sail_msg)
				pub_send_reset.publish(reset_msg)
				if reset_msg.data == True:
					reset_msg.data = False
				rate.sleep()
			nbr_ite+=1
			rospy.loginfo("Iteration nbr {}/{}".format(nbr_ite,nbr_episodes))
		save_data(Q_table)
		rospy.loginfo("End of the training")

