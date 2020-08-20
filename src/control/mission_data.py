#!/usr/bin/env python2
import numpy as np
import utm
import os

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose2D
from gps_common.msg import GPSFix

import sys
import rospkg
rospack = rospkg.RosPack()
pkg = rospack.get_path('plymouth2020')
sys.path.append(pkg+'/src/my_libs')
import control_lib as cl

def sawtooth(x):
	return (x+np.pi)%(2*np.pi)-np.pi   # or equivalently   2*arctan(tan(x/2))

##############################################################################################
#      ROS
##############################################################################################

def sub_gps_origin(data): # Vector3
	global lat_lon_origin, everything_is_ready
	lat_lon_origin[0] = [data.x, data.y]
	lat_lon_origin[1] = utm.from_latlon(data.x, data.y)

	everything_is_ready[0] = 1

def sub_gps(data): # Vector3
	global x, everything_is_ready
	res = utm.from_latlon(data.latitude, data.longitude)
	x[0,0] = -(lat_lon_origin[1][1]-res[1])
	x[1,0] = lat_lon_origin[1][0]-res[0]

	everything_is_ready[1] = 1

def sub_wind_direction(data):
	global psi, everything_is_ready
	psi = data.data

	everything_is_ready[2] = 1

def sub_wind_force(data):
	global awind, everything_is_ready
	awind = data.data

	everything_is_ready[3] = 1

def sub_euler_angles(data):
	global x, everything_is_ready
	x[2,0] = data.x
	everything_is_ready[4] = 1

##############################################################################################
#      Main
##############################################################################################

everything_is_ready = np.zeros(5)
lat_lon_origin = [[0.0,0.0],[0.0,0.0]]
x = np.array([[0.0],[0.0],[0.0]]) #x,y,theta
psi, awind = 0.0, 0.0

if __name__ == '__main__':

	#####	Ros init 	##############################################

	node_name = 'mission_data'
	rospy.init_node(node_name)
	mission_txt = rospy.get_param('mission',"mission.txt")
	print(mission_txt)
	rospy.Subscriber("launch_send_gps_origin", Vector3, sub_gps_origin)
	rospy.Subscriber("simu_send_gps", GPSFix, sub_gps)
	rospy.Subscriber("simu_send_wind_direction", Float32, sub_wind_direction)
	rospy.Subscriber("simu_send_wind_force", Float32, sub_wind_force)
	rospy.Subscriber("simu_send_theta", Vector3, sub_euler_angles)


	#####	Load mission table	######################################

	THIS_FOLDER = os.path.dirname(os.path.abspath(__file__))
	my_file = os.path.join(THIS_FOLDER, "missionFile/"+mission_txt)
	f = open(my_file,'r')
	doc = f.readlines()
	f.close()
	mission_tab = []
	for i in doc:
		doc_line = []
		s = i.split(',')
		for j in s:
			try:
				j = float(j)
			except:
				pass
			doc_line.append(j)
		mission_tab.append(doc_line)

	#####	Publish GPS origin 	######################################

	pub_send_gps_origin = rospy.Publisher('launch_send_gps_origin', Vector3, queue_size=10)
	gps_origin_msg = Vector3()
	rospy.sleep(1)
	rospy.loginfo("[{}] Got gps origin {},{}".format(node_name,mission_tab[0][1], mission_tab[0][2]))
	gps_origin_msg.x = mission_tab[0][1]
	gps_origin_msg.y = mission_tab[0][2]
	rospy.sleep(10)
	for i in range(5):
		pub_send_gps_origin.publish(gps_origin_msg)
		rospy.sleep(0.1)


	mission_tab = mission_tab[1:][:]
	print(mission_tab)

	#####	Wait all subscribers	##################################

	while (len(everything_is_ready) != np.sum(everything_is_ready)) and (not rospy.is_shutdown()):
		rospy.sleep(0.5)
		rospy.loginfo("[{}] Got {}/{} subscribers ready".format(node_name,int(np.sum(everything_is_ready)),len(everything_is_ready)))
		rospy.loginfo("[{}] EiR {}".format(node_name,everything_is_ready))
		
	rospy.loginfo("[{}] All subscribers ready".format(node_name))

	pub_send_u_rudder     = rospy.Publisher('control_send_u_rudder', Float32, queue_size=10)
	pub_send_u_sail       = rospy.Publisher('control_send_u_sail', Float32, queue_size=10)
	pub_send_thetabar     = rospy.Publisher('control_send_thetabar', Float32, queue_size=10)
	pub_send_line_begin   = rospy.Publisher('control_send_line_begin', Pose2D, queue_size=10)
	pub_send_line_end     = rospy.Publisher('control_send_line_end', Pose2D, queue_size=10)
	u_rudder_msg = Float32()
	u_sail_msg   = Float32()
	thetabar_msg = Float32()
	line_begin_msg = Pose2D()
	line_end_msg   = Pose2D()

	#####	Trough mission table	##################################

	for mission in mission_tab:
		if mission[0] == 1:
			rospy.loginfo("[{}] Line following between {},{} and {},{}".format(node_name,mission[1],mission[2],mission[3],mission[4]))

			res = utm.from_latlon(mission[1], mission[2])
			pax = -(lat_lon_origin[1][1]-res[1])
			pay = (lat_lon_origin[1][0]-res[0])
			a = np.array([[pax],[pay]])
			res = utm.from_latlon(mission[3], mission[4])
			pbx = -(lat_lon_origin[1][1]-res[1])
			pby = (lat_lon_origin[1][0]-res[0])
			b = np.array([[pbx],[pby]])
			theta_line = np.arctan2(pby-pay,pbx-pax)
			print('theta_line ',theta_line)
			line_begin_msg.x, line_begin_msg.y = mission[1], mission[2]
			line_end_msg.x, line_end_msg.y = mission[3], mission[4]

			for i in range(5):
				pub_send_line_begin.publish(line_begin_msg)
				pub_send_line_end.publish(line_end_msg)
				rospy.sleep(0.1)

			out_of_zone = False
			while (not out_of_zone) and (not rospy.is_shutdown()):

				thetabar = cl.get_thetabar_line_following(x,1,a,b,psi)
				u = cl.control_line_following(x,thetabar,psi)
				out_of_zone = np.linalg.det(np.array([[pbx-x[0,0], np.cos(theta_line-np.pi/2)],[pby-x[1,0],np.sin(theta_line-np.pi/2)]])) > 0
				u_rudder_msg.data = u[0,0]
				u_sail_msg.data = u[1,0]
				thetabar_msg.data = thetabar
				pub_send_u_rudder.publish(u_rudder_msg)
				pub_send_u_sail.publish(u_sail_msg)
				pub_send_thetabar.publish(thetabar_msg)
				rospy.sleep(0.2)


