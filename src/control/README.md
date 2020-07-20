# Controler part

THe files from the controler part: 

 * `controler.py` 

 Subscribe to :
 	+ simu_send_theta
	+ simu_send_wind_direction
	+ simu_send_wind_force
	+ launch_send_gps_origin
	+ simu_send_gps
	+ control_send_lines_to_follow

  Publish in :
	+ control_send_u_rudder
	+ control_send_u_sail

This program is controling the line following algorithm by taking the goal points to reach and the position of the boat. Then it uses the controler of the line following and publish the input for the boat.

 * `controler_station_keeping.py` 

  Subscribe to :
 	+ simu_send_theta
	+ simu_send_wind_direction
	+ simu_send_wind_force
	+ launch_send_gps_origin
	+ simu_send_gps
	+ control_send_zone_to_stay 

  Publish in :
	+ control_send_u_rudder
	+ control_send_u_sail

This program is controling the station keeping algorithm by taking the area to stay and the position of the boat. Then it uses the controler of the station keeping and publish the input for the boat.

 * `lines_to_follow.py` 

This program contain the point to follow for the simulation, and calulates the distance from the boat to the wanted point.  
If the boat is close enough to the point, then, the target changes.

 * `mode.py` 



 * `zone_to_stay.py` 

 This program creates a zone to stay and publish it.
