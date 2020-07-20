# Controler part

The files from the controler part: 

#### `controler.py` 

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


#### `controler_station_keeping.py` 

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


#### `lines_to_follow.py` 

This program contain the point to follow for the simulation, and calulates the distance from the boat to the wanted point.  
If the boat is close enough to the point, then, the target changes.


#### `mission.py` :

The different possible states :  
 - If simu = 0, then it's with the simulation.  
 - If simu = 1, then it' for the real mission.  
 - If mission[0] = 0, then it's a cap following algorithm.  
 - If mission[0] = 1, then it's a line following algorithm.  
 - If mission[0] = 2, then it's a stating keeping algorithm.  

Mission is defined in the file `mission.txt`in the file missionFile.

The launch you need to use and change if you want if the following :

    roslaunch plymouth2020 mission.launch 


#### `mode.py` 

Useful if you use the XBee. 


#### `zone_to_stay.py` 

 This program creates a zone to stay and publish it.
