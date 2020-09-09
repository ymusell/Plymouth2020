# Display part

This part is to display the algorithm on Rviz. The programs present are : 

#### `rviz_displayer.py`

Use the programm simu_boat to do the same thing as to print the boat in RViz. This program use the transformation of ROS.  
This program is launch with the following launch :  

    roslaunch plymouth2020 TestRviz.launch    

#### `rviz_displayer_fleet.py`

This program creates several boats and print them on scrren with RViz. Here are created classes for the boats, and for the RViz markers. Each boat can communicate with the Xbee communication. And then, the boats are displayed with there name.  
The launch to display this program is :

    roslaunch plymouth2020 fleet.launch 


#### `rviz_displayer_line_following.py`

The other programs needed to this python program are:  
	- `simu_boat.py`  
	- `lines_to_follow.py`  
	- `controler.py`  
	- `control_lib.py`  

This program create a boat and 3 points to follow. With the line following algorithms, the boat follow the line.  
All of it is printed on RViz.  
The launch to use is the following:

    roslaunch plymouth2020 line_following.launch 


#### `rviz_displayer_mission.py`

This algorithm display all the boat with the line to follow, the area to stay in in function of the type of the mission.  
The launch to use is the following:

    roslaunch plymouth2020 mission.launch 


#### `rviz_displayer_station_keeping.py`

The other programs needed to this python program are:  
	- `simu_boat.py`  
	- `zone_to_stay.py`  
	- `controler_station_keeping.py`  
	- `control_lib.py`  

This program create a boat and a zone to stay. With the line following algorithms, the boat follow the line to stay in an area.  
All of it is printed on RViz.  
The launch to use is the following:

    roslaunch plymouth2020 station_keeping.launch 

