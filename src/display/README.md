# Display part

This part is for the display of the algorithm on Rviz. The programs present are : 

 * `rviz_displayer.py`

Use the programm simu_boat to do the same thing as to print the boat in RViz. This program use the transformation of ROS.  
This program is launch with the following launch :  

    roslaunch plymouth2020 TestRviz.launch    

 * `rviz_displayer_fleet.py`

This program creates several boats and print them on scrren with RViz. Here are created classes for the boats, and for the RViz markers. Each boat can communicate with the Xbee communication. And then, the boats are displayed with there name.  
The launch to display this program is :

    roslaunch plymouth2020 fleet.launch 


<!--  * `rviz_displayer_line_following.py`



 * `rviz_displayer_mission.py`



 * `rviz_displayer_station_keeping.py` -->




