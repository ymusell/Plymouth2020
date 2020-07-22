# Timeline of the creation of this project

Here is a small summary of all the major steps made to advance the project:

  -  Firstly, being able to use the Raspberry Pi 3 and ROS is needed  
  -  Then downloading Arduino has been done and the configuration of it in order to locate of all the packages  
  -  Moreover, all the sensors have been studied to become familiar with them  
  -  Being able to see all the datas from the sensors on the Raspberry Pi 3 is mandatory  
  -  Thus, the first filtering algorith were used to calibrate the sensors  
  -  For the filtering part, the used of the Kalman filter was needed, so the course made by Luc Jaulin was studied. [Here](https://www.ensta-bretagne.fr/jaulin/kalmooc.html "link to the lesson about the Kalman filter")  is the link to the course of Luc Jaulin about the Kalman Filter  
  -  The IMU was calibrate and the sensor for the wind direction too  
  -  The efficience of the Kalman filter was tested and a new Kalman filter was created.  
  -  With all the calibration and the low level is finished  
  -  the state machine of the boat has been created in order to simulate the behavior of the boat for simple algorithms  
  -  Line following algorithm has been implemented and is simulationhas been done  
  -  The same applies for the station keeping algorith  
  -  Then the RViz display part was created for a better view of the boat  
  -  With Rviz, the line following and station keeping algorithm were implemented  
  -  For the creation of a more complexe algorithm the update of some knowledge was needed  