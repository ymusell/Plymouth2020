## The IMU  
The IMU used is the Grove IMU 9Dof, v2.2  
For this programm, we need to add the libraries wire.h, I2Cdev.h for the I2C communication and MPU9250.h.  
You can download this libraries or find other information about the sensor there: [link](https://wiki.seeedstudio.com/Grove-IMU_9DOF_v2.0/ "Information for the Grove-IMU_9DOF_v2.0")  


## Calibration part  
There are to types of calibration, the first one where we only consider the hard iron distortion, (the fastest algorithm) and on the second calibration, there are hard and the soft iron distortion taken into account.  



To use this program, you have to lauch the sensorCalibration launch
but before, you have to change the "?" in the line <arg name="calibration" default="?" />  in the launch.  
Values of the ? =  
* 0 : The program 	will use the previous calibration by considering only the hard iron distortion.  
* 1 : The program will calibrate the IMU, you have to follow the instruction on screen.  
* 2 : The program will use the previous calibration by considering the hard and soft iron distortion. Then you will have to launch the Magneto 1.2 that you can find [here](http://sailboatinstruments.blogspot.com/2011/09/improved-magnetometer-calibration-part.html "Magneto 1.2"). And you need to change the values of the offset in the imu.py file that gave you Magneto 1.2.  


And after,  

    roslaunch plymouth2020 sensorCalibration.launch

## Seeing the value of the full calibration

You can see if the new calibration of your magnetometer is good by changing the value of the offset in the magnetometerValueUnCalibrated.py.  
And run:  

    python2.7 magnetometerValueUnCalibrated.py