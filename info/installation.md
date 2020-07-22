# Prerequisites 

  -  First of all you need to download arduino IDE [click here](https://www.arduino.cc/en/Main/Software "Download Arduino IDE")  
  -  You need to install ROS on your Raspberry Pi 3 
  -  You maybe will have to enable the port use and to add your user to the group, then type:  


    ls -l /dev/ttyACM*
    sudo usermod -a -G dialout <username>


# Add some ROS packages 
  -  For the use of the GPS you will need gps-common 

	sudo apt-get install ros-melodic-gps-common 

  -  For the communication between Arduino and ROS, you will need rosserial_arduino and rosserial


    sudo apt-get install ros-melodic-rosserial-arduino
    sudo apt-get install ros-melodic-rosserial

	cd home/Arduino/libraries    //The place where is the library file for the arduino
	rm -rf ros_lib
	rosrun rosserial_arduino make_libraries.py 


# Installation of this package  
  
  - Create a ROS workspace `mkdir -p workspaceRos/src/`, `cd workspaceRos`, `catkin_make`.
  - In a terminal under `workspaceRos/src`, type `git clone https://github.com/ymusell/plymouth2020.git`.
  - Under `workspaceRos/`, type `catkin_make` then `. devel/setup.bash`.
  - You now have the package installed in `workspaceRos/src/plymouth2020`. 


# Uploading the arduino part

  -  Firstly, you need to create the good path to search all the libraries for your arduino or to copy/paste all the libraries in the good file. In arduino, you can find and change the path in:

`File/Preferences/ and Sketchbook localisation:`

  -  In order to see if all the sensors work you can try all the .ino program in the file:

`/plymouth2020/src/components/sensorActuatorTest/`


# Installation of python package

For the utm reading part:

    pip install utm 
