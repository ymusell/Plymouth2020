# The simulation part
  
The simulation will display the boat on matplotlib.


#### `command_sailboat.py` : 

Launch this program with python3 due to the use of greek symbols.  
This program is the simulate the behaviour of the boat with the state vector and the Euler formula, and the mission of this boat is to follow a triangle.  
The all process is print on screen with matplolib. 
  
#### `display_boat.py` :
This program allows to see the boat in real time on screen. It shows the position (the yaw) of the boat with the angle send to the motors for the rudder and the sail. There are also information about the wind, its direction and its intensity.  

#### `simu_boat.py` :

This program creates publishers to simulate the behaviour of the boat by taking into account, its position, the angles of the rudder and the sail.  
By taking into account this information, with the euler formula, we know where the boat will be in the next step. To do so, we have a constant and known wind.  
This code will be used to test the controllers with a simulator, and to be able to see the boat with Rviz.  

__Warning__: The initial position of the boat has to be defined before launching this program.

#### `station_keeping_infinite.py` :

With this program, we can test the station keeping algorithm with the infinite strategy.