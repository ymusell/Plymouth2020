## Navigation using machine learning algorithm  

The type of machine learning algorithm used is a reinforcement learning. In this type of machine learning problem, the algorithm used here is a Q-learning algorithm.  
In order to have the Q-table, the data will be saved in the file `/data`.  
The launch is the following:  

    roslaunch plymouth2020 station_keeping_deep_learning.launch
  
In this launch, the value of the "state" can be changed to:  
* 0 if the T-table matrix is already created in the file `/data` (default value)  
* 1 otherwise  
  
In this launch, the boat will be learning how to sail (stay in a position in a first place). The first iterations will alllow the boat to explore its environment and the latest iterations will exploit the values of the Q-table.
  
  
  
##### Other information  
To continue (not done):  
Other information about implemented a reinforcement learning using policy gradient can be found [here](https://lilianweng.github.io/lil-log/2018/04/08/policy-gradient-algorithms.html "Information about policy gradient")  and [here](https://medium.com/swlh/policy-gradient-reinforcement-learning-with-keras-57ca6ed32555 "Information about implementing policy gradient").
