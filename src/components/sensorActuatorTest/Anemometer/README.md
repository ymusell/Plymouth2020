## The Anemometer and the vane sensor  
This component is the Davis Anemometer which can give the speed and the orientation of the wind.  
For this programm, we need to add the library TimerOne.h.  
You can download this library or find other information here: [link](https://www.arduinolibraries.info/libraries/timer-one "Information for the library TimerOne.h")  

## Direction

Used an extended kalman filter.
  
And use the yaw calculated with the IMU to have the wind direction in the frame of earth.  

## Speed

Sometimes there are some random data such as 123, 216, etc ... when the wind speed is around 5 or 6 meters/seconds. A reactivity was needed, that is why a threshold has been applyied rather than a low_pass filter.  

Return the true wind and not the apparent wind. It need the speed given by the GPS to compute the true wind. If there is no value, the apparent wind is returned.
