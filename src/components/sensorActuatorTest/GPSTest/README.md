## The GPS  
The GPS used is the Globalsat BU-353-S4.  
This program check if we received the values from the GPS and put the message with $GPGGA format in a .txt. this program is a .py because the GPS is log to the Raspberry pi. This program works for python 2.7.  

The format of message used is $GPGGA you can find usefull information about this format [here](https://fr.wikipedia.org/wiki/NMEA_0183 "Information GPGGA message").  
  
### Library needed 
The libraries you will need: -the serial library  