import serial
 
port = "/dev/ttyUSB0"
 
def parseGPS(data,f):
#    print "raw:", data #prints raw data
	print(data)
	if data[:6] == '$GPGGA':
		f.write(data)

 
 
print("Receiving GPS data")
f = open("log.txt","w")		#Open the file which will coontain the values of the Gps
ser = serial.Serial(port, baudrate = 4800, timeout = 0.5)	#Create a link between the program and the port
while True:
	data = ser.readline()	#Reading of the value
	parseGPS(data,f)