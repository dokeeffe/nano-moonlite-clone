import serial 
import time 
arduino = serial.Serial(port='/dev/ttyACM1', baudrate=9600, timeout=.1) 
def write_read(): 
	arduino.write(bytes(':TL#', 'UTF-8')) 
	time.sleep(0.02) 
	data = arduino.readline() 
	return data 

print(write_read())
