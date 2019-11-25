#This program is setting up python-serial communication for IR Sensor readings
import serial
import cv2
import time
import numpy as np
"""
To check the port number on on Windows:
	Control panel > Hardware and Sound > Devices and Printers
	> double click on the bluetooth device > choose Hardware tab
"""
#change port when using
port = '/dev/cu.usbserial-1410' # 'COM4', 'COM7'
ser = serial.Serial(port, baudrate=9600, timeout=1)


#IRReading Format = [FR,FL,BR,BL]
IRArray=[]
while True:
	"""Test if python can recieve data"""
	IRArray=[]
	data = ser.readline().decode('latin-1').strip().split('\n')

	IRdata=(data[0].split(':'))
	if "SensorReading" in IRdata:
		for reading in IRdata:
			if reading != '' and reading != 'SensorReading':
				IRArray.append(int(reading,10))
		print(IRArray)
	else:
		print(data)
#IR Array sent to Localization IR database to get starting point for path planning 
