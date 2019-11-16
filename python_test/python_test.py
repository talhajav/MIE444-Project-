import serial 
import time

"""
To check the port number on on Windows:
	Control panel > Hardware and Sound > Devices and Printers 
	> double click on the bluetooth device > choose Hardware tab
"""
port = 'COM7' # 'COM4', 'COM7'
ser = serial.Serial(port, baudrate=9600, timeout=1)

while True:
	"""Test if python can recieve data"""
	data = ser.readline().decode('latin-1').strip()
	print(data)

	"""Test if python can send data (using milestone 2 arduino code)"""
	# ser.write('A'.encode())
