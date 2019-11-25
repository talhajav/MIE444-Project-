#function to convert sensor readings into array for wall

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


USArray=[0,0,0,0];
#example output
#{'North': 34, 'East1': 39, 'East2': 58, 'South': 28, 'West1': 13, 'West2': 16}

while True:
    data = ser.readline().decode('latin-1').strip()
    if "North:" in data:
        #data = f"North: 1. East1: 4. East2: 5. South: 10. West1: 6. West2: 5"
        readings = {direction.strip(): int(measurement.strip()) for direction, measurement in [reading.split(":") for reading in data.split(".")]}
        if int(readings["North"]) > 12:
            USArray[0]=1
        if int(readings["East1"]) > 12 and int(readings["East2"]) > 12:
            USArray[1]=1
        if int(readings["South"]) > 12:
            USArray[2]=1
        if int(readings["West1"]) > 12 and int(readings["West2"]) > 12:
            USArray[3]=1


    print(USArray)
