import serial 
import cv2
import time
import numpy as np
from localization import Localization
from visualize import Visualization
from path_planning import PathPlanning
"""
To check the port number on on Windows:
	Control panel > Hardware and Sound > Devices and Printers 
	> double click on the bluetooth device > choose Hardware tab
"""
# connect to bluetooth
port = 'COM5' # 'COM5', 'COM7'
ser = serial.Serial(port, baudrate=9600, timeout=1)
ser.write('A'.encode())

# initialize variables	
viz = Visualization()
us_sensor_readings = None
ir_sensor_readings = None
begin_localization = False

def get_us_sensor_reading(data):
	#  data = f"North: 1. East1: 4. East2: 5. South: 10. West1: 6. West2: 5"
	readings = {direction.strip(): int(measurement.strip()) for direction, measurement in [reading.split(":") for reading in data.split(".")]}
	viz.visualize(readings)

	us_sensor_readings = [readings["North"] > 12, 
						  readings["East1"] > 12 and readings["East2"] > 12,
						  readings["South"] > 12,
						  readings["West1"] > 12 and readings["West2"] > 12]
	us_sensor_readings = [0 if reading else 1 for reading in us_sensor_readings]

	return us_sensor_readings

while True:
	data = ser.readline().decode('latin-1').strip()
	if not data:
		continue

	if "North:" in data:
		us_sensor_readings = get_us_sensor_reading(data)
		# print(us_sensor_readings)
	elif "IRSensorReading" in data:
		ir_sensor_readings = [int(reading,10) for reading in data.split(":") if (reading != '' and reading != 'IRSensorReading')]
		print(ir_sensor_readings)
	else:
		print(data)

	if "Localization begin" in data:
		print("localization will begin!")
		begin_localization = True

	if begin_localization and us_sensor_readings and ir_sensor_readings:
		print("localizing.....")
		locz = Localization(us_sensor_readings, ir_sensor_readings)
		result = locz.localize()
		if result == "Move Forward":
			print("Moving forward to grab more data for localization")
			while True:
				ser.write("MOVEFORWARD\n".encode())
				data = ser.readline().decode('latin-1').strip()
				print(data)
				if "North:" in data:
					us_sensor_readings = get_us_sensor_reading(data)
				if "Stopped Moving" in data:
					break
			locz.update_wall_sensor_reading(us_sensor_readings)
			result = locz.localize2()

			if result == "Move Forward":
				print("Moving forward again to grab more data for localization")
				while True:		
					ser.write("MOVEFORWARD\n".encode());
					data = ser.readline().decode('latin-1').strip()
					if "North:" in data:
						us_sensor_readings = get_us_sensor_reading(data)
					if "Stopped Moving" in data:
						break
				locz.update_wall_sensor_reading2(us_sensor_reading)
				result = locz.localize3()

		if result == "Completed":
			ser.write("LOCALIZED\n".encode())
			planz = PathPlanning(locz.localized_coordinate)
			while(True):
				ser.write((str(planz.path_to_loading_zone) + "\n").encode())
				data = ser.readline().decode('latin-1').strip()
				if "Recieved the paths!" in data:
					break
		else:
			raise ValueError("LOCALIZATION FAILED")

		begin_localization = False

cv2.destroyAllWindows()