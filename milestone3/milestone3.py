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

# initialize variables
IRArray=[] # format = [front right, front left, back right, back left]
viz = Visualization()
us_sensor_readings = None
ir_sensor_readings = None
begin_localization = False

while True:
	data = ser.readline().decode('latin-1').strip()
	if not data:
		continue

	if "North:" in data:
		#  data = f"North: 1. East1: 4. East2: 5. South: 10. West1: 6. West2: 5"
		us_sensor_readings = {direction.strip(): int(measurement.strip()) for direction, measurement in [reading.split(":") for reading in data.split(".")]}
		viz.visualize(us_sensor_readings)
	elif "IRSensorReading" in data:
		IRdata = data.split(":")
		for reading in IRdata:
			if reading != '' and reading != 'IRSensorReading':
				ir_sensor_readings.append(int(reading,10))
	else:
		print(data)

	if "Localization begin" in data:
		begin_localization = True

	if begin_localization and us_sensor_readings and ir_sensor_readings:
		locz = Localization(us_sensor_readings, ir_sensor_readings)
		result = locz.localize()

		if result is "MoveForward":
			ser.write("w\n".encode());
			while True:
				data = ser.readline().decode('latin-1').strip()
				if "North:" in data:
					us_sensor_readings = {direction.strip(): int(measurement.strip()) for direction, measurement in [reading.split(":") for reading in data.split(".")]}
					viz.visualize(us_sensor_readings)
				if "Stopped Moving" in data:
					break
			locz.update_wall_sensor_reading(us_sensor_reading)
			result = locz.localize2()

			if result is "MoveForward":
				ser.write("w\n".encode());
				while True:
					data = ser.readline().decode('latin-1').strip()
					if "North:" in data:
						us_sensor_readings = {direction.strip(): int(measurement.strip()) for direction, measurement in [reading.split(":") for reading in data.split(".")]}
						viz.visualize(us_sensor_readings)
					if "Stopped Moving" in data:
						break
				locz.update_wall_sensor_reading2(us_sensor_reading)
				result = locz.localize3()

		if result is "Completed":
			ser.write("LOCALIZED\n")
			planz = PathPlanning(locz.localized_coordinate)
			ser.write((str(planz.path_to_loading_zone) + "\n").encode())
		else:
			raise ValueError("LOCALIZATION FAILED")

cv2.destroyAllWindows()