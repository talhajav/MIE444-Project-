import serial 
import cv2
import time
import numpy as np
from localization import Localization
"""
To check the port number on on Windows:
	Control panel > Hardware and Sound > Devices and Printers 
	> double click on the bluetooth device > choose Hardware tab
"""
# bluetooth variables
port = 'COM5' # 'COM5', 'COM7'
ser = serial.Serial(port, baudrate=9600, timeout=1)

# IR variables
IRArray=[] # format = [front right, front left, back right, back left]

# localization variables
locz = Localization()

# visualization variables
window_size = 800	
org = (int(window_size / 2), int(window_size / 2)) # (x, y)
rover_pos = [(org[0]-28, org[1]-50), (org[0]+32, org[1]+50)] # top left corner, bot right corner
font = cv2.FONT_HERSHEY_SIMPLEX 
thickness = 1
font_scale = 1
text_color = (255, 0, 255) 
line_color = (0, 0, 255)
pixel_cm_scale = 10
visual_limit = 30 # cm

def visualize(readings):
	image = np.zeros((window_size, window_size, 3), dtype="uint8")
	image = cv2.putText(image, str(readings["North"])+"cm", (org[0] - 2, org[1] - 300), font, font_scale, text_color, thickness, cv2.LINE_AA) 
	image = cv2.putText(image, str(readings["East1"])+"cm", (org[0] + 300, org[1] - 160), font, font_scale, text_color, thickness, cv2.LINE_AA) 
	image = cv2.putText(image, str(readings["East2"])+"cm", (org[0] + 300, org[1] + 160), font, font_scale, text_color, thickness, cv2.LINE_AA) 
	image = cv2.putText(image, str(readings["South"])+"cm", (org[0] - 2, org[1] + 300), font, font_scale, text_color, thickness, cv2.LINE_AA) 
	image = cv2.putText(image, str(readings["West1"])+"cm", (org[0] - 320, org[1] - 160), font, font_scale, text_color, thickness, cv2.LINE_AA) 
	image = cv2.putText(image, str(readings["West2"])+"cm", (org[0] - 320, org[1] + 160), font, font_scale, text_color, thickness, cv2.LINE_AA) 
	image = cv2.rectangle(image, rover_pos[0], rover_pos[1], text_color, thickness) # represent the rover
	if readings["North"] < visual_limit:
		image = cv2.line(image, (org[1]-100, rover_pos[0][1]-readings["North"]*pixel_cm_scale), 
								(org[1]+100, rover_pos[0][1]-readings["North"]*pixel_cm_scale), 
								line_color, thickness) 
	if readings["South"] < visual_limit:
		image = cv2.line(image, (org[1]-100, rover_pos[1][1]+readings["South"]*pixel_cm_scale), 
								(org[1]+100, rover_pos[1][1]+readings["South"]*pixel_cm_scale), 
								line_color, thickness) 
	if readings["East1"] < visual_limit and readings["East2"] < visual_limit:
		if abs(readings["East1"] - readings["East2"]) > 5:
			image = cv2.line(image, (rover_pos[1][0]+readings["East1"]*pixel_cm_scale, rover_pos[0][1]-50), 
									(rover_pos[1][0]+readings["East1"]*pixel_cm_scale, org[1]), 
									line_color, thickness) 
			image = cv2.line(image, (rover_pos[1][0]+readings["East2"]*pixel_cm_scale, org[1]), 
									(rover_pos[1][0]+readings["East2"]*pixel_cm_scale, rover_pos[1][1]+50), 
									line_color, thickness) 
		else:
			image = cv2.line(image, (rover_pos[1][0]+readings["East1"]*pixel_cm_scale, rover_pos[0][1]-50), 
									(rover_pos[1][0]+readings["East2"]*pixel_cm_scale, rover_pos[1][1]+50), 
									line_color, thickness) 
	if readings["West1"] < visual_limit and readings["West2"] < visual_limit:
		if abs(readings["West1"] - readings["West2"]) > 5:
			image = cv2.line(image, (rover_pos[0][0]-readings["West1"]*pixel_cm_scale, rover_pos[0][1]-50), 
									(rover_pos[0][0]-readings["West1"]*pixel_cm_scale, org[1]), 
									line_color, thickness) 
			image = cv2.line(image, (rover_pos[0][0]-readings["West2"]*pixel_cm_scale, org[1]), 
									(rover_pos[0][0]-readings["West2"]*pixel_cm_scale, rover_pos[1][1]+50), 
									line_color, thickness) 
		else:
			image = cv2.line(image, (rover_pos[0][0]-readings["West1"]*pixel_cm_scale, rover_pos[0][1]-50), 
									(rover_pos[0][0]-readings["West2"]*pixel_cm_scale, rover_pos[1][1]+50), 
									line_color, thickness) 
	cv2.imshow('Sensor Visualization', image)
	cv2.waitKey(1) # display window for 1 ms

# ser.write("A".encode())
while True:
	data = ser.readline().decode('latin-1').strip()
	if not data:
		continue

	IRdata=(data[0].split(':'))

	if "North:" in data:
		#  data = f"North: 1. East1: 4. East2: 5. South: 10. West1: 6. West2: 5"
		readings = {direction.strip(): int(measurement.strip()) for direction, measurement in [reading.split(":") for reading in data.split(".")]}
		visualize(readings)
	elif "IRSensorReading" in data:
		IRArray = []
		IRdata = data.split(":")
		for reading in IRdata:
			if reading != '' and reading != 'IRSensorReading':
				IRArray.append(int(reading,10))
		print(IRArray)
	else:
		print(data)

cv2.destroyAllWindows()