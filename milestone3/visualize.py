import cv2
import numpy as np

class Visualization:

	def __init__(self):
		# visualization variables
		self.window_size = 800	
		self.org = (int(self.window_size / 2), int(self.window_size / 2)) # (x, y)
		self.rover_pos = [(self.org[0]-28, self.org[1]-50), (self.org[0]+32, self.org[1]+50)] # top left corner, bot right corner
		self.font = cv2.FONT_HERSHEY_SIMPLEX 
		self.thickness = 1
		self.font_scale = 1
		self.text_color = (255, 0, 255) 
		self.line_color = (0, 0, 255)
		self.pixel_cm_scale = 10
		self.visual_limit = 30 # cm

	def visualize(self, readings):
		image = np.zeros((self.window_size, self.window_size, 3), dtype="uint8")
		image = cv2.putText(image, str(readings["North"])+"cm", (self.org[0] - 2, self.org[1] - 300), self.font, self.font_scale, self.text_color, self.thickness, cv2.LINE_AA) 
		image = cv2.putText(image, str(readings["East1"])+"cm", (self.org[0] + 300, self.org[1] - 160), self.font, self.font_scale, self.text_color, self.thickness, cv2.LINE_AA) 
		image = cv2.putText(image, str(readings["East2"])+"cm", (self.org[0] + 300, self.org[1] + 160), self.font, self.font_scale, self.text_color, self.thickness, cv2.LINE_AA) 
		image = cv2.putText(image, str(readings["South"])+"cm", (self.org[0] - 2, self.org[1] + 300), self.font, self.font_scale, self.text_color, self.thickness, cv2.LINE_AA) 
		image = cv2.putText(image, str(readings["West1"])+"cm", (self.org[0] - 320, self.org[1] - 160), self.font, self.font_scale, self.text_color, self.thickness, cv2.LINE_AA) 
		image = cv2.putText(image, str(readings["West2"])+"cm", (self.org[0] - 320, self.org[1] + 160), self.font, self.font_scale, self.text_color, self.thickness, cv2.LINE_AA) 
		image = cv2.rectangle(image, self.rover_pos[0], self.rover_pos[1], self.text_color, self.thickness) # represent the rover
		if readings["North"] < self.visual_limit:
			image = cv2.line(image, (self.org[1]-100, self.rover_pos[0][1]-readings["North"]*self.pixel_cm_scale), 
									(self.org[1]+100, self.rover_pos[0][1]-readings["North"]*self.pixel_cm_scale), 
									self.line_color, self.thickness) 
		if readings["South"] < self.visual_limit:
			image = cv2.line(image, (self.org[1]-100, self.rover_pos[1][1]+readings["South"]*self.pixel_cm_scale), 
									(self.org[1]+100, self.rover_pos[1][1]+readings["South"]*self.pixel_cm_scale), 
									self.line_color, self.thickness) 
		if readings["East1"] < self.visual_limit and readings["East2"] < self.visual_limit:
			if abs(readings["East1"] - readings["East2"]) > 5:
				image = cv2.line(image, (self.rover_pos[1][0]+readings["East1"]*self.pixel_cm_scale, self.rover_pos[0][1]-50), 
										(self.rover_pos[1][0]+readings["East1"]*self.pixel_cm_scale, self.org[1]), 
										self.line_color, self.thickness) 
				image = cv2.line(image, (self.rover_pos[1][0]+readings["East2"]*self.pixel_cm_scale, self.org[1]), 
										(self.rover_pos[1][0]+readings["East2"]*self.pixel_cm_scale, self.rover_pos[1][1]+50), 
										self.line_color, self.thickness) 
			else:
				image = cv2.line(image, (self.rover_pos[1][0]+readings["East1"]*self.pixel_cm_scale, self.rover_pos[0][1]-50), 
										(self.rover_pos[1][0]+readings["East2"]*self.pixel_cm_scale, self.rover_pos[1][1]+50), 
										self.line_color, self.thickness) 
		if readings["West1"] < self.visual_limit and readings["West2"] < self.visual_limit:
			if abs(readings["West1"] - readings["West2"]) > 5:
				image = cv2.line(image, (self.rover_pos[0][0]-readings["West1"]*self.pixel_cm_scale, self.rover_pos[0][1]-50), 
										(self.rover_pos[0][0]-readings["West1"]*self.pixel_cm_scale, self.org[1]), 
										self.line_color, self.thickness) 
				image = cv2.line(image, (self.rover_pos[0][0]-readings["West2"]*self.pixel_cm_scale, self.org[1]), 
										(self.rover_pos[0][0]-readings["West2"]*self.pixel_cm_scale, self.rover_pos[1][1]+50), 
										self.line_color, self.thickness) 
			else:
				image = cv2.line(image, (self.rover_pos[0][0]-readings["West1"]*self.pixel_cm_scale, self.rover_pos[0][1]-50), 
										(self.rover_pos[0][0]-readings["West2"]*self.pixel_cm_scale, self.rover_pos[1][1]+50), 
										self.line_color, self.thickness) 
		cv2.imshow('Sensor Visualization', image)
		cv2.waitKey(1) # display window for 1 ms