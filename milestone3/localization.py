import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import math
from maze import Maze

class Localization(Maze):

	def __init__(self, us_sensor_reading, ir_sensor_reading):
		super().__init__()
		#What arduino has to supply
		self.us_sensor_reading = us_sensor_reading # [1, 0, 0, 1] #ultrasonic reading from robot
		self.new_wall_sensor_reading = None
		self.new_wall_sensor_reading2 = None
		#IR Sensor reading
		self.ir_sensor_reading = ir_sensor_reading # [0, 1, 1, 1]

		self.drive_mode = 'B'
		self.localized = False
		self.localized_coordinate = (0, 0)

	def update_wall_sensor_reading(self, sensor_reading):
		self.new_wall_sensor_readings = sensor_reading

	def update_wall_sensor_reading2(self, sensor_reading):
		self.new_wall_sensor_readings2 = sensor_reading

	def print_localization(self):
		self.visual_map[self.localized_coordinate] = 'H'
		print(self.visual_map)

	def determine_direction(self, coordinate_original):
	    direction_value = coordinate_original[0][1]
	    switcher = {0: 1,
		            1: 2,
		            2: 3,
		            3: 4,
		            4: 1,
		            5: 2,
		            6: 3,
		            7: 4,
		            8: 1,
		            9: 2,
		            10: 3,
		            11: 4,
		            12: 1,
		            13: 2,
		            14: 3,
		            15: 4,
		            16: 1,
		            17: 2,
		            18: 3,
		            19: 4,
		            20: 1,
		            21: 2,
		            22: 3,
		            23: 4,
		            24: 1,
		            25: 2,
		            26: 3,
		            27: 4,
		            28: 1,
		            29: 2,
		            30: 3,
		            31: 4}
	    return switcher.get(direction_value, None)

	def determine_direction_additional(self, coordinate_original):
	    direction_value = coordinate_original[1]
	    switcher = {0: 1,
		            1: 2,
		            2: 3,
		            3: 4,
		            4: 1,
		            5: 2,
		            6: 3,
		            7: 4,
		            8: 1,
		            9: 2,
		            10: 3,
		            11: 4,
		            12: 1,
		            13: 2,
		            14: 3,
		            15: 4,
		            16: 1,
		            17: 2,
		            18: 3,
		            19: 4,
		            20: 1,
		            21: 2,
		            22: 3,
		            23: 4,
		            24: 1,
		            25: 2,
		            26: 3,
		            27: 4,
		            28: 1,
		            29: 2,
		            30: 3,
		            31: 4}
	    return switcher.get(direction_value, None)

    def remove_repeats (self, inputlist):
	    for i in range (0, len(inputlist)):
	        inputlist[i]= list(set(inputlist[i]))
	    return inputlist

	def row_indices(self, modifiedinputlist):
	    indexedoutput = []
	    for i in range(0, len(modifiedinputlist)):
	        if len(modifiedinputlist[i])== 0:
	            continue
	        for j in range(0, len(modifiedinputlist[i])):
	            indexedoutput.append([i, modifiedinputlist[i][j]])
	    return indexedoutput


	def wall_quadrants_prediction (self, wallrow, sensor_reading):
	    count = 0

	    wall_narrowed_list = [[],[],[],[]]
	    wall_narrowed_list_original = [[],[],[],[]]

	    wall_prediction = []

	    for h in range (0, len(wallrow)):
	        for i in range (0, len(wallrow[0])):
	            for j in range (0, len(wallrow[0][0])):
	                if wallrow [h][i][j] == sensor_reading[j]:
	                    count += 1
	            if count == 4:
	                wall_narrowed_list_original[h].append(i)
	                wall_narrowed_list[h].append(i//4)
	            count = 0

	    wall_narrowed_list_original = remove_repeats(wall_narrowed_list_original)

	    wall_narrowed_list = remove_repeats (wall_narrowed_list)

	#    for i in range (0, len(wall_narrowed_list_original)):
	#        wall_narrowed_list_original[i]= list(set(wall_narrowed_list_original[i]))
	#
	#    for i in range (0, len(wall_narrowed_list)):
	#        wall_narrowed_list[i]= list(set(wall_narrowed_list[i]))

	#    for i in range(0, len(wall_narrowed_list)):
	#        if len(wall_narrowed_list[i])== 0:
	#            continue
	#        for j in range(0, len(wall_narrowed_list[i])):
	#            wall_input_to_IR.append([i, wall_narrowed_list[i][j]])

	    wall_narrowed_list_original = row_indices (wall_narrowed_list_original)

	    wall_prediction = row_indices (wall_narrowed_list)

	    return (wall_prediction, wall_narrowed_list_original)

	def final_quadrant_prediction (self, wall_input_to_IR, ir_sensor_reading):

	    drive_mode = ""
	    count_ir = 0
	    quadrant = []

	    for h in range(0, len(wall_input_to_IR)):
	            for j in range (0, 4):
	                if (ir_sensor_reading[j] == self.ir_wall_config[(wall_input_to_IR[h][0])][(wall_input_to_IR[h][1])][j]):
	                    count_ir+=1
	            if count_ir == 4:
	                quadrant.append(wall_input_to_IR[h])
	            count_ir = 0

	    quadrant_set = set(tuple(x) for x in quadrant)
	    quadrant = [list(x) for x in quadrant_set]

	    #quadrant = [i for i in quadrant if (min(i)>=0 and max(i)>15)]

	    if len(quadrant) > 1:
	        drive_mode = "W"
	        return (drive_mode, quadrant)

	    else:
	        print (quadrant)
	        quadrant [0][1] = (quadrant[0][1])//4
	        direction = determine_direction (quadrant)
	        return (quadrant, direction)

	def additional_movement_localization(self, quadrant, new_wall_sensor_reading, wallrow, IRrow):
	    new_locations = []
	    final_localized_location = []
	    counter_check = 0
	    #print (quadrant)
	    for i in range (0, len(quadrant)):
	        a = determine_direction_additional (quadrant[i])
	        if (a==2):
	            new_locations.append([quadrant[i][0], (quadrant[i][1]-4)])
	        if (a==4):
	            new_locations.append([quadrant[i][0], (quadrant[i][1]+4)])
	        if (a==1):
	             new_locations.append([(quadrant[i][0]+1), quadrant[i][1]])
	        if (a==3):
	             new_locations.append([(quadrant[i][0]-1), quadrant[i][1]])

	    for i in range (0, len(new_locations)):
	        for j in range (0, len(new_wall_sensor_reading)):
	            if (wallrow [new_locations[i][0]][new_locations[i][1]][j] == new_wall_sensor_reading [j]):
	                counter_check +=1
	        if counter_check == 4:
	            final_localized_location.append([new_locations[i][0], (new_locations[i][1])])
	        counter_check = 0
	    return final_localized_location

	def localize(self):
		#function to make prediction of where it is based on US sensor
		US_wall_prediction = (wall_quadrants_prediction(self.us_wall_config, self.us_sensor_reading))[0]

		print(US_wall_prediction)

		#Merging IR sensor data from Arduino with predicitons from US sensor predicitons from above to identify location
		#output is motion direction if not localized or quadrant
		self.localized_location = final_quadrant_prediction(
								(wall_quadrants_prediction(self.us_wall_config, self.us_sensor_reading))[1], 
								self.ir_sensor_reading)

		if type(self.localized_location[0]) == str:
			self.drive_mode = self.localized_location[0]
			print(self.drive_mode)
			return "Move Forward"

		print(self.localized_location, self.drive_mode)
		startPoint = (self.localized_location[0][0])
		self.localized_coordinate = startPoint
		self.localized = True
		print(startPoint)

		return "Completed"

	def localize2(self):
		assert(self.new_wall_sensor_reading	is not None)
	    self.localized_location = additional_movement_localization(self.localized_location[1], 
		    													   self.new_wall_sensor_reading, 
		    													   self.us_wall_config, 
		    													   self.ir_wall_config)
	    print(self.localized_location)
	    if (len(self.localized_location) == 1):
	        self.localized_location [0][1] = self.localized_location[0][1]//4
	        self.drive_mode = "B"

	    print(self.drive_mode)
	    if (len(self.localized_location)>1):
	    	self.drive_mode = "W"
	        return "Move Forward"
	    print(self.localized_location, self.drive_mode)

		startPoint = (self.localized_location[0][0])
		self.localized_coordinate = startPoint
		self.localized = True
		print(startPoint)

		return "Completed"

	def localize3(self):
		assert(self.new_wall_sensor_reading	is not None)
        self.localized_location = additional_movement_localization(self.localized_location, 
	        													   self.new_wall_sensor_reading2,
	        													   self.us_wall_config, 
	    													  	   self.ir_wall_config)
        self.drive_mode = "B"
        self.localized_location [0][1] = (self.localized_location [0][1])
        direction = determine_direction_additional(self.localized_location[0])
        self.localized_location.append(direction)
        print(self.localized_location, self.drive_mode, direction)

		startPoint = (self.localized_location[0][0])
		self.localized_coordinate = startPoint
		self.localized = True
		print(startPoint)

		return "Completed"