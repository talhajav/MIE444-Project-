import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import math
from maze import Maze

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

class PathPlanning(Maze):

    def __init__(self, coord, left_lz=True):
        super().__init__()
        self.starting_corrdinate = coord
        self.left_loading_zone = (1, 7)
        self.right_loading_zone = (3, 5)
        # where we want robot to go when it finishes loading the block
        self.end_loading_zone = self.left_loading_zone if left_lz else self.right_loading_zone
        self.find_path()

    def find_path(self):
        #Call robotPathtoLZ function to get path to blocks LZ. This is using a map with LZ blocks not a part of path solution space so it doesn't travel in LZ and move block to hard to reach location
        self.path_to_loading_zone = (self.robotPathtoLZ(self.starting_corrdinate, self.end_loading_zone))
        print(self.path_to_loading_zone)

        self.quadrant_list = []
        for idx in range(0, len(self.path_to_loading_zone)):
            self.quadrant_list.append([self.path_to_loading_zone[idx], 
                                      self.maze_quadrant[self.path_to_loading_zone[idx][0]][self.path_to_loading_zone[idx][1]]])

    def astar(self, maze, start, end):
        """Returns a list of tuples as a path from the given start to the given end in the given maze"""

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1] # Return reversed path

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if maze[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)

    def plotMaze(self, maze):
        plt.figure(figsize=(8,4))
        cmap = colors.ListedColormap(['Blue','red','purple','green','y'])
        plt.pcolor(maze[::-1],cmap=cmap,edgecolors='k', linewidths=3)
        plt.show()


    def convertToList(self, path):
        pathList=[]
        for point in path:
            pathList.append([point[0],point[1]])
        return pathList

    def robotPathtoLZ(self, start,end):
        #calling a* function that will return a list of coordinates on map for robot path
        if any(start == lz for lz in [(2,6), (2,7), (3,6), (3,7)]):
            path = self.astar(self.original_maze, start, end)
        else:
            path = self.astar(self.maze_lz, start, end)
        path=list(path)
        #adjusting maze list with robot path for visualization later
        for point in path:
            self.maze3[point[0]][point[1]]=2
        #plotting maze with path visualzed from start to end point
        self.plotMaze(self.maze3)
        
        return self.convertToList(path)

    #function to path plan to dropoff point
    def robotPathtoB(self, start,end):
        #calling a* function that will return a list of coordinates on map for robot path
        path = self.astar(self.original_maze, start, end)
        path=list(path)
        #adjusting maze list with robot path for visualization later
        for point in path:
            self.maze3[point[0]][point[1]]=2
        #plotting maze with path visualzed from start to end point
        self.plotMaze(self.maze3)

        return self.convertToList(path)

    def convertToMotion(self, path):
        yslope=[]
        xslope=[]
        steps=[]
        for i in range(1,len(path)):
            dx=path[i][0]-path[i-1][0]
            dy=path[i][1]-path[i-1][1]
            yslope.append(dy)
            xslope.append(dx)
            if dy == 0 and dx != 0:
                if dx == -1:
                    steps.append('Down')
                elif dx == 1:
                    steps.append('Up')

            elif dx == 0 and dy!=0:
                if dy == -1:
                    steps.append('Right')
                elif dy == 1:
                    steps.append('Left')
        motion=[]
        count=0

        # for j in range(1,len(steps)):
        #     if steps[j] == steps[j-1]:
        #         motion.append('fwd')
        #         #if steps[j] == steps[j+1]:
        #         #continue
        #         #motion.append('fwd')
        #     elif steps[j] == 'down' and steps[j-1] == 'left':
        #         motion.append('turn left')
        #     elif steps[j] == 'down' and steps[j-1] == 'right':
        #         motion.append('turn right')
        #     elif steps[j] == 'up' and steps[j-1] == 'left':
        #         motion.append('turn right')
        #     elif steps[j] == 'up' and steps[j-1] == 'right':
        #         motion.append('turn left')

        #     elif steps[j] == 'left' and steps[j-1] == 'down':
        #         motion.append('turn right')
        #     elif steps[j] == 'right' and steps[j-1] == 'down':
        #         motion.append('turn left')
        #     elif steps[j] == 'right' and steps[j-1] == 'up':
        #         motion.append('turn right')
        #     elif steps[j] == 'left' and steps[j-1] == 'up':
        #         motion.append('turn left')

        return steps
    
    def determine_orientation(self, coordinate_index):
        direction_value = coordinate_index[1]

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

    def modified_heading (inputheading):
        convertedHeading = 1
        if inputheading == 'Right':
            convertedHeading = 2
        if inputheading == 'Down':
            convertedHeading = 3
        if inputheading == 'Left':
            convertedHeading = 4

        return convertedHeading

    def quadrantChange(self, previous_location, next_location, USSensorReading, wallrow, path_list, pathtoLZ):
        count = 0
        heading = self.modified_heading(path_list[0])
        motion = ''
        US_Sensor_next_location = wallrow [next_location[0]][(next_location[1])*4+(heading-1)]
        for i in range (0, len(USSensorReading)):
            if US_Sensor_next_location[i] == USSensorReading [i]:
                count+=1
        if count == 4:
            pathtoLZ.remove(previous_location)
            motion = 'B'
            previous_location = next_location
            next_location = pathtoLZ [1]
            path_list.remove(path_list[0])
            count = 0
        else:
            motion = 'W'
        print (previous_location)
        print (next_location)
        print (pathtoLZ)
        print (motion)
        print (path_list)
        return (previous_location, next_location, pathtoLZ, path_list, motion)


    def robotMotion(self, nextHeading, currentLocation, wallrow, USSensorReading):

        convertedHeading = 1
        if nextHeading == 'Right':
            convertedHeading = 2
        if nextHeading == 'Down':
            convertedHeading = 3
        if nextHeading == 'Left':
            convertedHeading = 4

        motion = 'W'

        currentHeading=0
        count = 0
        USList=[]
        for i in range (currentLocation[1]*4,(currentLocation[1])*4+4):
            for j in range(0,4):
                if wallrow[currentLocation[0]][i][j] == USSensorReading[j]:
                    count+=1
            if count == 4:
                USList.append(currentLocation[0])
                USList.append(i)
            count = 0
        currentHeading = self.determine_orientation(USList)
        print (currentHeading)

        #Make a do while loop in Arduino to keep turning until wall sensor readings match the global direction value for the same quadrant
        change = currentHeading - convertedHeading
        if change == -1 or change == 3: #Right by 90 deg
            motion = 'D'
        if change == -2 or change == 2: #Right by 180 deg
            motion = 'D'
        if change == -3 or change == 1: #Left by 90 deg
            motion = 'A'

        print (motion)
        return motion