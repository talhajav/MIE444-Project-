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
		self.left_loading_zone = (1, 0)
		self.right_loading_zone = (3, 2)
		# where we want robot to go when it finishes loading the block
		self.end_loading_zone = if left_lz self.left_loading_zone else self.right_loading_zone
		self.find_path()

	def find_path(self):
		#Call robotPathtoLZ function to get path to blocks LZ. This is using a map with LZ blocks not a part of path solution space so it doesn't travel in LZ and move block to hard to reach location
		self.path_to_loading_zone = (self.robotPathtoLZ(self.starting_corrdinate, self.end_loading_zone))
		print(self.path_to_loading_zone)

		self.quadrant_list = []
		for idx in range(0, len(self.path_to_loading_zone)):
		    self.quadrantList.append([self.path_to_loading_zone[idx], 
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
	    plt.pcolor(maze[::],cmap=cmap,edgecolors='k', linewidths=3)
	    plt.show()


	def convertToList(self, path):
	    pathList=[]
	    for point in path:
	        pathList.append([point[0],point[1]])
	    return pathList

	def robotPathtoLZ(self, start,end):
	    #calling a* function that will return a list of coordinates on map for robot path
	    path = astar(mazeOriginal, start, end)
	    path=list(path)
	    #adjusting maze list with robot path for visualization later
	    for point in path:
	        maze3[point[0]][point[1]]=2
		#plotting maze with path visualzed from start to end point
	    plotMaze(maze3)
	    
	    return convertToList(path)

	#function to path plan to dropoff point
	def robotPathtoB(self, start,end):
	    #calling a* function that will return a list of coordinates on map for robot path
	    path = astar(mazeOriginal, start, end)
	    path=list(path)
	    #adjusting maze list with robot path for visualization later
	    for point in path:
	        maze3[point[0]][point[1]]=2
		#plotting maze with path visualzed from start to end point
	    plotMaze(maze3)

	    return convertToList(path)
