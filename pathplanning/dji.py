#Origin at bottom left corner

import matplotlib.pyplot as plt
import math
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import colors


##Initialize the maze lists
mazeOriginal=[[0,0,0,0,0,0,1,0],
            [0,1,0,1,1,0,1,0],
            [0,0,1,0,0,0,0,0],
            [0,0,0,0,1,0,1,0]]


#maze with loading zone blocked off as potential region for robot path
mazeLZ=[[0,0,0,0,0,0,1,0],
      [0,1,0,1,1,0,1,0],
      [1,1,1,0,0,0,0,0],
      [1,1,0,0,1,0,1,0]]

#maze with loading zones(2) and dropoff locations(3), 2 blocks closest to loading zone entrance (4)
maze2=[[0,0,0,0,0,0,1,3],
      [4,1,3,1,1,0,1,0],
      [2,2,1,0,0,0,0,0],
      [2,2,4,0,1,3,1,3]]
#Maze list that will be changed to visualize path
maze3=[[0,0,0,0,0,0,1,0],
      [0,1,0,1,1,0,1,0],
      [0,0,1,0,0,0,0,0],
      [0,0,0,0,1,0,1,0]]

#loadingZone=[[3,0],[3,1],[2,0],[2,1]]
#dropoffPoint=[[1,2],[0,7],[3,5],[3,7]]

#locations of blocks just before entering dropff zone
LZRight=(3,2)
LZLeft=(1,0)

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


def astar(maze, start, end):
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

def plotMaze(maze):
    plt.figure(figsize=(8,4))
    cmap = colors.ListedColormap(['Blue','red','purple','green','y'])
    plt.pcolor(maze[::],cmap=cmap,edgecolors='k', linewidths=3)
    plt.show()

def robotPathtoLZ(start,end):
    #calling a* function that will return a list of coordinates on map for robot path
    path = astar(mazeLZ, start, end)
    path=list(path)
    #adjusting maze list with robot path for visualization later
    for point in path:
        maze3[point[0]][point[1]]=2
#plotting maze with path visualzed from start to end point
    plotMaze(maze3)
#function to path plan to dropoff point
def robotPathtoB(start,end):
    #calling a* function that will return a list of coordinates on map for robot path
    path = astar(mazeOriginal, start, end)
    path=list(path)
    #adjusting maze list with robot path for visualization later
    for point in path:
        maze3[point[0]][point[1]]=2
#plotting maze with path visualzed from start to end point
    plotMaze(maze3)

#example starting coordinates
start=(2,7) #given from localization backend
LZend=LZLeft #can be this or LZRight depending on what we decided


#Call robotPathtoLZ function to get path to blocks LZ. This is using a map with LZ blocks not a part of path solution space so it doesn't travel in LZ and move block to hard to reach location
pathtoLZ=robotPathtoLZ(start,LZend)
#call function to get path to dropoff zone
pathtoB=robotPathtoB(start,end)
