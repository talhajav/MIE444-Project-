#Origin at bottom left corner

import matplotlib.pyplot as plt
import math
import numpy as np

import matplotlib.pyplot as plt
from matplotlib import colors


##Initialize the maze lists
#if 1, then not possible to move there bc block
#[Front,Right,Back,]
wallList=[[[0,0,1,1],[1,0,1,0],[0,0,1,0],[1,0,1,0],[1,0,1,0],[0,1,1,0],1,[0,1,1,1]],
            [[0,1,0,1],1,[1,1,0,1],1,1,[0,1,0,1],1,[0,1,0,1]],
            [[0,0,0,1],[0,1,1,0],1,[0,0,1,1],[1,0,1,0],[0,0,0,0],[1,0,1,0],[0,1,0,0]],
            [[1,0,0,1],[1,0,0,0],[1,0,1,0],[1,1,0,0],1,[1,1,0,1],1,[1,1,0,1]]]




mazeQuadrant=[[4,2,1,2,2,4,5,3],
            [2,5,3,5,5,2,5,2],
            [1,4,5,4,2,0,2,1],
            [4,1,2,4,5,1,5,3]
            ]

mazeOriginal=[[0,1,0,0,0,0,0,0],
             [0,1,0,1,1,0,1,0],
             [0,0,0,0,0,1,0,0],
             [0,1,0,1,0,0,0,0]]


#maze with loading zone blocked off as potential region for robot path
mazeLZ=[[0,0,0,0,0,0,1,1],
      [0,1,1,1,1,0,1,0],
      [1,1,1,0,0,0,0,0],
      [1,1,0,0,1,1,1,1]]

#maze with loading zones(2) and dropoff locations(3), 2 blocks closest to loading zone entrance (4)
maze2=[[0,0,0,0,0,0,1,3],
      [4,1,3,1,1,0,1,0],
      [2,2,1,0,0,0,0,0],
      [2,2,4,0,1,3,1,3]]
#Maze list that will be changed to visualize path
maze3=[[0,1,0,0,0,0,0,0],
             [0,1,0,1,1,0,1,0],
             [0,0,0,0,0,1,0,0],
             [0,1,0,1,0,0,0,0]]

#loadingZone=[[3,0],[3,1],[2,0],[2,1]]
#dropoffPoint=[[1,2],[0,7],[3,5],[3,7]]


wallrow = np.array([[[0, 1, 1, 1], [1, 1, 1, 0], [1, 1, 0, 1], [1, 0, 1, 1],
                     [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1],
                     [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 1],
                     [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1],
                     [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1],
                     [0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1],
                     [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1],
                     [0, 0, 1, 1], [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1]],

                    [[0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0],
                     [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1],
                     [0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0],
                     [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1],
                     [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1],
                     [1, 1, 0, 1], [1, 0, 1, 1], [0, 1, 1, 1], [1, 1, 1, 0],
                     [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1],
                     [0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0]],

                    [[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1], [0, 0, 1, 0],
                     [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1],
                     [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0],
                     [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1],
                     [0, 0, 1, 1], [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1],
                     [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1],
                     [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 1],
                     [0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0]],

                    [[1, 1, 0, 1], [1, 0, 1, 1], [0, 1, 1, 1], [1, 1, 1, 0],
                     [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [0, 1, 0, 0],
                     [1, 1, 0, 1], [1, 0, 1, 1], [0, 1, 1, 1], [1, 1, 1, 0],
                     [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1],
                     [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0],
                     [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1],
                     [1, 0, 0, 0], [0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 0],
                     [1, 0, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0], [1, 1, 0, 0]]])


#locations of blocks just before entering dropff zone
LZRight=(3,5)
LZLeft=(1,7)

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
    plt.pcolor(maze[::-1],cmap=cmap,edgecolors='k', linewidths=3)
    plt.show()


def convertToList(path):
    pathList=[]
    for point in path:
        pathList.append([point[0],point[1]])
    return pathList

def robotPathtoLZ(start,end):
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
def robotPathtoB(start,end):
    #calling a* function that will return a list of coordinates on map for robot path
    path = astar(mazeOriginal, start, end)
    path=list(path)
    #adjusting maze list with robot path for visualization later
    for point in path:
        maze3[point[0]][point[1]]=2
#plotting maze with path visualzed from start to end point
    plotMaze(maze3)

    return convertToList(path)


#example starting coordinates
#start1=(2,7) #given from localization backend
start1=(0,0)
LZend=LZRight#can be this or LZRight depending on what we decided
#example path starting coordinates
#start2=(2,3)
#dropoff=(1,2)
#Call robotPathtoLZ function to get path to blocks LZ. This is using a map with LZ blocks not a part of path solution space so it doesn't travel in LZ and move block to hard to reach location

pathtoLZ=(robotPathtoLZ(start1,LZend))
print(pathtoLZ)


quadrantList=[]
for l in range(0,len(pathtoLZ)):
    quadrantList.append([pathtoLZ[l],mazeQuadrant[pathtoLZ[l][0]][pathtoLZ[l][1]]])

#print(quadrantList)

#Converting path list of vectors corresponding to wasd
def convertToMotion(path):
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

    for j in range(1,len(steps)):
        if steps[j] == steps[j-1]:
            motion.append('fwd')
            #if steps[j] == steps[j+1]:
            #continue
            #motion.append('fwd')
        elif steps[j] == 'down' and steps[j-1] == 'left':
            motion.append('turn left')
        elif steps[j] == 'down' and steps[j-1] == 'right':
            motion.append('turn right')
        elif steps[j] == 'up' and steps[j-1] == 'left':
            motion.append('turn right')
        elif steps[j] == 'up' and steps[j-1] == 'right':
            motion.append('turn left')

        elif steps[j] == 'left' and steps[j-1] == 'down':
            motion.append('turn right')
        elif steps[j] == 'right' and steps[j-1] == 'down':
            motion.append('turn left')
        elif steps[j] == 'right' and steps[j-1] == 'up':
            motion.append('turn right')
        elif steps[j] == 'left' and steps[j-1] == 'up':
            motion.append('turn left')

    return steps

#    print(motion)

#Know initial coordinate from localization and know our final destination; path is returned
#if two or more motion steps are the same, continue moving forward until state change

path_list = convertToMotion(pathtoLZ)
print (path_list)

def determine_orientation (coordinate_index):


    direction_value = coordinate_index[1]

    switcher = {
            0: 1,
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
            31: 4,
            }
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

def quadrantChange(previous_location, next_location, USSensorReading, wallrow, path_list, pathtoLZ):
    count = 0
    heading = modified_heading(path_list[0])
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


def robotMotion(nextHeading, currentLocation, wallrow, USSensorReading):

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
    currentHeading = determine_orientation(USList)
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

USSensorReading1 = [0, 1, 0, 1]
previous_or_initialLocation = [1, 0]
upcominglocation = [2,0]

robotMotion('Left',[2,0],wallrow,[0,0,0,1])
#print (path_list)
quadrantChange (previous_or_initialLocation, upcominglocation, USSensorReading1, wallrow, path_list, pathtoLZ)
