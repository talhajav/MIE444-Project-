import numpy as np

class Maze:

	def __init__(self):
		self.visual_maze = np.array([['o', 'o', 'o', 'o', 'x', 'o', 'x', 'o'],
									['o', 'o', 'x', 'o', 'o', 'o', 'o', 'o'],
						   			['o', 'x', 'o', 'x', 'x', 'o', 'x', 'o'],
						   			['o', 'o', 'o', 'o', 'o', 'o', 'x', 'o']])
		self.original_maze = np.array( [ [0,1,0,0,0,0,0,0],
							             [0,1,0,1,1,0,1,0],
							             [0,0,0,0,0,1,0,0],
							             [0,1,0,1,0,0,0,0]])
		self.maze_quadrant = np.array([	[4,2,1,2,2,4,5,3],
										[2,5,3,5,5,2,5,2],
										[1,4,5,4,2,0,2,1],
										[4,1,2,4,5,1,5,3]])
		self.wall_config_map = np.array([ [[0,0,1,1],[1,0,1,0],[0,0,1,0],[1,0,1,0],[1,0,1,0],[0,1,1,0],1,[0,1,1,1]],
								          [[0,1,0,1],1,[1,1,0,1],1,1,[0,1,0,1],1,[0,1,0,1]],
								          [[0,0,0,1],[0,1,1,0],1,[0,0,1,1],[1,0,1,0],[0,0,0,0],[1,0,1,0],[0,1,0,0]],
								          [[1,0,0,1],[1,0,0,0],[1,0,1,0],[1,1,0,0],1,[1,1,0,1],1,[1,1,0,1]]])
		
		# maze with loading zone blocked off as potential region for robot path
		self.maze_lz = np.array([[0,1,0,0,0,0,0,0],
					             [0,1,0,1,1,0,1,0],
					             [0,0,0,0,0,1,1,1],
					             [0,1,0,1,0,0,1,1]])
		# maze with loading zones as 2), dropoff locations as 3), 2 blocks closest to loading zone entrance as 4)
		self.maze2 = np.array([	[0,0,0,0,0,0,1,3],
							    [4,1,3,1,1,0,1,0],
							    [2,2,1,0,0,0,0,0],
							    [2,2,4,0,1,3,1,3]])
		# maze that will be changed to visualize path
		self.maze3 = np.array( [ [0,1,0,0,0,0,0,0],
					             [0,1,0,1,1,0,1,0],
					             [0,0,0,0,0,1,0,0],
					             [0,1,0,1,0,0,0,0]])

		# wall config from ultrasonic sensor readings				
										# N, E, S, W
		self.us_wall_config = np.array([[[0, 1, 1, 1], [1, 1, 1, 0], [1, 1, 0, 1], [1, 0, 1, 1],
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

										  # Front Right, Front Left, Back Right, Back Left
		self.ir_wall_config = np.array([ [[0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0],
					                      [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0],
					                      [0, 1, 1, 1], [1, 1, 1, 0], [1, 1, 0, 1], [1, 0, 1, 1],
					                      [0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1],
					                      [0, 1, 1, 1], [1, 1, 1, 0], [1, 1, 0, 1], [1, 0, 1, 1],
					                      [0, 1, 1, 1], [1, 1, 1, 0], [1, 1, 0, 1], [1, 0, 1, 1],
					                      [0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1],
					                      [1, 1, 1, 0], [1, 1, 0, 1], [1, 0, 1, 1], [0, 1, 1, 1]],

					                    [ [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1],
					                      [1, 1, 1, 0], [1, 1, 0, 1], [1, 0, 1, 1], [0, 1, 1, 1],
					                      [1, 0, 0, 0], [0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 0],
					                      [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0],
					                      [1, 0, 1, 1], [0, 1, 1, 1], [1, 1, 1, 0], [1, 1, 0, 1],
					                      [0, 1, 1, 1], [1, 1, 1, 0], [1, 1, 0, 1], [1, 0, 1, 1],
					                      [0, 0, 1, 1], [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1],
					                      [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1], [0, 0, 1, 0]],

					                    [ [0, 0, 1, 1], [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1],
					                      [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 1],
					                      [0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0],
					                      [1, 1, 1, 0], [1, 1, 0, 1], [1, 0, 1, 1], [0, 1, 1, 1],
					                      [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0],
					                      [1, 1, 0, 1], [1, 0, 1, 1], [0, 1, 1, 1], [1, 1, 1, 0],
					                      [1, 0, 1, 0], [0, 1, 0, 1], [1, 0, 1, 0], [0, 1, 0, 1],
					                      [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0]],

					                    [ [0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0],
					                      [1, 1, 1, 0], [1, 1, 0, 1], [1, 0, 1, 1], [0, 1, 1, 1],
					                      [1, 0, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0], [1, 1, 0, 0],
					                      [0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1],
					                      [0, 0, 1, 1], [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1],
					                      [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1],
					                      [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 1],
					                      [0, 1, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 1]]])