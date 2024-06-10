# Simple script that simulates a bot moving inside an Arena, following a series of commands
# by Alberto Tonda, 2018 <alberto.tonda@gmail.com>

import sys
import numpy as np

def lineRectCollision(wall : dict, x1 : int, y1 : int, x2 : int, y2 : int) -> bool:
	# check if the line defined by points (x1,y1) and (x2,y2) intersects with the rectangle defined by the wall
	# bottom-left corner of the wall
	xMin = wall["x"]
	yMin = wall["y"]
	# top-right corner of the wall
	xMax = wall["x"] + wall["width"]
	yMax = wall["y"] + wall["height"]
	
	# check if the line intersects with the rectangle
	# https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
	def ccw(A,B,C):
		return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
	
	def intersect(A,B,C,D):
		return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
	
	return intersect( (x1,y1), (x2,y2), (xMin,yMin), (xMax,yMin) ) or intersect( (x1,y1), (x2,y2), (xMax,yMin), (xMax,yMax) ) or intersect( (x1,y1), (x2,y2), (xMax,yMax), (xMin,yMax) ) or intersect( (x1,y1), (x2,y2), (xMin,yMax), (xMin,yMin) )


'''This function accepts in input a list of strings, and tries to parse them to update the position of a robot. Then returns distance from objective.'''
def fitnessRobot(listOfCommands, visualize=False) :

	# the Arena is a 100 x 100 pixel space
	arenaLength = 100
	arenaWidth = 100
	
	# let's also put a couple of walls in the arena; walls are described by a set of 4 (x,y) corners (bottom-left, top-left, top-right, bottom-right)
	walls = []

	wall1 = dict()
	wall1["x"] = 30
	wall1["y"] = 0
	wall1["width"] = 10
	wall1["height"] = 80

	wall2 = dict()
	wall2["x"] = 70
	wall2["y"] = 20
	wall2["width"] = 10
	wall2["height"] = 80

	walls.append(wall1)
	walls.append(wall2)
	
	# initial position and orientation of the robot
	startX = robotX = 10
	startY = robotY = 10
	startDegrees = 0 # 90°
	
	# position of the objective
	objectiveX = 90
	objectiveY = 90
	
	# this is a list of points that the robot will visit; used later to visualize its path
	positions = []
	positions.append( [robotX, robotY] )
	
	# TODO move robot, check that the robot stays inside the arena
	angle = startDegrees * np.pi / 180
	for command in listOfCommands:
		commandType, n = command.split(' ')
		if commandType == "rotate":
			angle += int(n) * np.pi / 180
		elif commandType == "move":
			x1 = robotX
			y1 = robotY
			x2 = robotX + int(int(n) * np.sin(angle))
			y2 = robotY + int(int(n) * np.cos(angle))
			collision = False
			for wall in walls:
				# collision with wall or arena boundaries
				if lineRectCollision(wall, x1, y1, x2, y2) or x2 < 0 or x2 > arenaWidth or y2 < 0 or y2 > arenaLength: 
					# print(f"Collision with wall {wall}")
					collision = True
					break
			if not collision:
				robotX = x2
				robotY = y2
				positions.append( [robotX, robotY] )

	# print(f"positions : {positions}")
	# TODO measure distance from objective
	distanceFromObjective = np.sqrt((objectiveX - robotX)**2 + (objectiveY - robotY)**2)
	
	# this is optional, argument "visualize" has to be explicitly set to "True" when function is called
	if visualize :
		
		import matplotlib.pyplot as plt
		import matplotlib.patches as patches
		figure = plt.figure()
		ax = figure.add_subplot(111)
		
		# plot initial position and objective
		ax.plot(startX, startY, 'r^', label="Initial position of the robot")
		ax.plot(objectiveX, objectiveY, 'gx', label="Position of the objective")
		
		# plot the walls
		for wall in walls :
			ax.add_patch(patches.Rectangle( (wall["x"], wall["y"]), wall["width"], wall["height"] ))
		
		# plot a series of lines describing the movement of the robot in the arena
		for i in range(1, len(positions)) :
			ax.plot( [ positions[i-1][0], positions[i][0] ], [ positions[i-1][1], positions[i][1] ], 'r-', label="Robot path" )
		
		ax.set_title("Movements of the robot inside the arena")
		ax.legend(loc='best')
		plt.show()

	return distanceFromObjective

################# MAIN
def main() :
	
	# first, let's see what happens with an empty list of commands
	listOfCommands = []
	fitnessRobot(listOfCommands, visualize=True)
	
	return 0

if __name__ == "__main__" :
	sys.exit( main() )

