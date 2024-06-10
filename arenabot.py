# Simple script that simulates a bot moving inside an Arena, following a series of commands
# by Alberto Tonda, 2018 <alberto.tonda@gmail.com>

import sys
import random
import numpy as np
import inspyred

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
	startDegrees = 90 # 90°
	
	# position of the objective
	objectiveX = 90
	objectiveY = 90
	
	# this is a list of points that the robot will visit; used later to visualize its path
	positions = []
	positions.append( [robotX, robotY] )
	
	# TODO move robot, check that the robot stays inside the arena
	angle = 0
	for command in listOfCommands:
		commandType, n = command.split(' ')
		n = int(n)
		if commandType == "rotate":
			angle += n*np.pi/180
		elif commandType == "move":
			robotX += int(n*np.sin(angle))
			robotY += int(n*np.cos(angle))
			positions.append( [robotX, robotY] )
	#print(f"New position {robotX}, {robotY}")


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



def generator_arenabot(random, args):
	commandList = []
	for i in range(3):
		n = random.randint(0, 360)
		commandList.append(f"rotate {n}")
		n = random.randint(0, 100)
		commandList.append(f"move {n}")
	return commandList

def evaluator_arenabot(candidates, args):
	return [fitnessRobot(c, False) for c in candidates]



def crossover_arenabot(random, candidates, args):
	crossover_rate = args['crossover_rate']

	# Crossover and mutation mechanisms
	children = []
	for c in range(len(candidates)):
		# Do the crossover by selecting two different candidates
		parent1, parent2 = random.choices(candidates, k=2)
		# parent1 will be the one with minimum length
		if len(parent1) > len(parent2):
			parent1, parent2 = parent2,parent1

		child = []
		for i in range(len(parent1)):
			child.append(random.choice([parent1[i], parent2[i]]))
		child += parent2[len(parent1):]
	children.append(child)


def mutator_arenabot(random, candidates, args):
	mutation_rate = args['mutation_rate']

	for candidate in candidates:
		if (random.random() < mutation_rate):
			randomIndex = random.randint(0, len(candidate)-1)
			name, param = candidate[randomIndex].split(' ')
			param = int(param)
			# 50% chance of changing command name
			if (random.randint(0, 1) == 1):
				if name == "move":
					name = "rotate"
					param = random.randint(0, 360)
				else:
					name = "move"
					param = random.randint(0, 100)
			
			# Little modification of numeric argument 
			if name == "move":
				param *= random.gauss(1, 0.1)
				param = int(max(min(param, 100), 0))
			else:
				param *= random.gauss(1, 0.8) 
				param  = int(param) % 360
			
			candidate[randomIndex] = f"{name} {param}"
	return candidates




################# MAIN
def main() :
	random_gen = random.Random()
	random_gen.seed(42)

	ev_algo = inspyred.ec.EvolutionaryComputation(random_gen)
	ev_algo.selector = inspyred.ec.selectors.tournament_selection
	ev_algo.variator = [mutator_arenabot]
	ev_algo.replacer = inspyred.ec.replacers.plus_replacement
	ev_algo.terminator = inspyred.ec.terminators.evaluation_termination
	ev_algo.observer = inspyred.ec.observers.best_observer
	
	final_pop = ev_algo.evolve(
		generator=generator_arenabot,
		evaluator=evaluator_arenabot,
		pop_size=50,
		num_selected=100,
		maximize=False,
		max_evaluations=2000,
		mutation_rate=0.2,
		crossover_rate=0.8
	)

	best = final_pop[0]
	print("Best fitness {}".format(best))
	
	return 0

if __name__ == "__main__" :
	sys.exit( main() )

