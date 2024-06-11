# Simple script that simulates a bot moving inside an Arena, following a series of commands
# by Alberto Tonda, 2018 <alberto.tonda@gmail.com>

import sys
import random
import time
import numpy as np
import inspyred

def lineRectCollision(wall : dict, x1 : int, y1 : int, x2 : int, y2 : int) -> bool:
	# check if the line defined by points (x1,y1) and (x2,y2) intersects with the rectangle defined by the wall
	# bottom-left corner of the wall
	xMin = wall["x"]
	yMin = wall["y"]
	# top-right corner of the wall
	xMax = wall["x"] + wall["width"]
	yMax = wall["y"] + wall["height"]
	
	# check if the line intersects with the rectangle
	def ccw(A,B,C):
		return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
	
	def intersect(A,B,C,D):
		return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
	
	return intersect( (x1,y1), (x2,y2), (xMin,yMin), (xMax,yMin) ) or intersect( (x1,y1), (x2,y2), (xMax,yMin), (xMax,yMax) ) or intersect( (x1,y1), (x2,y2), (xMax,yMax), (xMin,yMax) ) or intersect( (x1,y1), (x2,y2), (xMin,yMax), (xMin,yMin) )



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


# returns the sum of all the N in "move N" of commandList
# sum_of_distances(["move 10", "move 15", "rotate 120"]) returns 25
def sum_of_distances(commandList):
	total=0
	for command in commandList:
		name, n = command.split(' ')
		if name == "move":
			total += int(n)
	return total

'''This function accepts in input a list of strings, and tries to parse them to update the position of a robot. Then returns distance from objective.'''
def fitnessRobot(listOfCommands, visualize=False) :
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
		n = int(n)
		if commandType == "rotate":
			angle += n * np.pi / 180
		elif commandType == "move":
			x1 = robotX
			y1 = robotY
			x2 = robotX + int(n * np.sin(angle))
			y2 = robotY + int(n * np.cos(angle))
			collision = False
			for wall in walls:
				# collision with wall or arena boundaries
				if lineRectCollision(wall, x1, y1, x2, y2) or x2 < 0 or x2 >= arenaWidth or y2 < 0 or y2 >= arenaLength: 
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
			ax.plot( [ positions[i-1][0], positions[i][0] ], [ positions[i-1][1], positions[i][1] ], 'r-')
		
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



def crossover_arenabot1(random, candidates, args):
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
	return children

def crossover_arenabot2(random, candidates, args):
	crossover_rate = args['crossover_rate']
	children = []
	for c in range(len(candidates)):
		parent1, parent2 = random.choices(candidates, k=2)
		if random.random() < crossover_rate:
			randIndex1 = random.randint(0, len(parent1)-1)
			randIndex2 = random.randint(0, len(parent2)-1)
			child1 = parent1[0:randIndex1] + parent2[randIndex2:]
			child2 = parent2[0:randIndex2] + parent1[randIndex1:]
		else:
			child1 = parent1.copy()
			child2 = parent2.copy()
		children.append(child1)
		children.append(child2)
	
	children.sort(key=fitnessRobot)
	return children[:len(candidates)]
		


boundMax = {
	"move": 100,
	"rotate": 360 
}

def mutator_arenabot(random, candidates, args):
	mutation_rate = args['mutation_rate']

	new_candidates = []
	for candidate in candidates:
		# randomly mutate one existing command
		if (random.random() < mutation_rate):
			randomIndex = random.randint(0, len(candidate)-1)
			name, param = candidate[randomIndex].split(' ')
			param = int(param)
			# 50% chance of changing command name
			if (random.randint(0, 1) == 1):
				if name == "move":
					name = "rotate"
				else:
					name = "move"
				param = random.randint(0, boundMax[name])
			
			# Little modification of numeric argument 
			if name == "move":
				param *= random.gauss(1, 0.1)
				param = int(max(min(param, 100), 0))
			else:
				param *= random.gauss(1, 0.8) 
				param  = int(param) % 360
			
			candidate[randomIndex] = f"{name} {param}"


		# randomly add a new command
		if (random.random() < mutation_rate):
			name = random.choice(["move", "rotate"])
			param = random.randint(0, boundMax[name])
			candidate.append(f"{name} {param}")
		new_candidates.append(candidate)
			
	return new_candidates


# stops if either maximum number of evaluations has been reached or one individual has reached
# a fitness of zero
def terminator_arenabot(population, num_generations, num_evaluations, args):
	if inspyred.ec.terminators.evaluation_termination(population, num_generations, num_evaluations, args):
		return True
	if 0 in [ind.fitness for ind in population]:
		return True

	return False


def test_plan():
	random_gen = random.Random()
	random_gen.seed(time.time())

	ev_algo = inspyred.ec.EvolutionaryComputation(random_gen)
	ev_algo.selector = inspyred.ec.selectors.tournament_selection
	ev_algo.variator = [crossover_arenabot2, mutator_arenabot]
	ev_algo.replacer = inspyred.ec.replacers.plus_replacement
	ev_algo.terminator = terminator_arenabot
	#ev_algo.observer = inspyred.ec.observers.best_observer

	
	pop_size_tab = range(69, 73, 1)
	crossover_rate_tab = np.linspace(0.8, 1, 15)
	mutation_rate_tab = np.linspace(0, 0.2, 15)

	N = 3

	best_pop_size = pop_size_tab[0]
	best_cross_rate = crossover_rate_tab[0]
	best_mutation_rate = mutation_rate_tab[0]

	best_average = 1000

	
	i = 0
	for pop_size in pop_size_tab:
		for crossover_rate in crossover_rate_tab:
			for mutation_rate in mutation_rate_tab:
				if crossover_rate + mutation_rate < 1:
					cur_average = 0
					for i in range(N):
						final_pop = ev_algo.evolve(
								generator=generator_arenabot,
								evaluator=evaluator_arenabot,
								pop_size=pop_size,
								num_selected=pop_size*2,
								maximize=False,
								max_evaluations=5000,
								crossover_rate=crossover_rate,
								mutation_rate=mutation_rate
						)
						cur_average += final_pop[0].fitness
					cur_average /= N
					if cur_average < best_average:
						best_average = cur_average
						best_pop_size = pop_size
						best_cross_rate = crossover_rate
						best_mutation_rate = mutation_rate
					i += 1
					print(f"Current {i}: {cur_average} ({pop_size} {crossover_rate} {mutation_rate})")
	print(f"Best: {best_average} ({best_pop_size} {best_cross_rate} {best_mutation_rate})")





################# MAIN
def main() :
	random_gen = random.Random()
	random_gen.seed(time.time())

	ev_algo = inspyred.ec.EvolutionaryComputation(random_gen)
	ev_algo.selector = inspyred.ec.selectors.tournament_selection
	ev_algo.variator = [crossover_arenabot2, mutator_arenabot]
	ev_algo.replacer = inspyred.ec.replacers.plus_replacement
	ev_algo.terminator = terminator_arenabot
	ev_algo.observer = inspyred.ec.observers.best_observer
	
	final_pop = ev_algo.evolve(
		generator=generator_arenabot,
		evaluator=evaluator_arenabot,
		pop_size=70,
		num_selected=2*70,
		maximize=False,
		max_evaluations=10000,
		mutation_rate=0.1,
		crossover_rate=0.9
	)

	best = final_pop[0]
	print("Best fitness {}".format(best.fitness))
	fitnessRobot(best.candidate, True)
	
	return 0

if __name__ == "__main__" :
	sys.exit(test_plan())


