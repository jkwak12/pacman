from __future__ import print_function
import heapq
import sys
import math
import copy
from collections import deque

import pdb



Maze = open(sys.argv[1]).read().split('\n')


def getStart(maze):
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if maze[i][j] == 'P':
				return (i,j)

def getGoal(maze):
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if maze[i][j] == '.':
				return (i,j)


def maze2graph(maze):
	height = len(maze)
	width = len(maze[0]) if height else 0
	graph = {(i,j):[] for j in range(width) for i in range(height) if not maze[i][j] == '%'}
	for row, col in graph.keys():
		if row < height - 1 and not maze[row+1][col]=='%':
			graph[(row, col)].append((row + 1, col))
			graph[(row + 1, col)].append((row, col))
		if col < width - 1 and not maze[row][col + 1]=='%':
			graph[(row, col)].append((row, col + 1))
			graph[(row, col + 1)].append((row, col))
	return graph

def find_path_bfs(maze):
	start, goal = getStart(maze),getGoal(maze)
	queue = deque ([([start], start)])
	visited = set()
	graph = maze2graph(maze)
	while queue:
		path,current = queue.popleft()
		if current == goal:
			return (path,visited)
		if current in visited:
			continue
		visited.add(current)
		for neighbour in graph[current]:
			queue.append((path + [neighbour], neighbour))
	#return ''NO WAY''

def find_path_dfs(maze):
	start,goal = getStart(maze),getGoal(maze)
	stack = deque([([start], start)])
	visited = set()
	graph = maze2graph(maze)
	while stack:
		path,current = stack.pop()
		if current == goal:
			return (path,visited)
		if current in visited:
			continue
		visited.add(current)
		for neighbour in graph[current]:
			stack.append((path+[neighbour],neighbour))


def heuristic(cell, goal):
	return abs(cell[0]-goal[0]) + abs(cell[1] - goal[1])



def find_path_astar(maze):
	start, goal = getStart(maze), getGoal(maze)
	priority_queue = []
	heapq.heappush(priority_queue, (0+heuristic(start,goal), 0,[start], start))
	visited = set()
	graph = maze2graph(maze)
	while priority_queue:
		cost, g, path, current = heapq.heappop(priority_queue)
		if current == goal:
			return (path,visited)
		if current in visited:
			continue
		visited.add(current)
		for neighbour in graph[current]:
			heapq.heappush(priority_queue,(g + heuristic(neighbour,goal), g+1, path + [neighbour], neighbour))
	

def find_path_greedy_best_first(maze):
	start, goal = getStart(maze), getGoal(maze)
	priority_queue = []
	heapq.heappush(priority_queue, (heuristic(start,goal),[start], start))
	visited = set()
	graph = maze2graph(maze)
	while priority_queue:
		cost, path, current = heapq.heappop(priority_queue)
		if current == goal:
			return (path,visited)
		if current in visited:
			continue
		visited.add(current)
		for neighbour in graph[current]:
			heapq.heappush(priority_queue,(heuristic(neighbour,goal), path + [neighbour], neighbour))

def getDots(maze):
	dots = []
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if maze[i][j] == '.':
				dots.append((i,j))
	return dots

def getPacman(maze):
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if maze[i][j] == 'P':
				return (i,j)

#run an A* search between 2 dots
#return LENGTH of the path found
def dotDistance(dot1, dot2):
	start = dot1
	goal = dot2
	priority_queue = []
	heapq.heappush(priority_queue, (0+heuristic(start,goal), 0,[start], start))
	visited = set()
	graph = maze2graph(Maze)
	while priority_queue:
		cost, g, path, current = heapq.heappop(priority_queue)
		if current == goal:
			return len(path) 	#return the number of steps
		if current in visited:
			continue
		visited.add(current)
		for neighbour in graph[current]:
			heapq.heappush(priority_queue,(g + heuristic(neighbour,goal), g+1, path + [neighbour], neighbour))

#will return a dictionary with distances as a main value.
#Approximate run time is ----> (runtime of A* search)(n^2-n)/2, where n = # of dots in maze
#entry = (dot1 location, dot2 location):distance between dot1<-->dot2
def dotDistances(dots):
	distances = {}
	while len(dots) > 1:		# at least 2 dots to calculate distance?
		x = dots[0]
		i = 1     	#start with the 2nd dot in list
		dot1 = x	#beginning dot
		while i < len(dots):
			dot2 = dots[i]	#temporary goal dot
			distance = dotDistance(dot1, dot2)
			dotObject = (dot1, dot2)    		#will be key for dictionary
			distances[dotObject] = distance 	#update dictionary
			i += 1	
		dots = dots[1:]		#slice off the current 1st dot of our working list
	return distances

def find_distance_astar(maze, pos1, pos2):
	start, goal = pos1, pos2
	priority_queue = []
	heapq.heappush(priority_queue, (0+heuristic(start,goal), 0,[start], start))
	visited = set()
	graph = maze2graph(maze)
	while priority_queue:
		cost, g, path, current = heapq.heappop(priority_queue)
		if current == goal:
			return (path,visited) 
		if current in visited:
			continue
		visited.add(current)
		for neighbour in graph[current]:
			heapq.heappush(priority_queue,(g + heuristic(neighbour,goal), g+1, path + [neighbour], neighbour))

#will essentially perform part 1.2 of the MP
'''		As of this current build, the list "solutions" will hold the order of dots visited in the (hopefully)
shortest path possible (again, not sure due to how I disregard tiebreaking).
		Also not sure what the MP means by a "top-level routine" as I've ONLY used A* searching'''
def find_path_dots(maze):
	solutions = []				   #will hold ordered coordinates of visited dots
	pacman = getPacman(Maze)       #returns pacman's coordinate's
	dots = getDots(Maze)
	distances = dotDistances(dots)
	distancesKeys = distances.keys()
	while len(dots) > 1:
		'''dictionary key that holds largest distance value'''
		furthestDots = max(distances.iterkeys(), key=(lambda key: distances[key]))   
		largestDistance = distances[furthestDots]
		dot1 = furthestDots[0]
		dot2 = furthestDots[1]
		#print ('Coordinates of farthest dots:', furthestDots)
		#print ('Largest distance:', largestDistance)
		path1, visited1 = find_distance_astar(Maze, pacman, dot1)
		path2, visited2 = find_distance_astar(Maze, pacman, dot2)
		'''Note: I'm still undecided on how to handle 
		tie-breaking at this point...:('''
		if len(path1) <= len(path2):    
			truePath = path1
		else:
			truePath = path2
		#print ('List of dots in maze:', dots)
		#print ('Pacman path will be:', truePath)
		#pdb.set_trace()	was using pdb for debugging stuff...
		removedDot = dots[len(dots)-1]	
		for cell in truePath:		
			if cell in dots: 		#check whether current cell in true path is a dot
				solutions.append(cell)   #add dot to solutions
				removedDot = cell  #pacman will go to this dot
				break
		saveKeys = copy.deepcopy(distancesKeys)		#needed to make a deep copy of this or I would be affecting 
														#distanceKeys too when doing "saveKeys.remove(coord)"
		for coord in distancesKeys:
			if removedDot == coord[0] or removedDot == coord[1]:
				#print('coord:', coord)
				saveKeys.remove(coord)
				del distances[coord]	  #get rid of any distance associated with deleted dot
		del distancesKeys
		distancesKeys = saveKeys
		#sys.exit()
		dots.remove(removedDot)	 	#removed "eaten" dot from maze
		pacman = removedDot         #position pacman to be at the "eaten" dot
	#at this point, there is one dot left in the maze!
	removedDot = dots.pop()
	solutions.append(removedDot)
	pacman = removedDot 			#probably pointless but idgaf
	print('Order of dots collected is:', solutions)
	'''still undecided on the return types of this method...maybe returns "solutions...?'''
	return 
	#return (path, visited)
	

#******************MAIN******************

#Path alogorithm used
method = ''
process = raw_input("\nEnter the method you wish to use (single, multi):")
print('\n')
if process == 'multi':
	#path,visited = find_path_dots(Maze)
	find_path_dots(Maze)
	sys.exit()		#force quit script. Not elegant, I know
elif process == 'single':
	method = raw_input("\nEnter the algorithm you wish to use (dfs, bfs, greedy, a*): ")
	print('\n')
if method == 'dfs' and process == 'single':
	path,visited = find_path_dfs(Maze)
elif method == 'bfs' and process == 'single':
	path,visited = find_path_bfs(Maze)
elif method == 'greedy' and process == 'single':
	path,visited = find_path_greedy_best_first(Maze)
elif method == 'a*' and process == 'single':
	path,visited = find_path_astar(Maze)
else:
	#force quit the script if invalid method
	print('\nINVALID INPUT\nEXITING...')
	sys.exit()


#following print routines work if we're just path finding (single dot)
print ("Width of maze:", len(Maze[0]))
print ("Height of maze:", len(Maze))
print ("Total nodes calculated:", (len(visited)))
print ("Length of calculated path:", (len(path)))
print ("Solution.txt was printed in directory")
#print (Maze)
'''
for i in range(len(Maze)):
	for j in range(len(Maze[0])):
		if (i,j) in path and not (i,j) == getStart(Maze) and not (i,j) == getGoal(Maze):
			print('.', end = '')
		else:
			print(Maze[i][j], end = '')
	print('\n')
'''


#**********Solution***********
f = open('solution.txt','w')
for i in range(len(Maze)):
	for j in range(len(Maze[0])):
		if (i,j) in path and not (i,j) == getStart(Maze) and not (i,j) == getGoal(Maze):
			f.write('.')
			#print(".")
		else:
			f.write(Maze[i][j])
			#print(Maze[i][j])
	f.write('\n')

#print(Solution)
