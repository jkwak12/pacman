from __future__ import print_function
import heapq
import sys
import time
import shutil
import os
from collections import deque
import pdb

Maze = open(sys.argv[1]).read().split('\n')
#Maze = open('MediumMaze.txt').read().split('\n')

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

''' Get a list of dot positions'''
def getDots(maze):
	dots = []
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if maze[i][j] == '.':
				dots.append((i,j))
	return dots


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


def manhattan(cell, goal):
	return abs(cell[0]-goal[0]) + abs(cell[1] - goal[1])


def find_path_astar(maze):
	start, goal = getStart(maze), getGoal(maze)
	priority_queue = []
	heapq.heappush(priority_queue, (0+manhattan(start,goal), 0,[start], start))
	visited = set()
	graph = maze2graph(maze)
	while priority_queue:
		cost, g, path, current = heapq.heappop(priority_queue)
		#print(len(path))
		if current == goal:
			return (path,visited)
		if current in visited:
			continue
		visited.add(current)
		for neighbour in graph[current]:
			heapq.heappush(priority_queue,(g + manhattan(neighbour,goal), g+1, path + [neighbour], neighbour))


def find_path_greedy_best_first(maze):
	start, goal = getStart(maze), getGoal(maze)
	priority_queue = []
	heapq.heappush(priority_queue, (manhattan(start,goal),[start], start))
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
			heapq.heappush(priority_queue,(manhattan(neighbour,goal), path + [neighbour], neighbour))





''' heuristic = astar distance to closest dot + astar distance from the closest dot found to the farest dot
'''
def dotHeuristic(graph, cell,dots):
	heuristic = 0
	next_position = cell
	next_dots = dots
	#print(next_position)
	for i in range(2):
		distances = []
		for dot in next_dots:
			distances.append((dot, manhattan(next_position,dot)))
		#Object = (cell, dot)
		if (len(distances) == 0):
			break

		dot_closest, dist_closest = distances[0]
		for (dot, dist) in distances:
			if (i==0) and (dist < dist_closest):
				dot_closest = dot
				dist_closest = dist
			if (i==1) and (dist > dist_closest):
				dot_closest = dot
				dist_closest = dist
		heuristic += dist_closest
		next_position = dot_closest
		#next_dots.remove(dot_closest)
	return heuristic


def dotDistance(graph,dot1,dot2):
	start = dot1
	goal = dot2
	priority_queue = []
	heapq.heappush(priority_queue,(0+manhattan(start,goal), 0, [start], start))
	visited = set()
	while priority_queue:
		cost, g, path, current = heapq.heappop(priority_queue)
		if current == goal:
			return len(path)
		if current in visited:
			continue
		visited.add(current)
		for neighbour in graph[current]:
			heapq.heappush(priority_queue,(g + manhattan(neighbour,goal), g+1, path + [neighbour], neighbour))






dots = getDots(Maze)
#distances = dotDistances(graph, dots)
pacman = getStart(Maze)









''' .............................The following code run A* to visit each dot with the heuristic defined above........................... '''


def find_path_astar_multi(graph, start, dots):
	#start, goal = getStart(maze), getGoal(maze)
	priority_queue = []
	path_continued = []
	total_path = []
	dotOrder = []
	#goals = dots
	heapq.heappush(priority_queue, (0+dotHeuristic(graph,start,dots), 0, [start], start))
	visited = set()
	expanded = 0
	#graph = maze2graph(maze)
	while priority_queue:
		cost, g, path, current = heapq.heappop(priority_queue)
		
		
		if not len(dots):
			return (total_path, expanded, dotOrder)
		
		if current in dots:
			dots.remove(current)
			dotOrder.append(current)
			total_path = path
			expanded += len(visited)
			visited = set()
			priority_queue = []

		if current in visited:
			continue
		visited.add(current)
			
		#print(current)
		for neighbour in graph[current]:
			heapq.heappush(priority_queue,(g + dotHeuristic(graph,neighbour,dots), g+1, path + [neighbour], neighbour))





'''for main function placement, place this function at the end of your main function calls, 
		BUT before any print statements. This function will clear your terminal. 		'''
def animate_path(maze, dots, path):
	frameNumber = 0
	start = path[0]
	goal = path[len(path) - 1]
	pacmanStart = getStart(maze)
	writePacman = 1
	folder = 'animation'
	if not os.path.isdir(folder):	#just some checking for existence of frames folder
		os.mkdir(folder)
	for current in path:
		tempPath = path[0:(frameNumber + 1)] 
		frameString = 'animation/frame' + str(frameNumber) + '.txt'
		f = open(frameString, 'w+')
		for i in range(len(maze)):
			for j in range(len(maze[0])):
				if (i,j) == tempPath[len(tempPath) - 1] and writePacman:
					f.write('P')
					writePacman = 0
				elif (i,j) in tempPath and not (i,j) == start and not (i,j) == goal:
					f.write(' ')
				elif (i,j) == pacmanStart:
					f.write(' ')
				else:
					f.write(Maze[i][j])
			f.write('\n')
		#pdb.set_trace()
		frameNumber += 1
		writePacman = 1
		f.close()
	frame = 0
	delay = 0.1  		#the delay between frames. The more dots, the shorter the delay
	while frame < frameNumber:
		os.system('cls' if os.name == 'nt' else 'clear')		#clear screen
		f = open('animation/frame' + str(frame) + '.txt')
		print(f.read())
		f.close()
		time.sleep(delay)	  
		frame += 1
	shutil.rmtree(folder) #delete frame files
	return


#******************MAIN******************

#Path alogorithm used
method = ''
process = raw_input("\nEnter the method you wish to use (single, multi):")
print('\n')
if process == 'multi':
	#path,visited = find_path_dots(Maze)
	graph = maze2graph(Maze)
	path, expanded, dotOrder = find_path_astar_multi(graph, pacman, dots)
	#sys.exit()		#force quit script. Not elegant, I know
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


#dots_original = getDots(Maze)
animate_path(Maze,dotOrder,path)

#following print routines work if we're just path finding (single dot)
print ("Width of maze:", len(Maze[0]))
print ("Height of maze:", len(Maze))
print ("Total nodes calculated:", (expanded))
print ("Length of calculated path:", (len(path)))
print ("Solution.txt was printed in directory")
#print (Maze)





#**********Solution***********

'''Just drag this method into your main file somewhere'''


#assume the list 'dots' is already in the order they were eaten in
#also added back in the printing of dots along the path since a TA in office hours
#		said that they wanted the visible path alongside the order of dots eaten
def write_dots_eaten(maze, dots, path):
	f = open('solution.txt','w+')
	start = path[0]
	goal = path[len(path) - 1]
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if (i,j) in path and not (i,j) == start and not (i,j) == goal:
				f.write('.')
			else:
				f.write(Maze[i][j])
		f.write('\n')	
	'''done writing maze state with path of dots'''

	'''now including the order of initial dots eaten'''	
	f.seek(0,0)	#move file cursor to beginning
	contents = f.read()
	count = 48		#ascii value for '0'
	for dot in dots:
		x = dot[1]	#column value 
		y = dot[0]	#row value
		index = y*(len(maze[0]) + 1) + x
		contents = contents[:index] + chr(count) + contents[index+1:]	#some crazy offset
		count += 1
		if(count == 58): 	#is this ascii value '10'?
			count = 97		#ascii value for 'a'
		elif(count == 123):	#is this value greater than alphabet length?
			count = 65		#ascii value for 'A'
	f.seek(0,0)
	f.write(contents)
	f.close()

write_dots_eaten(Maze, dotOrder, path)
#print(Solution)
