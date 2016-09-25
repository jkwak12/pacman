from __future__ import print_function
import heapq
from collections import deque
import time
import sys
import pdb
import os
import shutil


''' Get coordinates of pacman'''
def getStart(maze):
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if maze[i][j] == 'P':
				return (i,j)

'''Get coordinates of a single dot (relevent only for finding a single dot)'''
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


''' Transfer 2D maze array to graph'''
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




'''Manhattan distance between 2 cells'''
def manhattan(cell, goal):
	return abs(cell[0]-goal[0]) + abs(cell[1] - goal[1])


'''*****************************START OF SINGLE PATH ALGORITHMS*********************************** '''


'''single path for bfs search'''
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

'''single path for dfs search'''
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


'''single path for a* search'''
def find_path_astar(maze):
	start, goal = getStart(maze), getGoal(maze)
	priority_queue = []
	heapq.heappush(priority_queue, (0+manhattan(start,goal), 0,[start], start))
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
			heapq.heappush(priority_queue,(g + manhattan(neighbour,goal), g+1, path + [neighbour], neighbour))
	

'''single path for greedy search'''
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

'''****************************************END OF SINGLE PATH ALGORITHMS******************** '''



''' heuristic = astar distance to closest dot + astar distance from the closest dot found to the farthest dot
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


'''This is our heuristic definition for the extra credit portion of the MP '''
def dotHeuristic_extra(graph, cell, dots):
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





''' ................The following code run 1.2 custom-A* to visit each dot with the heuristic defined above.................. '''


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
			return (total_path, dotOrder, expanded)
		
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


''' ...................The following code run 1.2 EXTRA-CREDIT custom-A* to visit each dot with the heuristic defined above.........'''
def find_path_astar_multi_extra(graph, start, dots):
	#start, goal = getStart(maze), getGoal(maze)
	priority_queue = []
	path_continued = []
	total_path = []
	dotOrder = []
	#goals = dots
	heapq.heappush(priority_queue, (0+dotHeuristic_extra(graph,start,dots), 0, [start], start))
	visited = set()
	expanded = 0
	#graph = maze2graph(maze)
	while priority_queue:
		cost, g, path, current = heapq.heappop(priority_queue)
		
		
		if not len(dots):
			return (total_path, dotOrder, expanded)
		
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
			heapq.heappush(priority_queue,(g + dotHeuristic_extra(graph,neighbour,dots), g+1, path + [neighbour], neighbour))


'''........................................bfs for debugging...........................................'''


def find_path_bfs(graph, start, dots):
	queue = deque ([([start], start)])
	visited = set()
	while queue:
		path,current = queue.popleft()
		if not len(dots):
			return total_path
		if current in dots:
			dots.remove(current)
			visited = set()
			total_path = path
			queue = deque ([([current], current)])
		if current in visited:
			continue
		visited.add(current)
		for neighbour in graph[current]:
			queue.append((path + [neighbour], neighbour))


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



'''for main function placement, place this function at the end of your main function calls, 
		BUT before any print statements. This function will clear your terminal. 		'''
def animate_path(maze, path):
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
	delay = (80/(len(maze)*len(maze[0])))  		#the delay between frames. The larger the maze, the shorter the delay
	while frame < frameNumber:
		os.system('cls' if os.name == 'nt' else 'clear')		#clear screen
		f = open('animation/frame' + str(frame) + '.txt')
		print(f.read())
		f.close()
		time.sleep(delay)	  
		frame += 1
	shutil.rmtree(folder) #delete frame files
	return



'''print single path as trail of dots'''
def print_single_path(maze, path):
	f = open('solution.txt','w')
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if (i,j) in path and not (i,j) == getStart(maze) and not (i,j) == getGoal(maze):
				f.write('.')
			else:
				f.write(maze[i][j])
		f.write('\n')
	


#******************************************MAIN***************************************

Maze = open(sys.argv[1]).read().split('\n')
graph = maze2graph(Maze)
dots = getDots(Maze)
pacman = getStart(Maze)

method = ''
animate = ''
extra = ''
process = raw_input("\nEnter the dot search method you wish to use (s/m) <--> (single, multi):")
if process == 'm' or process == 'M' or process == 'multi':
	extra = raw_input("\nAre you running the extra credit heuristic?\n(y/n, or just hit enter to run the main 1.2 heuristic):")
	if(extra == 'y' or extra == 'Y' or extra == 'yes'):
		path, solutions, expanded = find_path_astar_multi_extra(graph,pacman,dots)
	else:
		path, solutions, expanded = find_path_astar_multi(graph,pacman,dots)
		write_dots_eaten(Maze, solutions, path)
	animate = raw_input('Do you want a terminal animation of the maze (y/n)?')
elif process == 's' or process == 'S' or process == 'single':
	method = raw_input("\nEnter the algorithm you wish to use (dfs, bfs, greedy, a*): ")
	print('\n')
else:
	#force quit the script if invalid method
	print('\nINVALID INPUT\nEXITING...')
	sys.exit() 
if method == 'dfs' and (process == 's' or process == 'S' or process == 'single'):
	path, visited = find_path_dfs(Maze)
	expanded = len(visited)
	print_single_path(Maze, path)
	animate = raw_input('Do you want a terminal animation of the maze (y/n)?')
elif method == 'bfs' and (process == 's' or process == 'S' or process == 'single'):
	path, visited = find_path_bfs(Maze)
	expanded = len(visited)
	print_single_path(Maze, path)
	animate = raw_input('Do you want a terminal animation of the maze (y/n)?')
elif method == 'greedy' and (process == 's' or process == 'S' or process == 'single'):
	path, visited = find_path_greedy_best_first(Maze)
	expanded = len(visited)
	print_single_path(Maze, path)
	animate = raw_input('Do you want a terminal animation of the maze (y/n)?')
elif method == 'a*' and (process == 's' or process == 'S' or process == 'single'):
	path, visited = find_path_astar(Maze)
	expanded = len(visited)
	print_single_path(Maze, path)
	animate = raw_input('Do you want a terminal animation of the maze (y/n)?')
elif process == 's' or process == 'S' or process == 'single':
	#force quit the script if invalid method
	print('\nINVALID INPUT\nEXITING...')
	sys.exit() 

#animation prompt for user
if animate == 'y' or animate == 'Y' or animate == 'yes':
	animate_path(Maze,  path)

''' Print path and path length '''
print('\nPath length is', len(path), '(including start position of pacman)')
print('Path length is', (len(path) - 1), '(NOT including start position of pacman)')
print('Expanded node count is', expanded)
print('Solution path and/or order of dots traversed printed as solution.txt in current directory...')
