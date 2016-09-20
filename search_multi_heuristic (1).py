from __future__ import print_function
import heapq
from collections import deque
import time

Maze = open('TinySearch.txt').read().split('\n')

''' Get cordinates of pacman'''
def getStart(maze):
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if maze[i][j] == 'P':
				return (i,j)

''' Get a list of dot positions'''
def getDots(maze):
	dots = []
	for i in range(len(maze)):
		for j in range(len(maze[0])):
			if maze[i][j] == '.':
				dots.append((i,j))
	return dots

#goal = getGoal(Maze)
#print(goal)

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

graph = maze2graph(Maze)


'''Manhattan distance between 2 cells'''
def manhattan(cell, goal):
	return abs(cell[0]-goal[0]) + abs(cell[1] - goal[1])



''' heuristic = manhattan distance to closest dot + manhattan distance to farest dot
'''
def dotHeuristic(cell,dots):
	heuristic = 0
	next_position = cell
	next_dots = dots
	#print(next_position)
	for i in range(2):
		distances = []
		for dot in next_dots:
			distances.append((dot,manhattan(next_position,dot)))
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








dots = getDots(Maze)
#distances = dotDistances(graph, dots)
pacman = getStart(Maze)









''' .............................The following code run A* to visit each dot with the heuristic defined above........................... '''


def find_path_astar(graph, start, dots):
	#start, goal = getStart(maze), getGoal(maze)
	priority_queue = []
	path_continued = []
	#goals = dots
	heapq.heappush(priority_queue, (0+dotHeuristic(start,dots), 0, [start], start))
	visited = set()
	#graph = maze2graph(maze)
	while priority_queue:
		cost, g, path, current = heapq.heappop(priority_queue)
		#print(current)
		
		if not len(dots):
			return path, len(visited)
		#if current in visited:
		#	continue
		visited.add(current)
		
		if current in dots:
			dots.remove(current)
			#path_continued += path
			
		#print(current)
		for neighbour in graph[current]:
			#if len(goals):
			#if current in dots:
			#	heapq.heappush(priority_queue,(g + dotHeuristic(neighbour,dots), g+1, [neighbour], neighbour))
			heapq.heappush(priority_queue,(g + dotHeuristic(neighbour,dots), g+1, path + [neighbour], neighbour))



''' Print path and path length '''
path, nodesExpended = find_path_astar(graph,pacman,dots)

for i in range(len(Maze)):
	for j in range(len(Maze[0])):
		if (i,j) in path and not (i,j) == getStart(Maze) and not (i,j) == getDots(Maze):
			print('.', end = '')
		#if (i,j) in path and (i,j) in getDots(Maze):
		#	print(',', end = '')
		else:
			print(Maze[i][j], end = '')
	print('\n')

print('Path length is', len(path))
print('Number of nodes expended is', nodesExpended)
print(path)




