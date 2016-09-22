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



''' Calculate distance between each 2 dots in the dots list, and keep them a dictionary.
     The keys are dot pairs, and values are corresponding distances'''
def dotDistances(graph, dots):
	distances = {}
	while len(dots) >1:
		x = dots[0]
		i = 1
		dot1 = x
		while i < len (dots):
			dot2 = dots[i]
			distance = dotDistance(graph, dot1, dot2)
			dotObject = (dot1, dot2)
			distances[dotObject] = distance
			i += 1
		dots = dots[1:]
	return distances


''' Run A* search between 2 dots. 
	Return Length of the path found.'''
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



''' Run A* search between pacman start position and each dot. 
	Return the dot with the shortest path.
	Pacman starts from this nearest dot. (Later this dot will be assigned as the root of MST)
'''
def getFirstDot(graph,pacman,dots):
	MinDist = dotDistance(graph,pacman,dots[0])
	Firstdot = dots[0]
	for dot in dots:
		dist = dotDistance(graph,pacman,dot)
		if dist < MinDist:
			MinDist = dist
			Firstdot = dot
	return Firstdot


dots = getDots(Maze)
distances = dotDistances(graph, dots)
pacman = getStart(Maze)
root = getFirstDot(graph,pacman,dots)
#print(distances)
#nearestgoal = NearestGoal(getStart(Maze), getGoal(Maze))
#print(nearestgoal)
#print(dist)



''' ...............The following code find the order of dots pacman should visit based on minimum spanning tree (MST)................'''

''' First, arrange the dots in a dictionary structure that shows all the vertices and edges
	goalStates = {'vertices': [list of all dots], 'edges': set{(distance, dot1, dot2)}'''
def getGoalStates(distances, dots):
	goalStates = {'vertices':dots, 'edges': set()}
	for key, value in distances.items():
		edge = (value, key[0],key[1])
		goalStates['edges'].add(edge)
	return goalStates

goalStates = getGoalStates(distances,dots)
#print(goalStates)



''' Initialize. At the initial state, no vertice is connected. Thus the parent of each vertice is itself '''
parent = dict()
rank = dict()
def make_set(vertice):
    parent[vertice] = vertice
    rank[vertice] = 0

''' Recursively find the root of a vertice'''
def find(vertice):
    if parent[vertice] != vertice:
        parent[vertice] = find(parent[vertice])
    return parent[vertice]

''' If cycles will not be created, connect two vertices (set one as the parent of the other one) '''
def union(vertice1, vertice2):
    root1 = find(vertice1)
    root2 = find(vertice2)
    if root1 != root2:
        if rank[root1] > rank[root2]:
            parent[root2] = root1
        else:
            parent[root1] = root2
            if rank[root1] == rank[root2]: rank[root2] += 1

''' Kruskal algorithm 
	Returns the minimun spanning tree in the form set{(weight, vertice1, vertice2)}'''
def kruskal(goalStates):
    for vertice in goalStates['vertices']:
        make_set(vertice)

    minimum_spanning_tree = set()
    edges = list(goalStates['edges'])
    edges.sort()
    for edge in edges:
        weight, vertice1, vertice2 = edge
        if find(vertice1) != find(vertice2):
            union(vertice1, vertice2)
            minimum_spanning_tree.add(edge)
    return minimum_spanning_tree

minimum_spanning_tree = kruskal(goalStates)
#print(minimum_spanning_tree)

''' Rearrange the MST as a graph '''
def mst2graph(dots, minimum_spanning_tree):
	tree = {(i,j):[] for (i,j) in dots}
	for dis, dot1, dot2 in minimum_spanning_tree:
		tree[dot1].append(dot2)
		tree[dot2].append(dot1)
	#visitOrder = dfs(tree, root)
	return tree

''' Transverse the tree in pre-order (Run dfs to visit every node of the tree) 
	Return the order of goals pacman should visit'''
def dfs(tree, root):
	stack = deque([root])
	visited = []
	while stack:
		vertex = stack.pop()
		if vertex not in visited:
			visited.append(vertex)
			for neighbour in tree[vertex]:
				stack.append( neighbour)
	return visited





#root = (3,4)
tree = mst2graph(dots, minimum_spanning_tree)
visitOrder = dfs(tree, root)
#print(tree)
#print(visitOrder)






''' .............................The following code run A* to visit each dot, in the order found just now........................... '''


def find_path_astar(graph, start, goal):
	#start, goal = getStart(maze), getGoal(maze)
	priority_queue = []
	heapq.heappush(priority_queue, (0+manhattan(start,goal), 0,[start], start))
	visited = set()
	#graph = maze2graph(maze)
	while priority_queue:
		cost, g, path, current = heapq.heappop(priority_queue)
		if current == goal:
			return path
		if current in visited:
			continue
		visited.add(current)
		for neighbour in graph[current]:
			heapq.heappush(priority_queue,(g + manhattan(neighbour,goal), g+1, path + [neighbour], neighbour))


def find_path_multi_astar(graph,visitOrder):
	path = []
	for i in range(len(visitOrder)-1):
		path = path + find_path_astar(graph,visitOrder[i],visitOrder[i+1])
	return path


''' Print path and path length '''
path = find_path_multi_astar(graph,visitOrder)

for i in range(len(Maze)):
	for j in range(len(Maze[0])):
		if (i,j) in path and not (i,j) == getStart(Maze) and not (i,j) == getDots(Maze):
			print('.', end = '')
		else:
			print(Maze[i][j], end = '')
	print('\n')

print(len(path))




